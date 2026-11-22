// Decode RoboSense packets with rs_driver (the library inside this image) and print what comes
// out, so two rs_driver versions can be compared on the same packets.
//
// The packets go through rs_driver's own LidarDriver (input type RAW_PACKET, the path
// rslidar_sdk uses for packets from ROS): the same decoders, frame splitting, point cloud header
// and queues as an online sensor, without a socket.
//
//   decode_dump synth <LIDAR_TYPE> <seed> <msop_packets> <min_distance> <max_distance>
//       Make packets from a seeded generator (layout per model: header id, block ids and
//       azimuths for mechanical models, packet sequence numbers for MEMS models; everything
//       else, distances and intensities included, pseudo-random), decode them and print one
//       line per point cloud with a hash of every point (x, y, z, intensity, ring, timestamp).
//   decode_dump file <LIDAR_TYPE> <packets.bin> <min_distance> <max_distance> [--points]
//       Decode packets from a file (records: u8 kind 0=MSOP 1=DIFOP, u32 length little-endian,
//       bytes) and print the same lines; with --points also every point.
//
// Build: g++ -std=c++17 -O2 -DUNIT_TEST -DDISABLE_PCAP_PARSE -DENABLE_DIFOP_PARSE
//        -DENABLE_MODIFY_RECVBUF -I <rs_driver>/src decode_dump.cpp -lpthread
// (UNIT_TEST only makes the decoders' constant parameters readable; it changes no decoding.)

#include <functional>
#include <memory>

#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/driver/decoder/decoder_factory.hpp>
#include <rs_driver/macro/version.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>

#include <arpa/inet.h>

#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace robosense::lidar;
typedef PointCloudT<PointXYZIRT> Cloud;

static std::mutex g_mtx;
static std::atomic<uint64_t> g_clouds{0};
static bool g_points = false;

static uint64_t fnv(const void* data, size_t len, uint64_t h = 1469598103934665603ULL)
{
  const uint8_t* p = (const uint8_t*)data;
  for (size_t i = 0; i < len; i++)
  {
    h ^= p[i];
    h *= 1099511628211ULL;
  }
  return h;
}

static void putCloud(std::shared_ptr<Cloud> c)
{
  std::lock_guard<std::mutex> lg(g_mtx);
  uint64_t h = fnv(c->points.data(), c->points.size() * sizeof(PointXYZIRT));
  size_t nan = 0;
  for (auto& p : c->points)
    if (std::isnan(p.x))
      nan++;
  printf("cloud seq=%u ts=%.6f height=%u width=%u points=%zu nan=%zu dense=%d frame=%s hash=%016" PRIx64 "\n",
         c->seq, c->timestamp, c->height, c->width, c->points.size(), nan, (int)c->is_dense,
         c->frame_id.c_str(), h);
  if (g_points)
  {
    for (auto& p : c->points)
      printf("p %.6f %.6f %.6f %.1f %u %.6f\n", p.x, p.y, p.z, (double)p.intensity, (unsigned)p.ring, p.timestamp);
  }
  g_clouds++;
}

static std::shared_ptr<Cloud> getCloud()
{
  return std::make_shared<Cloud>();
}

struct Rng  // splitmix64: the same stream on every platform and compiler
{
  uint64_t s;
  uint64_t next()
  {
    uint64_t z = (s += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
  }
  void fill(std::vector<uint8_t>& b, uint8_t mask = 0xFF)
  {
    for (auto& x : b)
      x = (uint8_t)(next() & mask);
  }
};

template <typename P>
static void mech(std::vector<uint8_t>& b, const RSDecoderConstParam& cp, Rng& r, uint32_t& az, uint16_t step)
{
  if (sizeof(P) != cp.MSOP_LEN)
  {
    fprintf(stderr, "struct size %zu != MSOP_LEN %u\n", sizeof(P), (unsigned)cp.MSOP_LEN);
    exit(2);
  }
  b.resize(cp.MSOP_LEN);
  r.fill(b);
  memcpy(b.data(), cp.MSOP_ID, cp.MSOP_ID_LEN);
  P* p = (P*)b.data();
  for (uint16_t i = 0; i < cp.BLOCKS_PER_PKT; i++)
  {
    memcpy(p->blocks[i].id, cp.BLOCK_ID, 2);
    p->blocks[i].azimuth = htons((uint16_t)(az % 36000));
    az += step;
  }
}

template <typename P>
static void mems(std::vector<uint8_t>& b, const RSDecoderConstParam& cp, Rng& r, uint32_t& seq, uint32_t seqs)
{
  if (sizeof(P) != cp.MSOP_LEN)
  {
    fprintf(stderr, "struct size %zu != MSOP_LEN %u\n", sizeof(P), (unsigned)cp.MSOP_LEN);
    exit(2);
  }
  b.resize(cp.MSOP_LEN);
  r.fill(b);
  memcpy(b.data(), cp.MSOP_ID, cp.MSOP_ID_LEN);
  P* p = (P*)b.data();
  p->header.pkt_seq = htons((uint16_t)(seq % seqs + 1));
  seq++;
}

static void jumbo(std::vector<uint8_t>& b, const RSDecoderConstParam& cp, Rng& r, uint32_t& seq)
{
  b.resize(cp.MSOP_LEN);
  r.fill(b);
  RSM1_Jumbo* j = (RSM1_Jumbo*)b.data();
  for (auto& p : j->pkts)
  {
    memcpy(p.header.id, cp.MSOP_ID, cp.MSOP_ID_LEN);
    p.header.pkt_seq = htons((uint16_t)(seq % 630 + 1));
    seq++;
  }
}

// One MSOP packet of the given model. Returns false for a model this tool cannot make.
static bool makeMsop(LidarType t, std::vector<uint8_t>& b, const RSDecoderConstParam& cp, Rng& r,
                     uint32_t& state)
{
  switch (t)
  {
    case LidarType::RS16: mech<RS16MsopPkt>(b, cp, r, state, 20); break;
    case LidarType::RS32: mech<RS32MsopPkt>(b, cp, r, state, 20); break;
    case LidarType::RSBP: mech<RSBPMsopPkt>(b, cp, r, state, 20); break;
    case LidarType::RSAIRY: mech<RSAIRYMsopPkt>(b, cp, r, state, 20); break;
    case LidarType::RSHELIOS: mech<RSHELIOSMsopPkt>(b, cp, r, state, 20); break;
    case LidarType::RSHELIOS_16P: mech<RSHELIOSMsopPkt>(b, cp, r, state, 20); break;
    case LidarType::RS128: mech<RS128MsopPkt>(b, cp, r, state, 20); break;
    case LidarType::RS80: mech<RS80MsopPkt>(b, cp, r, state, 20); break;
    case LidarType::RS48: mech<RSP48MsopPkt>(b, cp, r, state, 20); break;
    case LidarType::RSP128: mech<RSP128MsopPkt>(b, cp, r, state, 20); break;
    case LidarType::RSP80: mech<RSP80MsopPkt>(b, cp, r, state, 20); break;
    case LidarType::RSP48: mech<RSP48MsopPkt>(b, cp, r, state, 20); break;
    case LidarType::RSM1: mems<RSM1MsopPkt>(b, cp, r, state, 630); break;
    case LidarType::RSM2: mems<RSM2MsopPkt>(b, cp, r, state, 1260); break;
    case LidarType::RSM3: mems<RSM3MsopPkt>(b, cp, r, state, 630); break;
    case LidarType::RSE1: mems<RSEOSMsopPkt>(b, cp, r, state, 630); break;
    case LidarType::RSMX: mems<RSMXMsopPkt>(b, cp, r, state, 630); break;
    case LidarType::RSM1_JUMBO: jumbo(b, cp, r, state); break;
#if RSLIDAR_VERSION_PATCH >= 20
    case LidarType::RSFAIRY: mech<RSFAIRYMsopPkt>(b, cp, r, state, 20); break;
    case LidarType::RSEMX: mems<RSEMXMsopPkt>(b, cp, r, state, 630); break;
#endif
    default: return false;
  }
  return true;
}

static int usage()
{
  fprintf(stderr, "usage: decode_dump synth TYPE SEED MSOP_PACKETS MIN MAX | file TYPE PACKETS.bin MIN MAX [--points]\n");
  return 1;
}

static void drain()
{
  // Wait until the driver's handle thread has published everything: no new cloud for 400 ms.
  uint64_t seen = g_clouds.load();
  for (int idle = 0; idle < 4;)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    uint64_t now = g_clouds.load();
    idle = (now == seen) ? idle + 1 : 0;
    seen = now;
  }
}

int main(int argc, char** argv)
{
  if (argc < 6)
    return usage();
  std::string mode = argv[1];
  LidarType type = strToLidarType(argv[2]);

  RSDriverParam param;
  param.input_type = InputType::RAW_PACKET;
  param.lidar_type = type;
  param.decoder_param.wait_for_difop = true;
  param.decoder_param.use_lidar_clock = true;  // stamps from the packets: the same on every run
  param.decoder_param.dense_points = false;
  param.decoder_param.ts_first_point = true;
  if (mode == "synth" && argc < 7)
    return usage();
  param.decoder_param.min_distance = std::stof(mode == "synth" ? argv[5] : argv[4]);
  param.decoder_param.max_distance = std::stof(mode == "synth" ? argv[6] : argv[5]);

  printf("rs_driver %d.%d.%d %s min=%g max=%g\n", RSLIDAR_VERSION_MAJOR, RSLIDAR_VERSION_MINOR,
         RSLIDAR_VERSION_PATCH, argv[2], param.decoder_param.min_distance, param.decoder_param.max_distance);

  std::shared_ptr<Decoder<Cloud>> probe = DecoderFactory<Cloud>::createDecoder(type, param.decoder_param);
  const RSDecoderConstParam cp = probe->const_param_;

  LidarDriver<Cloud> driver;
  driver.regPointCloudCallback(getCloud, putCloud);
  driver.regExceptionCallback([](const Error&) {});
  if (!driver.init(param) || !driver.start())
  {
    fprintf(stderr, "driver init failed\n");
    return 1;
  }

  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> packets;
  if (mode == "synth")
  {
    Rng r{std::stoull(argv[3])};
    int n = std::stoi(argv[4]);
    uint32_t state = (uint32_t)(r.next() % 36000);
    std::vector<uint8_t> difop(cp.DIFOP_LEN);
    // DIFOP: two low bits per byte only, so calibration angles stay in range and the decoders accept them.
    r.fill(difop, 0x03);
    memcpy(difop.data(), cp.DIFOP_ID, cp.DIFOP_ID_LEN);
    for (int i = 0; i < n; i++)
    {
      if (i % 200 == 0)
        packets.emplace_back(1, difop);
      std::vector<uint8_t> b;
      if (!makeMsop(type, b, cp, r, state))
      {
        fprintf(stderr, "no packet generator for %s\n", argv[2]);
        return 2;
      }
      packets.emplace_back(0, b);
    }
  }
  else if (mode == "file")
  {
    g_points = (argc > 6 && std::string(argv[6]) == "--points");
    std::ifstream in(argv[3], std::ios::binary);
    if (!in)
    {
      fprintf(stderr, "cannot read %s\n", argv[3]);
      return 1;
    }
    while (true)
    {
      uint8_t kind;
      uint8_t len[4];
      if (!in.read((char*)&kind, 1) || !in.read((char*)len, 4))
        break;
      uint32_t n = len[0] | (len[1] << 8) | (len[2] << 16) | ((uint32_t)len[3] << 24);
      std::vector<uint8_t> b(n);
      if (!in.read((char*)b.data(), n))
        return 1;
      packets.emplace_back(kind, b);
    }
  }
  else
  {
    return usage();
  }

  size_t fed = 0;
  for (auto& kp : packets)
  {
    Packet pkt;
    pkt.buf_ = kp.second;
    driver.decodePacket(pkt);
    // Keep the packet queue short (rs_driver 1.5.18 drops its whole queue above 10240 packets).
    if (++fed % 100 == 0)
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if (kp.first == 1)
      std::this_thread::sleep_for(std::chrono::milliseconds(20));  // DIFOP before the next MSOP
  }
  drain();
  driver.stop();
  printf("packets %zu clouds %" PRIu64 "\n", packets.size(), g_clouds.load());
  return 0;
}
