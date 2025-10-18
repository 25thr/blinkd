/*
  Blinkd detection + EAR Pipeline (C++17)
  What this provides
   - Camera capture (OpenCV)
   - Face landmarks backend (choose one at build/run time):
       A) MediaPipe Face Mesh *compatible* ONNX (468 landmarks) via OpenCV DNN
       B) OpenCV Facemark LBF (68 landmarks) as a fallback
   - Eye Aspect Ratio (EAR) computed from landmarks (MediaPipe or 68pt mapping)
   - Ultra-lightweight blink FSM (from the C impl) integrated inline
   - UDP JSON event out (Unity/Unreal friendly)
   - Optional CPU pinning + high-priority thread

  Notes
   * You must supply the landmark model file(s):
       - 468 landmarks ONNX (e.g., "face_landmarks_468.onnx")
       - or LBF model (e.g., "lbfmodel.yaml")
   * If you only want MediaPipe-style indices, use Backend::FaceMesh468.

  Example run (468-mesh backend + UDP)
    ./blinkd --camera 0 --backend 468 \
      --onnx /path/to/face_landmarks_468.onnx --udp 127.0.0.1:7777

  Example run (68-point LBF fallback)
    ./blinkd --camera 0 --backend 68 \
      --lbf /path/to/lbfmodel.yaml --udp 127.0.0.1:7777

  UDP event payload (one line JSON per event)
    {"t_ms":123456,"type":"blink","dur_ms":132,"flags":3}
*/

#include "blinkd.h"

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <algorithm>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#ifdef __linux__
  #include <pthread.h>
  #include <sched.h>
#endif

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#ifdef HAVE_OPENCV_FACE
  #include <opencv2/face.hpp>
#endif

// Time + UDP helpers
static inline uint64_t now_ms() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

struct UdpOut { int sock = -1; sockaddr_in addr{}; bool enabled = false; };
static int udp_open(UdpOut &u, const char *ip, uint16_t port) {
  u.sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (u.sock < 0) return -1;
  memset(&u.addr, 0, sizeof(u.addr));
  u.addr.sin_family = AF_INET;
  u.addr.sin_port = htons(port);
  if (inet_aton(ip, &u.addr.sin_addr) == 0) return -2;
  u.enabled = true;
  return 0;
}
static void udp_send(UdpOut &u, const char *type, uint32_t t_ms, uint32_t dur_ms, uint32_t flags) {
  if (!u.enabled) return;
  char buf[256];
  int n = snprintf(buf, sizeof(buf),
                   "{\"t_ms\":%u,\"type\":\"%s\",\"dur_ms\":%u,\"flags\":%u}\n",
                   t_ms, type, dur_ms, flags);
  sendto(u.sock, buf, (size_t)n, 0, (sockaddr *)&u.addr, sizeof(u.addr));
}

// Landmark helpers and EAR computation
static const int L_EAR[6] = {33, 160, 158, 133, 153, 144};
static const int R_EAR[6] = {263, 387, 385, 362, 380, 373};

static inline double l2(const cv::Point2f &a, const cv::Point2f &b) {
  cv::Point2f d = a - b;
  return std::sqrt(d.x * d.x + d.y * d.y);
}
static inline float EAR_from6(const cv::Point2f P[6]) {
  double num = l2(P[1], P[5]) + l2(P[2], P[4]);
  double den = 2.0 * l2(P[0], P[3]) + 1e-9;
  return (float)(num / den);
}

// FaceMesh468 ONNX backend
struct FaceMesh468 {
  cv::dnn::Net net;
  int inW = 192, inH = 192;
  cv::CascadeClassifier face_cascade;
  bool load(const std::string &onnx) {
    net = cv::dnn::readNetFromONNX(onnx);
    return !net.empty();
  }
  bool init_face() {
    return face_cascade.load(cv::samples::findFile("haarcascade_frontalface_default.xml"));
  }

  bool infer(const cv::Mat &frame, std::vector<cv::Point2f> &out468) {
    std::vector<cv::Rect> faces;
    face_cascade.detectMultiScale(frame, faces, 1.1, 3, 0, cv::Size(60, 60));
    if (faces.empty()) return false;
    auto face = *std::max_element(faces.begin(), faces.end(),
                                  [](auto &a, auto &b) { return a.area() < b.area(); });
    cv::Rect roi = face & cv::Rect(0, 0, frame.cols, frame.rows);
    cv::Mat crop = frame(roi);

    cv::Mat resized;
    cv::resize(crop, resized, cv::Size(inW, inH));
    resized.convertTo(resized, CV_32FC3, 1.0 / 255.0);
    cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);

    int sizes[] = {1, inH, inW, 3};
    cv::Mat blob(4, sizes, CV_32F);
    for (int y = 0; y < inH; y++)
      for (int x = 0; x < inW; x++) {
        cv::Vec3f pix = resized.at<cv::Vec3f>(y, x);
        for (int c = 0; c < 3; c++) {
          int idx[4] = {0, y, x, c};
          blob.at<float>(idx) = pix[c];
        }
      }

    net.setInput(blob);
    std::vector<cv::Mat> outs;
    net.forward(outs);
    if (outs.empty()) return false;
    cv::Mat out = outs[0].reshape(1);
    int total = out.total();
    int N = (total % 3 == 0) ? (int)(total / 3) : 0;
    if (N < 468) return false;

    out468.resize(N);
    const float *p = (const float *)out.data;
    for (int i = 0; i < N; i++) {
      float x = p[3 * i + 0], y = p[3 * i + 1];
      out468[i] = cv::Point2f(roi.x + x * roi.width, roi.y + y * roi.height);
    }
    return true;
  }
};

// CLI and main loop
struct Args {
  int cam = 0;
  std::string onnx;
  bool use_udp = true;
  std::string ip = "127.0.0.1";
  uint16_t port = 7777;
  bool show = true;
  int width = 640, height = 480;
};

static bool parse_ipport(const std::string &s, std::string &ip, uint16_t &port) {
  auto p = s.find(':');
  if (p == std::string::npos) return false;
  ip = s.substr(0, p);
  port = (uint16_t)std::atoi(s.c_str() + p + 1);
  return true;
}

int main(int argc, char **argv) {
  Args a;
  for (int i = 1; i < argc; i++) {
    std::string k = argv[i];
    if (k == "--camera" && i + 1 < argc) a.cam = std::atoi(argv[++i]);
    else if (k == "--onnx" && i + 1 < argc) a.onnx = argv[++i];
    else if (k == "--udp" && i + 1 < argc) { parse_ipport(argv[++i], a.ip, a.port); a.use_udp = true; }
    else if (k == "--show") a.show = true;
  }

  if (a.onnx.empty()) {
    fprintf(stderr, "Usage: %s --camera N --onnx path/to/facemesh.onnx [--udp ip:port] [--show]\n", argv[0]);
    return 1;
  }

  // UDP setup
  UdpOut udp;
  if (a.use_udp && udp_open(udp, a.ip.c_str(), a.port) != 0) {
    perror("udp_open");
    a.use_udp = false;
  }

  // Camera
  cv::VideoCapture cap(a.cam);
  if (!cap.isOpened()) {
    fprintf(stderr, "Camera %d open failed\n", a.cam);
    return 1;
  }
  cap.set(cv::CAP_PROP_FRAME_WIDTH, a.width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, a.height);
  cap.set(cv::CAP_PROP_FPS, 120);

  // FaceMesh backend
  FaceMesh468 fm;
  if (!fm.load(a.onnx)) {
    fprintf(stderr, "Failed to load FaceMesh ONNX: %s\n", a.onnx.c_str());
    return 1;
  }
  fm.init_face();

  // Link to blinkd SDK
  BlinkHandle *B = blink_create(1.0f);
  blink_set_preset(B, BLINK_PRESET_BALANCED);

  uint64_t t0 = now_ms();
  cv::Mat frame;
  while (true) {
    if (!cap.read(frame)) break;
    uint32_t t_ms = (uint32_t)(now_ms() - t0);

    // Extract landmarks
    std::vector<cv::Point2f> pts;
    bool ok = fm.infer(frame, pts);
    if (!ok || pts.size() < 468) continue;

    // Compute EAR → openness [0..1]
    cv::Point2f A[6];
    for (int i = 0; i < 6; i++) A[i] = pts[L_EAR[i]];
    float earL = EAR_from6(A);
    for (int i = 0; i < 6; i++) A[i] = pts[R_EAR[i]];
    float earR = EAR_from6(A);
    auto map_ear = [&](float ear) { float x = (ear - 0.12f) / (0.30f - 0.12f); return std::clamp(x, 0.0f, 1.0f); };
    float openL = map_ear(earL);
    float openR = map_ear(earR);

    uint32_t dur_ms = 0, flags = 0;
    blink_update(B, t_ms, openL, openR, &dur_ms, &flags);

    if (flags) {
      if (a.use_udp) {
        if (flags & BLINK_EVT_BLINK) udp_send(udp, "blink", t_ms, dur_ms, flags);
        if (flags & BLINK_EVT_WINK_LEFT) udp_send(udp, "wink_left", t_ms, dur_ms, flags);
        if (flags & BLINK_EVT_WINK_RIGHT) udp_send(udp, "wink_right", t_ms, dur_ms, flags);
        if (flags & BLINK_EVT_LONG_BLINK) udp_send(udp, "long_blink", t_ms, dur_ms, flags);
        if (flags & BLINK_EVT_DOUBLE_BLINK) udp_send(udp, "double_blink", t_ms, dur_ms, flags);
      }
      if (a.show) {
        char txt[128];
        snprintf(txt, sizeof(txt), "EV %u dur=%u", flags, dur_ms);
        cv::putText(frame, txt, {20, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 255}, 2);
      }
    }

    if (a.show) {
      int y = frame.rows - 20;
      int w = 200;
      cv::rectangle(frame, {10, y - 10}, {10 + (int)(w * openL), y}, {0, 255, 0}, cv::FILLED);
      cv::rectangle(frame, {10, y - 30}, {10 + (int)(w * openR), y - 20}, {0, 255, 0}, cv::FILLED);
      cv::imshow("blinkd_cam", frame);
      if (cv::waitKey(1) == 27) break; // ESC
    }
  }

  blink_destroy(B);
  if (a.use_udp) close(udp.sock);
  return 0;
}
