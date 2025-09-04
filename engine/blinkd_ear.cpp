/*
  Blink Detection + EAR Pipeline (single-binary C++17)
  What this provides
   - Camera capture (OpenCV)
   - Face landmarks backend (choose one at build/run time):
       A) MediaPipe Face Mesh *compatible* ONNX (468 landmarks) via OpenCV DNN
       B) OpenCV Facemark LBF (68 landmarks) as a fallback
   - Eye Aspect Ratio (EAR) computed from landmarks (MediaPipe or 68pt mapping)
   - Ultra-lightweight blink FSM (from previous C impl) integrated inline
   - UDP JSON event out (Unity/Unreal friendly)
   - Optional CPU pinning + high-priority thread

  Notes
   * You must supply the landmark model file(s):
       - 468 landmarks ONNX (e.g., "face_landmarks_468.onnx")
       - or LBF model (e.g., "lbfmodel.yaml")
   * This file focuses on a clean, minimal, *fast* single executable.
   * If you only want MediaPipe-style indices, use Backend::FaceMesh468.

  Build (Linux/macOS)
    g++ -O3 -march=native -DNDEBUG -std=c++17 -o blinkd_ear blinkd_ear.cpp \
        `pkg-config --cflags --libs opencv4` -lpthread

  Example run (468-mesh backend + UDP)
    ./blinkd_ear --camera 0 --backend 468 \
      --onnx /path/to/face_landmarks_468.onnx --udp 127.0.0.1:7777

  Example run (68-point LBF fallback)
    ./blinkd_ear --camera 0 --backend 68 \
      --lbf /path/to/lbfmodel.yaml --udp 127.0.0.1:7777

  UDP event payload (one line JSON per event)
    {"t_ms":123456,"type":"blink","dur_ms":132,"flags":3}
*/

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

// ========================= time helpers =========================
static inline uint64_t now_ms(){
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

// ========================= UDP =========================
struct UdpOut{ int sock=-1; sockaddr_in addr{}; bool enabled=false; };
static int udp_open(UdpOut& u, const char* ip, uint16_t port){
  u.sock = ::socket(AF_INET, SOCK_DGRAM, 0); if(u.sock<0) return -1;
  memset(&u.addr,0,sizeof(u.addr)); u.addr.sin_family = AF_INET; u.addr.sin_port = htons(port);
  if(inet_aton(ip, &u.addr.sin_addr)==0) return -2; u.enabled=true; return 0;
}
static void udp_send(UdpOut& u, const char* type, uint32_t t_ms, uint32_t dur_ms, uint32_t flags){
  if(!u.enabled) return; char buf[256];
  int n = snprintf(buf,sizeof(buf),
  "{\"t_ms\":%u,\"type\":\"%s\",\"dur_ms\":%u,\"flags\":%u}\n",
  t_ms,type,dur_ms,flags);
  sendto(u.sock, buf, (size_t)n, 0, (sockaddr*)&u.addr, sizeof(u.addr));
}

// ========================= Blink FSM =========================

enum EyeState {EYE_OPEN=0,EYE_CLOSING,EYE_CLOSED,EYE_REFRACT};

enum {
  EVT_NONE        = 0,
  EVT_BLINK       = 1<<0,
  EVT_LONG_BLINK  = 1<<1,
  EVT_DOUBLE_BLINK= 1<<2,
  EVT_WINK_LEFT   = 1<<3,
  EVT_WINK_RIGHT  = 1<<4
};

struct EyeDet{
  float ema_alpha=0.01f, noise_alpha=0.02f; // adaptive stats
  float close_k=2.5f, open_k=1.5f;          // thresholds in sigmas
  uint32_t min_ms=40, long_ms=400, max_ms=800, dbl_gap_ms=300, refr_ms=60;
  // adaptive
  float baseline=1.0f, dev=0.03f, last=1.0f; 
  // state
  EyeState st=EYE_OPEN; uint32_t enter_ms=0; uint32_t last_end_ms=0; bool last_was_blink=false;
};

static inline float clamp01(float x){return std::max(0.f,std::min(1.f,x));}
static inline float ema(float prev,float x,float a){return prev + a*(x-prev);} 

static inline void update_stats(EyeDet& d, float open, bool open_like){
  if(open_like) d.baseline = ema(d.baseline, open, d.ema_alpha);
  float absdev = std::fabs(open - d.baseline);
  d.dev = ema(d.dev, absdev, d.noise_alpha); if(d.dev<1e-3f) d.dev=1e-3f;
}

static uint32_t eyedet_update(EyeDet& d, uint32_t t_ms, float open, uint32_t* blink_ms){
  uint32_t ev=EVT_NONE; if(blink_ms) *blink_ms=0; open=clamp01(open); d.last=open;
  float sigma=d.dev; float close_thr=d.baseline - d.close_k*sigma; float open_thr=d.baseline - d.open_k*sigma;
  bool open_like = open>=open_thr; update_stats(d, open, open_like);

  switch(d.st){
    case EYE_OPEN:
      if(open < close_thr){ d.st=EYE_CLOSING; d.enter_ms=t_ms; }
      break;
    case EYE_CLOSING:
      if(open < close_thr){ d.st=EYE_CLOSED; d.enter_ms=t_ms; }
      else if(open >= open_thr){ d.st=EYE_OPEN; }
      break;
    case EYE_CLOSED:{
      uint32_t dur = t_ms - d.enter_ms;
      if(open >= open_thr){
        if(dur>=d.min_ms && dur<=d.max_ms){ if(blink_ms) *blink_ms=dur; ev|=EVT_BLINK; if(dur>=d.long_ms) ev|=EVT_LONG_BLINK; if(d.last_was_blink && (t_ms-d.last_end_ms)<=d.dbl_gap_ms) ev|=EVT_DOUBLE_BLINK; d.last_was_blink=true; d.last_end_ms=t_ms; }
        else d.last_was_blink=false;
        d.st=EYE_REFRACT; d.enter_ms=t_ms;
      }
    } break;
    case EYE_REFRACT:
      if((t_ms - d.enter_ms) >= d.refr_ms) d.st=EYE_OPEN; break;
  }
  return ev;
}

struct BlinkDet2 { EyeDet L,R; uint32_t wink_min_ms=60; };
static uint32_t blinkdet2_update(BlinkDet2& b, uint32_t t_ms, float openL, float openR, uint32_t* out_ms){
  uint32_t msL=0, msR=0; uint32_t evL=eyedet_update(b.L,t_ms,openL,&msL); uint32_t evR=eyedet_update(b.R,t_ms,openR,&msR); uint32_t ev=evL|evR; if(out_ms) *out_ms=(msL>msR?msL:msR);
  if((evL&EVT_BLINK) && !(evR&EVT_BLINK) && msL>=b.wink_min_ms && b.R.last > b.R.baseline-1.0f*b.R.dev) ev|=EVT_WINK_LEFT;
  if((evR&EVT_BLINK) && !(evL&EVT_BLINK) && msR>=b.wink_min_ms && b.L.last > b.L.baseline-1.0f*b.L.dev) ev|=EVT_WINK_RIGHT;
  return ev;
}

// ========================= Landmark backends =========================

enum class Backend{ FaceMesh468, Facemark68 };

// --- MediaPipe FaceMesh-compatible indices for EAR (left/right) ---
// Left eye: p1=33, p4=133, p2=160, p6=144, p3=158, p5=153
// Right eye: p1=263, p4=362, p2=387, p6=373, p3=385, p5=380
static const int L_EAR[6] = {33,160,158,133,153,144};
static const int R_EAR[6] = {263,387,385,362,380,373};

static inline double l2(const cv::Point2f& a, const cv::Point2f& b){ cv::Point2f d=a-b; return std::sqrt(d.x*d.x + d.y*d.y); }
static inline float EAR_from6(const cv::Point2f P[6]){
  double num = l2(P[1],P[5]) + l2(P[2],P[4]);
  double den = 2.0 * l2(P[0],P[3]) + 1e-9; return (float)(num/den);
}

// ========================= FaceMesh468 via ONNX (OpenCV DNN) =========================
struct FaceMesh468{
  cv::dnn::Net net; int inW=192, inH=192; float conf_thr=0.5f; // adjust per model
  bool load(const std::string& onnx){ net = cv::dnn::readNetFromONNX(onnx); return !net.empty(); }
  // Simple single-face pipeline: detect face via cv::Cascade as fallback; crop-center
  // For production, use a stronger face detector (YuNet, SCRFD, etc.).
  cv::CascadeClassifier face_cascade;
  bool init_face(){ return face_cascade.load(cv::samples::findFile("haarcascade_frontalface_default.xml")); }

  bool infer(const cv::Mat& frame, std::vector<cv::Point2f>& out468){
    // detect face ROI (very basic); choose biggest
    std::vector<cv::Rect> faces; face_cascade.detectMultiScale(frame, faces, 1.1, 3, 0, cv::Size(60,60));
    if(faces.empty()) return false; auto face = *std::max_element(faces.begin(), faces.end(),[](auto&a,auto&b){return a.area()<b.area();});
    cv::Rect roi = face & cv::Rect(0,0,frame.cols,frame.rows);

    cv::Mat crop = frame(roi);
    // Prepare the input in NHWC format manually:
    cv::Mat resized;
    cv::resize(crop, resized, cv::Size(inW, inH)); // resize to model input size

    // Convert to float and normalize [0,1]
    resized.convertTo(resized, CV_32FC3, 1.0/255.0);

    // OpenCV loads BGR by default, if your model expects RGB, convert:
    cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);

    // Now create a blob in NHWC order:
    int sizes[] = {1, inH, inW, 3}; // NHWC shape
    cv::Mat blob(4, sizes, CV_32F);

    // Copy data from resized into blob, ensuring NHWC layout:
    for (int y = 0; y < inH; y++) {
        for (int x = 0; x < inW; x++) {
            cv::Vec3f pix = resized.at<cv::Vec3f>(y, x);
            for (int c = 0; c < 3; c++) {
                int idx[4] = {0, y, x, c};
                blob.at<float>(idx) = pix[c];
            }
        }
    }
    net.setInput(blob);
    std::vector<cv::Mat> outs; net.forward(outs);
    if(outs.empty()) return false;
    // Assume output shape [1, N*3] or [1,N,3] with N=468 (normalize [0..1])
    cv::Mat out = outs[0].reshape(1);
    int total = out.total();
    int N = (total%3==0)? (int)(total/3) : 0; if(N<468) return false;
    out468.resize(N);
    const float* p = (const float*)out.data;
    for(int i=0;i<N;i++){
      float x = p[3*i+0]; float y = p[3*i+1]; // assume normalized in [0,1]
      // map back to frame coords (within ROI)
      out468[i] = cv::Point2f(roi.x + x*roi.width, roi.y + y*roi.height);
    }
    return true;
  }
};

// ========================= Facemark68 (OpenCV) =========================
#ifdef HAVE_OPENCV_FACE
struct Facemark68{
  cv::Ptr<cv::face::Facemark> fm;
  cv::CascadeClassifier face_cascade;
  bool load(const std::string& lbf){ fm = cv::face::FacemarkLBF::create(); ((cv::face::FacemarkLBF*)fm.get())->loadModel(lbf); return true; }
  bool init_face(){ return face_cascade.load(cv::samples::findFile("haarcascade_frontalface_default.xml")); }
  bool infer(const cv::Mat& frame, std::vector<cv::Point2f>& out68){
    std::vector<cv::Rect> faces; face_cascade.detectMultiScale(frame, faces, 1.1, 3, 0, cv::Size(60,60)); if(faces.empty()) return false;
    std::vector<std::vector<cv::Point2f>> landmarks; bool ok=fm->fit(frame, faces, landmarks); if(!ok || landmarks.empty()) return false; out68 = landmarks[0]; return true;
  }
};
#endif

// Map 68-pt indices to EAR (same as dlib’s convention)
// Left eye: [36,37,38,39,40,41]  Right eye: [42,43,44,45,46,47]
static inline float EAR_from68(const std::vector<cv::Point2f>& L, bool left){
  int base = left?36:42; cv::Point2f P[6] = { L[base+0], L[base+1], L[base+2], L[base+3], L[base+4], L[base+5] };
  return EAR_from6(P);
}

// ========================= CPU pinning (Linux only) =========================
static void try_pin_to_core(int core){
#ifdef __linux__
  cpu_set_t set; CPU_ZERO(&set); if(core>=0){ CPU_SET(core,&set); pthread_setaffinity_np(pthread_self(), sizeof(set), &set); }
#endif
}

// ========================= CLI =========================
struct Args{
  int cam=0; Backend backend=Backend::FaceMesh468; std::string onnx; std::string lbf;
  bool use_udp=false; std::string ip="127.0.0.1"; uint16_t port=7777; int pin_core=-1; bool show=false; int width=640, height=480;
};

static void usage(const char* exe){
  fprintf(stderr,
  "Usage: %s --camera N [--backend 468|68] [--onnx path] [--lbf path]\n"
  "              [--udp ip:port] [--pin core] [--show] [--size WxH]\n",
  exe);
}

static bool parse_ipport(const std::string& s, std::string& ip, uint16_t& port){ auto p=s.find(':'); if(p==std::string::npos) return false; ip=s.substr(0,p); port=(uint16_t)std::atoi(s.c_str()+p+1); return true; }
static bool parse_size(const std::string& s, int& w,int& h){ auto p=s.find('x'); if(p==std::string::npos) return false; w=std::atoi(s.substr(0,p).c_str()); h=std::atoi(s.substr(p+1).c_str()); return true; }

int main(int argc, char** argv){
  Args a; for(int i=1;i<argc;i++){
    std::string k=argv[i];
    if(k=="--camera" && i+1<argc) a.cam=std::atoi(argv[++i]);
    else if(k=="--backend" && i+1<argc){ std::string b=argv[++i]; a.backend = (b=="68"? Backend::Facemark68: Backend::FaceMesh468); }
    else if(k=="--onnx" && i+1<argc) a.onnx=argv[++i];
    else if(k=="--lbf" && i+1<argc) a.lbf=argv[++i];
    else if(k=="--udp" && i+1<argc){ parse_ipport(argv[++i], a.ip, a.port); a.use_udp=true; }
    else if(k=="--pin" && i+1<argc) a.pin_core=std::atoi(argv[++i]);
    else if(k=="--show") a.show=true;
    else if(k=="--size" && i+1<argc){ parse_size(argv[++i], a.width,a.height); }
    else { usage(argv[0]); }
  }

  if(a.pin_core>=0) try_pin_to_core(a.pin_core);

  UdpOut udp; if(a.use_udp) { if(udp_open(udp, a.ip.c_str(), a.port)!=0){ perror("udp_open"); a.use_udp=false; } }

  cv::VideoCapture cap(a.cam); if(!cap.isOpened()){ fprintf(stderr,"Camera %d open failed\n", a.cam); return 1; }
  cap.set(cv::CAP_PROP_FRAME_WIDTH, a.width); cap.set(cv::CAP_PROP_FRAME_HEIGHT, a.height); cap.set(cv::CAP_PROP_FPS, 120);

  BlinkDet2 B; // defaults are fine; baseline ~1.0 (open)
  B.L.baseline = B.R.baseline = 1.0f; B.L.dev=B.R.dev=0.03f;

  // Backends
  FaceMesh468 fm468; bool have468=false; bool have68=false;
  if(a.backend==Backend::FaceMesh468){
    if(a.onnx.empty()){ fprintf(stderr,"--onnx is required for backend 468\n"); return 1; }
    if(!fm468.load(a.onnx)){ fprintf(stderr, "Failed to load ONNX: %s\n", a.onnx.c_str()); return 1; }
    fm468.init_face(); have468=true;
  }
#ifdef HAVE_OPENCV_FACE
  Facemark68 fm68;
if (a.backend == Backend::Facemark68) {
  if (a.lbf.empty()) {
    fprintf(stderr, "--lbf is required for backend 68\n");
    return 1;
  }
  if (!fm68.load(a.lbf)) {
    fprintf(stderr, "Failed to load LBF model\n");
    return 1;
  }
  fm68.init_face();
  have68 = true;
}

#else
  if(a.backend==Backend::Facemark68) {
    fprintf(stderr, "OpenCV FACE module not available; rebuild OpenCV with contrib\n");
    return 1;
  }
#endif

  uint64_t t0=now_ms();
  cv::Mat frame; std::vector<cv::Point2f> Lmk;
  while(true){
    if(!cap.read(frame)) break; if(frame.empty()) continue; uint32_t t_ms = (uint32_t)(now_ms()-t0);
    cv::Mat gray; cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    float openL=1.0f, openR=1.0f; bool ok=false;

    if(have468){
      std::vector<cv::Point2f> pts; ok = fm468.infer(gray, pts);
      if(ok && (int)pts.size()>=468){
        cv::Point2f A[6]; for(int i=0;i<6;i++) A[i]=pts[L_EAR[i]]; float earL=EAR_from6(A);
        for(int i=0;i<6;i++) A[i]=pts[R_EAR[i]]; float earR=EAR_from6(A);
        // Map EAR to openness ~[0..1] with adaptive scaling
        // Typical EAR open ~0.25-0.35, closed ~0.12-0.18 (rough). Normalize via running baselines inside EyeDet.
        // Here we provide a simple linear mapping with clamp.
        auto map_ear=[&](float ear){ float x=(ear-0.12f)/(0.30f-0.12f); return std::max(0.f,std::min(1.f,x)); };
        openL = map_ear(earL); openR = map_ear(earR);
        if(a.show){ for(int i=0;i<6;i++) cv::circle(frame, pts[L_EAR[i]], 1, {0,255,0}, -1); for(int i=0;i<6;i++) cv::circle(frame, pts[R_EAR[i]], 1, {0,255,0}, -1); }
      }
    }
#ifdef HAVE_OPENCV_FACE
    else if(have68){
      std::vector<cv::Point2f> pts; ok = fm68.infer(gray, pts);
      if(ok && (int)pts.size()>=48){
        float earL = EAR_from68(pts,true); float earR=EAR_from68(pts,false);
        auto map_ear=[&](float ear){ float x=(ear-0.12f)/(0.30f-0.12f); return std::max(0.f,std::min(1.f,x)); };
        openL = map_ear(earL); openR = map_ear(earR);
        if(a.show){ for(int i=36;i<=41;i++) cv::circle(frame, pts[i], 1, {255,0,0}, -1); for(int i=42;i<=47;i++) cv::circle(frame, pts[i], 1, {255,0,0}, -1); }
      }
    }
#endif

    uint32_t dur_ms=0; uint32_t ev = blinkdet2_update(B, t_ms, openL, openR, &dur_ms);
    if(ev){
      if(a.use_udp){ if(ev & EVT_BLINK) udp_send(udp, "blink", t_ms, dur_ms, ev); if(ev & EVT_WINK_LEFT) udp_send(udp, "wink_left", t_ms, dur_ms, ev); if(ev & EVT_WINK_RIGHT) udp_send(udp, "wink_right", t_ms, dur_ms, ev); if(ev & EVT_LONG_BLINK) udp_send(udp, "long_blink", t_ms, dur_ms, ev); if(ev & EVT_DOUBLE_BLINK) udp_send(udp, "double_blink", t_ms, dur_ms, ev); }
      if(a.show){ char txt[128]; snprintf(txt,sizeof(txt),"BLINK dur=%u flags=%u",dur_ms,ev); cv::putText(frame, txt, {20,30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0,255,255}, 2); }
    }

    if(a.show){
      // visualize openness
      int y=frame.rows-20; int w=200; cv::rectangle(frame, {10,y-10}, {10+(int)(w*openL), y}, {0,255,0}, cv::FILLED);
      cv::rectangle(frame, {10,y-30}, {10+(int)(w*openR), y-20}, {0,255,0}, cv::FILLED);
      cv::imshow("blink_ear", frame); if(cv::waitKey(1)==27) break; // ESC
    }
  }
  return 0;
}
