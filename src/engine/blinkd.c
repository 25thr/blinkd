/*
  Blinkd: A low-level, hardware-agnostic, eye-blink input layer, like a "blink driver" for real-time interactions.
  - Ultra-lightweight, dependency-free core detector (O(1) per sample)
  - Works with 1 or 2 eye channels (openness in [0..1])
  - Adaptive baseline (thresholds using EMA) + robust noise deviation estimator
  - Finite State Machine (FSM) with hysteresis + refractory to avoid chatter
  - Calibration and sensitivity presets
  - Emits events: BLINK, LONG_BLINK, DOUBLE_BLINK, WINK_LEFT, WINK_RIGHT
  - IPC: UDP JSON broadcast (for Unity, Unreal, Godot)
  - Optional POSIX shared-memory ring (for IPC with camera producers)
  - C99 API, ABI-stable

  Integrate with a camera pipeline by mapping any eye openness estimator to [0..1]
  (e.g., Eye Aspect Ratio, iris distance, or segmentation percent-open).

  Author: 03C0
*/

#define _GNU_SOURCE
#include "blinkd.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <errno.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Platform compatibility layer
#ifdef _WIN32
  // Prevent <windows.h> from including the old <winsock.h>
  #ifndef WIN32_LEAN_AND_MEAN
  #define WIN32_LEAN_AND_MEAN
  #endif
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #include <windows.h>
  #pragma comment(lib, "ws2_32.lib")

  #define close closesocket
  #define usleep(x) Sleep((x)/1000)

  // clock_gettime substitute for Windows
  static inline int clock_gettime_monotonic(struct timespec* ts) {
      static LARGE_INTEGER freq, start;
      static int initialized = 0;
      LARGE_INTEGER counter;
      if (!initialized) {
          QueryPerformanceFrequency(&freq);
          QueryPerformanceCounter(&start);
          initialized = 1;
      }
      QueryPerformanceCounter(&counter);
      double elapsed = (double)(counter.QuadPart - start.QuadPart) / (double)freq.QuadPart;
      ts->tv_sec  = (time_t)elapsed;
      ts->tv_nsec = (long)((elapsed - ts->tv_sec) * 1e9);
      return 0;
  }
  #define clock_gettime(id, ts) clock_gettime_monotonic(ts)

#else
  // POSIX/Linux/macOS
  #include <unistd.h>
  #include <sys/types.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
#endif

// Internal state machine
typedef enum { EYE_OPEN=0, EYE_CLOSING, EYE_CLOSED, EYE_REFRACT } eye_state_t;

typedef struct {
  float ema_alpha, noise_alpha;
  float close_k, open_k;
  uint32_t min_ms, long_ms, max_ms, dbl_gap_ms, refr_ms;
  float baseline, dev, last;
  eye_state_t st;
  uint32_t enter_ms, last_end_ms;
  uint8_t last_was_blink;
} EyeDet;

typedef struct BlinkdHandle {
  EyeDet L, R;
  uint32_t wink_min_ms;
} BlinkdHandle;

// Utility helpers
static inline uint64_t now_monotonic_ms(void){
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec*1000ull + (uint64_t)ts.tv_nsec/1000000ull;
}
static inline float clamp01(float x){return x<0?0:(x>1?1:x);}
static inline float ema(float prev,float x,float a){return prev + a*(x-prev);}

static void eyedet_init(EyeDet* d,float init_open){
  memset(d,0,sizeof(*d));
  d->ema_alpha=0.01f; d->noise_alpha=0.02f;
  d->close_k=2.5f; d->open_k=1.5f;
  d->min_ms=40; d->long_ms=400; d->max_ms=800;
  d->dbl_gap_ms=300; d->refr_ms=60;
  d->baseline=init_open; d->dev=0.03f; d->last=init_open;
  d->st=EYE_OPEN; d->enter_ms = 0; // accept relative timestamp streams (t_ms starting at 0)
}

static inline void update_stats(EyeDet* d,float open,int open_like){
  if(open_like) d->baseline=ema(d->baseline,open,d->ema_alpha);
  float absdev=fabsf(open-d->baseline);
  d->dev=ema(d->dev,absdev,d->noise_alpha);
  if(d->dev<1e-3f) d->dev=1e-3f;
}

static uint32_t blinkd_detector_update(EyeDet* d,uint32_t t_ms,float open,uint32_t* blink_ms){
  uint32_t ev=0; if(blink_ms)*blink_ms=0; open=clamp01(open);
  if(t_ms < d->enter_ms) return 0; // ignore out-of-order
  d->last=open;
  float sigma=d->dev;
  float close_thr=d->baseline - d->close_k*sigma;
  float open_thr =d->baseline - d->open_k*sigma;
  int open_like=open>=open_thr;
  update_stats(d,open,open_like);

  switch(d->st){
    case EYE_OPEN:
      if(open<close_thr){ d->st=EYE_CLOSING; d->enter_ms=t_ms; }
      break;
    case EYE_CLOSING:
      if(open<close_thr){ d->st=EYE_CLOSED; d->enter_ms=t_ms; }
      else if(open>=open_thr){ d->st=EYE_OPEN; }
      break;
    case EYE_CLOSED:{
      uint32_t dur=t_ms - d->enter_ms;
      if(open>=open_thr){
        if(dur>=d->min_ms && dur<=d->max_ms){
          if(blink_ms)*blink_ms=dur;
          ev|=BLINKD_EVT_BLINK;
          if(dur>=d->long_ms) ev|=BLINKD_EVT_LONG_BLINK;
          if(d->last_was_blink && (t_ms - d->last_end_ms)<=d->dbl_gap_ms)
            ev|=BLINKD_EVT_DOUBLE_BLINK;
          d->last_was_blink=1; d->last_end_ms=t_ms;
        } else d->last_was_blink=0;
        d->st=EYE_REFRACT; d->enter_ms=t_ms;
      }
    } break;
    case EYE_REFRACT:
      if((t_ms - d->enter_ms)>=d->refr_ms) d->st=EYE_OPEN;
      break;
  }
  return ev;
}

// Public API implementation
BlinkdHandle* blinkd_create(float init_open){
  BlinkdHandle* h=(BlinkdHandle*)calloc(1,sizeof(BlinkdHandle));
  if(!h) return NULL;
  eyedet_init(&h->L,init_open);
  eyedet_init(&h->R,init_open);
  h->wink_min_ms=60;
  return h;
}

void blinkd_destroy(BlinkdHandle* h){ if(h) free(h); }

void blinkd_set_ema_alpha(BlinkdHandle* h,float a){ h->L.ema_alpha=h->R.ema_alpha=a; }
void blinkd_set_noise_alpha(BlinkdHandle* h,float a){ h->L.noise_alpha=h->R.noise_alpha=a; }
void blinkd_set_thresholds(BlinkdHandle* h,float close_k,float open_k){
  h->L.close_k=h->R.close_k=close_k; h->L.open_k=h->R.open_k=open_k;
}
void blinkd_set_timing(BlinkdHandle* h,uint32_t min_blink_ms,uint32_t long_blink_ms,
                      uint32_t max_blink_ms,uint32_t double_gap_ms,uint32_t refractory_ms){
  h->L.min_ms=h->R.min_ms=min_blink_ms;
  h->L.long_ms=h->R.long_ms=long_blink_ms;
  h->L.max_ms=h->R.max_ms=max_blink_ms;
  h->L.dbl_gap_ms=h->R.dbl_gap_ms=double_gap_ms;
  h->L.refr_ms=h->R.refr_ms=refractory_ms;
}
void blinkd_set_wink_min(BlinkdHandle* h,uint32_t wink_min_ms){ h->wink_min_ms=wink_min_ms; }

void blinkd_set_preset(BlinkdHandle* h, BlinkdPreset p){
  switch(p){
    case BLINKD_PRESET_LOW:   blinkd_set_thresholds(h,3.0f,2.0f); blinkd_set_timing(h,60,500,1000,300,80); break;
    case BLINKD_PRESET_HIGH:  blinkd_set_thresholds(h,2.0f,1.2f); blinkd_set_timing(h,30,300,700,250,50); break;
    default: /* balanced */  blinkd_set_thresholds(h,2.5f,1.5f); blinkd_set_timing(h,40,400,800,300,60); break;
  }
}

int blinkd_update(BlinkdHandle* h,uint32_t t_ms,float openL,float openR,
                 uint32_t* out_dur_ms,uint32_t* out_flags){
  uint32_t msL=0,msR=0;
  uint32_t evL=blinkd_detector_update(&h->L,t_ms,openL,&msL);
  uint32_t evR=blinkd_detector_update(&h->R,t_ms,openR,&msR);
  uint32_t ev=evL|evR;
  if(out_dur_ms) *out_dur_ms=(msL>msR?msL:msR);
  // Wink detection
  if((evL&BLINKD_EVT_BLINK) && !(evR&BLINKD_EVT_BLINK) &&
     msL>=h->wink_min_ms && h->R.last > h->R.baseline - 1.0f*h->R.dev)
    ev|=BLINKD_EVT_WINK_LEFT;
  if((evR&BLINKD_EVT_BLINK) && !(evL&BLINKD_EVT_BLINK) &&
     msR>=h->wink_min_ms && h->L.last > h->L.baseline - 1.0f*h->L.dev)
    ev|=BLINKD_EVT_WINK_RIGHT;
  if(out_flags) *out_flags=ev;
  return (int)ev;
}

int blinkd_update_single(BlinkdHandle* h,uint32_t t_ms,float open,
                        uint32_t* out_dur_ms,uint32_t* out_flags){
  uint32_t ev=blinkd_detector_update(&h->L,t_ms,open,out_dur_ms);
  if(out_flags) *out_flags=ev;
  return (int)ev;
}

void blinkd_calibration_reset(BlinkdHandle* h,float baseline,float dev){
  h->L.baseline=h->R.baseline=baseline;
  h->L.dev=h->R.dev=dev;
}

void blinkd_calibration_update(BlinkdHandle* h,float open){
  // simple calibration using open-eye samples
  h->L.baseline=ema(h->L.baseline,open,h->L.ema_alpha);
  h->R.baseline=ema(h->R.baseline,open,h->R.ema_alpha);
}

// UDP IPC (cross-platform)
int blinkd_udp_open(const char* ip, uint16_t port) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return -1;

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

#ifdef _WIN32
    // Use InetPtonA on Windows
    if (InetPtonA(AF_INET, ip, &addr.sin_addr) != 1) {
        closesocket(sock);
        return -2;
    }
#else
    // POSIX
    if (inet_aton(ip, &addr.sin_addr) == 0) {
        close(sock);
        return -2;
    }
#endif

    if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
#ifdef _WIN32
        closesocket(sock);
#else
        close(sock);
#endif
        return -3;
    }

    return sock;
}

void blinkd_udp_send(int sock, const char* type, uint32_t t_ms, uint32_t dur_ms, uint32_t flags) {
    if (sock < 0) return;

    char buf[256];
    int n = snprintf(buf, sizeof(buf),
        "{\"t_ms\":%u,\"type\":\"%s\",\"dur_ms\":%u,\"flags\":%u}\n",
        t_ms, type, dur_ms, flags);

    if (n > 0) send(sock, buf, (size_t)n, 0);
}

void blinkd_udp_close(int sock) {
#ifdef _WIN32
    if (sock >= 0) closesocket(sock);
#else
    if (sock >= 0) close(sock);
#endif
}

#ifdef BLINKD_DEBUG
void blinkd_debug_dump(const BlinkdHandle* h) {
    if (!h) return;
    const float baseL = h->L.baseline, devL = h->L.dev;
    const float baseR = h->R.baseline, devR = h->R.dev;
    fprintf(stderr,
        "[blinkd-debug] L(base=%.3f, dev=%.3f) R(base=%.3f, dev=%.3f)\n",
        baseL, devL, baseR, devR);
}
#endif

// POSIX shared-memory ring buffer
#ifdef __unix__
#include <sys/mman.h>
#include <fcntl.h>

typedef struct {
  uint32_t t_ms;
  float openL, openR;
} BlinkdSample;

typedef struct BlinkdShm {
  size_t cap;
  volatile size_t head;
  volatile size_t tail;
  BlinkdSample data[1];
} BlinkdShmRaw;

struct BlinkdShm {
  BlinkdShmRaw* r;
  size_t bytes;
  char name[64];
  int fd;
};

BlinkdShm* blinkd_shm_open(const char* name, size_t capacity){
  if(!name || name[0] != '/') { fprintf(stderr,"shm name must start with '/'\n"); return NULL; }
  size_t bytes = sizeof(BlinkdShmRaw) + (capacity-1)*sizeof(BlinkdSample);
  int fd = shm_open(name, O_CREAT|O_RDWR, 0600);
  if(fd < 0){ perror("shm_open"); return NULL; }
  if(ftruncate(fd, (off_t)bytes) < 0){ perror("ftruncate"); close(fd); return NULL; }
  BlinkdShmRaw* r = (BlinkdShmRaw*)mmap(NULL, bytes, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
  if(r == MAP_FAILED){ perror("mmap"); close(fd); return NULL; }
  r->cap = capacity; r->head = r->tail = 0;
  BlinkdShm* h = (BlinkdShm*)calloc(1,sizeof(BlinkdShm));
  h->r = r; h->bytes = bytes; h->fd = fd; strncpy(h->name, name, sizeof(h->name)-1);
  return h;
}

int blinkd_shm_push(BlinkdShm* shm, uint32_t t_ms, float openL, float openR){
  if(!shm || !shm->r) return -1;
  BlinkdShmRaw* r = shm->r;
  size_t next = (r->head + 1) % r->cap;
  if(next == r->tail) return -1; // full
  BlinkdSample s = { t_ms, openL, openR };
  r->data[r->head] = s;
  __sync_synchronize();
  r->head = next;
  return 0;
}

void blinkd_shm_close(BlinkdShm* shm){
  if(!shm) return;
  if(shm->r && shm->bytes) munmap((void*)shm->r, shm->bytes);
  if(shm->fd >= 0) close(shm->fd);
  shm_unlink(shm->name);
  free(shm);
}
#endif // __unix__
