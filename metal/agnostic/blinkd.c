/*
  Blink Detection + IPC
  - Ultra-lightweight, dependency-free core detector (O(1) per sample)
  - Works with 1 or 2 eye channels (openness in [0..1])
  - Adaptive thresholds using EMA + robust noise scale
  - Finite State Machine with hysteresis + refractory to avoid chatter
  - Emits events: BLINK, LONG_BLINK, DOUBLE_BLINK, WINK_LEFT, WINK_RIGHT
  - IPC: UDP JSON broadcast (localhost) + optional POSIX shared-memory ring buffer

  Build:
    gcc -O3 -march=native -Wall -Wextra -o blinkd blinkd.c -lm -lpthread

  Example run (stdin streaming openness values at 200 Hz):
    # feed time(ms) openL openR
    python feeder.py | ./blinkd --udp 127.0.0.1:7777 --rate 200

  Integrate with a camera pipeline by mapping any eye openness estimator to [0..1]
  (e.g., Eye Aspect Ratio, iris distance, or segmentation percent-open).
*/

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Time util
static uint64_t now_monotonic_ms(void) {
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec*1000ull + (uint64_t)ts.tv_nsec/1000000ull;
}

// Detector
typedef enum {
  EYE_OPEN=0, EYE_CLOSING=1, EYE_CLOSED=2, EYE_OPENING=3, EYE_REFRACT=4
} eye_state_t;

// Events bitmask
enum {
  EVT_NONE        = 0,
  EVT_BLINK       = 1<<0,
  EVT_LONG_BLINK  = 1<<1,
  EVT_DOUBLE_BLINK= 1<<2,
  EVT_WINK_LEFT   = 1<<3,
  EVT_WINK_RIGHT  = 1<<4
};

typedef struct {

  // configuration
  float ema_alpha;          // smoothing for baseline (0.001..0.05)
  float noise_alpha;        // smoothing for deviation (0.001..0.1)
  float close_thresh_k;     // how many sigmas below baseline is \"closed\"
  float open_thresh_k;      // how many sigmas above closed to re-open
  uint32_t min_blink_ms;    // minimum closed duration to count as blink
  uint32_t long_blink_ms;   // threshold for long blink
  uint32_t max_blink_ms;    // cap single blink duration
  uint32_t double_gap_ms;   // max gap between blinks for double-blink
  uint32_t refractory_ms;   // ignore chatter after event

  // adaptive stats
  float baseline;           // EMA of openness when open
  float dev;                // robust deviation (~sigma)

  // state
  eye_state_t st;
  uint32_t st_enter_ms;
  uint32_t last_blink_end_ms;
  uint8_t  last_was_blink;

  // measurements
  float last_openness;
} EyeDet;

static void eyedet_init(EyeDet* d, float init_open) {
  memset(d,0,sizeof(*d));
  d->ema_alpha = 0.01f;
  d->noise_alpha = 0.02f;
  d->close_thresh_k = 2.5f; // stricter for fewer false positives
  d->open_thresh_k  = 1.5f; // hysteresis to reopen
  d->min_blink_ms = 40;     // human blink ~100-400ms; accept >=40ms
  d->long_blink_ms = 400;   // long blink marker
  d->max_blink_ms = 800;    // beyond this treat as hold/ignore
  d->double_gap_ms = 300;   // two quick blinks within this window
  d->refractory_ms = 60;    // ignore chatter post event

  d->baseline = init_open;
  d->dev = 0.03f; // start with small noise
  d->st = EYE_OPEN;
  d->st_enter_ms = 0;
  d->last_blink_end_ms = 0;
  d->last_was_blink = 0;
  d->last_openness = init_open;
}

static inline float clamp01(float x){return x<0?0:(x>1?1:x);} 
static inline float ema(float prev, float x, float a){return prev + a*(x-prev);} 

// Update adaptive baseline only when reasonably open to avoid drift during closes.
static inline void update_stats(EyeDet* d, float open, int is_open_like){
  if(is_open_like) d->baseline = ema(d->baseline, open, d->ema_alpha);
  float absdev = fabsf(open - d->baseline);
  d->dev = ema(d->dev, absdev, d->noise_alpha);
  if(d->dev < 1e-3f) d->dev = 1e-3f; // floor
}

// Returns events bitmask; optionally outputs blink duration in *blink_ms
static uint32_t eyedet_update(EyeDet* d, uint32_t t_ms, float openness, uint32_t* blink_ms){
  uint32_t ev = EVT_NONE; if(blink_ms) *blink_ms = 0; 
  openness = clamp01(openness);
  d->last_openness = openness;

  float sigma = d->dev;
  float close_thr = d->baseline - d->close_thresh_k * sigma;
  float open_thr  = d->baseline - d->open_thresh_k  * sigma; // must go above this to reopen

  int is_open_like = openness >= open_thr; // for stats update
  update_stats(d, openness, is_open_like);

  switch(d->st){
    case EYE_OPEN:
      if(openness < close_thr){ d->st = EYE_CLOSING; d->st_enter_ms = t_ms; }
      break;
    case EYE_CLOSING:
      if(openness < close_thr){ d->st = EYE_CLOSED; d->st_enter_ms = t_ms; }
      else if(openness >= open_thr){ d->st = EYE_OPEN; } // bounce
      break;
    case EYE_CLOSED: {
      uint32_t dur = t_ms - d->st_enter_ms;
      if(openness >= open_thr){
        // evaluate blink
        if(dur >= d->min_blink_ms && dur <= d->max_blink_ms){
          if(blink_ms) *blink_ms = dur;
          ev |= EVT_BLINK;
          if(dur >= d->long_blink_ms) ev |= EVT_LONG_BLINK;
          // double-blink detection
          if(d->last_was_blink && (t_ms - d->last_blink_end_ms) <= d->double_gap_ms) ev |= EVT_DOUBLE_BLINK;
          d->last_was_blink = 1; d->last_blink_end_ms = t_ms;
        } else {
          d->last_was_blink = 0;
        }
        d->st = EYE_REFRACT; d->st_enter_ms = t_ms; // refractory
      }
    } break;
    case EYE_REFRACT:
      if((t_ms - d->st_enter_ms) >= d->refractory_ms) d->st = EYE_OPEN;
      break;
    case EYE_OPENING:
      // unused
      d->st = EYE_OPEN; break;
  }

  return ev;
}

// Binaural wrapper
typedef struct {
  EyeDet L, R;
  uint32_t wink_min_ms; // require at least this closed duration on one side
} BlinkDet2;

static void blinkdet2_init(BlinkDet2* b, float init_open_L, float init_open_R){
  eyedet_init(&b->L, init_open_L);
  eyedet_init(&b->R, init_open_R);
  b->wink_min_ms = 60;
}

static uint32_t blinkdet2_update(BlinkDet2* b, uint32_t t_ms, float open_L, float open_R, uint32_t* blink_ms_out){
  uint32_t evL=0, evR=0; uint32_t msL=0, msR=0; 
  evL = eyedet_update(&b->L, t_ms, open_L, &msL);
  evR = eyedet_update(&b->R, t_ms, open_R, &msR);

  uint32_t ev = evL | evR; 
  if(blink_ms_out) *blink_ms_out = (msL>msR?msL:msR);

  // wink detection: one side blink, the other stays open-like
  if((evL & EVT_BLINK) && !(evR & EVT_BLINK) && msL >= b->wink_min_ms && b->R.last_openness > b->R.baseline - 1.0f*b->R.dev)
    ev |= EVT_WINK_LEFT;
  if((evR & EVT_BLINK) && !(evL & EVT_BLINK) && msR >= b->wink_min_ms && b->L.last_openness > b->L.baseline - 1.0f*b->L.dev)
    ev |= EVT_WINK_RIGHT;

  return ev;
}

// IPC: UDP JSON
typedef struct { int sock; struct sockaddr_in addr; int enabled; } UdpOut;

static int udp_open(UdpOut* u, const char* ip, uint16_t port){
  u->sock = socket(AF_INET, SOCK_DGRAM, 0);
  if(u->sock<0) return -1;
  memset(&u->addr,0,sizeof(u->addr));
  u->addr.sin_family = AF_INET;
  u->addr.sin_port = htons(port);
  if(inet_aton(ip, &u->addr.sin_addr)==0) return -2;
  u->enabled = 1; return 0;
}

static void udp_send_event(UdpOut* u, const char* type, uint32_t t_ms, uint32_t dur_ms, uint32_t flags){
  if(!u->enabled) return;
  char buf[256];
  // minimal JSON; avoid locale issues
  int n = snprintf(buf, sizeof(buf),
    "{\"t_ms\":%u,\"type\":\"%s\",\"dur_ms\":%u,\"flags\":%u}\n",
    t_ms, type, dur_ms, flags);
  sendto(u->sock, buf, (size_t)n, 0, (struct sockaddr*)&u->addr, sizeof(u->addr));
}

// POSIX shm ring buffer: single-producer, single-consumer for eye samples. Each sample: t_ms, openL, openR
typedef struct { uint32_t t_ms; float openL, openR; } EyeSample;

typedef struct {
  size_t cap; volatile size_t head; volatile size_t tail; EyeSample data[1];
} Ring;

static Ring* ring_create(const char* name, size_t capacity){
  size_t bytes = sizeof(Ring) + (capacity-1)*sizeof(EyeSample);
  int fd = shm_open(name, O_CREAT|O_RDWR, 0600);
  if(fd<0){perror("shm_open"); return NULL;}
  if(ftruncate(fd, (off_t)bytes)<0){perror("ftruncate"); return NULL;}
  Ring* r = (Ring*)mmap(NULL, bytes, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
  if(r==MAP_FAILED){perror("mmap"); return NULL;}
  r->cap = capacity; r->head = r->tail = 0; return r;
}

static inline int ring_push(Ring* r, const EyeSample* s){
  size_t next = (r->head + 1) % r->cap; if(next==r->tail) return -1; // full
  r->data[r->head] = *s; __sync_synchronize(); r->head = next; return 0;
}

static inline int ring_pop(Ring* r, EyeSample* s){
  if(r->tail == r->head) return -1; // empty
  *s = r->data[r->tail]; __sync_synchronize(); r->tail = (r->tail+1)%r->cap; return 0;
}

// CLI / Glue
typedef struct {
  int use_udp; char ip[64]; uint16_t port;
  int use_shm; char shm_name[64]; size_t shm_cap;
  uint32_t sample_rate_hz; // for stdin mode
} Args;

static void usage(const char* exe){
  fprintf(stderr,
  "Usage: %s [--udp ip:port] [--shm /blinkring N] [--rate Hz]\n\n"
  "Input modes:\n"
  "  * Stdin: lines \"t_ms openL openR\" (floats allowed), with --rate to help timing.\n"
  "  * Shared memory: producer writes EyeSample to ring named --shm.\n"
  , exe);
}

static int parse_ipport(const char* s, char* ip, uint16_t* port){
  const char* c = strchr(s, ':'); if(!c) return -1; size_t n = (size_t)(c-s);
  if(n>=63) return -1; memcpy(ip, s, n); ip[n]='\0'; *port = (uint16_t)atoi(c+1); return 0;
}

int main(int argc, char** argv){
  Args a = {0}; a.sample_rate_hz = 200; a.shm_cap = 4096;
  for(int i=1;i<argc;i++){
    if(strcmp(argv[i],"--udp")==0 && i+1<argc){ parse_ipport(argv[++i], a.ip, &a.port); a.use_udp=1; }
    else if(strcmp(argv[i],"--shm")==0 && i+2<argc){ strncpy(a.shm_name, argv[++i],63); a.shm_cap = (size_t)strtoul(argv[++i],NULL,10); a.use_shm=1; }
    else if(strcmp(argv[i],"--rate")==0 && i+1<argc){ a.sample_rate_hz = (uint32_t)strtoul(argv[++i],NULL,10);} 
    else { usage(argv[0]); }
  }

  UdpOut udp={0}; if(a.use_udp){ if(udp_open(&udp, a.ip, a.port)!=0){ perror("udp_open"); a.use_udp=0; } }

  BlinkDet2 B; blinkdet2_init(&B, 1.0f, 1.0f);

  Ring* ring = NULL; if(a.use_shm){ ring = ring_create(a.shm_name, a.shm_cap); if(!ring){ fprintf(stderr,"Failed to create shm ring\n"); return 1; } }

  uint64_t t0 = now_monotonic_ms();
  uint32_t last_emit_ms = 0;
  uint32_t dur_ms = 0; (void)last_emit_ms;

  if(ring){
    // consume from shared memory ring
    EyeSample s; 
    for(;;){
      if(ring_pop(ring, &s)==0){
        uint32_t ev = blinkdet2_update(&B, s.t_ms, s.openL, s.openR, &dur_ms);
        if(ev & EVT_BLINK){ udp_send_event(&udp, "blink", s.t_ms, dur_ms, ev); }
        if(ev & EVT_WINK_LEFT){ udp_send_event(&udp, "wink_left", s.t_ms, dur_ms, ev); }
        if(ev & EVT_WINK_RIGHT){ udp_send_event(&udp, "wink_right", s.t_ms, dur_ms, ev); }
      } else { usleep(1000); }
    }
  } else {
    // stdin mode: t_ms openL openR per line (or we synthesize time if t_ms<0)
    char line[256];
    while(fgets(line,sizeof(line),stdin)){
      float t_ms_f, L, R; if(sscanf(line, "%f %f %f", &t_ms_f, &L, &R)!=3) continue;
      uint32_t t_ms = (t_ms_f>=0)? (uint32_t)t_ms_f : (uint32_t)(now_monotonic_ms()-t0);
      uint32_t ev = blinkdet2_update(&B, t_ms, L, R, &dur_ms);
      if(ev){
        if(ev & EVT_BLINK){ udp_send_event(&udp, "blink", t_ms, dur_ms, ev); printf("EVT BLINK dur=%u flags=%u\n", dur_ms, ev); }
        if(ev & EVT_LONG_BLINK){ printf("EVT LONG_BLINK\n"); }
        if(ev & EVT_DOUBLE_BLINK){ printf("EVT DOUBLE_BLINK\n"); }
        if(ev & EVT_WINK_LEFT){ udp_send_event(&udp, "wink_left", t_ms, dur_ms, ev); printf("EVT WINK_LEFT\n"); }
        if(ev & EVT_WINK_RIGHT){ udp_send_event(&udp, "wink_right", t_ms, dur_ms, ev); printf("EVT WINK_RIGHT\n"); }
        fflush(stdout);
      }
      // pacing if user supplies only openness w/o timestamps
      if(t_ms_f < 0 && a.sample_rate_hz>0){ usleep(1000000u / a.sample_rate_hz); }
    }
  }
  return 0;
}
