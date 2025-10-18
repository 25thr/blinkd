/*
  Blinkd: A low-latency, hardware-agnostic, eye-blink input layer, like a "blink driver" for real-time interactions.
  - Ultra-lightweight, dependency-free core detector (O(1) per sample)
  - Works with 1 or 2 eye channels (openness in [0..1])
  - Adaptive baseline (thresholds using EMA) + robust noise deviation estimator
  - Finite State Machine with hysteresis + refractory to avoid chatter
  - Calibration and sensitivity presets
  - Emits events: BLINK, LONG_BLINK, DOUBLE_BLINK, WINK_LEFT, WINK_RIGHT
  - IPC: UDP JSON broadcast (for Unity, Unreal, Godot)
  - Optional POSIX shared-memory ring (for IPC with camera producers)
  - C99 API, ABI-stable

  Integrate with a camera pipeline by mapping any eye openness estimator to [0..1]
  (e.g., Eye Aspect Ratio, iris distance, or segmentation percent-open).

  Author: 03C0
*/

#ifndef BLINKD_H
#define BLINKD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// -------------------------------------------------------------
// Types / constants
// -------------------------------------------------------------
typedef struct BlinkHandle BlinkHandle;

enum {
  BLINK_EVT_NONE        = 0,
  BLINK_EVT_BLINK       = 1<<0,
  BLINK_EVT_LONG_BLINK  = 1<<1,
  BLINK_EVT_DOUBLE_BLINK= 1<<2,
  BLINK_EVT_WINK_LEFT   = 1<<3,
  BLINK_EVT_WINK_RIGHT  = 1<<4
};

// -------------------------------------------------------------
// Lifecycle
// -------------------------------------------------------------
BlinkHandle* blink_create(float init_open);
void blink_destroy(BlinkHandle* h);

// -------------------------------------------------------------
// Configuration (optional; call before first update)
// -------------------------------------------------------------
void blink_set_ema_alpha(BlinkHandle* h, float a);
void blink_set_noise_alpha(BlinkHandle* h, float a);
void blink_set_thresholds(BlinkHandle* h, float close_k, float open_k);
void blink_set_timing(BlinkHandle* h,
                      uint32_t min_blink_ms, uint32_t long_blink_ms,
                      uint32_t max_blink_ms, uint32_t double_gap_ms,
                      uint32_t refractory_ms);
void blink_set_wink_min(BlinkHandle* h, uint32_t wink_min_ms);

// Presets for convenience
typedef enum { BLINK_PRESET_LOW=0, BLINK_PRESET_BALANCED=1, BLINK_PRESET_HIGH=2 } BlinkPreset;
void blink_set_preset(BlinkHandle* h, BlinkPreset p);

// -------------------------------------------------------------
// Runtime updates
// -------------------------------------------------------------
int blink_update(BlinkHandle* h, uint32_t t_ms,
                 float openL, float openR,
                 uint32_t* out_dur_ms, uint32_t* out_flags);

int blink_update_single(BlinkHandle* h, uint32_t t_ms,
                        float open, uint32_t* out_dur_ms, uint32_t* out_flags);

// Calibration helpers
void blink_reset_baseline(BlinkHandle* h, float baseline, float dev);
void blink_calibrate_sample(BlinkHandle* h, float open);

// -------------------------------------------------------------
// IPC utilities (optional, for engines / external listeners)
// -------------------------------------------------------------
int  blink_udp_open(const char* ip, uint16_t port);
void blink_udp_send(int sock, const char* type,
                    uint32_t t_ms, uint32_t dur_ms, uint32_t flags);
void blink_udp_close(int sock);

// -------------------------------------------------------------
// Shared-memory producer API (POSIX only)
// -------------------------------------------------------------
#ifdef __unix__

// Single-producer, single-consumer ring buffer
typedef struct BlinkShm BlinkShm;

// Create or open a shared-memory ring buffer
// name: e.g. "/blinkring" (must begin with '/')
// capacity: number of samples (recommend 1024+)
BlinkShm* blink_shm_open(const char* name, size_t capacity);

// Push one sample (t_ms, openL, openR) into the ring.
// Returns 0 on success, -1 if full.
int blink_shm_push(BlinkShm* shm, uint32_t t_ms, float openL, float openR);

// Destroy/unmap shared memory
void blink_shm_close(BlinkShm* shm);

#endif // __unix__

#ifdef __cplusplus
}
#endif
#endif // BLINKD_H
