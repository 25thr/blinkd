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

  NOTE: This OSS edition expects the caller to compute openness [0..1].
  Blinkd Pro SDK includes a native custom-built Mediapipe-based landmark tracking integration that computes openness automatically.
  Production pipelines should use the Pro SDK's ML-based backend.

  Author: 03C0
*/

#ifndef BLINKD_H
#define BLINKD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Types / constants
typedef struct BlinkdHandle BlinkdHandle;

enum {
  BLINKD_EVT_NONE        = 0,
  BLINKD_EVT_BLINK       = 1<<0,
  BLINKD_EVT_LONG_BLINK  = 1<<1,
  BLINKD_EVT_DOUBLE_BLINK= 1<<2,
  BLINKD_EVT_WINK_LEFT   = 1<<3,
  BLINKD_EVT_WINK_RIGHT  = 1<<4
};

// Lifecycle
BlinkdHandle* blinkd_create(float init_open);
void blinkd_destroy(BlinkdHandle* h);

// Configuration (optional; call before first update)
void blinkd_set_ema_alpha(BlinkdHandle* h, float a);
void blinkd_set_noise_alpha(BlinkdHandle* h, float a);
void blinkd_set_thresholds(BlinkdHandle* h, float close_k, float open_k);
void blinkd_set_timing(BlinkdHandle* h,
                      uint32_t min_blinkd_ms, uint32_t long_blinkd_ms,
                      uint32_t max_blinkd_ms, uint32_t double_gap_ms,
                      uint32_t refractory_ms);
void blinkd_set_wink_min(BlinkdHandle* h, uint32_t wink_min_ms);

// Presets for convenience
// For high-accuracy, noise-resistant landmark extraction with minimal setup, look into Blinkd Pro SDK
typedef enum { BLINKD_PRESET_LOW=0, BLINKD_PRESET_BALANCED=1, BLINKD_PRESET_HIGH=2 } BlinkdPreset;
void blinkd_set_preset(BlinkdHandle* h, BlinkdPreset p);

// Runtime updates
int blinkd_update(BlinkdHandle* h, uint32_t t_ms,
                 float openL, float openR,
                 uint32_t* out_dur_ms, uint32_t* out_flags);

int blinkd_update_single(BlinkdHandle* h, uint32_t t_ms,
                        float open, uint32_t* out_dur_ms, uint32_t* out_flags);

// Calibration helpers
void blinkd_calibration_reset(BlinkdHandle* h, float baseline, float dev);
void blinkd_calibration_update(BlinkdHandle* h, float open);

// IPC utilities (optional, for engines / external listeners)
int  blinkd_udp_open(const char* ip, uint16_t port);
void blinkd_udp_send(int sock, const char* type,
                    uint32_t t_ms, uint32_t dur_ms, uint32_t flags);
void blinkd_udp_close(int sock);

// Shared-memory producer API (POSIX only)
#ifdef __unix__

// Single-producer, single-consumer ring buffer
typedef struct BlinkdShm BlinkdShm;

// Create or open a shared-memory ring buffer
// name: e.g. "/blinkdring" (must begin with '/')
// capacity: number of samples (recommend 1024+)
BlinkdShm* blinkd_shm_open(const char* name, size_t capacity);

// Push one sample (t_ms, openL, openR) into the ring.
// Returns 0 on success, -1 if full.
int blinkd_shm_push(BlinkdShm* shm, uint32_t t_ms, float openL, float openR);

// Destroy/unmap shared memory
void blinkd_shm_close(BlinkdShm* shm);

#endif // __unix__

#ifdef BLINKD_DEBUG
void blinkd_debug_dump(const BlinkdHandle* h);
#endif

#ifdef __cplusplus
}
#endif
#endif // BLINKD_H
