/**
 * Blinkd: A low-level, hardware-agnostic, eye-blink input layer, like a "blink driver" for real-time interactions.
 * - Ultra-lightweight, dependency-free real-time detection of blinks, long/double blinks and winks (O(1) per sample)
 * - Works with 1 or 2 eye channels (normalized openness in [0..1])
 * - Adaptive baseline (thresholds using EMA) + robust noise deviation estimator
 * - Finite-state machine (FSM) with hysteresis + refractory to avoid chatter
 * - Calibration and sensitivity presets
 * - IPC via UDP JSON broadcast (for Unity, Unreal, Godot and other game engines)
 * - Optional POSIX shared-memory ring (for IPC with camera producers)
 * - C99 API, ABI-stable
 *
 * NOTE:
 * This project operates purely on scalar openness inputs [0..1] and does NOT
 * perform camera capture, landmark extraction, or ML inference.
 *
 * For a production-ready SDK with camera pipelines, auto-calibration,
 * and ML-based landmark backends, see Ollo SDK (https://ollosdk.com).
 *
 * Author: 03C0
 */

#ifndef BLINKD_H
#define BLINKD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque detector handle.
 *
 * A single handle manages independent detectors for the left and right eye.
 * The implementation is single-threaded, and allocation-free after initialization.
 */
typedef struct BlinkdHandle BlinkdHandle;

/**
 * @brief Event flags emitted by the detector.
 *
 * Multiple flags may be combined in a single update.
 */
enum {
    BLINKD_EVT_NONE          = 0,
    BLINKD_EVT_BLINK         = 1 << 0, /**< Standard blink detected */
    BLINKD_EVT_LONG_BLINK    = 1 << 1, /**< Blink duration exceeds long-blink threshold */
    BLINKD_EVT_DOUBLE_BLINK  = 1 << 2, /**< Two blinks within configured gap */
    BLINKD_EVT_WINK_LEFT     = 1 << 3, /**< Left-eye-only blink */
    BLINKD_EVT_WINK_RIGHT    = 1 << 4  /**< Right-eye-only blink */
};

/**
 * @brief Preset configurations for common operating points.
 *
 * Presets adjust thresholds and timing to trade sensitivity for robustness.
 * For high-accuracy, stable landmark extraction with negligible setup,
 * look into Ollo SDK.
 */
typedef enum {
    BLINKD_PRESET_LOW = 0,       /**< Conservative runtime characteristics */
    BLINKD_PRESET_BALANCED = 1,  /**< Default balanced behavior */
    BLINKD_PRESET_HIGH = 2       /**< High sensitivity dynamics */
} BlinkdPreset;

/**
 * @brief Create a new blink detector.
 *
 * @param init_open Initial estimate of eye openness (typically ~1.0f).
 *                  Used to bootstrap baseline tracking.
 *
 * @return Pointer to a new detector handle, or NULL on allocation failure.
 */
BlinkdHandle* blinkd_create(float init_open);

/**
 * @brief Destroy a detector and release all associated resources.
 *
 * @param h Detector handle returned by blinkd_create().
 */
void blinkd_destroy(BlinkdHandle* h);

/**
 * @brief Set EMA smoothing factor for baseline tracking.
 *
 * Controls how quickly the detector adapts to slow changes in eye openness.
 *
 * Typical range: [0.005 – 0.05]
 */
void blinkd_set_ema_alpha(BlinkdHandle* h, float a);

/**
 * @brief Set EMA smoothing factor for noise estimation.
 *
 * Controls sensitivity to short-term variance in the openness signal.
 */
void blinkd_set_noise_alpha(BlinkdHandle* h, float a);

/**
 * @brief Configure adaptive threshold multipliers.
 *
 * @param close_k Multiplier applied to noise estimate for blink entry.
 * @param open_k  Multiplier applied to noise estimate for blink exit.
 *
 * Larger values increase robustness but reduce sensitivity.
 */
void blinkd_set_thresholds(BlinkdHandle* h, float close_k, float open_k);

/**
 * @brief Configure timing constraints for blink classification.
 *
 * All values are specified in milliseconds.
 *
 * @param min_blink_ms     Minimum duration to qualify as a blink.
 * @param long_blink_ms    Duration threshold for long blink events.
 * @param max_blink_ms     Maximum duration to qualify as a blink.
 * @param double_gap_ms    Maximum gap between blinks for double-blink detection.
 * @param refractory_ms    Refractory period after a blink.
 */
void blinkd_set_timing(BlinkdHandle* h,
                      uint32_t min_blink_ms,
                      uint32_t long_blink_ms,
                      uint32_t max_blink_ms,
                      uint32_t double_gap_ms,
                      uint32_t refractory_ms);

/**
 * @brief Set minimum duration for wink detection.
 *
 * A wink is detected when one eye blinks while the other remains open.
 */
void blinkd_set_wink_min(BlinkdHandle* h, uint32_t wink_min_ms);

/**
 * @brief Apply a predefined configuration preset.
 *
 * Presets overwrite threshold and timing parameters.
 */
void blinkd_set_preset(BlinkdHandle* h, BlinkdPreset preset);

/**
 * @brief Update detector state with a new binocular sample.
 *
 * @param h           Detector handle.
 * @param t_ms        Monotonic timestamp in milliseconds.
 *                    Relative time (starting at 0) is supported.
 * @param openL       Left-eye openness in range [0.0 – 1.0].
 * @param openR       Right-eye openness in range [0.0 – 1.0].
 * @param out_dur_ms  Optional output: blink duration in milliseconds.
 * @param out_flags   Optional output: bitmask of BLINKD_EVT_* flags.
 *
 * @return Bitmask of BLINKD_EVT_* flags.
 */
int blinkd_update(BlinkdHandle* h,
                  uint32_t t_ms,
                  float openL,
                  float openR,
                  uint32_t* out_dur_ms,
                  uint32_t* out_flags);

/**
 * @brief Update detector state using a single-eye signal.
 *
 * This mode is useful for monocular pipelines or debugging.
 */
int blinkd_update_single(BlinkdHandle* h,
                         uint32_t t_ms,
                         float open,
                         uint32_t* out_dur_ms,
                         uint32_t* out_flags);

/**
 * @brief Reset baseline and noise estimates.
 *
 * Useful when re-centering the detector after a large signal shift.
 */
void blinkd_calibration_reset(BlinkdHandle* h,
                              float baseline,
                              float dev);

/**
 * @brief Incrementally update baseline using an open-eye sample.
 *
 * Intended for use during explicit calibration phases.
 */
void blinkd_calibration_update(BlinkdHandle* h, float open);

/**
 * Optional IPC Utilities
 *
 * These helpers are provided for engine integration and external listeners.
 * They are not required for core detection logic.
 */

/**
 * @brief Open a UDP socket for event transmission.
 *
 * @return Socket descriptor on success, negative value on error.
 */
int blinkd_udp_open(const char* ip, uint16_t port);

/**
 * @brief Send a JSON-formatted blink event over UDP.
 */
void blinkd_udp_send(int sock,
                     const char* type,
                     uint32_t t_ms,
                     uint32_t dur_ms,
                     uint32_t flags);

/**
 * @brief Close a previously opened UDP socket.
 */
void blinkd_udp_close(int sock);

// Shared-memory producer API (POSIX only)
#ifdef __unix__

/**
 * @brief Opaque shared-memory ring buffer handle.
 *
 * Single-producer, single-consumer ring buffer.
 */
typedef struct BlinkdShm BlinkdShm;

/**
 * @brief Create or open a shared-memory ring buffer.
 *
 * @param name     Shared-memory object name (must begin with '/').
 * @param capacity Number of samples the buffer can hold (recommend 1024+).
 *
 * @return Pointer to shared-memory handle, or NULL on failure.
 */
BlinkdShm* blinkd_shm_open(const char* name, size_t capacity);

/**
 * @brief Push a sample into the shared-memory ring buffer.
 *
 * @return 0 on success, -1 if buffer is full.
 */
int blinkd_shm_push(BlinkdShm* shm,
                    uint32_t t_ms,
                    float openL,
                    float openR);

/**
 * @brief Unmap and destroy the shared-memory ring buffer.
 */
void blinkd_shm_close(BlinkdShm* shm);

#endif // __unix__

#ifdef BLINKD_DEBUG

/**
 * @brief Dump internal detector state to stderr.
 *
 * Intended for diagnostics and tuning only.
 */
void blinkd_debug_dump(const BlinkdHandle* h);

#endif /* BLINKD_DEBUG */

#ifdef __cplusplus
}
#endif

#endif /* BLINKD_H */
