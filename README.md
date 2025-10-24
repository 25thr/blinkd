<img align="left" style="width:260px" src="https://github.com/0x48piraj/blinkd/blob/master/docs/assets/blinkd_logo.png" width="288px">

**Real-time blink detection engine turning blinks into actions for games, accessibility, and next-gen interfaces.**

`blinkd` is essentially a **low-latency, hardware-agnostic, eye-blink input layer**, like a **"blink driver"** for real-time interactions.

**Note for builders**

*`blinkd` is a low-latency input kernel. It does one thing well: turn eyelid signals into clean events. Designed to be real-time and input-source neutral. If you're building with it, you're building from the metal up.*

This could become foundational tech for:

* **Games**
* **Accessibility**
* **AR/VR**
* **Biometric interfaces** _(fatigue detection, attentiveness)_
* **Expression-based UI** _(hands-free UI control)_

We're building a **real-time eye input kernel**, which:

1. Ingests camera data
2. Processes it into a clean binary event stream (`BLINK`, `LONG_BLINK`, `WINK_LEFT`, etc.)
3. Exposes that stream via standard IPC (UDP, shared memory)
4. Can be extended as a **system input driver**

## Features

The C implementation which is language-agnostic comes with,

- **Real-time detector**: EMA-based adaptive thresholds + hysteresis + refractory
- Handles **one or two eyes**: Emits `BLINK`, `LONG_BLINK`, `DOUBLE_BLINK`, `WINK_LEFT`, `WINK_RIGHT`
- **IPC built-in**: UDP JSON events for game engines (Unreal Engine, Godot, Unity), plus an _optional POSIX shared-memory ring buffer_ for high-rate pipelines
- **O(1)** per sample: No external deps for the core

## Architecture

<p align="center">
  <img src="https://github.com/0x48piraj/blinkd/blob/master/docs/assets/blinkd_process_pipeline.png" alt="Architecture diagram">
</p>

### Feeding

Provide _openness_ in `[0..1]` per eye at ~120–240 Hz. Map any upstream signal to that range, e.g.:

- **Eye Aspect Ratio (EAR)**: normalize per-user to `[0..1]`
- **Iris/eyelid distance** or **segmentation % open**
- **IR camera "eye open probability"** from a ML model
- The detector handles baseline drift and noise adaptively.

### IPC for engines

* **UDP JSON** (zero dependencies):
  `{"t_ms":123456,"type":"blink","dur_ms":132,"flags":3}`

We can also expose a local WebSocket API _(to be implemented)_,

```bash
ws://localhost:8765
```

#### Shared-memory IPC support (POSIX only)

* It creates a named shared memory region (`/blinkdring`) that both producer (camera pipeline) and consumer (game engine or monitor tool) can map.
* Each slot is a `BlinkdSample` (timestamp + two openness floats).
* The ring uses simple head/tail pointers with `__sync_synchronize()` memory barrier.
* You can push samples at 200 Hz safely from one process and consume from another with minimal overhead.

**Usage example (producer):**

```c
BlinkShm* shm = blink_shm_open("/blinkdring", 4096);
while(1){
  float L = 0.9f, R = 0.8f;
  uint32_t t = (uint32_t)(clock()*1000/CLOCKS_PER_SEC);
  blink_shm_push(shm, t, L, R);
  usleep(5000);
}
blink_shm_close(shm);
```

The consumer (another process) can map the same `/blinkdring` and read in a loop.

### Tuning

The `blinkd` SDK provides several configuration options to fine-tune blink detection based on camera quality, user behavior, or latency tolerance. You can adjust sensitivity, detection timing, and noise rejection using the tuning API.

### Default behavior

When you create a `BlinkdHandle` via `blinkd_create()`, it initializes with **default balanced parameters**:

```c
blinkd_set_thresholds(h, 2.5f, 1.5f);  // close_k, open_k
blinkd_set_timing(h, 40, 400, 800, 300, 60); // min, long, max, dbl_gap, refractory
```

#### Thresholds

These control how tightly the system defines a "closed" or "open" eye, relative to baseline noise:

```c
void blinkd_set_thresholds(BlinkdHandle* h, float close_k, float open_k);
```

* `close_k`: How many deviations below baseline to consider "closed"
* `open_k`: How many deviations below baseline to consider "open"

Lower values increase sensitivity; higher values are more conservative.

#### Timing

```c
void blinkd_set_timing(BlinkdHandle* h,
    uint32_t min_blink_ms,
    uint32_t long_blink_ms,
    uint32_t max_blink_ms,
    uint32_t double_gap_ms,
    uint32_t refractory_ms);
```

* `min_blink_ms`: Minimum duration to count as a blink
* `long_blink_ms`: Threshold for long blinks
* `max_blink_ms`: Max duration before ignoring as noise
* `double_gap_ms`: Time window for detecting double blinks
* `refractory_ms`: Cooldown time after a blink before accepting another

#### EMA

```c
void blinkd_set_ema_alpha(BlinkdHandle* h, float alpha);       // default: 0.01
void blinkd_set_noise_alpha(BlinkdHandle* h, float alpha);     // default: 0.02
```

These control how quickly the algorithm adapts to baseline changes or noise. Smaller = smoother, but slower.

#### Wink detection

```c
void blinkd_set_wink_min(BlinkdHandle* h, uint32_t wink_min_ms);
```

This sets the **minimum duration** for a one-eye blink to qualify as a wink.

#### Presets

To simplify tuning, you can use one of the built-in presets:

```c
void blinkd_set_preset(BlinkdHandle* h, BlinkPreset preset);
```

Available presets:

* `BLINK_PRESET_LOW`: For noisy input (e.g. low-res cams, shaky face)
* `BLINK_PRESET_HIGH`: For clean input, fast detection
* `BLINK_PRESET_BALANCED` *(default)*: Safe middle ground

### Example: Custom configuration

```c
BlinkdHandle* h = blinkd_create(init_open);
blinkd_set_thresholds(h, 2.8f, 1.4f);
blinkd_set_timing(h, 50, 300, 750, 250, 50);
blinkd_set_ema_alpha(h, 0.02f);
blinkd_set_noise_alpha(h, 0.03f);
blinkd_set_wink_min(h, 70);
```

## Build & run

```bash
cmake --build . --config Release
# Stream samples via stdin (time_ms openL openR), broadcast to localhost:7777
./blinkdctl
```

Clients (apps, game engines, etc.) can easily consume them.
