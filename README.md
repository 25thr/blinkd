<img align="left" style="width:260px" src="https://github.com/0x48piraj/blinkd/blob/master/docs/assets/blinkd_logo.png" width="288px">

**Real-time blink detection engine turning blinks into actions for games, accessibility, and next-gen interfaces.**

`blinkd` is essentially a **low-latency, hardware-agnostic, eye-blink input layer**, like a **"blink driver"** for real-time interactions.

**Note for builders**

*`blinkd` is a low-latency input kernel. It does one thing well: turn eyelid signals into clean events. Designed to be real-time and input-source neutral. If you're building with it, you're building from the metal up.*

<br>

This could become foundational tech for:

* **Games** (blink-to-shoot, wink-to-jump, etc.)
* **Accessibility** (hands-free UI control)
* **AR/VR** (gaze + blink fusion input)
* **Biometric interfaces** (fatigue detection, attentiveness)
* **Expression-based UI** (long blink = cancel, double = confirm)

We're building a **real-time eye input kernel**, which:

1. Ingests camera data
2. Processes it into a clean binary event stream (`BLINK`, `LONG_BLINK`, `WINK_LEFT`, etc.)
3. Exposes that stream via standard IPC (UDP, shared memory)
4. Can be extended as a **system input driver**

## Features

The C implementation which is language-agnostic comes with,

- **Real-time detector**: EMA-based adaptive thresholds + hysteresis + refractory
- Handles **one or two eyes**: Emits `BLINK`, `LONG_BLINK`, `DOUBLE_BLINK`, `WINK_LEFT`, `WINK_RIGHT`
- **IPC built-in**: UDP JSON events to your game engine, plus an _optional POSIX shared-memory ring buffer_ for high-rate pipelines
- **O(1)** per sample: No external deps for the core

### Feeding

Provide _openness_ in `[0..1]` per eye at ~120–240 Hz. Map any upstream signal to that range, e.g.:

- **Eye Aspect Ratio (EAR)**: normalize per-user to `[0..1]`
- **Iris/eyelid distance** or **segmentation % open**
- **IR camera “eye open probability”** from a tiny model
- The detector handles baseline drift and noise adaptively.

### IPC for engines

* **UDP JSON** (zero dependencies):
  `{"t_ms":123456,"type":"blink","dur_ms":132,"flags":3}`
* In **Unity**: read with `UdpClient` and parse JSON; map to gameplay (e.g., slow-mo on `DOUBLE_BLINK`, pause on `LONG_BLINK`, left/right abilities on winks).
* In **Unreal**: a small UDP receiver (UE’s `FUdpSocketReceiver`) → Blueprint events.

We can also expose a local WebSocket API _(to be implemented)_,

```bash
ws://localhost:8765
```

### Tuning

* `min_blink_ms` (≥40 ms), `long_blink_ms` (\~400 ms), `double_gap_ms` (\~300 ms)
* `close_thresh_k` / `open_thresh_k` control sensitivity & hysteresis
* `refractory_ms` prevents chatter in rapid eyelid tremors

## Build & run

```bash
gcc -O3 -march=native -Wall -Wextra -o blinkd blinkd.c -lm -lpthread
# Stream samples via stdin (time_ms openL openR), broadcast to localhost:7777
./blinkd --udp 127.0.0.1:7777 --rate 200
```

Clients (games, Unity, Godot, etc.) can easily consume them.

## Architecture

```
[Camera + EAR] → [Blink FSM] → [Event Queue] → 
→ [uinput / SendInput / WebSocket / Game Plugin] → [Games & Apps]
```
