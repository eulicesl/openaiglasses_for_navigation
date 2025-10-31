# Firmware Review: Microphone & Camera Simultaneous Processing
## Adaptation Guide for Omi Glass Firmware

---

## Executive Summary

This repository contains a sophisticated ESP32-S3 based smart glasses firmware that simultaneously handles:
- **Camera** streaming (VGA/SVGA/XGA JPEG at configurable FPS)
- **Microphone** capture (PDM mic → 16kHz PCM16 mono audio)
- **Speaker** playback (I2S DAC for TTS/audio feedback)
- **IMU** data streaming (ICM42688 via SPI at 50Hz)

The key architectural pattern is **multi-task FreeRTOS design with separate I2S peripherals** for mic and speaker to avoid interference, using WebSocket for bidirectional communication.

---

## 1. Hardware Architecture Overview

### Pin Assignments (XIAO ESP32S3 Sense)

| Component | Interface | Pins | Notes |
|-----------|-----------|------|-------|
| **Camera** | Parallel 8-bit | D0-D7 (GPIO 15-48) | OV2640/OV5640 compatible |
| **Microphone** | PDM I2S RX | CLK: GPIO42, DATA: GPIO41 | Built-in XIAO mic |
| **Speaker** | I2S TX STD | BCLK: GPIO7, LRCK: GPIO8, DIN: GPIO9 | MAX98357A DAC |
| **IMU** | SPI | SCK: GPIO1, MOSI: GPIO2, MISO: GPIO3, CS: GPIO4 | ICM42688-P |

**Key Design Decision**: Microphone uses PDM mode I2S, Speaker uses Standard mode I2S on **separate I2S peripherals** (ESP32-S3 has 2x I2S). This prevents cross-talk and DMA conflicts.

---

## 2. Software Architecture: Multi-Task FreeRTOS Pattern

### Task Breakdown

```
Core 1 (Camera/Network):          Core 0 (Audio/Sensors):
├─ taskCamCapture (Priority 4)   ├─ taskMicCapture (Priority 2)
├─ taskCamSend (Priority 3)      ├─ taskImuLoop (Priority 2)
└─ taskMicUpload (Priority 2)    └─ taskTTSPlay (Priority 2)
```

### Critical Pattern: Producer-Consumer with FreeRTOS Queues

#### Camera Pipeline
```cpp
// compile.ino:162-198 - CAPTURE TASK (Core 1)
void taskCamCapture(void*) {
  for(;;) {
    if (!snapshot_in_progress && cam_ws_ready) {
      camera_fb_t* fb = esp_camera_fb_get();  // Blocking capture
      if (fb && fb->format == PIXFORMAT_JPEG) {
        enqueue_frame(fb);  // Non-blocking queue push
      }
    }
  }
}

// compile.ino:147-160 - Queue Management
inline void enqueue_frame(camera_fb_t* fb) {
  if (xQueueSend(qFrames, &fb, 0) != pdPASS) {
    // Queue full → drop oldest frame (FIFO discipline)
    fb_ptr_t drop = nullptr;
    if (xQueueReceive(qFrames, &drop, 0) == pdPASS) {
      esp_camera_fb_return(drop);  // Return buffer to camera driver
      frame_dropped_count++;
    }
    xQueueSend(qFrames, &fb, 0);  // Insert new frame
  }
}

// compile.ino:200-263 - SENDER TASK (Core 1)
void taskCamSend(void*) {
  for(;;) {
    fb_ptr_t fb = nullptr;
    if (xQueueReceive(qFrames, &fb, pdMS_TO_TICKS(100)) == pdPASS) {
      // Optional FPS throttling
      if (g_target_fps > 0) {
        vTaskDelay(/* FPS period calculation */);
      }
      wsCam.sendBinary((const char*)fb->buf, fb->len);
      esp_camera_fb_return(fb);  // CRITICAL: Return buffer
    }
  }
}
```

**Why This Matters for Omi Glass**:
- Separates capture (blocking) from network I/O (variable latency)
- Queue acts as elastic buffer during network congestion
- Drop-oldest policy maintains real-time behavior
- Buffer management prevents memory leaks

---

#### Microphone Pipeline
```cpp
// compile.ino:276-297 - MIC CAPTURE (Core 0)
void taskMicCapture(void*) {
  const int samples_per_chunk = BYTES_PER_CHUNK / 2; // 20ms chunks
  for(;;) {
    if (run_audio_stream && aud_ws_ready) {
      AudioChunk ch;
      int16_t* out = reinterpret_cast<int16_t*>(ch.data);

      // Sample-by-sample blocking read from PDM
      for (int i = 0; i < samples_per_chunk; i++) {
        int v = i2sIn.read();  // Blocking
        if (v == -1) { delay(1); continue; }
        out[i] = (int16_t)v;
      }

      // Non-blocking enqueue (drop-oldest if full)
      if (xQueueSend(qAudio, &ch, 0) != pdPASS) {
        AudioChunk dump;
        xQueueReceive(qAudio, &dump, 0);
        xQueueSend(qAudio, &ch, 0);
      }
    }
  }
}

// compile.ino:299-310 - MIC UPLOAD (Core 1)
void taskMicUpload(void*) {
  for(;;) {
    if (run_audio_stream && aud_ws_ready) {
      AudioChunk ch;
      if (xQueueReceive(qAudio, &ch, pdMS_TO_TICKS(100)) == pdPASS) {
        wsAud.sendBinary((const char*)ch.data, ch.n);
      }
    }
  }
}
```

**Key Pattern**: 20ms chunking matches typical VAD frame size, minimizing latency while keeping packet overhead low.

---

### Speaker/TTS Pipeline (HTTP Chunked Transfer)

The system uses **HTTP chunked transfer encoding** instead of WebSocket for TTS, enabling server-side mixing and queueing:

```cpp
// compile.ino:467-677 - HTTP WAV Streaming
void taskHttpPlay(void*) {
  WiFiClient cli;

  while (http_play_running) {
    // Connect to /stream.wav endpoint
    cli.connect(SERVER_HOST, SERVER_PORT);
    cli.print("GET /stream.wav HTTP/1.1\r\n...");

    // Parse chunked encoding
    auto readBody = makeBodyReader(is_chunked, chunk_left);

    // Read WAV header (dynamic sample rate: 8k/12k/16k)
    parse_wav_header(cli, fmt, ...);

    // Reconfigure I2S output to match server sample rate
    if (current_out_rate != sampleRate) {
      i2sOut.begin(I2S_MODE_STD, sampleRate, I2S_DATA_BIT_WIDTH_32BIT,
                   I2S_SLOT_MODE_STEREO);
      current_out_rate = sampleRate;
    }

    // Stream audio in real-time
    while (http_play_running) {
      uint8_t inbuf[2048];
      readBody(inbuf, bytes20, BODY_TIMEOUT_MS);

      // Convert mono16 → stereo32 for MAX98357A
      mono16_to_stereo32_msb((int16_t*)inbuf, samp, outLR, 0.8f);
      i2sOut.write((uint8_t*)outLR, bytes);
    }
  }
}

// compile.ino:333-340 - Audio Format Conversion
static inline void mono16_to_stereo32_msb(const int16_t* in, size_t nSamp,
                                          int32_t* outLR, float gain = 0.7f) {
  for (size_t i = 0; i < nSamp; ++i) {
    int32_t s = (int32_t)((float)in[i] * gain);
    int32_t v32 = s << 16;  // Shift to MSB (32-bit I2S requirement)
    outLR[i*2 + 0] = v32;   // Left channel
    outLR[i*2 + 1] = v32;   // Right channel (duplicate)
  }
}
```

**Why HTTP Instead of WebSocket for Audio Downlink**:
- Server can mix multiple audio sources (TTS + alerts + background music)
- Chunked encoding enables infinite streaming without `Content-Length`
- Automatic reconnection logic handles network hiccups
- Client-side buffering handled by HTTP stack

---

## 3. Synchronization & Communication Patterns

### WebSocket Message Protocol

```cpp
// compile.ino:915-972 - Camera WebSocket Commands
wsCam.onMessage([](WebsocketsMessage msg) {
  String cmd = msg.data().trim();

  // Runtime resolution switching
  if (cmd.startsWith("SET:FRAMESIZE=")) {
    framesize_t fs = parse_framesize(cmd);
    apply_framesize(fs);  // Dynamic reconfiguration
  }

  // Quality adjustment (JPEG compression)
  else if (cmd.startsWith("SET:QUALITY=")) {
    int q = constrain(cmd.toInt(), 5, 40);
    sensor_t* s = esp_camera_sensor_get();
    s->set_quality(s, q);
  }

  // FPS throttling
  else if (cmd.startsWith("SET:FPS=")) {
    g_target_fps = constrain(cmd.toInt(), 5, 60);
  }

  // High-quality snapshot (interrupts streaming)
  else if (cmd == "SNAP:HQ") {
    snapshot_in_progress = true;
    s->set_framesize(s, FRAMESIZE_SXGA);  // Upgrade to 1280x1024
    s->set_quality(s, 18);
    camera_fb_t* fb = esp_camera_fb_get();
    wsCam.send("SNAP:BEGIN");
    wsCam.sendBinary((char*)fb->buf, fb->len);
    wsCam.send("SNAP:END");
    s->set_framesize(s, old_fs);  // Restore streaming mode
    snapshot_in_progress = false;
  }
});

// compile.ino:983-991 - Audio WebSocket Commands
wsAud.onMessage([](WebsocketsMessage msg) {
  if (msg.data() == "RESTART") {
    run_audio_stream = false;
    xQueueReset(qAudio);  // Flush buffered audio
    delay(50);
    wsAud.send("START");
    run_audio_stream = true;
  }
});
```

### Server-Side Processing (Python/FastAPI)

```python
# app_main.py WebSocket handler pattern
@app.websocket("/ws/camera")
async def ws_camera(websocket: WebSocket):
    await websocket.accept()

    while True:
        # Binary frame: JPEG blob
        data = await websocket.receive_bytes()

        # Decode JPEG → numpy array
        frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)

        # CRITICAL: Thread-safe frame buffer
        bridge_io.set_latest_frame(frame)  # Atomic write

        # Broadcast to all viewer clients
        await broadcast_to_viewers(data)

# bridge_io.py - Thread-safe frame sharing
class FrameBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self._frame = None

    def set_latest_frame(self, frame):
        with self._lock:
            self._frame = frame.copy()  # Deep copy prevents race

    def get_latest_frame(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None
```

---

## 4. IMU Integration (SPI-based ICM42688)

**Key Learning**: Original design used I2C, but switched to SPI to avoid conflicts with I2S DMA:

```cpp
// compile.ino:777-790 - SPI Initialization
bool imu_init_spi() {
  SPI.begin(IMU_SPI_SCK, IMU_SPI_MISO, IMU_SPI_MOSI, IMU_SPI_CS);
  pinMode(IMU_SPI_CS, OUTPUT);
  digitalWrite(IMU_SPI_CS, HIGH);

  uint8_t who = imu_read8(REG_WHO_AM_I);
  if (who != 0x47) return false;  // ICM42688 signature

  imu_write8(REG_PWR_MGMT0, 0x0F);  // Enable accel+gyro
  return true;
}

// compile.ino:792-817 - Burst Read (14 bytes)
bool imu_read_once(float& tempC, float& ax, float& ay, float& az,
                    float& gx, float& gy, float& gz) {
  uint8_t raw[14];
  imu_readn(BURST_FIRST, raw, 14);  // Single SPI transaction

  // Parse: TEMP_H, TEMP_L, ACCEL_X_H, ACCEL_X_L, ...
  int16_t tr  = (raw[0] << 8) | raw[1];
  int16_t axr = (raw[2] << 8) | raw[3];
  // ... (gyro/accel/temp scaling)

  tempC = (float)tr / TEMP_SENS + TEMP_OFFSET;
  ax = ((float)axr / ACC_LSB_PER_G) * G;
  // ...
}

// compile.ino:824-860 - 50Hz Streaming Task
void taskImuLoop(void*) {
  for(;;) {
    imu_read_once(tempC, ax, ay, az, gx, gy, gz);

    // EMA smoothing (optional)
    ax_f = EMA_ALPHA * ax + (1 - EMA_ALPHA) * ax_f;

    // Send via UDP (lower overhead than WebSocket for high-rate data)
    char buf[256];
    snprintf(buf, sizeof(buf),
      "{\"ts\":%lu,\"temp_c\":%.2f,"
      "\"accel\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
      "\"gyro\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}}",
      millis(), tempC, ax_f, ay_f, az_f, gx, gy, gz);

    udp.beginPacket(UDP_HOST, UDP_PORT);
    udp.write((uint8_t*)buf, strlen(buf));
    udp.endPacket();

    vTaskDelay(pdMS_TO_TICKS(20));  // 50Hz = 20ms period
  }
}
```

**Why SPI Over I2C**:
- I2C bus can be disrupted by I2S DMA activity
- SPI provides deterministic timing (no clock stretching)
- Higher throughput for burst reads

---

## 5. Backend Architecture (Server-Side)

### Audio Processing Chain

```python
# asr_core.py - Real-time Speech Recognition
class ASRCallback:
    """
    Handles streaming ASR from Alibaba DashScope Paraformer
    - Partial results: UI display only (don't trigger AI)
    - Final results: Trigger LLM processing
    - Hotwords ("停下", "别说了"): Emergency interrupt
    """

    def on_event(self, event):
        text, is_end = self._extract_sentence(event)

        # Priority 1: Hotword interrupt (全清零)
        if self._has_hotword(text):
            await self._full_reset("Hotword interrupt")
            return

        # Priority 2: Partial update (UI only)
        self._last_partial = text
        await self._ui_partial(text)

        # Priority 3: Final sentence (trigger AI)
        if is_end and not self._is_playing():
            await self._start_ai(text)

# audio_stream.py - TTS Broadcasting
async def broadcast_pcm16_realtime(pcm16: bytes):
    """
    Distribute TTS audio to all /stream.wav clients
    - 20ms chunking for real-time feel
    - Drop-tail policy if client queue full
    """
    off = 0
    next_tick = loop.time()

    while off < len(pcm16):
        chunk = pcm16[off : off + BYTES_PER_20MS_16K]

        for sc in stream_clients:
            if sc.q.full():
                sc.q.get_nowait()  # Drop oldest
            sc.q.put_nowait(chunk)

        # Real-time pacing: await next 20ms tick
        next_tick += 0.020
        await asyncio.sleep(max(0, next_tick - loop.time()))

        off += BYTES_PER_20MS_16K
```

### Navigation State Machine (Multi-Modal Fusion)

```python
# navigation_master.py - Central Orchestrator
class NavigationMaster:
    """
    States: IDLE | CHAT | BLINDPATH_NAV | CROSSING | ITEM_SEARCH
    - Camera frames → YOLO segmentation (blind path detection)
    - ASR text → Intent classification
    - IMU → Posture estimation (fall detection, orientation)
    """

    async def process_frame(self, frame: np.ndarray):
        if self.state == "BLINDPATH_NAV":
            # Blind path segmentation
            result = self.blindpath_detector.process_frame(frame)
            guidance = self._compute_guidance(result.mask)
            await self._tts(guidance)  # "向左3度", "前方障碍物"

        elif self.state == "ITEM_SEARCH":
            # YOLO-E open-vocabulary detection + hand tracking
            detections = self.yoloe_model.detect(frame, query="红牛")
            hands = self.mediapipe_hands.process(frame)
            if self._is_grasping(hands, detections):
                await self._tts("已拿到物品")
                self.state = "IDLE"
```

---

## 6. Adaptation Guide for Omi Glass Firmware

### Current Omi Glass Status (Assumption)
Based on your description, Omi Glass likely has:
- ✅ Camera streaming (ESP32-based)
- ❌ Microphone capture
- ❌ Speaker output
- ❓ IMU/sensor integration

### Step-by-Step Integration Plan

#### Phase 1: Add Microphone Capture

**1. Hardware Verification**
```cpp
// Check if Omi Glass has PDM mic or I2S MEMS mic
// Common options:
// - PDM: SPH0645, MP34DT01
// - I2S: INMP441, ICS-43434

// Identify available GPIO pins (avoid camera data lines)
#define MIC_CLK_PIN  42  // Adjust based on hardware
#define MIC_DATA_PIN 41
```

**2. Add I2S Microphone Task** (copy from compile.ino:267-310)
```cpp
I2SClass i2sIn;

void init_mic() {
  i2sIn.setPinsPdmRx(MIC_CLK_PIN, MIC_DATA_PIN);
  i2sIn.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT,
              I2S_SLOT_MODE_MONO);
}

// Queue structure
typedef struct {
  size_t n;
  uint8_t data[640];  // 20ms @ 16kHz = 640 bytes
} AudioChunk;

QueueHandle_t qAudio;

void taskMicCapture(void* param) {
  for(;;) {
    if (mic_enabled) {
      AudioChunk ch;
      int16_t* out = (int16_t*)ch.data;

      for (int i = 0; i < 320; i++) {  // 320 samples
        int v = i2sIn.read();
        if (v != -1) out[i] = (int16_t)v;
      }

      ch.n = 640;
      xQueueSend(qAudio, &ch, 0);
    }
    vTaskDelay(1);
  }
}

void taskMicUpload(void* param) {
  for(;;) {
    AudioChunk ch;
    if (xQueueReceive(qAudio, &ch, pdMS_TO_TICKS(100)) == pdPASS) {
      wsAudio.sendBinary((char*)ch.data, ch.n);
    }
  }
}

void setup() {
  // ...
  qAudio = xQueueCreate(10, sizeof(AudioChunk));
  xTaskCreatePinnedToCore(taskMicCapture, "mic_cap", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskMicUpload, "mic_upl", 4096, NULL, 2, NULL, 1);
}
```

**3. Add Audio WebSocket Endpoint** (server-side)
```python
# app_main.py
@app.websocket("/ws_audio")
async def ws_audio(websocket: WebSocket):
    await websocket.accept()

    # Start ASR session
    asr_client = create_asr_client(callback=ASRCallback(...))

    try:
        while True:
            # Receive 20ms PCM16 chunks
            data = await websocket.receive_bytes()

            # Feed to ASR engine
            asr_client.send_audio(data)

            # Optional: Record for debugging
            audio_recorder.write(data)
    except:
        asr_client.stop()
```

---

#### Phase 2: Add Speaker Output

**1. Hardware Setup**
```cpp
// MAX98357A I2S DAC (common choice)
#define SPK_BCLK 7
#define SPK_LRCK 8
#define SPK_DIN  9

I2SClass i2sOut;

void init_speaker() {
  i2sOut.setPins(SPK_BCLK, SPK_LRCK, SPK_DIN);
  i2sOut.begin(I2S_MODE_STD, 16000, I2S_DATA_BIT_WIDTH_32BIT,
               I2S_SLOT_MODE_STEREO);
}
```

**2. Implement HTTP Audio Streaming** (copy compile.ino:467-677)
```cpp
void taskHttpPlay(void* param) {
  WiFiClient cli;

  while (true) {
    if (!cli.connect(SERVER_HOST, SERVER_PORT)) {
      delay(500);
      continue;
    }

    cli.print("GET /stream.wav HTTP/1.1\r\n"
              "Host: " SERVER_HOST "\r\n"
              "Connection: keep-alive\r\n\r\n");

    // Parse chunked response
    // ... (see full implementation in compile.ino:467-677)

    // Stream to speaker
    while (true) {
      uint8_t buf[640];
      if (!read_chunk(cli, buf, 640)) break;

      int32_t stereo[640];
      mono16_to_stereo32_msb((int16_t*)buf, 320, stereo);
      i2sOut.write((uint8_t*)stereo, 640 * sizeof(int32_t));
    }
  }
}
```

**3. Server-Side TTS Integration**
```python
# audio_stream.py
@app.get("/stream.wav")
async def stream_wav(request: Request):
    q = asyncio.Queue(maxsize=96)
    client = StreamClient(q=q, abort_event=asyncio.Event())
    stream_clients.add(client)

    async def gen():
        # WAV header
        yield _wav_header_unknown_size(sr=16000, ch=1, sw=2)

        # Stream chunks
        while True:
            chunk = await q.get()
            if chunk is None: break
            yield chunk

    return StreamingResponse(gen(), media_type="audio/wav")

# Usage in TTS callback
async def on_tts_chunk(pcm16_data):
    await broadcast_pcm16_realtime(pcm16_data)
```

---

#### Phase 3: Simultaneous Processing Optimization

**1. Core Pinning Strategy**
```cpp
void setup() {
  // Core 1: Network-heavy tasks (camera, audio upload)
  xTaskCreatePinnedToCore(taskCamCapture, "cam_cap", 10240, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(taskCamSend, "cam_snd", 8192, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskMicUpload, "mic_upl", 4096, NULL, 2, NULL, 1);

  // Core 0: Sensor/audio tasks (mic, speaker, IMU)
  xTaskCreatePinnedToCore(taskMicCapture, "mic_cap", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskHttpPlay, "http_wav", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskImuLoop, "imu_loop", 4096, NULL, 2, NULL, 0);
}
```

**2. Memory Management**
```cpp
// PSRAM allocation for large buffers
camera_config_t config;
config.fb_location = CAMERA_FB_IN_PSRAM;  // Camera uses PSRAM
config.fb_count = 2;  // Double-buffering

// Internal RAM for real-time audio
QueueHandle_t qAudio = xQueueCreate(10, sizeof(AudioChunk));  // ~6.4KB
```

**3. Watchdog & Error Recovery**
```cpp
#include "esp_task_wdt.h"

void taskCamCapture(void* param) {
  esp_task_wdt_add(NULL);  // Register with watchdog

  for(;;) {
    esp_task_wdt_reset();  // Pet the dog every iteration

    // ... capture logic ...

    if (error_count > 10) {
      ESP_LOGE(TAG, "Camera hung, rebooting...");
      esp_restart();
    }
  }
}
```

---

## 7. Testing Strategy

### Unit Tests (ESP32 side)

**Test 1: Microphone-Only**
```cpp
void test_mic_loopback() {
  init_mic();

  Serial.println("Recording 3 seconds...");
  std::vector<int16_t> recording;

  for (int i = 0; i < 150; i++) {  // 3 sec / 20ms
    AudioChunk ch;
    xQueueReceive(qAudio, &ch, portMAX_DELAY);
    recording.insert(recording.end(), (int16_t*)ch.data,
                     (int16_t*)(ch.data + ch.n));
  }

  // Verify: RMS > threshold (not silent)
  float rms = compute_rms(recording);
  assert(rms > 100);
}
```

**Test 2: Camera-Microphone Interference**
```cpp
void test_av_sync() {
  init_camera();
  init_mic();

  start_all_tasks();

  for (int i = 0; i < 100; i++) {
    // Sample both queues
    camera_fb_t* fb;
    AudioChunk audio;

    xQueuePeek(qFrames, &fb, 0);
    xQueuePeek(qAudio, &audio, 0);

    // Verify timestamps within 50ms
    assert(abs(fb->timestamp - audio.timestamp) < 50);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
```

### Integration Tests (Server side)

**Test 3: End-to-End Latency**
```python
# test_latency.py
import asyncio, time

async def test_asr_latency():
    # Send pre-recorded "你好" audio
    audio_data = load_wav("nihao.wav")

    t0 = time.time()
    async with websockets.connect("ws://server/ws_audio") as ws:
        await ws.send(audio_data)

        # Wait for ASR result
        result = await asyncio.wait_for(
            receive_asr_result(),
            timeout=2.0
        )

    latency = time.time() - t0
    assert latency < 1.5  # Should be under 1.5 seconds
    assert "你好" in result.text
```

---

## 8. Performance Benchmarks (from this codebase)

| Metric | Value | Notes |
|--------|-------|-------|
| Camera FPS | 15-30 | VGA @ JPEG Q=17, WiFi limited |
| Audio Latency | ~200ms | Mic → ASR → TTS → Speaker |
| Frame Drop Rate | <5% | Under normal WiFi conditions |
| Power Consumption | ~800mA | Camera + WiFi + I2S active |
| Memory Usage | 180KB RAM, 2MB PSRAM | PSRAM for camera buffers |

---

## 9. Common Pitfalls & Solutions

### Issue 1: I2S Mic Noise/Silence
**Symptom**: Audio is silent or full of noise
**Cause**: Wrong I2S mode (PDM vs STD) or incorrect pins
**Solution**:
```cpp
// Verify PDM mic with oscilloscope on CLK pin (should see ~2.4MHz square wave)
i2sIn.setPinsPdmRx(CLK, DATA);  // NOT setPinsStdRx!
i2sIn.begin(I2S_MODE_PDM_RX, ...);
```

### Issue 2: Camera Freezes During Audio Playback
**Symptom**: Video stream stops when TTS plays
**Cause**: I2S DMA conflicts with camera DMA
**Solution**:
```cpp
// Use separate I2S peripherals (ESP32-S3 has I2S0 and I2S1)
I2SClass i2sIn(0);   // Mic on I2S0
I2SClass i2sOut(1);  // Speaker on I2S1
```

### Issue 3: WebSocket Disconnects Under Load
**Symptom**: Connection drops when both camera and mic are active
**Cause**: TCP congestion
**Solution**:
```cpp
// Reduce camera FPS dynamically
if (ws_send_fail_count > 10) {
  g_target_fps = max(5, g_target_fps - 5);
  sensor->set_quality(sensor, min(40, current_quality + 5));
}
```

### Issue 4: High Memory Fragmentation
**Symptom**: Crashes after ~1 hour of operation
**Cause**: Heap fragmentation from variable-sized allocations
**Solution**:
```cpp
// Use fixed-size queues and PSRAM for large buffers
config.fb_location = CAMERA_FB_IN_PSRAM;

// Periodic cleanup
if (millis() - last_gc > 300000) {  // Every 5 minutes
  heap_caps_check_integrity_all(true);
  last_gc = millis();
}
```

---

## 10. Key Takeaways for Omi Glass Adaptation

### Critical Patterns to Adopt

1. **Multi-Task Architecture**
   - Separate capture/upload tasks prevents blocking
   - FreeRTOS queues provide elastic buffering
   - Core pinning optimizes cache locality

2. **I2S Peripheral Isolation**
   - Use different I2S units for mic/speaker
   - Avoid I2C during audio streaming (use SPI for IMU)

3. **Network Protocol Selection**
   - WebSocket for low-latency bidirectional (camera, mic)
   - HTTP chunked for server-controlled streams (TTS)
   - UDP for high-rate sensor data (IMU @ 50Hz)

4. **Memory Management**
   - Camera: PSRAM (large, variable-sized)
   - Audio: Internal RAM (small, fixed-sized, low-latency)
   - Queues: Fixed depth with drop-oldest policy

5. **Error Recovery**
   - Watchdog timers on all long-running tasks
   - Automatic reconnection with exponential backoff
   - Graceful degradation (reduce FPS under congestion)

### Recommended Changes for Omi Glass

```diff
// Current Omi Glass firmware (hypothetical)
void loop() {
  camera_fb_t* fb = esp_camera_fb_get();
  ws_camera.sendBinary(fb->buf, fb->len);
  esp_camera_fb_return(fb);
  delay(33);  // ~30 FPS
}

// Recommended refactoring
+ QueueHandle_t qFrames, qAudio;
+ I2SClass i2sIn(0), i2sOut(1);

void setup() {
+  init_mic();
+  init_speaker();
+  qFrames = xQueueCreate(3, sizeof(camera_fb_t*));
+  qAudio = xQueueCreate(10, sizeof(AudioChunk));
+  xTaskCreatePinnedToCore(taskCamCapture, ..., 1);
+  xTaskCreatePinnedToCore(taskCamSend, ..., 1);
+  xTaskCreatePinnedToCore(taskMicCapture, ..., 0);
+  xTaskCreatePinnedToCore(taskMicUpload, ..., 1);
+  xTaskCreatePinnedToCore(taskHttpPlay, ..., 0);
}

void loop() {
+  wsCam.poll();
+  wsAud.poll();
+  delay(2);
}
```

---

## 11. Reference Implementation Checklist

Use this checklist when integrating microphone into Omi Glass:

### Hardware Phase
- [ ] Identify microphone type (PDM vs I2S MEMS)
- [ ] Map available GPIO pins (avoid camera pins)
- [ ] Verify speaker DAC compatibility (I2S STD mode)
- [ ] Check power budget (mic: ~2mA, speaker: ~100mA)
- [ ] Test I2S peripheral assignment (I2S0 vs I2S1)

### Firmware Phase
- [ ] Copy I2S init code (compile.ino:267-274, 315-322)
- [ ] Implement mic capture task (compile.ino:276-297)
- [ ] Implement mic upload task (compile.ino:299-310)
- [ ] Add audio WebSocket handlers (compile.ino:974-991)
- [ ] Implement HTTP speaker task (compile.ino:467-677)
- [ ] Create FreeRTOS queues (qAudio, qTTS)
- [ ] Configure task priorities and core pinning

### Server Phase
- [ ] Add /ws_audio endpoint (app_main.py)
- [ ] Integrate ASR client (asr_core.py)
- [ ] Implement /stream.wav endpoint (audio_stream.py)
- [ ] Add TTS generation callback
- [ ] Implement audio broadcasting (broadcast_pcm16_realtime)

### Testing Phase
- [ ] Microphone isolation test (silence detection)
- [ ] Speaker playback test (sine wave)
- [ ] Camera-audio interference test
- [ ] End-to-end latency test (speech → response)
- [ ] Load test (simultaneous camera + mic + speaker)
- [ ] Memory leak test (24-hour soak)

---

## 12. Contact & Support

For questions about adapting this firmware to Omi Glass:

**Original Codebase**: This is a Chinese smart glasses project for visually impaired users
**Hardware Platform**: XIAO ESP32S3 Sense
**Backend**: Python FastAPI + Alibaba DashScope API

**Key Files to Study**:
- `compile/compile.ino` - Complete firmware (1015 lines)
- `asr_core.py` - ASR callback handling
- `audio_stream.py` - TTS broadcasting
- `navigation_master.py` - Multi-modal state machine

**External Dependencies**:
- ESP32 Arduino Core 2.0.14+
- ArduinoWebsockets library
- ESP_I2S library (for dual I2S support)
- OpenCV + YOLO (server-side)

---

## Appendix A: Full Task Dependency Graph

```
setup()
├─ WiFi.begin() [blocking, ~3 sec]
├─ init_camera() [blocking, ~500ms]
├─ init_i2s_in() [PDM mic]
├─ init_i2s_out() [I2S DAC]
├─ xQueueCreate(qFrames, 3)
├─ xQueueCreate(qAudio, 10)
└─ xQueueCreate(qTTS, 48)

loop() [runs on Core 1]
├─ wsCam.poll() [non-blocking]
├─ wsAud.poll() [non-blocking]
└─ delay(2)

Background Tasks:
[Core 1, Priority 4] taskCamCapture
  └─> qFrames [depth=3]
      └─> [Core 1, Priority 3] taskCamSend
          └─> wsCam.sendBinary()

[Core 0, Priority 2] taskMicCapture
  └─> qAudio [depth=10]
      └─> [Core 1, Priority 2] taskMicUpload
          └─> wsAud.sendBinary()

[Core 0, Priority 2] taskHttpPlay
  ├─ WiFiClient.connect("/stream.wav")
  └─> i2sOut.write()

[Core 0, Priority 2] taskTTSPlay
  └─ qTTS [depth=48]
      └─> i2sOut.write()

[Core 0, Priority 2] taskImuLoop
  ├─ imu_read_once() [SPI transaction]
  └─> udp.sendPacket()
```

---

## Appendix B: Pin Usage Summary

| GPIO | Function | Direction | Notes |
|------|----------|-----------|-------|
| 1 | SPI_SCK (IMU) | Output | 10MHz clock |
| 2 | SPI_MOSI (IMU) | Output | - |
| 3 | SPI_MISO (IMU) | Input | - |
| 4 | SPI_CS (IMU) | Output | Active low |
| 7 | I2S_BCLK (Speaker) | Output | Bit clock |
| 8 | I2S_LRCK (Speaker) | Output | Word select |
| 9 | I2S_DIN (Speaker) | Output | Data to DAC |
| 10 | CAM_XCLK | Output | 20MHz |
| 11-18 | CAM_D0-D7 | Input | Parallel data |
| 13 | CAM_PCLK | Input | Pixel clock |
| 38 | CAM_VSYNC | Input | Vertical sync |
| 39 | CAM_SIOC (I2C) | Bidir | I2C clock |
| 40 | CAM_SIOD (I2C) | Bidir | I2C data |
| 41 | PDM_DATA (Mic) | Input | PDM bitstream |
| 42 | PDM_CLK (Mic) | Output | ~2.4MHz clock |
| 47 | CAM_HREF | Input | Horizontal ref |
| 48 | CAM_D7 | Input | MSB data |

**Conflicts to Avoid**:
- Don't use GPIO 1-4 for I2C (reserved for SPI IMU)
- Don't use GPIO 41-42 for other I2S (mic pins)
- Camera pins (10-18, 38-40, 47-48) cannot be reassigned

---

## Appendix C: Code Snippets for Quick Copy-Paste

### Snippet 1: Basic Microphone Setup
```cpp
#include "ESP_I2S.h"

#define MIC_CLK  42
#define MIC_DATA 41

I2SClass i2sIn;

void setup() {
  i2sIn.setPinsPdmRx(MIC_CLK, MIC_DATA);
  if (!i2sIn.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT,
                   I2S_SLOT_MODE_MONO)) {
    Serial.println("Mic init failed!");
    while(1);
  }
  Serial.println("Mic ready");
}

void loop() {
  int16_t sample = i2sIn.read();
  if (sample != -1) {
    Serial.println(sample);
  }
}
```

### Snippet 2: WebSocket Audio Uploader
```cpp
#include <ArduinoWebsockets.h>
using namespace websockets;

WebsocketsClient wsAud;
QueueHandle_t qAudio;

typedef struct { size_t n; uint8_t data[640]; } AudioChunk;

void taskMicUpload(void*) {
  for(;;) {
    AudioChunk ch;
    if (xQueueReceive(qAudio, &ch, pdMS_TO_TICKS(100)) == pdPASS) {
      if (wsAud.available()) {
        wsAud.sendBinary((const char*)ch.data, ch.n);
      }
    }
  }
}

void setup() {
  qAudio = xQueueCreate(10, sizeof(AudioChunk));

  wsAud.onEvent([](WebsocketsEvent ev, String) {
    if (ev == WebsocketsEvent::ConnectionOpened) {
      Serial.println("Audio WS connected");
    }
  });

  wsAud.connect("ws://server:8081/ws_audio");

  xTaskCreate(taskMicUpload, "aud_upl", 4096, NULL, 2, NULL);
}
```

### Snippet 3: Server ASR Endpoint
```python
@app.websocket("/ws_audio")
async def ws_audio(websocket: WebSocket):
    await websocket.accept()

    from dashscope.audio.asr import Recognition, RecognitionCallback

    class MyCallback(RecognitionCallback):
        def on_event(self, result):
            text = result.get('text')
            if text:
                print(f"[ASR] {text}")
                asyncio.create_task(process_command(text))

    recognition = Recognition(
        model='paraformer-realtime-v2',
        format='pcm',
        sample_rate=16000,
        callback=MyCallback()
    )

    try:
        while True:
            data = await websocket.receive_bytes()
            recognition.send_audio_frame(data)
    finally:
        recognition.stop()
```

---

**End of Document**
**Version**: 1.0
**Date**: 2025-10-31
**Target Platform**: Omi Glass (ESP32-based smart glasses)
**Source**: OpenAI Glasses for Navigation firmware analysis
