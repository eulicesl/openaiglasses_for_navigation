# Omi Glass Microphone Integration Guide

## 📋 Overview

This guide provides step-by-step instructions for adding microphone functionality to Omi Glass firmware (currently camera-only) based on the proven architecture from this project's ESP32-S3 implementation.

**Source Firmware**: `compile/compile.ino` (1014 lines)  
**Target Hardware**: Omi Glass (ESP32-based with camera, needs microphone)  
**Integration Goal**: Add PDM microphone + bidirectional audio streaming

---

## 🔍 Current Firmware Analysis

### Hardware Configuration (XIAO ESP32S3 Sense)

#### Camera Pins (Already in Omi Glass)
```cpp
// From camera_pins.h - XIAO ESP32S3 configuration
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40  // I2C SDA
#define SIOC_GPIO_NUM     39  // I2C SCL

// Parallel data pins
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13
```

**USED PINS**: 10-18, 38-40, 47-48 (18 pins for camera)

#### Audio Pins (TO ADD to Omi Glass)
```cpp
// Microphone (PDM Interface)
#define I2S_MIC_CLOCK_PIN 42  // PDM CLK
#define I2S_MIC_DATA_PIN  41  // PDM DATA

// Speaker (I2S Interface) - OPTIONAL for Omi Glass
#define I2S_SPK_BCLK 7   // Bit Clock
#define I2S_SPK_LRCK 8   // Left/Right Clock (WS)
#define I2S_SPK_DIN  9   // Data In
```

**NEW PINS NEEDED**: 41-42 (microphone only) or 7-9, 41-42 (with speaker)

#### IMU Pins (SPI - in current design)
```cpp
#define IMU_SPI_SCK   1   // D0 - Clock
#define IMU_SPI_MOSI  2   // D1 - Master Out
#define IMU_SPI_MISO  3   // D2 - Master In  
#define IMU_SPI_CS    4   // D3 - Chip Select
```

**USED PINS**: 1-4 (if IMU present in Omi Glass)

---

## 🔧 Pin Conflict Analysis for Omi Glass

### Available GPIO Pins on ESP32-S3

**All ESP32-S3 GPIO**: 0-48 (not all exposed on every board)

**Reserved/Unusable**:
- GPIO 0: Boot mode (strapping pin)
- GPIO 19-20: USB (D- / D+)
- GPIO 26-32: SPI Flash/PSRAM (internal)
- GPIO 33-37: SPI Flash (internal)

**Omi Glass Camera Uses**: 10-18, 38-40, 47-48

**Recommended for Microphone**:
- **Option 1** (BEST): GPIO 41-42 (PDM microphone)
- **Option 2**: GPIO 5-6 (if 41-42 unavailable)
- **Option 3**: GPIO 21-22 (check schematic first)

### ⚠️ Critical Conflicts to Avoid

1. **DO NOT use GPIO 26-37**: Internal Flash/PSRAM
2. **DO NOT use GPIO 10-18**: Camera data lines
3. **DO NOT reuse I2C pins (39-40)**: Camera control

### ✅ Safe Pin Assignments for Omi Glass

```cpp
// Microphone (No conflicts)
#define I2S_MIC_CLOCK_PIN 42
#define I2S_MIC_DATA_PIN  41

// Alternative if GPIO 41-42 unavailable
// #define I2S_MIC_CLOCK_PIN 6
// #define I2S_MIC_DATA_PIN  5

// Speaker (Optional - if Omi Glass needs audio output)
#define I2S_SPK_BCLK 7
#define I2S_SPK_LRCK 8
#define I2S_SPK_DIN  9
```

---

## 📦 Firmware Architecture Overview

### Task Structure (6 FreeRTOS Tasks)

```
Core 0 (Real-time Audio)        Core 1 (CPU-intensive Video)
├── taskMicCapture (P2)         ├── taskCamCapture (P4)
├── taskImuLoop (P2)             ├── taskCamSend (P3)
└── taskTTSPlay (P2)             └── taskMicUpload (P2)

P = Priority (4=highest)
```

**Key Design Pattern**: Capture tasks run at full speed with non-blocking queues; send tasks throttle based on network capacity.

### Memory Allocation

```cpp
// Queue depths (critical for stability)
#define AUDIO_QUEUE_DEPTH 10    // ~200ms audio buffer
#define TTS_QUEUE_DEPTH 48      // ~1 second TTS buffer
qFrames (3 slots)               // 3x JPEG frames in queue

// Stack sizes (in bytes)
taskCamCapture:  10240  // 10KB - JPEG encoding needs space
taskCamSend:      8192  // 8KB
taskMicCapture:   4096  // 4KB
taskMicUpload:    4096  // 4KB
taskImuLoop:      4096  // 4KB
taskTTSPlay:      4096  // 4KB
```

---

## 🚀 Step-by-Step Integration Guide

### Step 1: Add Hardware Definitions

**File Location**: Create or modify `audio_config.h` in Omi Glass firmware

```cpp
// audio_config.h - Add to Omi Glass firmware
#ifndef AUDIO_CONFIG_H
#define AUDIO_CONFIG_H

// ===== Microphone Configuration =====
#define I2S_MIC_CLOCK_PIN 42
#define I2S_MIC_DATA_PIN  41
#define SAMPLE_RATE       16000  // 16kHz for ASR
#define CHUNK_MS          20     // 20ms chunks (320 bytes)
#define BYTES_PER_CHUNK   (SAMPLE_RATE * CHUNK_MS / 1000 * 2)

// ===== Audio Queue =====
#define AUDIO_QUEUE_DEPTH 10

// ===== Speaker Configuration (Optional) =====
// Uncomment if Omi Glass has speaker output
// #define ENABLE_SPEAKER
#ifdef ENABLE_SPEAKER
  #define I2S_SPK_BCLK 7
  #define I2S_SPK_LRCK 8
  #define I2S_SPK_DIN  9
  #define TTS_RATE     16000
  #define TTS_QUEUE_DEPTH 48
#endif

#endif // AUDIO_CONFIG_H
```

### Step 2: Add Global Variables to Main Firmware

**Location**: After `#include` statements in Omi Glass main `.ino` file

```cpp
// ===== Add these includes =====
#include "ESP_I2S.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "audio_config.h"  // Your new file

// ===== Audio I2S Objects =====
I2SClass i2sIn;   // Microphone input

#ifdef ENABLE_SPEAKER
  I2SClass i2sOut;  // Speaker output
#endif

// ===== Audio Queue =====
typedef struct {
  size_t n;
  uint8_t data[BYTES_PER_CHUNK];
} AudioChunk;

QueueHandle_t qAudio = nullptr;
const int SILENCE_20MS = BYTES_PER_CHUNK;

#ifdef ENABLE_SPEAKER
  typedef struct { 
    uint16_t n; 
    uint8_t data[2048]; 
  } TTSChunk;
  
  QueueHandle_t qTTS = nullptr;
  volatile bool tts_playing = false;
#endif

// ===== WebSocket for Audio =====
WebsocketsClient wsAud;  // Separate from camera WebSocket
volatile bool aud_ws_ready = false;
volatile bool run_audio_stream = false;

// ===== Audio Server Configuration =====
// Update these to match your server
const char* AUD_WS_PATH = "/ws_audio";
```

### Step 3: Initialize Microphone Hardware

**Location**: Add to `setup()` function AFTER camera initialization

```cpp
void setup() {
  // ... existing camera initialization ...
  
  // ===== Initialize Microphone =====
  Serial.println("[AUDIO] Initializing microphone...");
  
  i2sIn.setPinsPdmRx(I2S_MIC_CLOCK_PIN, I2S_MIC_DATA_PIN);
  if (!i2sIn.begin(I2S_MODE_PDM_RX, SAMPLE_RATE, 
                   I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("[AUDIO] ERROR: Microphone init failed!");
    // Don't halt - camera can still work
  } else {
    Serial.println("[AUDIO] Microphone initialized @ 16kHz");
  }
  
#ifdef ENABLE_SPEAKER
  // ===== Initialize Speaker (Optional) =====
  Serial.println("[AUDIO] Initializing speaker...");
  i2sOut.setPins(I2S_SPK_BCLK, I2S_SPK_LRCK, I2S_SPK_DIN);
  if (!i2sOut.begin(I2S_MODE_STD, TTS_RATE, 
                    I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO)) {
    Serial.println("[AUDIO] ERROR: Speaker init failed!");
  } else {
    Serial.println("[AUDIO] Speaker initialized @ 16kHz");
  }
#endif

  // ===== Create Audio Queues =====
  qAudio = xQueueCreate(AUDIO_QUEUE_DEPTH, sizeof(AudioChunk));
  if (!qAudio) {
    Serial.println("[AUDIO] ERROR: Failed to create audio queue!");
  }
  
#ifdef ENABLE_SPEAKER
  qTTS = xQueueCreate(TTS_QUEUE_DEPTH, sizeof(TTSChunk));
#endif

  // ... rest of setup ...
}
```

### Step 4: Create Microphone Capture Task

**Location**: Add these functions BEFORE `setup()`

```cpp
// ===== Microphone Capture Task =====
void taskMicCapture(void* param) {
  const int samples_per_chunk = BYTES_PER_CHUNK / 2; // int16 samples
  Serial.println("[MIC] Capture task started");
  
  for (;;) {
    if (run_audio_stream && aud_ws_ready) {
      AudioChunk ch;
      ch.n = BYTES_PER_CHUNK;
      int16_t* out = reinterpret_cast<int16_t*>(ch.data);
      
      // Read exactly one 20ms chunk (320 samples)
      int i = 0;
      while (i < samples_per_chunk) {
        int v = i2sIn.read();
        if (v == -1) {
          delay(1);  // Wait for data
          continue;
        }
        out[i++] = (int16_t)v;
      }
      
      // Enqueue with drop-oldest policy on overflow
      if (xQueueSend(qAudio, &ch, 0) != pdPASS) {
        AudioChunk dump;
        xQueueReceive(qAudio, &dump, 0);  // Drop oldest
        xQueueSend(qAudio, &ch, 0);       // Add new
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(5));  // Sleep when not streaming
    }
  }
}

// ===== Microphone Upload Task =====
void taskMicUpload(void* param) {
  Serial.println("[MIC] Upload task started");
  
  for (;;) {
    if (run_audio_stream && aud_ws_ready) {
      AudioChunk ch;
      if (xQueueReceive(qAudio, &ch, pdMS_TO_TICKS(100)) == pdPASS) {
        // Send to server via WebSocket
        wsAud.sendBinary((const char*)ch.data, ch.n);
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}
```

### Step 5: Launch Audio Tasks in setup()

**Location**: Add to `setup()` AFTER queue creation, BEFORE existing camera tasks

```cpp
void setup() {
  // ... microphone & queue initialization above ...
  
  // ===== Launch FreeRTOS Tasks =====
  // IMPORTANT: Launch in this order for proper initialization
  
  // Audio tasks on Core 0 (real-time priority)
  xTaskCreatePinnedToCore(
    taskMicCapture,     // Function
    "mic_cap",          // Name
    4096,               // Stack size (bytes)
    NULL,               // Parameters
    2,                  // Priority (0-5, 5=highest)
    NULL,               // Task handle
    0                   // Core 0 (dedicated to audio)
  );
  
  xTaskCreatePinnedToCore(
    taskMicUpload, 
    "mic_upl", 
    4096, 
    NULL, 
    2, 
    NULL, 
    1  // Core 1 (network I/O)
  );
  
  // ... existing camera tasks ...
  // xTaskCreatePinnedToCore(taskCamCapture, ...);
  // xTaskCreatePinnedToCore(taskCamSend, ...);
  
  Serial.println("[SYSTEM] All tasks launched");
}
```

### Step 6: Add Audio WebSocket Management

**Location**: Modify your existing `loop()` function

```cpp
void loop() {
  // ===== Camera WebSocket (Existing) =====
  if (!wsCam.available()) {
    if (wsCam.connect(SERVER_HOST, SERVER_PORT, CAM_WS_PATH)) {
      Serial.println("[WS-CAM] Connected");
    } else {
      delay(1000);
    }
  }
  
  // ===== Audio WebSocket (NEW) =====
  if (!wsAud.available()) {
    if (wsAud.connect(SERVER_HOST, SERVER_PORT, AUD_WS_PATH)) {
      Serial.println("[WS-AUD] Connected");
      delay(50);
      
      // Start audio streaming
      run_audio_stream = true;
      wsAud.send("START");  // Tell server to start ASR
      
      Serial.println("[AUDIO] Streaming started");
    } else {
      Serial.println("[WS-AUD] Connection failed, retry in 2s");
      delay(2000);
    }
  }
  
  // ===== Poll Both WebSockets =====
  wsCam.poll();
  wsAud.poll();
  
  delay(2);  // Small delay to prevent watchdog
}
```

### Step 7: Add Audio WebSocket Event Handlers

**Location**: Add to `setup()` AFTER WebSocket initialization

```cpp
void setup() {
  // ... hardware init ...
  
  // ===== Camera WebSocket Events (Existing) =====
  // ... your existing wsCam.onEvent() ...
  
  // ===== Audio WebSocket Events (NEW) =====
  wsAud.onEvent([](WebsocketsEvent ev, String data) {
    if (ev == WebsocketsEvent::ConnectionOpened) {
      aud_ws_ready = true;
      Serial.println("[WS-AUD] Connection opened");
    }
    else if (ev == WebsocketsEvent::ConnectionClosed) {
      aud_ws_ready = false;
      run_audio_stream = false;
      Serial.println("[WS-AUD] Connection closed");
    }
  });
  
  wsAud.onMessage([](WebsocketsMessage msg) {
    if (msg.isText()) {
      String cmd = msg.data();
      cmd.trim();
      
      if (cmd == "RESTART") {
        // Server requests restart (e.g., after ASR error)
        run_audio_stream = false;
        xQueueReset(qAudio);  // Clear buffer
        delay(50);
        
        wsAud.send("START");
        run_audio_stream = true;
        Serial.println("[AUDIO] Restarted by server");
      }
      else if (cmd == "STOP") {
        run_audio_stream = false;
        Serial.println("[AUDIO] Stopped by server");
      }
    }
  });
  
  Serial.println("[WS] Event handlers registered");
}
```

---

## 🎛️ Optional: Speaker Output (TTS Playback)

If Omi Glass needs to play audio responses, add this task:

```cpp
#ifdef ENABLE_SPEAKER

// ===== Speaker Playback Task =====
void taskTTSPlay(void* param) {
  static int32_t stereo32Buf[1024 * 2];  // Stereo buffer
  Serial.println("[SPK] Playback task started");
  
  for (;;) {
    if (!tts_playing) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    
    TTSChunk ch;
    if (xQueueReceive(qTTS, &ch, pdMS_TO_TICKS(50)) == pdPASS) {
      size_t inSamp = ch.n / 2;  // Mono 16-bit samples
      int16_t* inPtr = (int16_t*)ch.data;
      
      // Convert mono 16-bit to stereo 32-bit MSB
      size_t outPairs = 0;
      for (size_t i = 0; i < inSamp; ++i) {
        int32_t s = (int32_t)inPtr[i];
        s = (s * 19660) / 32768;  // Gain reduction (60%)
        int32_t v32 = s << 16;    // Shift to MSB
        
        stereo32Buf[outPairs * 2 + 0] = v32;  // Left
        stereo32Buf[outPairs * 2 + 1] = v32;  // Right
        outPairs++;
        
        if (outPairs >= 1024) {
          // Flush buffer to I2S
          size_t bytes = outPairs * 2 * sizeof(int32_t);
          size_t off = 0;
          while (off < bytes) {
            size_t wrote = i2sOut.write((uint8_t*)stereo32Buf + off, bytes - off);
            if (wrote == 0) vTaskDelay(pdMS_TO_TICKS(1));
            else off += wrote;
          }
          outPairs = 0;
        }
      }
      
      // Flush remaining
      if (outPairs > 0) {
        size_t bytes = outPairs * 2 * sizeof(int32_t);
        size_t off = 0;
        while (off < bytes) {
          size_t wrote = i2sOut.write((uint8_t*)stereo32Buf + off, bytes - off);
          if (wrote == 0) vTaskDelay(pdMS_TO_TICKS(1));
          else off += wrote;
        }
      }
    }
  }
}

// Launch in setup()
xTaskCreatePinnedToCore(taskTTSPlay, "tts_play", 4096, NULL, 2, NULL, 0);

#endif // ENABLE_SPEAKER
```

---

## 📊 Testing & Validation

### Step 1: Serial Monitor Validation

After flashing, you should see:

```
[AUDIO] Initializing microphone...
[AUDIO] Microphone initialized @ 16kHz
[MIC] Capture task started
[MIC] Upload task started
[WS] Event handlers registered
[WS-AUD] Connected
[AUDIO] Streaming started
```

### Step 2: Check Audio Data Flow

Add debug output to capture task (temporary):

```cpp
void taskMicCapture(void* param) {
  unsigned long last_log = 0;
  unsigned long chunk_count = 0;
  
  for (;;) {
    // ... existing capture code ...
    chunk_count++;
    
    // Log every 5 seconds
    if (millis() - last_log > 5000) {
      Serial.printf("[MIC] Captured %lu chunks (%.1f sec audio)\n", 
                    chunk_count, chunk_count * 0.02);
      last_log = millis();
      chunk_count = 0;
    }
  }
}
```

Expected output:
```
[MIC] Captured 250 chunks (5.0 sec audio)
[MIC] Captured 250 chunks (5.0 sec audio)
```

### Step 3: Server-Side Validation

On your Python server (`app_main.py`), you should see:

```python
[AUDIO] client connected
[AUDIO] START received
[ASR PARTIAL] len=15 text='你好世界'
[ASR FINAL] len=15 text='你好世界'
```

---

## 🐛 Troubleshooting

### Problem 1: "Microphone init failed"

**Symptoms**: 
```
[AUDIO] ERROR: Microphone init failed!
```

**Solutions**:
1. **Check pin conflicts**: Verify GPIO 41-42 not used elsewhere
2. **Check hardware**: Ensure PDM mic properly connected (CLK, DATA, GND, VCC)
3. **Try alternate pins**: Use GPIO 5-6 instead:
   ```cpp
   #define I2S_MIC_CLOCK_PIN 6
   #define I2S_MIC_DATA_PIN  5
   ```
4. **Check I2S peripheral**: ESP32-S3 has 2 I2S peripherals; ensure not conflicting with other devices

### Problem 2: No Audio Data Captured

**Symptoms**:
```
[MIC] Captured 0 chunks (0.0 sec audio)
```

**Solutions**:
1. **Check `run_audio_stream` flag**: Should be `true` after WebSocket connect
2. **Verify WebSocket connection**:
   ```cpp
   Serial.printf("[DEBUG] aud_ws_ready=%d run_audio=%d\n", 
                 aud_ws_ready, run_audio_stream);
   ```
3. **Test I2S read**:
   ```cpp
   int v = i2sIn.read();
   if (v == -1) Serial.println("[DEBUG] I2S read returns -1");
   ```
4. **Check microphone power**: Ensure MEMS mic has 3.3V supply

### Problem 3: Audio Quality Issues (Crackling/Noise)

**Symptoms**: Server receives garbled audio

**Solutions**:
1. **Check sample rate mismatch**:
   - ESP32: `SAMPLE_RATE = 16000`
   - Server: `SAMPLE_RATE = 16000` (must match!)
   
2. **Reduce WiFi congestion**: Lower camera FPS
   ```cpp
   g_target_fps = 10;  // Reduce from 30 to 10 FPS
   ```

3. **Increase audio queue depth**:
   ```cpp
   #define AUDIO_QUEUE_DEPTH 20  // Was 10
   ```

4. **Check ground connections**: Poor ground = noise

### Problem 4: Microphone Works, Camera Stops

**Symptoms**: After adding audio, camera stops capturing

**Solutions**:
1. **Check PSRAM allocation**:
   ```cpp
   config.fb_count = 2;  // Must be 2+, not 1
   ```

2. **Increase camera task priority**:
   ```cpp
   xTaskCreatePinnedToCore(taskCamCapture, ..., 5, ...);  // Was 4
   ```

3. **Check WiFi power**: Insufficient power = brownouts
   ```cpp
   WiFi.setTxPower(WIFI_POWER_15dBm);  // Reduce from 19.5dBm
   ```

### Problem 5: WebSocket Disconnects Frequently

**Symptoms**:
```
[WS-AUD] Connection closed
[WS-AUD] Connection failed, retry in 2s
```

**Solutions**:
1. **Increase server timeout** (`app_main.py`):
   ```python
   ws = await ws.accept()
   ws.keepalive_ping_interval = 30  # Ping every 30s
   ```

2. **Add watchdog reset** in capture task:
   ```cpp
   void taskMicCapture(void* param) {
     for (;;) {
       // ... capture code ...
       vTaskDelay(1);  // Prevent watchdog trigger
     }
   }
   ```

3. **Check network stability**: Use Ethernet if available

---

## 📏 Performance Benchmarks

### Expected Performance (ESP32-S3 @ 240MHz)

| Metric | Value | Notes |
|--------|-------|-------|
| Camera FPS | 15-30 | VGA resolution, JPEG quality 17 |
| Audio Latency | ~60ms | 20ms chunk + 40ms network |
| CPU Load (Core 0) | 40-60% | Audio + IMU |
| CPU Load (Core 1) | 60-80% | Camera + network |
| WiFi Throughput | 2-5 Mbps | Combined audio + video |
| PSRAM Usage | 3-5 MB | Camera framebuffers |
| Heap Usage | 50-100 KB | Queues + tasks |

### Optimization Tips

1. **Reduce camera resolution** if audio drops:
   ```cpp
   g_frame_size = FRAMESIZE_HVGA;  // 480x320 instead of VGA
   ```

2. **Reduce camera quality** (smaller JPEG):
   ```cpp
   config.jpeg_quality = 25;  // Was 17 (lower=better, bigger)
   ```

3. **Disable speaker** if not needed:
   ```cpp
   // #define ENABLE_SPEAKER  // Comment out
   ```

4. **Use shorter audio chunks** for lower latency:
   ```cpp
   #define CHUNK_MS 10  // 10ms instead of 20ms
   ```

---

## 🔌 Hardware Recommendations

### Microphone Options

1. **PDM MEMS Microphone** (Recommended)
   - Model: SPM1423, ICS-43434, MP34DT01
   - Interface: PDM (simple 2-wire)
   - Pins: CLK + DATA
   - Cost: $1-3
   - **Pros**: High quality, low noise, simple interface
   
2. **I2S MEMS Microphone** (Alternative)
   - Model: INMP441, ICS-43432
   - Interface: I2S (4-wire)
   - Pins: SCK, WS, SD, GND
   - Cost: $2-5
   - **Pros**: Higher bitrate options, standard I2S

### Speaker/Amplifier Options (if needed)

1. **MAX98357A** (Class D Amplifier)
   - Interface: I2S
   - Pins: BCLK, LRCK, DIN
   - Output: 3W mono
   - Cost: $2-5
   - **Pros**: Tiny, efficient, no analog needed

2. **PAM8403** (Stereo Amplifier)
   - Interface: Analog (needs DAC)
   - Output: 3W stereo
   - Cost: $1-2
   - **Cons**: Requires I2S DAC chip

### Power Considerations

- **Microphone**: ~0.5-1 mA @ 3.3V
- **Speaker Amp**: 10-100 mA (idle-max)
- **Total Addition**: ~10-100 mA depending on volume
- **Recommendation**: Use 500+ mA power supply (1A recommended)

---

## 📚 Integration Checklist

### Pre-Integration

- [ ] Review Omi Glass schematic for pin conflicts
- [ ] Identify available GPIO pins (41-42 recommended)
- [ ] Purchase compatible PDM microphone (SPM1423 or similar)
- [ ] Test microphone with breakout board (if available)
- [ ] Backup existing Omi Glass firmware

### Code Integration

- [ ] Add `audio_config.h` with pin definitions
- [ ] Add I2S includes and global variables
- [ ] Initialize microphone hardware in `setup()`
- [ ] Create audio queues
- [ ] Implement `taskMicCapture()` function
- [ ] Implement `taskMicUpload()` function
- [ ] Launch audio tasks with proper core affinity
- [ ] Add audio WebSocket connection in `loop()`
- [ ] Add audio WebSocket event handlers

### Server-Side Preparation

- [ ] Verify server has `/ws_audio` endpoint (see `app_main.py` line 727)
- [ ] Test server ASR functionality with existing setup
- [ ] Configure DashScope API key in server `.env`
- [ ] Check server can handle 16kHz PCM audio

### Testing

- [ ] Flash modified firmware to Omi Glass
- [ ] Verify serial monitor shows audio initialization
- [ ] Check WebSocket connection to `/ws_audio`
- [ ] Test audio capture (debug output)
- [ ] Verify server receives audio data
- [ ] Test ASR recognition with simple phrases
- [ ] Validate camera still works after audio addition
- [ ] Test simultaneous camera + audio streaming

### Validation

- [ ] Check CPU load (should be <80% per core)
- [ ] Monitor heap usage (should not grow)
- [ ] Test for 30+ minutes (check stability)
- [ ] Verify audio quality (clear speech recognition)
- [ ] Test WiFi disconnection/reconnection
- [ ] Document any issues or modifications needed

---

## 🎓 Advanced Topics

### Adaptive Audio Quality

Adjust audio quality based on network conditions:

```cpp
// In taskMicUpload()
unsigned long last_send_time = 0;

if (xQueueReceive(qAudio, &ch, pdMS_TO_TICKS(100)) == pdPASS) {
  unsigned long now = millis();
  unsigned long send_duration = now - last_send_time;
  
  if (send_duration > 100) {
    // Network slow - reduce audio quality
    // (requires implementing compression)
    Serial.println("[AUDIO] Network congestion detected");
  }
  
  wsAud.sendBinary((const char*)ch.data, ch.n);
  last_send_time = now;
}
```

### Echo Cancellation

If Omi Glass has both microphone and speaker:

```cpp
// In taskMicCapture() - basic echo suppression
if (tts_playing) {
  // Reduce gain when speaker is active
  for (int i = 0; i < samples_per_chunk; i++) {
    out[i] = out[i] >> 2;  // Reduce by 75%
  }
}
```

(For production, use proper AEC algorithm)

### Dynamic Sample Rate

Support multiple sample rates:

```cpp
void setSampleRate(int rate) {
  i2sIn.end();
  delay(50);
  i2sIn.begin(I2S_MODE_PDM_RX, rate, 
              I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
  Serial.printf("[AUDIO] Sample rate changed to %d Hz\n", rate);
}

// In wsAud.onMessage()
if (cmd.startsWith("SET_RATE:")) {
  int rate = cmd.substring(9).toInt();
  setSampleRate(rate);
}
```

---

## 📞 Support & Resources

### Reference Code Locations

| Feature | File | Lines |
|---------|------|-------|
| Microphone Init | `compile.ino` | 267-273 |
| Mic Capture Task | `compile.ino` | 276-296 |
| Mic Upload Task | `compile.ino` | 299-310 |
| Audio Queue | `compile.ino` | 78-82 |
| WebSocket Setup | `compile.ino` | 974-992 |
| Server ASR Handler | `app_main.py` | 727-860 |

### External Documentation

- [ESP32-S3 Technical Reference](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf)
- [ESP-IDF I2S Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/i2s.html)
- [PDM Microphone Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/i2s.html#pdm-mode)
- [FreeRTOS Task Management](https://www.freertos.org/a00019.html)

### Community Support

- **Based Hardware Omi**: Check their GitHub for hardware specs
- **ESP32 Forum**: https://esp32.com/
- **This Project**: https://github.com/eulicesl/openaiglasses_for_navigation

---

## 📄 License

This integration guide is provided as-is for educational purposes. Test thoroughly before deployment in production environments.

**Last Updated**: 2025-01-XX  
**Firmware Version**: v2.4-SPIIMU  
**Compatible Hardware**: ESP32-S3, Omi Glass (ESP32-based)

