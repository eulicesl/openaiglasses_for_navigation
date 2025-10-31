# Omi Glass Microphone Integration - Quick Start

## 🎯 30-Minute Integration Guide

This is a condensed version for experienced developers. For detailed explanations, see `OMI_GLASS_MICROPHONE_INTEGRATION_GUIDE.md`.

---

## Step 1: Hardware Connections (5 minutes)

### PDM Microphone Wiring

```
PDM Mic (SPM1423)    →    Omi Glass ESP32
─────────────────────────────────────────
CLK (Clock)          →    GPIO 42
DATA (Data)          →    GPIO 41
VDD (Power)          →    3.3V
GND (Ground)         →    GND
```

**Pin Conflicts Check**: These pins must NOT be used by camera or other peripherals.

---

## Step 2: Code Addition (15 minutes)

### 2.1: Add Includes (Top of .ino file)

```cpp
#include "ESP_I2S.h"
#include "freertos/queue.h"

// Audio config
#define I2S_MIC_CLOCK_PIN 42
#define I2S_MIC_DATA_PIN  41
#define SAMPLE_RATE       16000
#define CHUNK_MS          20
#define BYTES_PER_CHUNK   640
#define AUDIO_QUEUE_DEPTH 10

const char* AUD_WS_PATH = "/ws_audio";
```

### 2.2: Add Global Variables (After includes)

```cpp
// I2S & WebSocket
I2SClass i2sIn;
WebsocketsClient wsAud;

// Queue
typedef struct { size_t n; uint8_t data[640]; } AudioChunk;
QueueHandle_t qAudio = nullptr;

// Flags
volatile bool aud_ws_ready = false;
volatile bool run_audio_stream = false;
```

### 2.3: Add to setup() - Microphone Init

```cpp
void setup() {
  // ... existing camera init ...
  
  // Microphone
  i2sIn.setPinsPdmRx(I2S_MIC_CLOCK_PIN, I2S_MIC_DATA_PIN);
  if (!i2sIn.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("[ERROR] Mic init failed");
  }
  
  // Queue
  qAudio = xQueueCreate(10, sizeof(AudioChunk));
  
  // Tasks (Core 0 = audio, Core 1 = network)
  xTaskCreatePinnedToCore(taskMicCapture, "mic_cap", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskMicUpload, "mic_upl", 4096, NULL, 2, NULL, 1);
  
  // WebSocket handlers
  wsAud.onEvent([](WebsocketsEvent ev, String){
    if (ev == WebsocketsEvent::ConnectionOpened) { 
      aud_ws_ready = true;
      Serial.println("[WS-AUD] Open");
    }
    if (ev == WebsocketsEvent::ConnectionClosed) { 
      aud_ws_ready = false; 
      run_audio_stream = false;
    }
  });
  
  wsAud.onMessage([](WebsocketsMessage msg){
    if (msg.isText() && msg.data().trim() == "RESTART") {
      run_audio_stream = false; 
      xQueueReset(qAudio); 
      delay(50);
      wsAud.send("START"); 
      run_audio_stream = true;
    }
  });
}
```

### 2.4: Add Tasks (Before setup())

```cpp
void taskMicCapture(void*) {
  const int samples = BYTES_PER_CHUNK / 2;
  for(;;) {
    if (run_audio_stream && aud_ws_ready) {
      AudioChunk ch; ch.n = BYTES_PER_CHUNK;
      int16_t* out = (int16_t*)ch.data;
      
      int i = 0;
      while (i < samples) {
        int v = i2sIn.read();
        if (v == -1) { delay(1); continue; }
        out[i++] = (int16_t)v;
      }
      
      if (xQueueSend(qAudio, &ch, 0) != pdPASS) {
        AudioChunk dump;
        xQueueReceive(qAudio, &dump, 0);
        xQueueSend(qAudio, &ch, 0);
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }
}

void taskMicUpload(void*) {
  for(;;) {
    if (run_audio_stream && aud_ws_ready) {
      AudioChunk ch;
      if (xQueueReceive(qAudio, &ch, pdMS_TO_TICKS(100)) == pdPASS) {
        wsAud.sendBinary((const char*)ch.data, ch.n);
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}
```

### 2.5: Modify loop() - Add Audio Connection

```cpp
void loop() {
  // Camera (existing)
  if (!wsCam.available()) {
    wsCam.connect(SERVER_HOST, SERVER_PORT, "/ws/camera");
  }
  
  // Audio (NEW)
  if (!wsAud.available()) {
    if (wsAud.connect(SERVER_HOST, SERVER_PORT, AUD_WS_PATH)) {
      delay(50);
      run_audio_stream = true;
      wsAud.send("START");
    } else {
      delay(2000);
    }
  }
  
  wsCam.poll();
  wsAud.poll();
  delay(2);
}
```

---

## Step 3: Server Configuration (5 minutes)

### 3.1: Verify Server Endpoint

Check `app_main.py` has this WebSocket:

```python
@app.websocket("/ws_audio")
async def ws_audio(ws: WebSocket):
    await ws.accept()
    # ... ASR handling ...
```

✅ **Already implemented in line 727-860**

### 3.2: Set API Key

Create `.env` file in server directory:

```bash
DASHSCOPE_API_KEY=sk-your-key-here
```

---

## Step 4: Testing (5 minutes)

### 4.1: Flash & Monitor

```bash
# Arduino IDE: Upload to board
# Serial Monitor: 115200 baud

# Expected output:
[AUDIO] Microphone initialized @ 16kHz
[MIC] Capture task started
[MIC] Upload task started
[WS-AUD] Connected
[AUDIO] Streaming started
```

### 4.2: Server Logs

```bash
python app_main.py

# Expected:
[AUDIO] client connected
[AUDIO] START received
[ASR PARTIAL] text='测试'
[ASR FINAL] text='测试'
```

---

## 🐛 Quick Troubleshooting

| Problem | Solution |
|---------|----------|
| "Mic init failed" | Check wiring: GPIO 41/42, 3.3V, GND |
| No audio data | Check `run_audio_stream` flag, verify WebSocket connected |
| Garbled audio | Verify sample rate: ESP32=16000, Server=16000 |
| Camera stops | Increase `config.fb_count = 2`, reduce `g_target_fps = 10` |
| Frequent disconnects | Lower camera FPS, check WiFi signal strength |

---

## 📊 Validation Commands

### Check Task Status

Add to `loop()` for debugging:

```cpp
if (millis() - last_debug > 5000) {
  Serial.printf("[DEBUG] Audio: ready=%d stream=%d queue=%d\n", 
                aud_ws_ready, run_audio_stream, 
                uxQueueMessagesWaiting(qAudio));
  last_debug = millis();
}
```

### Check Memory

```cpp
Serial.printf("[MEM] Free heap: %d bytes, Min: %d bytes\n",
              ESP.getFreeHeap(), ESP.getMinFreeHeap());
```

---

## 🎯 Success Criteria

- [x] Serial shows "Microphone initialized"
- [x] WebSocket connects to `/ws_audio`
- [x] Server shows "ASR PARTIAL/FINAL" messages
- [x] Camera continues working (15+ FPS)
- [x] Audio latency < 200ms
- [x] No memory leaks after 10+ minutes

---

## 📚 Next Steps

1. **Test audio quality**: Record 30 seconds, check for drops
2. **Stress test**: Run both camera + audio for 1 hour
3. **Optimize**: Adjust `AUDIO_QUEUE_DEPTH` if needed
4. **Add speaker** (optional): See main guide for TTS output

---

## 🔗 References

- **Full Guide**: `OMI_GLASS_MICROPHONE_INTEGRATION_GUIDE.md`
- **Source Firmware**: `compile/compile.ino` (lines 265-310)
- **Server Handler**: `app_main.py` (lines 727-860)
- **ESP32-S3 I2S Docs**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/i2s.html

---

**Estimated Time**: 30 minutes  
**Difficulty**: Intermediate  
**Required Skills**: Arduino, C++, WebSocket basics

**Questions?** Check main guide or open an issue on GitHub.
