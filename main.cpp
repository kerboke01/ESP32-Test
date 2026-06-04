#include "secret.h"
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "driver/i2s.h"
#include "HT_SSD1306Wire.h"

// =========================
// OLED
// =========================
static SSD1306Wire display(
  0x3c,
  500000,
  SDA_OLED,    // GPIO17
  SCL_OLED,    // GPIO18
  GEOMETRY_128_64,
  RST_OLED     // GPIO21
);

// =========================
// Heltec LoRa ESP32 V3 Pin Mapping (CORRECTED)
// =========================

// --- System Pins ---
#define BUTTON_PIN      0     // Boot button (input-only but works with PULLUP)
#define LED_PIN         35    // On-board LED (write only, active low)
#define VEXT_PIN        36    // External power control

// --- I2S Recording (INMP441 MEMS Mic) ---
// Using safe ADC-capable GPIOs (1,2,4,5,6,7 are recommended for external use)
#define I2S_MIC_WS      4    // Word Select (WS) for microphone
#define I2S_MIC_SCK     6    // Bit Clock (BCLK) for microphone
#define I2S_MIC_SD      5    // Data Output (DOUT) from microphone
#define I2S_MIC_PORT    I2S_NUM_0

// --- I2S Playback (MAX98357 Audio Amplifier) ---
// Using safe general-purpose GPIOs for audio output
#define I2S_SPK_LRC     1    // LRC/WS for speaker (Left-Right Clock)
#define I2S_SPK_BCLK    2    // BCLK for speaker (Bit Clock)
#define I2S_SPK_DIN     7    // DIN for speaker (Data Input)
#define I2S_SPK_PORT    I2S_NUM_1

// --- Audio Parameters ---
static const uint32_t SAMPLE_RATE = 16000;
static const uint8_t CHANNELS = 1;
static const uint8_t BITS_PER_SAMPLE = 16;
static const uint8_t RECORD_TIME_SECS = 3;
static const size_t WAV_HEADER_SIZE = 44;
static const size_t PCM_SIZE = SAMPLE_RATE * RECORD_TIME_SECS * 2;
static const size_t WAV_SIZE = WAV_HEADER_SIZE + PCM_SIZE;

// --- TTS Audio Buffer ---
static const size_t TTS_BUFFER_SIZE = 8192;
static uint8_t* ttsBuffer = nullptr;

// =========================
// Globals
// =========================
uint8_t* audioBuffer = nullptr;
bool lastButtonState = HIGH;

// =========================
// Pin Structure Display
// =========================
/*
╔═══════════════════════════════════════════════════════════════════════════════╗
║                    HELTEC LORA ESP32 V3 - PIN ASSIGNMENTS                     ║
╠═══════════════════════════════════════════════════════════════════════════════╣
║                                                                                ║
║  ┌─────────────────────────────────────────────────────────────────────────┐    ║
║  │                      ON-BOARD CONNECTIONS                              │    ║
║  ├───────────────────┬─────────────────────────────────────────────────────┤    ║
║  │ Function          │ Pin (GPIO)    Notes                                │    ║
║  ├───────────────────┼─────────────────────────────────────────────────────┤    ║
║  │ OLED SDA          │ 17       ✓    I2C Data                             │    ║
║  │ OLED SCL          │ 18       ✓    I2C Clock                            │    ║
║  │ OLED RST          │ 21       ✓    Display Reset                        │    ║
║  │ Button            │ 0        ✓    Boot/User Button (INPUT_PULLUP)      │    ║
║  │ On-board LED      │ 35       ✓    Write-only (Active Low)              │    ║
║  │ Vext Control      │ 36       ✓    External Power Enable (Active Low)   │    ║
║  └───────────────────┴─────────────────────────────────────────────────────┘    ║
║                                                                                ║
║  ┌─────────────────────────────────────────────────────────────────────────┐    ║
║  │               MAX98357 I2S AUDIO AMPLIFIER (OUTPUT)                   │    ║
║  ├───────────────────┬─────────────────────────────────────────────────────┤    ║
║  │ MAX98357 Pin      │ ESP32 GPIO     Description                           │    ║
║  ├───────────────────┼─────────────────────────────────────────────────────┤    ║
║  │ VIN               │ 3V3 (3.3V)     Power Supply       ok                  │    ║
║  │ GND               │ GND            Ground              ok                │    ║
║  │ LRC (WS)          │ 1        ✓     Word Select / Left-Right Clock   ok    │    ║
║  │ BCLK              │ 2        ✓     Bit Clock                    ok        │    ║
║  │ DIN               │ 7        ✓     Serial Data Input      ok             │    ║
║  │ GAIN              │ NC (9dB)       Default gain (or tie to VCC/GND) ok   │    ║
║  │ SD                │ NC             Stereo mix mode (or 100K to 3V3)    │    ║
║  │ SPK+              │ Speaker+       Positive speaker terminal            │    ║
║  │ SPK-              │ Speaker-       Negative speaker terminal            │    ║
║  └───────────────────┴─────────────────────────────────────────────────────┘    ║
║                                                                                ║
║  ┌─────────────────────────────────────────────────────────────────────────┐    ║
║  │                  INMP441 MEMS MICROPHONE (INPUT)                       │    ║
║  ├───────────────────┬─────────────────────────────────────────────────────┤    ║
║  │ INMP441 Pin       │ ESP32 GPIO     Description                           │    ║
║  ├───────────────────┼─────────────────────────────────────────────────────┤    ║
║  │ VDD               │ 3V3 (3.3V)     Power Supply       ok                  │    ║
║  │ GND               │ GND            Ground              ok                │    ║
║  │ SCK               │ 6        ✓     I2S Clock           ok                 │    ║
║  │ WS                │ 4        ✓     Word Select          ok                │    ║
║  │ SD (DOUT)         │ 5        ✓     Serial Data Output     ok              │    ║
║  │ L/R               │ GND            Left channel mode      ok               │    ║
║  └───────────────────┴─────────────────────────────────────────────────────┘    ║
║                                                                                ║
║  ┌─────────────────────────────────────────────────────────────────────────┐    ║
║  │                  WHY THESE PINS WERE CHOSEN                            │    ║
║  ├─────────────────────────────────────────────────────────────────────────┤    ║
║  │ ✓ GPIOs 1,2,4,5,6,7: Safe ADC-capable GPIOs (officially recommended)    │    ║
║  │ ✓ No Flash/SPI conflicts (GPIO33-38 avoided)                            │    ║
║  │ ✓ No JTAG conflicts (GPIO39-42 avoided)                                  │    ║
║  │ ✓ Separate I2S ports for recording (I2S0) and playback (I2S1)           │    ║
║  │ ✓ Better performance: dedicated ports = no port sharing overhead       │    ║
║  └─────────────────────────────────────────────────────────────────────────┘    ║
║                                                                                ║
╚═══════════════════════════════════════════════════════════════════════════════╝
*/

// =========================
// Helpers
// =========================
void VextON() {
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);
}

int getStringLinesCount(String text, int maxWidth) {
    if (text.length() == 0) return 0;

    int lineCount = 1;
    int currentLineWidth = 0;
    int currentWordWidth = 0;
    int spaceWidth = 4;

    for (unsigned int i = 0; i < text.length(); i++) {
        char c = text.charAt(i);
        int charWidth = 6;

        if (c == '\n') {
            lineCount++;
            currentLineWidth = 0;
            currentWordWidth = 0;
            continue;
        }

        if (c == ' ') {
            if (currentLineWidth + currentWordWidth > maxWidth) {
                lineCount++;
                currentLineWidth = currentWordWidth + spaceWidth;
            } else {
                currentLineWidth += currentWordWidth + spaceWidth;
            }
            currentWordWidth = 0;
        } else {
            currentWordWidth += charWidth;
        }
    }

    if (currentLineWidth + currentWordWidth > maxWidth) {
        lineCount++;
    }

    return lineCount;
}

void displayScrollingText(String longText, int scrollSpeedMs) {
    int lineHeight = 13;
    int totalLines = getStringLinesCount(longText, display.width());
    int totalTextHeight = totalLines * lineHeight;
    int screenHeight = display.height();

    if (totalTextHeight <= screenHeight) {
        display.clear();
        display.drawStringMaxWidth(0, 0, display.width(), longText);
        display.display();
        delay(3000);
        return;
    }

    for (int y = 0; y <= (totalTextHeight - screenHeight + 4); y += 2) {
        display.clear();
        display.drawStringMaxWidth(0, -y, display.width(), longText);
        display.display();
        delay(scrollSpeedMs);
    }
}

void showMessage(const String& text) {
    display.clear();
    display.setFont(ArialMT_Plain_16);
    displayScrollingText(text, 100);
    display.display();
    Serial.println(text);
}

// =========================
// WiFi Connection
// =========================
bool connectWiFi() {
    WiFi.begin(ssid, password);
    showMessage("Connecting WiFi...");

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
        delay(500);
    }

    if (WiFi.status() == WL_CONNECTED) {
        showMessage("WiFi ready\n" + WiFi.localIP().toString());
        return true;
    }

    showMessage("WiFi failed");
    return false;
}

// =========================
// Buffer Initialization
// =========================
bool initBuffer() {
    audioBuffer = (uint8_t*)malloc(WAV_SIZE);
    if (!audioBuffer) {
        showMessage("Buffer alloc failed");
        return false;
    }
    memset(audioBuffer, 0, WAV_SIZE);

    ttsBuffer = (uint8_t*)malloc(TTS_BUFFER_SIZE);
    if (!ttsBuffer) {
        showMessage("TTS buffer alloc failed");
        free(audioBuffer);
        audioBuffer = nullptr;
        return false;
    }
    memset(ttsBuffer, 0, TTS_BUFFER_SIZE);

    return true;
}

// =========================
// Display Initialization
// =========================
void initDisplay() {
    VextON();
    delay(100);
    display.init();
    display.clear();
    display.display();
}

// =========================
// I2S Configuration (Recording - INMP441)
// =========================
void setupI2SRecording() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 1024,
        .use_apll = true  // APLL for better audio quality
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_MIC_SCK,   // GPIO6
        .ws_io_num = I2S_MIC_WS,     // GPIO4
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_MIC_SD    // GPIO5
    };

    i2s_driver_install(I2S_MIC_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_MIC_PORT, &pin_config);
    i2s_zero_dma_buffer(I2S_MIC_PORT);
}

// =========================
// I2S Configuration (Playback - MAX98357)
// =========================
void setupI2SPlayback() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,  // MAX98357 mono output
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = true
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SPK_BCLK,  // GPIO2
        .ws_io_num = I2S_SPK_LRC,   // GPIO1
        .data_out_num = I2S_SPK_DIN, // GPIO7
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    i2s_driver_install(I2S_SPK_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_SPK_PORT, &pin_config);
    i2s_zero_dma_buffer(I2S_SPK_PORT);
}

// =========================
// WAV Header Generation
// =========================
void generateWavHeader(uint8_t* header, uint32_t wavDataSize) {
    uint32_t totalChunkSize = wavDataSize + 36;
    uint32_t byteRate = SAMPLE_RATE * CHANNELS * (BITS_PER_SAMPLE / 8);
    uint16_t blockAlign = CHANNELS * (BITS_PER_SAMPLE / 8);

    header[0] = 'R'; header[1] = 'I'; header[2] = 'F'; header[3] = 'F';
    header[4] = (totalChunkSize & 0xff);
    header[5] = ((totalChunkSize >> 8) & 0xff);
    header[6] = ((totalChunkSize >> 16) & 0xff);
    header[7] = ((totalChunkSize >> 24) & 0xff);

    header[8] = 'W'; header[9] = 'A'; header[10] = 'V'; header[11] = 'E';
    header[12] = 'f'; header[13] = 'm'; header[14] = 't'; header[15] = ' ';
    header[16] = 16; header[17] = 0; header[18] = 0; header[19] = 0;
    header[20] = 1; header[21] = 0;
    header[22] = CHANNELS; header[23] = 0;

    header[24] = (SAMPLE_RATE & 0xff);
    header[25] = ((SAMPLE_RATE >> 8) & 0xff);
    header[26] = ((SAMPLE_RATE >> 16) & 0xff);
    header[27] = ((SAMPLE_RATE >> 24) & 0xff);

    header[28] = (byteRate & 0xff);
    header[29] = ((byteRate >> 8) & 0xff);
    header[30] = ((byteRate >> 16) & 0xff);
    header[31] = ((byteRate >> 24) & 0xff);

    header[32] = (blockAlign & 0xff);
    header[33] = ((blockAlign >> 8) & 0xff);
    header[34] = BITS_PER_SAMPLE;
    header[35] = 0;

    header[36] = 'd'; header[37] = 'a'; header[38] = 't'; header[39] = 'a';
    header[40] = (wavDataSize & 0xff);
    header[41] = ((wavDataSize >> 8) & 0xff);
    header[42] = ((wavDataSize >> 16) & 0xff);
    header[43] = ((wavDataSize >> 24) & 0xff);
}

// =========================
// Audio Recording
// =========================
bool recordAudio() {
    if (!audioBuffer) return false;

    showMessage("Recording...");
    memset(audioBuffer, 0, WAV_SIZE);
    generateWavHeader(audioBuffer, PCM_SIZE);

    uint8_t* writePtr = audioBuffer + WAV_HEADER_SIZE;
    size_t totalBytesRead = 0;

    while (totalBytesRead < PCM_SIZE) {
        size_t bytesRead = 0;
        i2s_read(I2S_MIC_PORT, writePtr + totalBytesRead, 1024, &bytesRead, portMAX_DELAY);
        totalBytesRead += bytesRead;
    }

    showMessage("Recording done");
    return true;
}

// =========================
// Audio Playback (TTS via MAX98357)
// =========================
void playAudio(const uint8_t* data, size_t len) {
    if (!data || len == 0) return;

    size_t pos = 0;
    while (pos < len) {
        size_t toWrite = len - pos;
        if (toWrite > 1024) toWrite = 1024;
        
        size_t written = 0;
        i2s_write(I2S_SPK_PORT, data + pos, toWrite, &written, 100);
        pos += written;
    }
}

// =========================
// Base64 Encoding
// =========================
String base64Encode(const uint8_t* data, size_t len) {
    const char* table = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    String out;
    out.reserve(((len + 2) / 3) * 4);

    for (size_t i = 0; i < len; i += 3) {
        uint32_t n = data[i] << 16;
        if (i + 1 < len) n |= data[i + 1] << 8;
        if (i + 2 < len) n |= data[i + 2];

        out += table[(n >> 18) & 63];
        out += table[(n >> 12) & 63];
        out += (i + 1 < len) ? table[(n >> 6) & 63] : '=';
        out += (i + 2 < len) ? table[n & 63] : '=';
    }

    return out;
}

// =========================
// Send Audio to OpenRouter
// =========================
String sendAudioToOpenRouter() {
    if (WiFi.status() != WL_CONNECTED) return "WiFi not connected";
    if (!audioBuffer) return "No audio buffer";

    showMessage("Encoding WAV...");
    String audioBase64 = base64Encode(audioBuffer, WAV_SIZE);

    HTTPClient http;
    http.begin("https://openrouter.ai/api/v1/chat/completions");
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", "Bearer " + String(openRouterKey));
    http.addHeader("HTTP-Referer", "https://esp32-project.com");
    http.addHeader("X-OpenRouter-Title", "ESP32 Voice Assistant");

    DynamicJsonDocument doc(20000);
    doc["model"] = "google/gemini-2.5-flash";

    JsonArray messages = doc.createNestedArray("messages");
    JsonObject msg = messages.createNestedObject();
    msg["role"] = "user";

    JsonArray content = msg.createNestedArray("content");

    JsonObject textPart = content.createNestedObject();
    textPart["type"] = "text";
    textPart["text"] = "Please transcribe this wav audio and answer briefly.";

    JsonObject audioPart = content.createNestedObject();
    audioPart["type"] = "input_audio";

    JsonObject inputAudio = audioPart.createNestedObject("input_audio");
    inputAudio["data"] = audioBase64;
    inputAudio["format"] = "wav";

    String body;
    serializeJson(doc, body);

    showMessage("Uploading...");
    int httpCode = http.POST(body);

    if (httpCode <= 0) {
        http.end();
        return "HTTP error: " + String(httpCode);
    }

    String response = http.getString();
    http.end();

    DynamicJsonDocument responseDoc(12000);
    DeserializationError err = deserializeJson(responseDoc, response);
    if (err) return "JSON parse error";

    if (!responseDoc["choices"][0]["message"]["content"].is<const char*>()) {
        return "No message content";
    }

    return String((const char*)responseDoc["choices"][0]["message"]["content"]);
}

// =========================
// TTS: Simple Tone Generation (Fallback)
// =========================
void playTone(int frequency, int durationMs) {
    const int samples = (SAMPLE_RATE * durationMs) / 1000;
    static int16_t toneBuffer[4096];
    
    for (int i = 0; i < samples && i < 4096; i++) {
        toneBuffer[i] = (int16_t)(32767 * sin(2.0 * PI * frequency * i / SAMPLE_RATE));
    }
    
    playAudio((uint8_t*)toneBuffer, samples * 2);
}

// =========================
// TTS: Character Beep Feedback
// =========================
void playBeep() {
    playTone(880, 50);  // A5 note, 50ms
}

// =========================
// Button Detection
// =========================
bool isButtonPressed() {
    bool currentState = digitalRead(BUTTON_PIN);
    bool pressed = (lastButtonState == HIGH && currentState == LOW);
    lastButtonState = currentState;
    return pressed;
}

// =========================
// Setup / Loop
// =========================
void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);  // LED off (active low)

    initDisplay();

    if (!initBuffer()) return;

    setupI2SRecording();   // Configure INMP441 microphone
    setupI2SPlayback();     // Configure MAX98357 speaker
    
    if (!connectWiFi()) {
        showMessage("WiFi required for TTS");
        delay(2000);
    }

    showMessage("Press button\nto record");
}

void loop() {
    if (isButtonPressed()) {
        delay(30);
        digitalWrite(LED_PIN, LOW);  // LED on during recording

        if (recordAudio()) {
            String result = sendAudioToOpenRouter();
            digitalWrite(LED_PIN, HIGH);  // LED off
            
            showMessage(result);
            
            // Play feedback beep if speech was received
            if (!result.startsWith("WiFi") && !result.startsWith("No") && !result.startsWith("HTTP")) {
                playBeep();
            }
            
            delay(4000);
            showMessage("Press button\nto record");
        } else {
            digitalWrite(LED_PIN, HIGH);
        }
    }

    delay(20);
}