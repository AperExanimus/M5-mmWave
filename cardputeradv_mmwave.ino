#include <M5Cardputer.h>
#include <Wire.h>

// ============================================================
// PIN & CONFIGURATION DEFINITIONS
// ============================================================
#define SENSOR_RX_PIN 15   // CardPuter GPIO15 ← Sensor TX
#define SENSOR_TX_PIN 13   // CardPuter GPIO13 ← Sensor RX
// Engineering mode detection + control
bool engineeringDetected = false;          // set true when we see long payloads / many nonzero bytes
const uint16_t ENGINEERING_LEN_THRESH = 120; // if a frame's payloadLen > this, assume engineering mode

// Serial/log baud (for Serial Monitor)
const unsigned long LOG_BAUD = 115200;

// NOTE: set this to the baud you've verified the sensor uses.
const int SENSOR_BAUD = 256000;

HardwareSerial ld2410(1);

String statusMessage = "Ready";
unsigned long lastKeyTime = 0;
const unsigned long KEY_DELAY_MS = 200;
char lastCapturedKey = 0;

// SENSOR STATUS TRACKING
int byteCount = 0;
bool anyDataReceived = false;
unsigned long dataTimestamp = 0;

// Frame counters
int validFrames = 0;

M5Canvas canvas(&M5Cardputer.Display);

// ----------------------- Frame parser config -----------------------
const uint8_t FRAME_HEADER[4] = {0xFD, 0xFC, 0xFB, 0xFA};
const uint8_t FRAME_FOOTER[4] = {0x04, 0x03, 0x02, 0x01};
const size_t MAX_PAYLOAD = 2048; // safety limit for payload length

// ---------------- Detection configuration (tuneable) ----------------
// Person detection uses a sliding window of per-frame metrics.
int personWindowFrames = 8;     // default window (in frames)
int personThreshold = 10;       // detection threshold (interpretation depends on metric mode)
bool autoBaselineEnabled = true; // if true, calibration on Space uses multiplicative factor
float baselineFactor = 2.0f;    // detection when avg > baseline * baselineFactor

// ---------------- Metric modes ----------------
enum MetricMode { INTERFRAME_DIFF = 0, PAYLOAD_SUM = 1, NONZERO_COUNT = 2 };
MetricMode metricMode = INTERFRAME_DIFF;

// Buffers and sliding-window metric storage
const int MAX_WINDOW = 512;
unsigned long metricsBuf[MAX_WINDOW];
int metricsIdx = 0;
int metricsCount = 0;
unsigned long metricsSum = 0;

bool personDetected = false;
unsigned long lastFrameMetric = 0;

// Baseline calibration state
unsigned long baselineValue = 0;
bool hasBaseline = false;

// Parser state
enum ParseState { SEARCH_HEADER, READ_LEN, READ_PAYLOAD, READ_FOOTER };
ParseState parseState = SEARCH_HEADER;
uint8_t headerMatchIdx = 0;
uint8_t footerMatchIdx = 0;
uint8_t lenBytes[2] = {0, 0};
uint16_t payloadLen = 0;
uint16_t payloadIdx = 0;
uint8_t payloadBuf[MAX_PAYLOAD];

// last payload for INTERFRAME_DIFF
uint8_t lastPayloadBuf[MAX_PAYLOAD];
uint16_t lastPayloadLen = 0;

void resetParser() {
  parseState = SEARCH_HEADER;
  headerMatchIdx = 0;
  footerMatchIdx = 0;
  payloadLen = 0;
  payloadIdx = 0;
  lenBytes[0] = 0;
  lenBytes[1] = 0;
}

// sliding-window add
void addMetric(unsigned long v) {
  if (personWindowFrames < 1) personWindowFrames = 1;
  if (personWindowFrames > MAX_WINDOW) personWindowFrames = MAX_WINDOW;

  if (metricsCount < personWindowFrames) {
    metricsBuf[metricsIdx] = v;
    metricsSum += v;
    metricsIdx = (metricsIdx + 1) % personWindowFrames;
    metricsCount++;
  } else {
    // replace oldest
    int replace = metricsIdx % personWindowFrames;
    metricsSum -= metricsBuf[replace];
    metricsBuf[replace] = v;
    metricsSum += v;
    metricsIdx = (replace + 1) % personWindowFrames;
  }
}

unsigned long metricsAverage() {
  if (metricsCount == 0) return 0;
  return metricsSum / metricsCount;
}

// Compute metrics
unsigned long compute_interframe_diff(uint8_t *buf, uint16_t len) {
  if (lastPayloadLen == 0 || len == 0) return 0;
  uint32_t sum = 0;
  uint16_t n = (len < lastPayloadLen) ? len : lastPayloadLen;
  for (uint16_t i = 0; i < n; ++i) sum += abs((int)buf[i] - (int)lastPayloadBuf[i]);
  if (len > lastPayloadLen) {
    for (uint16_t i = lastPayloadLen; i < len; ++i) sum += abs((int)buf[i]);
    n = len;
  } else if (lastPayloadLen > len) {
    for (uint16_t i = len; i < lastPayloadLen; ++i) sum += abs((int)lastPayloadBuf[i]);
    n = lastPayloadLen;
  }
  if (n == 0) return 0;
  return sum / n; // average difference per byte
}

unsigned long compute_payload_sum(uint8_t *buf, uint16_t len) {
  uint32_t s = 0;
  for (uint16_t i = 0; i < len; ++i) s += buf[i];
  return s;
}

unsigned long compute_nonzero_count(uint8_t *buf, uint16_t len) {
  unsigned int c = 0;
  for (uint16_t i = 0; i < len; ++i) if (buf[i] != 0) c++;
  return c;
}

void processFrame(uint8_t *buf, uint16_t len) {
  unsigned long metric = 0;
  switch (metricMode) {
    case INTERFRAME_DIFF:
      metric = compute_interframe_diff(buf, len);
      break;
    case PAYLOAD_SUM:
      metric = compute_payload_sum(buf, len);
      break;
    case NONZERO_COUNT:
      metric = compute_nonzero_count(buf, len);
      break;
  }
  lastFrameMetric = metric;

  // store last payload for next interframe comparison
  if (len <= MAX_PAYLOAD) {
    memcpy(lastPayloadBuf, buf, len);
    lastPayloadLen = len;
  } else {
    lastPayloadLen = 0;
  }

  validFrames++;

    // --- auto-detect engineering mode by payload length / content ---
  if (!engineeringDetected) {
    // If payload length is large, or contains many non-zero bytes, it's almost certainly engineering-mode gate data
    int nonzero = 0;
    for (uint16_t i = 0; i < len; ++i) if (buf[i] != 0) nonzero++;
    if (len >= ENGINEERING_LEN_THRESH || nonzero > (len/4)) {
      engineeringDetected = true;
      metricMode = PAYLOAD_SUM; // switch to absolute-energy metric automatically
      Serial.printf("[AUTO] Engineering-mode detected (len=%u nonzero=%d). Switching to PAYLOAD_SUM\n", (unsigned)len, nonzero);
      // reset any sliding windows/buffers so we don't use old inapplicable values
      metricsIdx = 0; metricsCount = 0; metricsSum = 0;
    }
  }

  addMetric(metric);
  unsigned long avg = metricsAverage();

  // If baseline exists and auto mode, use baselineFactor; otherwise compare to personThreshold
  if (hasBaseline && autoBaselineEnabled) {
    personDetected = (avg > (unsigned long)(baselineValue * baselineFactor));
  } else {
    personDetected = (avg >= (unsigned long)personThreshold);
  }

  Serial.printf("[FRAME] len=%u metric=%lu avg(%d)=%lu baseline=%lu detected=%s\n",
                (unsigned)len, metric, personWindowFrames, avg, baselineValue, personDetected ? "YES" : "no");
}

// feed bytes to parser (streaming)
void feedByteToParser(uint8_t b) {
  switch (parseState) {
    case SEARCH_HEADER:
      if (b == FRAME_HEADER[headerMatchIdx]) {
        headerMatchIdx++;
        if (headerMatchIdx == 4) {
          parseState = READ_LEN;
          headerMatchIdx = 0;
          lenBytes[0] = 0; lenBytes[1] = 0;
        }
      } else {
        headerMatchIdx = (b == FRAME_HEADER[0]) ? 1 : 0;
      }
      break;
    case READ_LEN:
      if (lenBytes[0] == 0 && lenBytes[1] == 0) {
        lenBytes[0] = b;
      } else {
        lenBytes[1] = b;
        payloadLen = (uint16_t)lenBytes[0] | ((uint16_t)lenBytes[1] << 8);
        lenBytes[0] = 0; lenBytes[1] = 0;
        if (payloadLen == 0) {
          payloadIdx = 0;
          parseState = READ_FOOTER;
          footerMatchIdx = 0;
        } else if (payloadLen <= MAX_PAYLOAD) {
          payloadIdx = 0;
          parseState = READ_PAYLOAD;
        } else {
          Serial.printf("[PARSER] Invalid payloadLen=%u, resync\n", (unsigned)payloadLen);
          resetParser();
        }
      }
      break;
    case READ_PAYLOAD:
      payloadBuf[payloadIdx++] = b;
      if (payloadIdx >= payloadLen) {
        parseState = READ_FOOTER;
        footerMatchIdx = 0;
      }
      break;
    case READ_FOOTER:
      if (b == FRAME_FOOTER[footerMatchIdx]) {
        footerMatchIdx++;
        if (footerMatchIdx == 4) {
          // frame complete
          processFrame(payloadBuf, payloadLen);
          resetParser();
        }
      } else {
        // mismatch: resync and try consider this byte as possible header start
        Serial.println("[PARSER] Footer mismatch, resync");
        resetParser();
        if (b == FRAME_HEADER[0]) headerMatchIdx = 1;
      }
      break;
  }
}

// ---------------- Helpers for calibration and UI ----------------
void resetMetricsBuffer(const char *msg) {
  metricsIdx = 0;
  metricsCount = 0;
  metricsSum = 0;
  lastPayloadLen = 0;
  hasBaseline = false;
  baselineValue = 0;
  personDetected = false;
  statusMessage = msg;
}

// Run a small calibration capture (N frames) to compute baseline
void runCalibration(int captureFrames = 8) {
  Serial.printf("Calibration started: capturing %d frames for baseline (please stand away)...\n", captureFrames);
  // capture frames by blocking until enough frames observed
  unsigned long start = millis();
  unsigned int captured = 0;
  unsigned long sum = 0;
  int oldMetricsWindow = personWindowFrames;
  // Temporarily set window small so the stream updates quickly
  personWindowFrames = captureFrames;
  metricsIdx = 0;
  metricsCount = 0;
  metricsSum = 0;
  unsigned long timeout = 5000 + captureFrames * 500; // avoid hang
  while (captured < (unsigned)captureFrames && millis() - start < timeout) {
    // We rely on processFrame to push metrics; just wait
    if (metricsCount > 0) {
      // consume newly added metrics
      while (metricsCount > 0 && captured < captureFrames) {
        // Actually metricsCount is sliding; read metricsSum/metricsCount later
        // For simplicity, poll metricsAverage after each frame
        captured = captured + 1; // increment by frame observed
        delay(10);
      }
    }
    delay(10);
  }
  // take the average currently in the buffer
  if (metricsCount > 0) {
    baselineValue = metricsAverage();
    hasBaseline = true;
    Serial.printf("Calibration done: baseline=%lu (factor %.2f)\n", baselineValue, baselineFactor);
  } else {
    Serial.println("Calibration failed: no frames captured");
  }
  // restore window to previous
  personWindowFrames = oldMetricsWindow;
  // clear metrics to repopulate
  metricsIdx = 0; metricsCount = 0; metricsSum = 0;
}

// Optional: send engineering mode enable command at startup
const bool ENABLE_ENGINEERING_MODE_AT_START = false;
void sendEnableEngineeringMode() {
  // Send the "enable engineering mode" command (command word 0x0062).
  // The "send data" payload for this command is likely:
  // header + length + commandWordLSB + commandWordMSB + [commandValue if any] + footer
  // For the "no value" example length is 0x0004 in the PDF's ACK example, but
  // for "send" usually 0x0002 (command only). We'll send the small command shown in many examples:
  // Example format used in PDF for "read firmware": FD FC FB FA 02 00 A0 00 04 03 02 01
  uint8_t cmd[] = { 0xFD,0xFC,0xFB,0xFA, 0x02,0x00, 0x62,0x00, 0x04,0x03,0x02,0x01 };
  ld2410.write(cmd, sizeof(cmd));
  Serial.println("Sent enable-engineering-mode candidate frame to sensor (please verify ACK).");
}

// send enable engineering mode and wait up to `timeoutMs` for an ACK
void sendEnableEngineeringModeOnce(unsigned long timeoutMs = 800) {
  // Send command: header + len(0x0002) + cmdWord(0x0062 LSB-first) + footer
  uint8_t cmd[] = { 0xFD,0xFC,0xFB,0xFA, 0x02,0x00, 0x62,0x00, 0x04,0x03,0x02,0x01 };
  ld2410.write(cmd, sizeof(cmd));
  Serial.println("Sent enable-engineering-mode candidate frame; waiting for ACK...");

  // simple blocking scan for the ACK pattern in incoming bytes
  const uint8_t ACK_PREFIX[] = {0xFD,0xFC,0xFB,0xFA, 0x04,0x00, 0x62,0x01};
  int match = 0;
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (ld2410.available()) {
      uint8_t b = ld2410.read();
      // optionally print bytes while waiting (comment out if too noisy)
      // Serial.printf("%02X ", b);
      if (b == ACK_PREFIX[match]) {
        match++;
        if (match == (int)(sizeof(ACK_PREFIX))) {
          // we saw header+len+ack-cmd; next two bytes should be status
          // read status bytes if available
          unsigned long t0 = millis();
          while (ld2410.available() < 2 && millis() - t0 < 200) delay(5);
          if (ld2410.available() >= 2) {
            uint8_t s0 = ld2410.read();
            uint8_t s1 = ld2410.read();
            uint16_t status = (uint16_t)s0 | ((uint16_t)s1 << 8);
            if (status == 0) {
              Serial.println("ACK: engineering mode enabled (status=0).");
              engineeringDetected = true;
              metricMode = PAYLOAD_SUM;
              metricsIdx = 0; metricsCount = 0; metricsSum = 0;
              return;
            } else {
              Serial.printf("ACK received but status=%u (failure)\n", (unsigned)status);
              return;
            }
          } else {
            Serial.println("ACK prefix matched but status bytes not received in time.");
            return;
          }
        }
      } else {
        match = (b == ACK_PREFIX[0]) ? 1 : 0;
      }
    } else {
      delay(5);
    }
  }
  Serial.println("No ACK received for enable-engineering command (timeout).");
}

// ---------------- Setup & UI ----------------
void setup() {
  Serial.begin(LOG_BAUD);
  Serial.println("\n=== CARDPUTER + LD2410C - FRAME PARSER (with calibration & metrics) ===");

  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);
  M5Cardputer.Display.setRotation(1);
  uint16_t w = M5Cardputer.Display.width();
  uint16_t h = M5Cardputer.Display.height();
  canvas.createSprite(w - 4, h - 4);
  canvas.setTextColor(WHITE);
  canvas.setTextSize(1);

  Serial.printf("[UART] Initializing sensor UART at %d\n", SENSOR_BAUD);
  ld2410.begin(SENSOR_BAUD, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);
  delay(200);

  if (ENABLE_ENGINEERING_MODE_AT_START) {
    sendEnableEngineeringMode();
  }

  Serial.println("[INIT] Complete. Controls: W/Q thresh +/- | A/S window +/- | C cycle metric | Space calibrate | Esc restart");
  resetParser();
  resetMetricsBuffer("Ready");
}

void drawInterface() {
  canvas.fillSprite(BLACK);
  canvas.setCursor(5,5);
  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.println("LD2410C Interactive Scanner");
  canvas.drawFastHLine(0,18,canvas.width()-10,WHITE);

  canvas.setCursor(5,24);
  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.printf("W/Q: Thresh +/-   A/S: Win +/-   C: Mode   Space: Calib");

  canvas.setTextSize(1);
  canvas.setTextColor(YELLOW);
  canvas.setCursor(5,38);
  canvas.printf("Window: %d   Thresh: %d", personWindowFrames, personThreshold);

  canvas.setTextSize(1);
  canvas.setCursor(5,52);
  canvas.setTextColor(WHITE);
  const char* modeName = (metricMode==INTERFRAME_DIFF) ? "INTER_DIFF" : (metricMode==PAYLOAD_SUM) ? "PAYLOAD_SUM" : "NONZERO";
  canvas.printf("Metric: %s   Baseline: %lu", modeName, baselineValue);

  canvas.setTextSize(2);
  if (personDetected) {
    canvas.setTextColor(RED);
    canvas.setCursor(5,84);
    canvas.printf("Person: YES");
  } else {
    canvas.setTextColor(GREEN);
    canvas.setCursor(5,84);
    canvas.printf("Person: NO ");
  }

  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.setCursor(5, canvas.height()-14);
  canvas.printf("Status: %s", statusMessage.c_str());

  canvas.pushSprite(2,2);
}

void loop() {
  M5Cardputer.update();

  // feed incoming bytes to parser
  while (ld2410.available()) {
    uint8_t b = (uint8_t)ld2410.read();
    byteCount++;
    anyDataReceived = true;
    dataTimestamp = millis();

    // optional hex debug
    if (b < 16) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ");
    if (byteCount % 32 == 0) Serial.println();

    feedByteToParser(b);
  }

  if (anyDataReceived && millis() - dataTimestamp > 2000) {
    statusMessage = "No recent data...";
  }

  // keyboard controls
  if (M5Cardputer.Keyboard.isChange()) {
    if (M5Cardputer.Keyboard.isPressed()) {
      Keyboard_Class::KeysState ks = M5Cardputer.Keyboard.keysState();
      for (auto k : ks.word) {
        lastCapturedKey = k;
        if (millis() - lastKeyTime > KEY_DELAY_MS) {
          if (k == 'w' || k == 'W') {
            personThreshold++;
            statusMessage = "Threshold ++";
          } else if (k == 'q' || k == 'Q') {
            if (personThreshold > 0) personThreshold--;
            statusMessage = "Threshold --";
          } else if (k == 'a' || k == 'A') {
            if (personWindowFrames > 1) personWindowFrames = max(1, personWindowFrames/2);
            // clear metrics on big window change
            metricsIdx = 0; metricsCount = 0; metricsSum = 0;
            statusMessage = "Window decreased";
          } else if (k == 's' || k == 'S') {
            if (personWindowFrames < MAX_WINDOW) personWindowFrames = min(MAX_WINDOW, personWindowFrames*2);
            metricsIdx = 0; metricsCount = 0; metricsSum = 0;
            statusMessage = "Window increased";
          } else if (k == 'c' || k == 'C') {
            metricMode = MetricMode((metricMode + 1) % 3);
            statusMessage = "Metric mode changed";
            metricsIdx = 0; metricsCount = 0; metricsSum = 0;
          } else if (k == ' ') {
            // calibration
            runCalibration(8);
            statusMessage = "Calibration done";
          } else if (k == '\x1B') {
            esp_restart();
          }
          else if (k == 'e' || k == 'E') {
           sendEnableEngineeringModeOnce();
           statusMessage = "Send enable cmd (see Serial)";
}
          lastKeyTime = millis();
        }
      }
    }
  }

  drawInterface();
}
