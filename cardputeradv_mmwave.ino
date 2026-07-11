/*
  cardputeradv_mmwave.ino
  Complete single-file sketch:
  - Frame-aware parser
  - Engineering-mode auto-detect and gate metrics
  - Calibration and runtime controls
*/

#include <M5Cardputer.h>
#include <Wire.h>

// ============================================================
// PIN & CONFIGURATION DEFINITIONS
// ============================================================
#define SENSOR_RX_PIN 15   // CardPuter GPIO15 ← Sensor TX
#define SENSOR_TX_PIN 13   // CardPuter GPIO13 ← Sensor RX

const unsigned long LOG_BAUD = 115200;
const int SENSOR_BAUD = 256000; // sensor UART (change if needed)

// Serial connection to sensor
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
const size_t MAX_PAYLOAD = 4096; // safety limit for payload length

// ---------------- Detection configuration (tuneable) ----------------
int personWindowFrames = 8;     // sliding window size (frames)
unsigned long personThreshold = 10;       // absolute fallback threshold (units vary by metric)
bool autoBaselineEnabled = true;
float baselineFactor = 2.0f;    // detection if avg > baseline * baselineFactor

// engineering-mode detection
bool engineeringDetected = false;
const uint16_t ENGINEERING_LEN_THRESH = 120; // bytes (heuristic)
int gateStart = 0;
int gateEnd = -1; // -1 means use payloadLen-1
int perGateThreshold = 10; // per-gate threshold for "count above" metric

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

// Forward declarations (no default args here)
void resetParser();
void feedByteToParser(uint8_t b);
void processFrame(uint8_t *buf, uint16_t len);
void addMetric(unsigned long v);
unsigned long metricsAverage();
unsigned long compute_interframe_diff(uint8_t *buf, uint16_t len);
unsigned long compute_payload_sum(uint8_t *buf, uint16_t len);
unsigned long compute_nonzero_count(uint8_t *buf, uint16_t len);
void resetMetricsBuffer(const char *msg);
void runCalibration(int captureFrames);
void sendEnableEngineeringModeOnce(unsigned long timeoutMs);

// ---------------- Parser helpers ----------------
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
  uint64_t sum = 0;
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
  return (unsigned long)(sum / n); // average difference per byte
}

unsigned long compute_payload_sum(uint8_t *buf, uint16_t len) {
  uint64_t s = 0;
  for (uint16_t i = 0; i < len; ++i) s += buf[i];
  return (unsigned long)s;
}

unsigned long compute_nonzero_count(uint8_t *buf, uint16_t len) {
  unsigned int c = 0;
  for (uint16_t i = 0; i < len; ++i) if (buf[i] != 0) c++;
  return c;
}

// ---------------- Frame processing ----------------
void processFrame(uint8_t *buf, uint16_t len) {
  // early guard
  if (len == 0) {
    lastFrameMetric = 0;
    addMetric(0);
    return;
  }

  validFrames++;

  // Auto-detect engineering mode if not already detected
  if (!engineeringDetected) {
    int nonzero = 0;
    for (uint16_t i = 0; i < len; ++i) if (buf[i] != 0) nonzero++;
    if ((uint16_t)len >= ENGINEERING_LEN_THRESH || nonzero > (len / 4)) {
      engineeringDetected = true;
      metricMode = PAYLOAD_SUM; // switch to absolute energy metric automatically
      Serial.printf("[AUTO] Engineering-mode detected (len=%u nonzero=%d). Switching to PAYLOAD_SUM\n", (unsigned)len, nonzero);
      // reset metrics so prior small-frame data doesn't pollute windows
      metricsIdx = 0; metricsCount = 0; metricsSum = 0;
      lastPayloadLen = 0; // reset interframe baseline
    }
  }

  unsigned long metric = 0;

  if (engineeringDetected) {
    // treat payload as gate energy array (1 byte per gate)
    int gs = (gateEnd < 0) ? (int)len - 1 : gateEnd;
    if (gs >= (int)len) gs = (int)len - 1;
    int s = gateStart;
    if (s < 0) s = 0;
    if (s > gs) { s = 0; gs = (int)len - 1; }

    unsigned long gateSum = 0;
    unsigned int countAbove = 0;
    unsigned int gateMax = 0;
    for (int i = s; i <= gs; ++i) {
      unsigned int v = buf[i];
      gateSum += v;
      if (v > (unsigned)perGateThreshold) countAbove++;
      if (v > gateMax) gateMax = v;
    }

    // choose metric (PAYLOAD_SUM default in engineering mode)
    if (metricMode == PAYLOAD_SUM) metric = gateSum;
    else if (metricMode == NONZERO_COUNT) metric = compute_nonzero_count(buf, len);
    else metric = compute_interframe_diff(buf, len); // fallback if user explicitly set it

    lastFrameMetric = metric;
    addMetric(metric);
    unsigned long avg = metricsAverage();

    // detection rules: either avg > baseline * factor (if baseline available), or avg >= personThreshold
    if (hasBaseline && autoBaselineEnabled) {
      personDetected = (avg > (unsigned long)(baselineValue * baselineFactor));
    } else {
      personDetected = (avg >= personThreshold);
    }

    Serial.printf("[ENG] len=%u gates=%d..%d sum=%lu max=%u above=%u avg(%d)=%lu -> detected=%s\n",
                  (unsigned)len, s, gs, gateSum, gateMax, countAbove, personWindowFrames, avg, personDetected ? "YES" : "no");

  } else {
    // Non-engineering: try to detect target-list style frames (small, structured)
    // Heuristic: if first byte is small <= 10, treat as target count.
    if (len >= 1 && buf[0] > 0 && buf[0] < 20) {
      uint8_t maybeCount = buf[0];
      personDetected = (maybeCount > 0);
      metric = maybeCount;
      lastFrameMetric = metric;
      addMetric(metric);
      Serial.printf("[TL] target_count=%u -> detected=%s\n", maybeCount, personDetected ? "YES" : "no");
    } else {
      // fallback: compute chosen metric (default interframe diff)
      if (metricMode == INTERFRAME_DIFF) metric = compute_interframe_diff(buf, len);
      else if (metricMode == PAYLOAD_SUM) metric = compute_payload_sum(buf, len);
      else metric = compute_nonzero_count(buf, len);

      lastFrameMetric = metric;
      addMetric(metric);
      unsigned long avg = metricsAverage();
      if (hasBaseline && autoBaselineEnabled) {
        personDetected = (avg > (unsigned long)(baselineValue * baselineFactor));
      } else {
        personDetected = (avg >= personThreshold);
      }
      Serial.printf("[FALLBACK] len=%u metric=%lu avg(%d)=%lu -> detected=%s\n",
                    (unsigned)len, metric, personWindowFrames, avg, personDetected ? "YES" : "no");
    }
  }

  // Store current payload as last payload for inter-frame diff calculations
  if (len <= MAX_PAYLOAD) {
    memcpy(lastPayloadBuf, buf, len);
    lastPayloadLen = len;
  } else {
    lastPayloadLen = 0;
  }
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
          if (payloadLen > MAX_PAYLOAD) {
            Serial.printf("[PARSER] payloadLen > MAX_PAYLOAD (%u) - resync\n", (unsigned)payloadLen);
            resetParser();
          } else {
            payloadIdx = 0;
            parseState = READ_PAYLOAD;
          }
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
          // valid frame complete
          processFrame(payloadBuf, payloadLen);
          resetParser();
        }
      } else {
        // mismatch -> resync conservatively
        Serial.println("[PARSER] Footer mismatch, resyncing");
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

// Run a small calibration capture (N frames) to compute baseline.
// This is a blocking routine: it waits for captureFrames parsed frames.
void runCalibration(int captureFrames = 8) {
  if (captureFrames < 1) captureFrames = 1;
  Serial.printf("[CAL] Capturing %d frames for baseline... (stand away)\n", captureFrames);

  // Clear metrics buffer to start collecting
  metricsIdx = 0; metricsCount = 0; metricsSum = 0;
  unsigned long start = millis();
  unsigned int captured = 0;
  unsigned long timeoutMs = 6000 + (unsigned long)captureFrames * 500;

  // We'll poll metricsCount to see when frames are appended to the window
  while (captured < (unsigned)captureFrames && (millis() - start) < timeoutMs) {
    // If metricsCount increases, assume a frame was processed
    if (metricsCount > captured) {
      // set captured to metricsCount but clamp to captureFrames
      captured = metricsCount;
    }
    // Allow other background tasks (not strictly necessary)
    delay(20);
  }

  if (metricsCount > 0) {
    baselineValue = metricsAverage();
    hasBaseline = true;
    Serial.printf("[CAL] Done: baseline=%lu (factor %.2f)\n", baselineValue, baselineFactor);
    statusMessage = "Calibration done";
  } else {
    Serial.println("[CAL] Failed: no frames captured");
    statusMessage = "Calibration failed";
  }

  // reset metric window after calibration, to allow fresh smoothing
  metricsIdx = 0; metricsCount = 0; metricsSum = 0;
}

// send enable engineering mode and wait for ACK (blocking up to timeoutMs)
void sendEnableEngineeringModeOnce(unsigned long timeoutMs = 800) {
  // Send command: header + len(0x0002) + cmdWord(0x0062 LSB-first) + footer
  uint8_t cmd[] = { 0xFD,0xFC,0xFB,0xFA, 0x02,0x00, 0x62,0x00, 0x04,0x03,0x02,0x01 };
  ld2410.write(cmd, sizeof(cmd));
  Serial.println("[CMD] Sent enable-engineering-mode; waiting for ACK...");

  // ACK pattern prefix
  const uint8_t ACK_PREFIX[] = {0xFD,0xFC,0xFB,0xFA, 0x04,0x00, 0x62,0x01};
  int match = 0;
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    if (ld2410.available()) {
      uint8_t b = ld2410.read();
      // Serial.printf("%02X ", b); // optional raw echo
      if (b == ACK_PREFIX[match]) {
        match++;
        if (match == (int)sizeof(ACK_PREFIX)) {
          // next two bytes are status
          unsigned long t0 = millis();
          while (ld2410.available() < 2 && millis() - t0 < 200) delay(5);
          if (ld2410.available() >= 2) {
            uint8_t s0 = ld2410.read();
            uint8_t s1 = ld2410.read();
            uint16_t status = (uint16_t)s0 | ((uint16_t)s1 << 8);
            if (status == 0) {
              Serial.println("[ACK] Engineering mode enabled (status=0).");
              engineeringDetected = true;
              metricMode = PAYLOAD_SUM;
              metricsIdx = 0; metricsCount = 0; metricsSum = 0;
              return;
            } else {
              Serial.printf("[ACK] NACK status=%u\n", (unsigned)status);
              return;
            }
          } else {
            Serial.println("[ACK] Prefix matched but status bytes missing.");
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
  Serial.println("[CMD] No ACK received (timeout).");
}

// ---------------- Setup & UI ----------------
void setup() {
  Serial.begin(LOG_BAUD);
  Serial.println("\n=== CARDPUTER + LD2410C - FRAME PARSER (Full) ===");

  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);
  M5Cardputer.Display.setRotation(1);

  uint16_t w = M5Cardputer.Display.width();
  uint16_t h = M5Cardputer.Display.height();
  canvas.createSprite(w - 4, h - 4);
  canvas.setTextColor(WHITE);
  canvas.setTextSize(1);

  Serial.printf("[UART] Initializing at %d baud\n", SENSOR_BAUD);
  ld2410.begin(SENSOR_BAUD, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);
  delay(200);

  resetParser();
  resetMetricsBuffer("Ready");

  Serial.println("[INIT] Controls: W/Q thr +/- | A/S win +/- | C cycle metric | E send enable | Space calibrate | Esc restart");
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
  canvas.printf("W/Q: Thresh +/-   A/S: Win +/-   C: Mode   E: Enable ENG   Space: Calib");

  canvas.setTextSize(1);
  canvas.setTextColor(YELLOW);
  canvas.setCursor(5,38);
  const char *modeName = (metricMode==INTERFRAME_DIFF) ? "INTER_DIFF" : (metricMode==PAYLOAD_SUM) ? "PAYLOAD_SUM" : "NONZERO";
  canvas.printf("Mode: %s   Window: %d   Thresh: %lu", modeName, personWindowFrames, personThreshold);

  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.setCursor(5,52);
  canvas.printf("Baseline: %lu  EngMode:%s", baselineValue, engineeringDetected ? "YES" : "no");

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

// ---------------- Main loop ----------------
void loop() {
  M5Cardputer.update();

  // feed incoming bytes to parser
  while (ld2410.available()) {
    uint8_t b = (uint8_t)ld2410.read();
    byteCount++;
    anyDataReceived = true;
    dataTimestamp = millis();

    // optional hex debug print (comment out if too verbose)
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
          } else if (k == 'e' || k == 'E') {
            sendEnableEngineeringModeOnce(1000);
            statusMessage = "Enable command sent";
          } else if (k == ' ') {
            runCalibration(8);
            statusMessage = "Calibration done";
          } else if (k == '\x1B') {
            esp_restart();
          }
          lastKeyTime = millis();
        }
      }
    }
  }

  drawInterface();
}
