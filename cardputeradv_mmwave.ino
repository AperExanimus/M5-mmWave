/*
  cardputeradv_mmwave.ino
  Full sketch:
  - Command frame parser: FD FC FB FA ... 04 03 02 01
  - Report frame parser: 0xAA ... 0x55  (per PDF pages 16-17)
  - Engineering-mode detection + gate metrics
  - Calibration + runtime controls
*/

#include <M5Cardputer.h>
#include <Wire.h>

struct ParsedLd2410Report {
  bool ok = false;
  bool enhanced = false;      // true for type=0x01
  uint8_t type = 0;           // 0x01 enhanced, 0x02 normal
  uint8_t targetState = 0xFF; // table 12: 0=no target,1=moving,2=stationary,3=both

  uint16_t movingDistanceCm = 0;
  uint8_t  movingEnergy = 0;

  uint16_t stationaryDistanceCm = 0;
  uint8_t  stationaryEnergy = 0;

  uint16_t detectionDistanceCm = 0;

  uint8_t movingGateMax = 0;     // N
  uint8_t stationaryGateMax = 0; // N

  // 1-byte energies per gate (0..N), capped to 9 (gate 0..8 typical)
  uint8_t movingGateEnergy[9] = {0};
  uint8_t stationaryGateEnergy[9] = {0};

  uint8_t lightLevel = 0; // optional enhanced tail
  uint8_t outLevel = 0;   // optional enhanced tail
};

// ---------------- Data frame parser state (outer report framing) ----------------
// Outer data frame format (MyLD2410 style):
// Header: F4 F3 F2 F1
// Length: 2 bytes little-endian
// Payload: <length> bytes
// Tail: F8 F7 F6 F5

const uint8_t DATA_HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};
const uint8_t DATA_TAIL[4]   = {0xF8, 0xF7, 0xF6, 0xF5};
const size_t DATA_PAYLOAD_MAX = 512; // realistic upper bound; safe for LD2410

enum DataParseState {
  D_SEARCH_HEADER = 0,
  D_READ_LEN_0,
  D_READ_LEN_1,
  D_READ_PAYLOAD_AND_TAIL
};

DataParseState dataParseState = D_SEARCH_HEADER;
uint8_t dataHeaderMatchIdx = 0;
uint16_t dataExpectedLen = 0;       // payload length from frame
uint16_t dataReadCount = 0;         // bytes read in payload+tail phase
uint8_t dataPayloadBuf[DATA_PAYLOAD_MAX];
uint8_t dataTailBuf[4];

// ============================================================
// PIN & CONFIGURATION DEFINITIONS
// ============================================================
#define SENSOR_RX_PIN 15   // CardPuter GPIO15 ← Sensor TX
#define SENSOR_TX_PIN 13   // CardPuter GPIO13 ← Sensor RX

const unsigned long LOG_BAUD = 115200;
const int SENSOR_BAUD = 256000; // sensor UART

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

// ----------------------- Command frame parser config -----------------------
const uint8_t CMD_HEADER[4] = {0xFD, 0xFC, 0xFB, 0xFA};
const uint8_t CMD_FOOTER[4] = {0x04, 0x03, 0x02, 0x01};
const size_t MAX_PAYLOAD = 4096; // safety limit for command payload length

// ----------------------- Report frame parser config -----------------------
// Per PDF table: data frame contains Head=0xAA ... Tail=0x55
const uint8_t REPORT_HEAD = 0xAA;
const uint8_t REPORT_TAIL = 0x55;
const size_t REPORT_MAX = 4096;

// ---------------- Detection configuration ----------------
int personWindowFrames = 8;
unsigned long personThreshold = 10; // absolute fallback threshold
bool autoBaselineEnabled = true;
float baselineFactor = 2.0f;        // detection when avg > baseline * factor

// Engineering-mode detection (heuristic)
bool engineeringDetected = false;
const uint16_t ENGINEERING_LEN_THRESH = 120; // report payload length heuristic
int gateStart = 0;
int gateEnd = -1;          // -1 => use end of payload
int perGateThreshold = 10; // for countAbove logging

// ---------------- Metric modes ----------------
enum MetricMode { INTERFRAME_DIFF = 0, PAYLOAD_SUM = 1, NONZERO_COUNT = 2 };
MetricMode metricMode = INTERFRAME_DIFF;

// Sliding window metric storage
const int MAX_WINDOW = 512;
unsigned long metricsBuf[MAX_WINDOW];
int metricsIdx = 0;
int metricsCount = 0;
unsigned long metricsSum = 0;

bool personDetected = false;
unsigned long lastFrameMetric = 0;

// Baseline state
unsigned long baselineValue = 0;
bool hasBaseline = false;

// ---------------- Command parser state ----------------
enum CmdParseState { CMD_SEARCH_HEADER, CMD_READ_LEN, CMD_READ_PAYLOAD, CMD_READ_FOOTER };
CmdParseState cmdParseState = CMD_SEARCH_HEADER;
uint8_t cmdHeaderMatchIdx = 0;
uint8_t cmdFooterMatchIdx = 0;
uint8_t cmdLenBytes[2] = {0, 0};
uint16_t cmdPayloadLen = 0;
uint16_t cmdPayloadIdx = 0;
uint8_t cmdPayloadBuf[MAX_PAYLOAD];


// last payload for interframe diff mode
uint8_t lastPayloadBuf[MAX_PAYLOAD];
uint16_t lastPayloadLen = 0;

// Forward declarations
void resetCmdParser();
void feedByteToCmdParser(uint8_t b);
void reportFeedByte(uint8_t b);
void processReportFrame(uint8_t *buf, uint16_t len);
void addMetric(unsigned long v);
unsigned long metricsAverage();
void resetMetricsBuffer(const char *msg);
void runCalibration(int captureFrames = 8);

unsigned long compute_interframe_diff(uint8_t *buf, uint16_t len);
unsigned long compute_payload_sum(uint8_t *buf, uint16_t len);
unsigned long compute_nonzero_count(uint8_t *buf, uint16_t len);

void sendEnableEngineeringModeOnce(unsigned long timeoutMs = 1000);
void sendRestartModuleOnce(unsigned long timeoutMs = 1000);

// -------------------------------------------------------------
// Utility: sliding window
// -------------------------------------------------------------
void addMetric(unsigned long v) {
  if (personWindowFrames < 1) personWindowFrames = 1;
  if (personWindowFrames > MAX_WINDOW) personWindowFrames = MAX_WINDOW;

  if (metricsCount < personWindowFrames) {
    metricsBuf[metricsIdx] = v;
    metricsSum += v;
    metricsIdx = (metricsIdx + 1) % personWindowFrames;
    metricsCount++;
  } else {
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

// -------------------------------------------------------------
// Metric calculators
// -------------------------------------------------------------

bool parseLd2410DataPayload(const uint8_t* p, size_t len, ParsedLd2410Report& out) {
  out = ParsedLd2410Report();

  // Must contain at least: type + 0xAA + base fields up to detection distance
  // Offsets as used by MyLD2410:
  // [0]=type, [1]=0xAA, [2]=status, [3..4]=mDist, [5]=mSig, [6..7]=sDist, [8]=sSig, [9..10]=distance
  if (!p || len < 11) return false;
  if (!((p[0] == 0x01) || (p[0] == 0x02))) return false;
  if (p[1] != 0xAA) return false;

  out.type = p[0];
  out.enhanced = (p[0] == 0x01);

  out.targetState = p[2] & 0x07; // 0..3 relevant for presence
  out.movingDistanceCm = (uint16_t)p[3] | ((uint16_t)p[4] << 8);
  out.movingEnergy = p[5];
  out.stationaryDistanceCm = (uint16_t)p[6] | ((uint16_t)p[7] << 8);
  out.stationaryEnergy = p[8];
  out.detectionDistanceCm = (uint16_t)p[9] | ((uint16_t)p[10] << 8);

  if (!out.enhanced) {
    out.ok = true;
    return true;
  }

  // Enhanced extension
  // [11]=moving N, [12]=stationary N, then moving (N+1) bytes, then stationary (N+1) bytes, then light/out optional
  if (len < 13) return false;

  uint8_t mN = p[11];
  uint8_t sN = p[12];
  if (mN > 8) mN = 8;
  if (sN > 8) sN = 8;
  out.movingGateMax = mN;
  out.stationaryGateMax = sN;

  size_t idx = 13;
  if (idx + (size_t)mN + 1 > len) return false;
  for (uint8_t i = 0; i <= mN; i++) out.movingGateEnergy[i] = p[idx++];

  if (idx + (size_t)sN + 1 > len) return false;
  for (uint8_t i = 0; i <= sN; i++) out.stationaryGateEnergy[i] = p[idx++];

  // Optional extras if present
  if (idx < len) out.lightLevel = p[idx++];
  if (idx < len) out.outLevel = p[idx++];

  out.ok = true;
  return true;
}

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
  return (unsigned long)(sum / n);
}

unsigned long compute_payload_sum(uint8_t *buf, uint16_t len) {
  uint64_t s = 0;
  for (uint16_t i = 0; i < len; ++i) s += buf[i];
  return (unsigned long)s;
}

unsigned long compute_nonzero_count(uint8_t *buf, uint16_t len) {
  unsigned long c = 0;
  for (uint16_t i = 0; i < len; ++i) if (buf[i] != 0) c++;
  return c;
}

// -------------------------------------------------------------
// Command parser (FD FC FB FA ... 04 03 02 01)
// -------------------------------------------------------------
void resetCmdParser() {
  cmdParseState = CMD_SEARCH_HEADER;
  cmdHeaderMatchIdx = 0;
  cmdFooterMatchIdx = 0;
  cmdPayloadLen = 0;
  cmdPayloadIdx = 0;
  cmdLenBytes[0] = 0;
  cmdLenBytes[1] = 0;
}

void feedByteToCmdParser(uint8_t b) {
  switch (cmdParseState) {
    case CMD_SEARCH_HEADER:
      if (b == CMD_HEADER[cmdHeaderMatchIdx]) {
        cmdHeaderMatchIdx++;
        if (cmdHeaderMatchIdx == 4) {
          cmdParseState = CMD_READ_LEN;
          cmdHeaderMatchIdx = 0;
          cmdLenBytes[0] = 0;
          cmdLenBytes[1] = 0;
        }
      } else {
        cmdHeaderMatchIdx = (b == CMD_HEADER[0]) ? 1 : 0;
      }
      break;

    case CMD_READ_LEN:
      if (cmdLenBytes[0] == 0 && cmdLenBytes[1] == 0) {
        cmdLenBytes[0] = b;
      } else {
        cmdLenBytes[1] = b;
        cmdPayloadLen = (uint16_t)cmdLenBytes[0] | ((uint16_t)cmdLenBytes[1] << 8);
        cmdLenBytes[0] = 0;
        cmdLenBytes[1] = 0;

        if (cmdPayloadLen == 0) {
          cmdPayloadIdx = 0;
          cmdParseState = CMD_READ_FOOTER;
          cmdFooterMatchIdx = 0;
        } else if (cmdPayloadLen <= MAX_PAYLOAD) {
          cmdPayloadIdx = 0;
          cmdParseState = CMD_READ_PAYLOAD;
        } else {
          Serial.printf("[CMD-PARSER] Invalid payloadLen=%u, resync\n", (unsigned)cmdPayloadLen);
          resetCmdParser();
        }
      }
      break;

    case CMD_READ_PAYLOAD:
      cmdPayloadBuf[cmdPayloadIdx++] = b;
      if (cmdPayloadIdx >= cmdPayloadLen) {
        cmdParseState = CMD_READ_FOOTER;
        cmdFooterMatchIdx = 0;
      }
      break;

    case CMD_READ_FOOTER:
      if (b == CMD_FOOTER[cmdFooterMatchIdx]) {
        cmdFooterMatchIdx++;
        if (cmdFooterMatchIdx == 4) {
          // Full command frame decoded; mostly ACK/debug visibility
          Serial.printf("\n[CMD-FRAME] payloadLen=%u payload:", (unsigned)cmdPayloadLen);
          for (uint16_t i = 0; i < cmdPayloadLen; ++i) {
            Serial.printf(" %02X", cmdPayloadBuf[i]);
          }
          Serial.println();
          resetCmdParser();
        }
      } else {
        Serial.println("[CMD-PARSER] Footer mismatch, resync");
        resetCmdParser();
        if (b == CMD_HEADER[0]) cmdHeaderMatchIdx = 1;
      }
      break;
  }
}


// -------------------------------------------------------------
// Process report payload (target/gate data)
// -------------------------------------------------------------
void processReportFrame(uint8_t *buf, uint16_t len) {
  validFrames++;

  ParsedLd2410Report r;
  if (parseLd2410DataPayload(buf, len, r)) {
    // Authoritative detection from status (Table 12)
    personDetected = (r.targetState >= 1 && r.targetState <= 3);

    // Keep metric buffer for smoothing/telemetry
    unsigned long metric = 0;
    if (r.enhanced && metricMode == PAYLOAD_SUM) {
      // sum moving + stationary gate energies
      for (uint8_t i = 0; i <= r.movingGateMax; i++) metric += r.movingGateEnergy[i];
      for (uint8_t i = 0; i <= r.stationaryGateMax; i++) metric += r.stationaryGateEnergy[i];
    } else if (metricMode == NONZERO_COUNT) {
      metric = (r.movingEnergy > 0) + (r.stationaryEnergy > 0);
    } else {
      // default: use combined basic energies
      metric = (unsigned long)r.movingEnergy + (unsigned long)r.stationaryEnergy;
    }

    lastFrameMetric = metric;
    addMetric(metric);
    unsigned long avg = metricsAverage();

    // Optional override with baseline mode:
    if (hasBaseline && autoBaselineEnabled) {
      personDetected = personDetected || (avg > (unsigned long)(baselineValue * baselineFactor));
    } else {
      personDetected = personDetected || (avg >= personThreshold);
    }

    engineeringDetected = r.enhanced;

    Serial.printf("\n[PARSED] type=%02X state=%u mDist=%u mE=%u sDist=%u sE=%u d=%u eng=%s metric=%lu avg=%lu detected=%s\n",
                  r.type, r.targetState, r.movingDistanceCm, r.movingEnergy,
                  r.stationaryDistanceCm, r.stationaryEnergy, r.detectionDistanceCm,
                  r.enhanced ? "YES" : "no", metric, avg, personDetected ? "YES" : "no");
    return;
  }

  // fallback if parser fails (keep old logic)
  Serial.printf("\n[PARSE-FAIL] len=%u (fallback path)\n", (unsigned)len);
  unsigned long metric = compute_payload_sum(buf, len);
  lastFrameMetric = metric;
  addMetric(metric);
  unsigned long avg = metricsAverage();
  personDetected = (avg >= personThreshold);
}

void resetDataParser() {
  dataParseState = D_SEARCH_HEADER;
  dataHeaderMatchIdx = 0;
  dataExpectedLen = 0;
  dataReadCount = 0;
}

bool dataTailValid() {
  for (int i = 0; i < 4; ++i) {
    if (dataTailBuf[i] != DATA_TAIL[i]) return false;
  }
  return true;
}

void feedByteToDataParser(uint8_t b) {
  switch (dataParseState) {
    case D_SEARCH_HEADER:
      if (b == DATA_HEADER[dataHeaderMatchIdx]) {
        dataHeaderMatchIdx++;
        if (dataHeaderMatchIdx == 4) {
          dataParseState = D_READ_LEN_0;
          dataHeaderMatchIdx = 0;
        }
      } else {
        dataHeaderMatchIdx = (b == DATA_HEADER[0]) ? 1 : 0;
      }
      break;

    case D_READ_LEN_0:
      dataExpectedLen = b; // low byte
      dataParseState = D_READ_LEN_1;
      break;

    case D_READ_LEN_1:
      dataExpectedLen |= ((uint16_t)b << 8); // high byte
      if (dataExpectedLen == 0 || dataExpectedLen > DATA_PAYLOAD_MAX) {
        Serial.printf("[DATA-PARSER] invalid payload len=%u, resync\n", (unsigned)dataExpectedLen);
        resetDataParser();
      } else {
        dataReadCount = 0;
        dataParseState = D_READ_PAYLOAD_AND_TAIL;
      }
      break;

    case D_READ_PAYLOAD_AND_TAIL:
      // First read payload bytes, then 4 tail bytes
      if (dataReadCount < dataExpectedLen) {
        dataPayloadBuf[dataReadCount] = b;
      } else if (dataReadCount < dataExpectedLen + 4) {
        dataTailBuf[dataReadCount - dataExpectedLen] = b;
      }

      dataReadCount++;

      if (dataReadCount >= dataExpectedLen + 4) {
        if (dataTailValid()) {
          // Pass parsed payload to your inner parser
          processReportFrame(dataPayloadBuf, dataExpectedLen);
        } else {
          Serial.println("[DATA-PARSER] tail mismatch");
        }
        resetDataParser();
      }
      break;
  }
}

// -------------------------------------------------------------
// Commands
// -------------------------------------------------------------
void sendEnableEngineeringModeOnce(unsigned long timeoutMs) {
  // Send: FD FC FB FA 02 00 62 00 04 03 02 01
  uint8_t cmd[] = { 0xFD,0xFC,0xFB,0xFA, 0x02,0x00, 0x62,0x00, 0x04,0x03,0x02,0x01 };
  ld2410.write(cmd, sizeof(cmd));
  Serial.println("[CMD] Sent enable-engineering command; waiting for ACK...");

  const uint8_t ACK_PREFIX[] = {0xFD,0xFC,0xFB,0xFA, 0x04,0x00, 0x62,0x01};
  int match = 0;
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    if (ld2410.available()) {
      uint8_t b = ld2410.read();
      if (b == ACK_PREFIX[match]) {
        match++;
        if (match == (int)sizeof(ACK_PREFIX)) {
          unsigned long t0 = millis();
          while (ld2410.available() < 2 && millis() - t0 < 200) delay(5);
          if (ld2410.available() >= 2) {
            uint8_t s0 = ld2410.read();
            uint8_t s1 = ld2410.read();
            uint16_t status = (uint16_t)s0 | ((uint16_t)s1 << 8);
            if (status == 0) {
              Serial.println("[ACK] Engineering mode enabled.");
              engineeringDetected = true;
              metricMode = PAYLOAD_SUM;
              metricsIdx = 0; metricsCount = 0; metricsSum = 0;
            } else {
              Serial.printf("[ACK] Enable engineering failed, status=%u\n", (unsigned)status);
            }
            return;
          } else {
            Serial.println("[ACK] Prefix matched but status bytes missing");
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
  Serial.println("[CMD] No ACK for enable-engineering (timeout).");
}

void sendRestartModuleOnce(unsigned long timeoutMs) {
  // Per your pasted page 16-17:
  // Send: FD FC FB FA 02 00 A3 00 04 03 02 01
  uint8_t cmd[] = { 0xFD,0xFC,0xFB,0xFA, 0x02,0x00, 0xA3,0x00, 0x04,0x03,0x02,0x01 };
  ld2410.write(cmd, sizeof(cmd));
  Serial.println("[CMD] Sent restart command; waiting for ACK...");

  const uint8_t ACK_PREFIX[] = {0xFD,0xFC,0xFB,0xFA, 0x04,0x00, 0xA3,0x01};
  int match = 0;
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    if (ld2410.available()) {
      uint8_t b = ld2410.read();
      if (b == ACK_PREFIX[match]) {
        match++;
        if (match == (int)sizeof(ACK_PREFIX)) {
          unsigned long t0 = millis();
          while (ld2410.available() < 2 && millis() - t0 < 200) delay(5);
          if (ld2410.available() >= 2) {
            uint8_t s0 = ld2410.read();
            uint8_t s1 = ld2410.read();
            uint16_t status = (uint16_t)s0 | ((uint16_t)s1 << 8);
            Serial.printf("[ACK] Restart status=%u\n", (unsigned)status);
            return;
          } else {
            Serial.println("[ACK] Restart prefix matched but status missing");
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
  Serial.println("[CMD] No ACK for restart (timeout).");
}

// -------------------------------------------------------------
// Calibration
// -------------------------------------------------------------
void runCalibration(int captureFrames) {
  if (captureFrames < 1) captureFrames = 1;
  Serial.printf("[CAL] Capturing %d frames for baseline (stand away)...\n", captureFrames);

  metricsIdx = 0;
  metricsCount = 0;
  metricsSum = 0;

  unsigned long start = millis();
  unsigned long timeoutMs = 6000 + (unsigned long)captureFrames * 500;

  // Wait until enough report frames contributed metrics
  while ((metricsCount < captureFrames) && (millis() - start < timeoutMs)) {
    // let loop continue reading serial
    delay(20);
  }

  if (metricsCount > 0) {
    baselineValue = metricsAverage();
    hasBaseline = true;
    Serial.printf("[CAL] Done: baseline=%lu factor=%.2f\n", baselineValue, baselineFactor);
    statusMessage = "Calibration done";
  } else {
    Serial.println("[CAL] Failed: no frames captured");
    statusMessage = "Calibration failed";
  }

  // clear smoothing buffer after calibration
  metricsIdx = 0;
  metricsCount = 0;
  metricsSum = 0;
}

// -------------------------------------------------------------
// UI
// -------------------------------------------------------------
void drawInterface() {
  canvas.fillSprite(BLACK);

  canvas.setCursor(5, 5);
  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.println("LD2410C Interactive Scanner");
  canvas.drawFastHLine(0, 18, canvas.width() - 10, WHITE);

  canvas.setCursor(5, 24);
  canvas.setTextColor(WHITE);
  canvas.setTextSize(1);
  canvas.printf("W/Q Th+/-  A/S Win+/-  C Mode  E EngOn  R Restart  Space Cal");

  const char* modeName = (metricMode == INTERFRAME_DIFF) ? "INTER_DIFF" :
                         (metricMode == PAYLOAD_SUM) ? "PAYLOAD_SUM" : "NONZERO";

  canvas.setCursor(5, 38);
  canvas.setTextColor(YELLOW);
  canvas.printf("Mode:%s  Win:%d  Th:%lu", modeName, personWindowFrames, personThreshold);

  canvas.setCursor(5, 52);
  canvas.setTextColor(WHITE);
  canvas.printf("Eng:%s Baseline:%lu Last:%lu", engineeringDetected ? "YES" : "no", baselineValue, lastFrameMetric);

  canvas.setTextSize(2);
  canvas.setCursor(5, 84);
  if (personDetected) {
    canvas.setTextColor(RED);
    canvas.printf("Person: YES");
  } else {
    canvas.setTextColor(GREEN);
    canvas.printf("Person: NO ");
  }

  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.setCursor(5, canvas.height() - 14);
  canvas.printf("Status: %s", statusMessage.c_str());

  canvas.pushSprite(2, 2);
}

// -------------------------------------------------------------
// Setup / Loop
// -------------------------------------------------------------
void setup() {
  Serial.begin(LOG_BAUD);
  Serial.println("\n=== CARDPUTER + LD2410C - CMD+REPORT PARSER ===");

  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);
  M5Cardputer.Display.setRotation(1);

  uint16_t w = M5Cardputer.Display.width();
  uint16_t h = M5Cardputer.Display.height();
  canvas.createSprite(w - 4, h - 4);

  Serial.printf("[UART] Sensor UART init @ %d\n", SENSOR_BAUD);
  ld2410.begin(SENSOR_BAUD, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);

  resetCmdParser();
  resetMetricsBuffer("Ready");
  statusMessage = "Ready";

  Serial.println("[INIT] Controls: W/Q Th+/- | A/S Win+/- | C Mode | E Enable Eng | R Restart | Space Cal | Esc Reboot");
}

void loop() {
  M5Cardputer.update();

  while (ld2410.available()) {
    uint8_t b = (uint8_t)ld2410.read();
    byteCount++;
    anyDataReceived = true;
    dataTimestamp = millis();

    // Optional raw hex stream debug
    if (b < 16) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ");
    if (byteCount % 32 == 0) Serial.println();

    // Feed both parsers
    feedByteToCmdParser(b);  // for command/ACK frames
    feedByteToDataParser(b);      // for report data frames (0xAA ... 0x55)
  }

  if (anyDataReceived && millis() - dataTimestamp > 2000) {
    statusMessage = "No recent data...";
  }

  // Keyboard controls
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
            if (personWindowFrames > 1) personWindowFrames = max(1, personWindowFrames / 2);
            metricsIdx = 0; metricsCount = 0; metricsSum = 0;
            statusMessage = "Window decreased";
          } else if (k == 's' || k == 'S') {
            if (personWindowFrames < MAX_WINDOW) personWindowFrames = min(MAX_WINDOW, personWindowFrames * 2);
            metricsIdx = 0; metricsCount = 0; metricsSum = 0;
            statusMessage = "Window increased";
          } else if (k == 'c' || k == 'C') {
            metricMode = MetricMode((metricMode + 1) % 3);
            metricsIdx = 0; metricsCount = 0; metricsSum = 0;
            statusMessage = "Metric mode changed";
          } else if (k == 'e' || k == 'E') {
            sendEnableEngineeringModeOnce(1000);
            statusMessage = "Enable engineering sent";
          } else if (k == 'r' || k == 'R') {
            sendRestartModuleOnce(1000);
            statusMessage = "Restart command sent";
          } else if (k == ' ') {
            runCalibration(8);
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
