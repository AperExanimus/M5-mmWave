/*
  cardputeradv_mmwave.ino
  Final hybrid parser build for LD2410 variants on CardPuter

  Features:
  - Command/ACK parser: FD FC FB FA ... 04 03 02 01
  - Strict data parser:  F4 F3 F2 F1 + len(LE) + payload + F8 F7 F6 F5
  - Auto-fallback parser: scans stream for [type(01/02),AA,...,55,00]
  - MyLD2410-compatible payload decode
  - Presence from target state + metric support
  - UI + runtime controls

  Keys:
    E : send engineering ON command (0x62)
    N : send engineering OFF command (0x63)
    R : send reboot command (0xA3)
    C : cycle metric mode
    W/Q : threshold +/- 
    A/S : window /2 or *2
    Space : calibration
    Esc : reboot CardPuter
*/

#include <M5Cardputer.h>
#include <Wire.h>

// ============================================================
// Pins / serial
// ============================================================
#define SENSOR_RX_PIN 15
#define SENSOR_TX_PIN 13

const unsigned long LOG_BAUD = 115200;
const int SENSOR_BAUD = 256000;

HardwareSerial ld2410(1);
M5Canvas canvas(&M5Cardputer.Display);

// ============================================================
// Protocol constants
// ============================================================
// Config/ACK envelope
const uint8_t CFG_HEAD[4] = {0xFD, 0xFC, 0xFB, 0xFA};
const uint8_t CFG_TAIL[4] = {0x04, 0x03, 0x02, 0x01};

// Strict data envelope (MyLD2410)
const uint8_t DATA_HEAD[4] = {0xF4, 0xF3, 0xF2, 0xF1};
const uint8_t DATA_TAIL[4] = {0xF8, 0xF7, 0xF6, 0xF5};

const size_t MAX_FRAME = 512;

// ============================================================
// UI / runtime state
// ============================================================
String statusMessage = "Ready";
unsigned long lastKeyTime = 0;
const unsigned long KEY_DELAY_MS = 180;

bool anyDataReceived = false;
unsigned long lastDataMs = 0;
unsigned long rawByteCount = 0;

// Parser health
unsigned long parsedDataFrames = 0;
unsigned long parsedAckFrames = 0;
unsigned long parseFailCount = 0;
unsigned long lastParsedMs = 0;
bool fallbackMode = false;   // auto enabled if strict parser starves
const unsigned long FALLBACK_TIMEOUT_MS = 1500;

// ============================================================
// Detection / metric
// ============================================================
enum MetricMode { METRIC_STATUS = 0, METRIC_BASE_ENERGY = 1, METRIC_GATE_SUM = 2 };
MetricMode metricMode = METRIC_STATUS;

int personWindowFrames = 8;
const int MAX_WINDOW = 512;
unsigned long metricBuf[MAX_WINDOW];
int metricIdx = 0;
int metricCount = 0;
unsigned long metricSum = 0;

unsigned long personThreshold = 1; // default works for METRIC_STATUS
bool autoBaselineEnabled = true;
float baselineFactor = 1.8f;
bool hasBaseline = false;
unsigned long baselineValue = 0;
unsigned long lastMetric = 0;

bool personDetected = false;
bool enhancedModeSeen = false;

// ============================================================
// Parsed payload model
// ============================================================
struct ParsedData {
  bool ok = false;
  bool enhanced = false;    // type=0x01
  uint8_t type = 0;         // 0x01 enhanced, 0x02 normal
  uint8_t status = 0xFF;    // 0=no target,1=moving,2=stationary,3=both

  uint16_t mDist = 0;
  uint8_t  mSig = 0;
  uint16_t sDist = 0;
  uint8_t  sSig = 0;
  uint16_t dist = 0;

  uint8_t mN = 0;
  uint8_t sN = 0;
  uint8_t mG[9] = {0};
  uint8_t sG[9] = {0};
  uint8_t light = 0;
  uint8_t out = 0;
} gData;

// ============================================================
// Generic strict frame parser state
// ============================================================
enum ParseState { SEARCH_HEAD, READ_LEN0, READ_LEN1, READ_BODY };
struct StreamParser {
  ParseState state = SEARCH_HEAD;
  uint8_t headMatch = 0;
  uint16_t expectedLen = 0;
  uint16_t readCount = 0;
  uint8_t payload[MAX_FRAME];
  uint8_t tail[4];
};

StreamParser cfgParser;
StreamParser dataParser;

// ============================================================
// Fallback extractor ring buffer
// ============================================================
const size_t RAW_RING_MAX = 1024;
uint8_t rawRing[RAW_RING_MAX];
size_t rawLen = 0;

// ============================================================
// Helpers
// ============================================================
void metricReset() {
  metricIdx = 0; metricCount = 0; metricSum = 0;
}
void metricAdd(unsigned long v) {
  if (personWindowFrames < 1) personWindowFrames = 1;
  if (personWindowFrames > MAX_WINDOW) personWindowFrames = MAX_WINDOW;

  if (metricCount < personWindowFrames) {
    metricBuf[metricIdx] = v;
    metricSum += v;
    metricIdx = (metricIdx + 1) % personWindowFrames;
    metricCount++;
  } else {
    int rep = metricIdx % personWindowFrames;
    metricSum -= metricBuf[rep];
    metricBuf[rep] = v;
    metricSum += v;
    metricIdx = (rep + 1) % personWindowFrames;
  }
}
unsigned long metricAvg() {
  return (metricCount == 0) ? 0 : metricSum / metricCount;
}
bool arr4eq(const uint8_t *a, const uint8_t *b) {
  for (int i = 0; i < 4; ++i) if (a[i] != b[i]) return false;
  return true;
}
void resetParser(StreamParser &ps) {
  ps.state = SEARCH_HEAD;
  ps.headMatch = 0;
  ps.expectedLen = 0;
  ps.readCount = 0;
}

// ============================================================
// Payload parser (MyLD2410-compatible)
// ============================================================
bool parseDataPayload(const uint8_t *p, size_t len, ParsedData &out) {
  out = ParsedData();

  if (!p || len < 11) return false;
  if (!((p[0] == 0x01) || (p[0] == 0x02))) return false;
  if (p[1] != 0xAA) return false;

  out.ok = true;
  out.type = p[0];
  out.enhanced = (p[0] == 0x01);
  out.status = p[2] & 0x07;

  out.mDist = (uint16_t)p[3] | ((uint16_t)p[4] << 8);
  out.mSig  = p[5];
  out.sDist = (uint16_t)p[6] | ((uint16_t)p[7] << 8);
  out.sSig  = p[8];
  out.dist  = (uint16_t)p[9] | ((uint16_t)p[10] << 8);

  if (!out.enhanced) return true;

  if (len < 13) return false;
  uint8_t mN = p[11]; if (mN > 8) mN = 8;
  uint8_t sN = p[12]; if (sN > 8) sN = 8;
  out.mN = mN; out.sN = sN;

  size_t idx = 13;
  if (idx + (size_t)mN + 1 > len) return false;
  for (uint8_t i = 0; i <= mN; ++i) out.mG[i] = p[idx++];

  if (idx + (size_t)sN + 1 > len) return false;
  for (uint8_t i = 0; i <= sN; ++i) out.sG[i] = p[idx++];

  if (idx < len) out.light = p[idx++];
  if (idx < len) out.out = p[idx++];

  return true;
}

// ============================================================
// Detection update
// ============================================================
void updateDetection(const ParsedData &pd) {
  bool statusPresence = (pd.status >= 1 && pd.status <= 3);

  unsigned long metric = 0;
  switch (metricMode) {
    case METRIC_STATUS:
      metric = statusPresence ? 1 : 0;
      break;
    case METRIC_BASE_ENERGY:
      metric = (unsigned long)pd.mSig + (unsigned long)pd.sSig;
      break;
    case METRIC_GATE_SUM:
      if (pd.enhanced) {
        for (uint8_t i = 0; i <= pd.mN; ++i) metric += pd.mG[i];
        for (uint8_t i = 0; i <= pd.sN; ++i) metric += pd.sG[i];
      } else {
        metric = (unsigned long)pd.mSig + (unsigned long)pd.sSig;
      }
      break;
  }

  lastMetric = metric;
  metricAdd(metric);
  unsigned long avg = metricAvg();

  bool metricPresence = false;
  if (hasBaseline && autoBaselineEnabled) metricPresence = (avg > (unsigned long)(baselineValue * baselineFactor));
  else metricPresence = (avg >= personThreshold);

  personDetected = statusPresence || metricPresence;

  Serial.printf("\n[PARSED] type=%02X st=%u mD=%u mE=%u sD=%u sE=%u d=%u enh=%s metric=%lu avg=%lu det=%s\n",
                pd.type, pd.status, pd.mDist, pd.mSig, pd.sDist, pd.sSig, pd.dist,
                pd.enhanced ? "YES" : "no", metric, avg, personDetected ? "YES" : "no");
}

// ============================================================
// Frame handlers
// ============================================================
void onCfgFrame(const uint8_t *payload, uint16_t len) {
  parsedAckFrames++;
  Serial.printf("\n[ACK] len=%u:", (unsigned)len);
  for (uint16_t i = 0; i < len; ++i) Serial.printf(" %02X", payload[i]);
  Serial.println();

  if (len >= 4) {
    uint16_t cmd = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    uint16_t st  = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
    Serial.printf("[ACK] cmd=0x%04X status=%u\n", cmd, st);
  }
}

void onDataPayload(const uint8_t *payload, uint16_t len) {
  ParsedData p;
  if (parseDataPayload(payload, len, p)) {
    gData = p;
    enhancedModeSeen = p.enhanced;
    parsedDataFrames++;
    lastParsedMs = millis();
    updateDetection(p);
  } else {
    parseFailCount++;
  }
}

void onDataFrame(const uint8_t *payload, uint16_t len) {
  // strict frame payload -> parse directly
  onDataPayload(payload, len);
}

// ============================================================
// Strict parser feed
// ============================================================
void feedParserByte(
    StreamParser &ps,
    uint8_t b,
    const uint8_t *head,
    const uint8_t *tail,
    void (*onFrame)(const uint8_t*, uint16_t)) {

  switch (ps.state) {
    case SEARCH_HEAD:
      if (b == head[ps.headMatch]) {
        ps.headMatch++;
        if (ps.headMatch == 4) {
          ps.state = READ_LEN0;
          ps.headMatch = 0;
        }
      } else {
        ps.headMatch = (b == head[0]) ? 1 : 0;
      }
      break;

    case READ_LEN0:
      ps.expectedLen = b;
      ps.state = READ_LEN1;
      break;

    case READ_LEN1:
      ps.expectedLen |= ((uint16_t)b << 8);
      if (ps.expectedLen == 0 || ps.expectedLen > MAX_FRAME) {
        resetParser(ps);
      } else {
        ps.readCount = 0;
        ps.state = READ_BODY;
      }
      break;

    case READ_BODY:
      if (ps.readCount < ps.expectedLen) {
        ps.payload[ps.readCount] = b;
      } else if (ps.readCount < ps.expectedLen + 4) {
        ps.tail[ps.readCount - ps.expectedLen] = b;
      }
      ps.readCount++;

      if (ps.readCount >= ps.expectedLen + 4) {
        if (arr4eq(ps.tail, tail)) onFrame(ps.payload, ps.expectedLen);
        else parseFailCount++;
        resetParser(ps);
      }
      break;
  }
}

// ============================================================
// Fallback extractor: detect [01/02, AA, ..., 55, 00]
// ============================================================
void feedByteToFallbackExtractor(uint8_t b) {
  if (rawLen < RAW_RING_MAX) rawRing[rawLen++] = b;
  else {
    memmove(rawRing, rawRing + 1, RAW_RING_MAX - 1);
    rawRing[RAW_RING_MAX - 1] = b;
    rawLen = RAW_RING_MAX;
  }

  // scan packet candidates
  for (size_t i = 1; i + 4 < rawLen; ++i) {
    uint8_t type = rawRing[i - 1];
    if (!((type == 0x01) || (type == 0x02))) continue;
    if (rawRing[i] != 0xAA) continue;

    for (size_t j = i + 1; j + 1 < rawLen; ++j) {
      if (rawRing[j] == 0x55 && rawRing[j + 1] == 0x00) {
        size_t start = i - 1;
        size_t end = j + 1;
        size_t pktLen = end - start + 1;
        if (pktLen >= 11 && pktLen <= 160) {
          uint8_t tmp[160];
          memcpy(tmp, rawRing + start, pktLen);
          onDataPayload(tmp, (uint16_t)pktLen);

          // consume through end
          size_t remain = rawLen - (end + 1);
          memmove(rawRing, rawRing + end + 1, remain);
          rawLen = remain;
          return;
        }
      }
    }
  }
}

// ============================================================
// Commands
// ============================================================
void sendCfgCommandRaw(const uint8_t *cmd, uint8_t cmdLen) {
  ld2410.write(CFG_HEAD, 4);
  ld2410.write(cmd, cmdLen);
  ld2410.write(CFG_TAIL, 4);
  ld2410.flush();
}

void sendEnableEngineering() {
  const uint8_t cmd[] = {0x02, 0x00, 0x62, 0x00};
  sendCfgCommandRaw(cmd, sizeof(cmd));
  statusMessage = "Sent eng ON";
  Serial.println("[CMD] eng ON sent");
}
void sendDisableEngineering() {
  const uint8_t cmd[] = {0x02, 0x00, 0x63, 0x00};
  sendCfgCommandRaw(cmd, sizeof(cmd));
  statusMessage = "Sent eng OFF";
  Serial.println("[CMD] eng OFF sent");
}
void sendRebootSensor() {
  const uint8_t cmd[] = {0x02, 0x00, 0xA3, 0x00};
  sendCfgCommandRaw(cmd, sizeof(cmd));
  statusMessage = "Sent sensor reboot";
  Serial.println("[CMD] sensor reboot sent");
}

// ============================================================
// Calibration
// ============================================================
void runCalibration(int frames = 12) {
  if (frames < 1) frames = 1;
  metricReset();
  Serial.printf("[CAL] collecting %d parsed frames...\n", frames);

  unsigned long start = millis();
  unsigned long timeout = 10000;
  int before = metricCount;

  while ((metricCount - before) < frames && (millis() - start) < timeout) {
    M5Cardputer.update();
    while (ld2410.available()) {
      uint8_t b = (uint8_t)ld2410.read();
      rawByteCount++;
      anyDataReceived = true;
      lastDataMs = millis();

      feedParserByte(cfgParser, b, CFG_HEAD, CFG_TAIL, onCfgFrame);
      if (!fallbackMode) feedParserByte(dataParser, b, DATA_HEAD, DATA_TAIL, onDataFrame);
      else feedByteToFallbackExtractor(b);
    }

    if (!fallbackMode && (millis() - lastParsedMs > FALLBACK_TIMEOUT_MS)) {
      fallbackMode = true;
      statusMessage = "Fallback parser ON";
      Serial.println("[PARSER] fallback enabled during calibration");
    }
    delay(5);
  }

  if (metricCount > 0) {
    baselineValue = metricAvg();
    hasBaseline = true;
    Serial.printf("[CAL] baseline=%lu factor=%.2f\n", baselineValue, baselineFactor);
    statusMessage = "Calibration done";
  } else {
    Serial.println("[CAL] failed (no parsed frames)");
    statusMessage = "Calibration failed";
  }
  metricReset();
}

// ============================================================
// UI
// ============================================================
void drawUI() {
  canvas.fillSprite(BLACK);
  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);

  const char* modeName =
      (metricMode == METRIC_STATUS) ? "STATUS" :
      (metricMode == METRIC_BASE_ENERGY) ? "BASE_E" : "GATE_SUM";

  canvas.setCursor(4, 4);
  canvas.printf("LD2410 Hybrid Parser");

  canvas.setCursor(4, 16);
  canvas.printf("Mode:%s Win:%d Thr:%lu", modeName, personWindowFrames, personThreshold);

  canvas.setCursor(4, 28);
  canvas.printf("Base:%lu Eng:%s Fallback:%s",
                baselineValue,
                enhancedModeSeen ? "Y" : "N",
                fallbackMode ? "ON" : "off");

  canvas.setCursor(4, 40);
  canvas.printf("ParsedD:%lu ACK:%lu Fail:%lu",
                parsedDataFrames, parsedAckFrames, parseFailCount);

  canvas.setCursor(4, 52);
  canvas.printf("st:%u mD:%u sD:%u d:%u", gData.status, gData.mDist, gData.sDist, gData.dist);

  canvas.setCursor(4, 64);
  canvas.printf("mE:%u sE:%u lastM:%lu", gData.mSig, gData.sSig, lastMetric);

  canvas.setTextSize(2);
  canvas.setCursor(4, 84);
  if (personDetected) {
    canvas.setTextColor(RED);
    canvas.print("Person: YES");
  } else {
    canvas.setTextColor(GREEN);
    canvas.print("Person: NO ");
  }

  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.setCursor(4, canvas.height() - 14);
  canvas.printf("%s", statusMessage.c_str());

  canvas.pushSprite(2, 2);
}

// ============================================================
// setup / loop
// ============================================================
void setup() {
  Serial.begin(LOG_BAUD);
  Serial.println("\n=== CardPuter LD2410 Final Hybrid ===");

  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);
  M5Cardputer.Display.setRotation(1);

  uint16_t w = M5Cardputer.Display.width();
  uint16_t h = M5Cardputer.Display.height();
  canvas.createSprite(w - 4, h - 4);

  ld2410.begin(SENSOR_BAUD, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);

  resetParser(cfgParser);
  resetParser(dataParser);
  metricReset();
  rawLen = 0;

  lastParsedMs = millis();
  statusMessage = "Ready: E/N/R C WQ AS Space Esc";

  Serial.println("[INIT] keys: E engON, N engOFF, R reboot sensor, C mode, W/Q thr, A/S window, Space calib, Esc reboot");
}

void loop() {
  M5Cardputer.update();

  while (ld2410.available()) {
    uint8_t b = (uint8_t)ld2410.read();

    rawByteCount++;
    anyDataReceived = true;
    lastDataMs = millis();

    // raw hex output
    if (b < 16) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ");
    if (rawByteCount % 32 == 0) Serial.println();

    // always parse config ack frames
    feedParserByte(cfgParser, b, CFG_HEAD, CFG_TAIL, onCfgFrame);

    // data parser path
    if (!fallbackMode) feedParserByte(dataParser, b, DATA_HEAD, DATA_TAIL, onDataFrame);
    else feedByteToFallbackExtractor(b);
  }

  // auto fallback when strict parser starves
  if (!fallbackMode && (millis() - lastParsedMs > FALLBACK_TIMEOUT_MS)) {
    fallbackMode = true;
    statusMessage = "Fallback parser ON";
    Serial.println("\n[PARSER] strict parser starved -> fallback ON");
  }

  if (anyDataReceived && (millis() - lastDataMs > 2000)) {
    statusMessage = "No recent data";
  }

  // keyboard controls
  if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
    auto ks = M5Cardputer.Keyboard.keysState();
    for (auto k : ks.word) {
      if (millis() - lastKeyTime < KEY_DELAY_MS) continue;
      lastKeyTime = millis();

      if (k == 'e' || k == 'E') sendEnableEngineering();
      else if (k == 'n' || k == 'N') sendDisableEngineering();
      else if (k == 'r' || k == 'R') sendRebootSensor();
      else if (k == 'c' || k == 'C') {
        metricMode = (MetricMode)((metricMode + 1) % 3);
        metricReset();
        statusMessage = "Metric mode changed";
      } else if (k == 'w' || k == 'W') {
        personThreshold++;
        statusMessage = "Threshold++";
      } else if (k == 'q' || k == 'Q') {
        if (personThreshold > 0) personThreshold--;
        statusMessage = "Threshold--";
      } else if (k == 'a' || k == 'A') {
        personWindowFrames = max(1, personWindowFrames / 2);
        metricReset();
        statusMessage = "Window down";
      } else if (k == 's' || k == 'S') {
        personWindowFrames = min(MAX_WINDOW, personWindowFrames * 2);
        metricReset();
        statusMessage = "Window up";
      } else if (k == ' ') {
        runCalibration(12);
      } else if (k == '\x1B') {
        esp_restart();
      }
    }
  }

  drawUI();
}
