#include <M5Cardputer.h>
#include <Wire.h>

// ============================================================
// CardPuter + LD2410 (MyLD2410-compatible frame parsing)
// ============================================================
#define SENSOR_RX_PIN 15   // CardPuter GPIO15 <- Sensor TX
#define SENSOR_TX_PIN 13   // CardPuter GPIO13 -> Sensor RX

const unsigned long LOG_BAUD = 115200;
const int SENSOR_BAUD = 256000; // LD2410 default

HardwareSerial ld2410(1);
M5Canvas canvas(&M5Cardputer.Display);

// ---------------- UI / state ----------------
String statusMessage = "Ready";
unsigned long lastKeyTime = 0;
const unsigned long KEY_DELAY_MS = 180;

bool anyDataReceived = false;
unsigned long lastDataMs = 0;
unsigned long frameCount = 0;

// ---------------- protocol constants ----------------
// Config/ACK frames
const uint8_t CFG_HEAD[4] = {0xFD, 0xFC, 0xFB, 0xFA};
const uint8_t CFG_TAIL[4] = {0x04, 0x03, 0x02, 0x01};

// Data frames (MyLD2410)
const uint8_t DATA_HEAD[4] = {0xF4, 0xF3, 0xF2, 0xF1};
const uint8_t DATA_TAIL[4] = {0xF8, 0xF7, 0xF6, 0xF5};

const size_t MAX_FRAME = 512;

// ---------------- detection / metrics ----------------
enum MetricMode { METRIC_STATUS = 0, METRIC_BASE_ENERGY = 1, METRIC_GATE_SUM = 2 };
MetricMode metricMode = METRIC_STATUS;

int personWindowFrames = 8;
const int MAX_WINDOW = 512;
unsigned long metricBuf[MAX_WINDOW];
int metricIdx = 0, metricCount = 0;
unsigned long metricSum = 0;

unsigned long personThreshold = 1; // for non-status metrics
bool autoBaselineEnabled = true;
float baselineFactor = 1.8f;
bool hasBaseline = false;
unsigned long baselineValue = 0;
unsigned long lastMetric = 0;

bool personDetected = false;
bool enhancedModeSeen = false;

// ---------------- parsed report data ----------------
struct ParsedData {
  bool ok = false;
  bool enhanced = false;    // type=0x01
  uint8_t type = 0;
  uint8_t status = 0xFF;    // 0..3 valid target states

  uint16_t mDist = 0;       // moving distance cm
  uint8_t  mSig = 0;        // moving energy
  uint16_t sDist = 0;       // stationary distance cm
  uint8_t  sSig = 0;        // stationary energy
  uint16_t dist = 0;        // detected distance cm

  uint8_t mN = 0;
  uint8_t sN = 0;
  uint8_t mG[9] = {0};      // gate energies moving
  uint8_t sG[9] = {0};      // gate energies stationary
  uint8_t light = 0;
  uint8_t out = 0;
} d;

// ---------------- parser states ----------------
enum ParseState { SEARCH_HEAD, READ_LEN0, READ_LEN1, READ_BODY };
struct StreamParser {
  ParseState state = SEARCH_HEAD;
  uint8_t headMatch = 0;
  uint16_t expectedLen = 0; // payload length
  uint16_t readCount = 0;   // payload + tail bytes read
  uint8_t payload[MAX_FRAME];
  uint8_t tail[4];
};

// one parser for cfg/ack, one for data
StreamParser cfgParser;
StreamParser dataParser;

// ============================================================
// helpers
// ============================================================
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
void metricReset() {
  metricIdx = 0; metricCount = 0; metricSum = 0;
}

bool endsWith4(const uint8_t *arr, const uint8_t *pat) {
  for (int i = 0; i < 4; ++i) if (arr[i] != pat[i]) return false;
  return true;
}

// ============================================================
// payload parse (MyLD2410 offsets)
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
// detection update
// ============================================================
void updateDetectionFromParsed(const ParsedData &pd) {
  // Primary truth from status (table 12): 1,2,3 => target
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

  // combine: status presence is authoritative, metric presence supports robustness
  personDetected = statusPresence || metricPresence;

  Serial.printf(
      "\n[DATA] type=%02X status=%u mDist=%u mSig=%u sDist=%u sSig=%u dist=%u enh=%s metric=%lu avg=%lu detected=%s\n",
      pd.type, pd.status, pd.mDist, pd.mSig, pd.sDist, pd.sSig, pd.dist,
      pd.enhanced ? "YES" : "no", metric, avg, personDetected ? "YES" : "no");
}

// ============================================================
// complete frame handlers
// ============================================================
void onCfgFrame(const uint8_t *payload, uint16_t len) {
  // payload typically: cmd(2) + status(2) + extra...
  Serial.printf("\n[ACK] len=%u payload:", (unsigned)len);
  for (uint16_t i = 0; i < len; ++i) Serial.printf(" %02X", payload[i]);
  Serial.println();

  if (len >= 4) {
    uint16_t cmd = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    uint16_t st  = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
    Serial.printf("[ACK] cmd=0x%04X status=%u\n", cmd, st);
  }
}

void onDataFrame(const uint8_t *payload, uint16_t len) {
  frameCount++;
  anyDataReceived = true;
  lastDataMs = millis();

  ParsedData p;
  if (parseDataPayload(payload, len, p)) {
    d = p;
    enhancedModeSeen = p.enhanced;
    updateDetectionFromParsed(p);
  } else {
    Serial.printf("\n[DATA] parse failed, len=%u\n", (unsigned)len);
  }
}

// ============================================================
// generic parser feed
// ============================================================
void resetParser(StreamParser &ps) {
  ps.state = SEARCH_HEAD;
  ps.headMatch = 0;
  ps.expectedLen = 0;
  ps.readCount = 0;
}

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
        if (endsWith4(ps.tail, tail)) {
          onFrame(ps.payload, ps.expectedLen);
        } else {
          Serial.println("\n[PARSER] tail mismatch");
        }
        resetParser(ps);
      }
      break;
  }
}

// ============================================================
// command send
// ============================================================
void sendCfgCommandRaw(const uint8_t *cmd, uint8_t cmdLen) {
  // cmd already includes: [lenL,lenH,cmdL,cmdH,...]
  ld2410.write(CFG_HEAD, 4);
  ld2410.write(cmd, cmdLen);
  ld2410.write(CFG_TAIL, 4);
  ld2410.flush();
}

void sendEnableEngineering() {
  // 02 00 62 00
  const uint8_t cmd[] = {0x02, 0x00, 0x62, 0x00};
  sendCfgCommandRaw(cmd, sizeof(cmd));
  statusMessage = "Sent eng ON";
  Serial.println("[CMD] enable engineering sent");
}
void sendDisableEngineering() {
  // 02 00 63 00
  const uint8_t cmd[] = {0x02, 0x00, 0x63, 0x00};
  sendCfgCommandRaw(cmd, sizeof(cmd));
  statusMessage = "Sent eng OFF";
  Serial.println("[CMD] disable engineering sent");
}
void sendRebootSensor() {
  // 02 00 A3 00
  const uint8_t cmd[] = {0x02, 0x00, 0xA3, 0x00};
  sendCfgCommandRaw(cmd, sizeof(cmd));
  statusMessage = "Sent sensor reboot";
  Serial.println("[CMD] reboot sensor sent");
}

// ============================================================
// calibration
// ============================================================
void runCalibration(int frames = 12) {
  if (frames < 1) frames = 1;
  metricReset();

  Serial.printf("[CAL] collecting %d frames...\n", frames);
  unsigned long start = millis();
  unsigned long timeout = 8000;

  while (metricCount < frames && (millis() - start) < timeout) {
    // allow loop parsing to continue naturally
    M5Cardputer.update();
    while (ld2410.available()) {
      uint8_t b = (uint8_t)ld2410.read();

      // keep both parsers alive during calibration
      feedParserByte(cfgParser,  b, CFG_HEAD,  CFG_TAIL,  onCfgFrame);
      feedParserByte(dataParser, b, DATA_HEAD, DATA_TAIL, onDataFrame);
    }
    delay(5);
  }

  if (metricCount > 0) {
    baselineValue = metricAvg();
    hasBaseline = true;
    Serial.printf("[CAL] baseline=%lu factor=%.2f\n", baselineValue, baselineFactor);
    statusMessage = "Calibration done";
  } else {
    Serial.println("[CAL] failed (no frames)");
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
  canvas.printf("LD2410 Parser (MyLD2410 style)");

  canvas.setCursor(4, 18);
  canvas.printf("Mode:%s Win:%d Thr:%lu", modeName, personWindowFrames, personThreshold);

  canvas.setCursor(4, 30);
  canvas.printf("Baseline:%lu EngSeen:%s", baselineValue, enhancedModeSeen ? "YES" : "no");

  canvas.setCursor(4, 42);
  canvas.printf("Frames:%lu LastMetric:%lu", frameCount, lastMetric);

  canvas.setCursor(4, 54);
  canvas.printf("Status:%u mD:%u sD:%u d:%u", d.status, d.mDist, d.sDist, d.dist);

  canvas.setCursor(4, 66);
  canvas.printf("mSig:%u sSig:%u", d.mSig, d.sSig);

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
  Serial.println("\n=== CardPuter LD2410 full parser ===");

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

  statusMessage = "Ready. E/N/R, C, WQ, AS, Space";
  Serial.println("[INIT] keys: E engON, N engOFF, R sensor reboot, C mode, W/Q thr, A/S window, Space calib, Esc reboot");
}

void loop() {
  M5Cardputer.update();

  // parse all incoming bytes using both parsers
  while (ld2410.available()) {
    uint8_t b = (uint8_t)ld2410.read();

    // optional raw byte print:
    if (b < 16) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ");

    feedParserByte(cfgParser,  b, CFG_HEAD,  CFG_TAIL,  onCfgFrame);
    feedParserByte(dataParser, b, DATA_HEAD, DATA_TAIL, onDataFrame);
  }

  if (anyDataReceived && (millis() - lastDataMs > 2000)) {
    statusMessage = "No recent data";
  }

  // keyboard
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
      }
      else if (k == 'w' || k == 'W') {
        personThreshold++;
        statusMessage = "Threshold++";
      }
      else if (k == 'q' || k == 'Q') {
        if (personThreshold > 0) personThreshold--;
        statusMessage = "Threshold--";
      }
      else if (k == 'a' || k == 'A') {
        personWindowFrames = max(1, personWindowFrames / 2);
        metricReset();
        statusMessage = "Window down";
      }
      else if (k == 's' || k == 'S') {
        personWindowFrames = min(MAX_WINDOW, personWindowFrames * 2);
        metricReset();
        statusMessage = "Window up";
      }
      else if (k == ' ') {
        runCalibration(12);
      }
      else if (k == '\x1B') {
        esp_restart();
      }
    }
  }

  drawUI();
}
