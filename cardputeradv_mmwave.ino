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
const uint8_t CFG_HEAD[4] = {0xFD, 0xFC, 0xFB, 0xFA};
const uint8_t CFG_TAIL[4] = {0x04, 0x03, 0x02, 0x01};

const uint8_t DATA_HEAD[4] = {0xF4, 0xF3, 0xF2, 0xF1};
const uint8_t DATA_TAIL[4] = {0xF8, 0xF7, 0xF6, 0xF5};

const size_t MAX_FRAME = 512;

// ============================================================
// UI/runtime state
// ============================================================
String statusMessage = "Ready";
unsigned long lastKeyTime = 0;
const unsigned long KEY_DELAY_MS = 180;

bool anyDataReceived = false;
unsigned long lastDataMs = 0;
unsigned long rawByteCount = 0;

// parser health
unsigned long parsedDataFrames = 0;
unsigned long parsedAckFrames = 0;
unsigned long parseFailCount = 0;
unsigned long fallbackExtractHits = 0;
unsigned long lastParsedMs = 0;
bool fallbackMode = false;
const unsigned long FALLBACK_TIMEOUT_MS = 1500;

// fallback telemetry only
unsigned long rawPatternMetric = 0;

// ============================================================
// Target-first output
// ============================================================
enum TargetKind { TK_UNKNOWN=0, TK_NONE, TK_MOVING, TK_STATIC, TK_BOTH };
TargetKind targetKind = TK_UNKNOWN;

bool parsedStatusValid = false;
uint8_t parsedStatus = 0xFF;

uint16_t dispMovingDist = 0;
uint16_t dispStaticDist = 0;
uint16_t dispDetectDist = 0;
uint8_t  dispMovingEnergy = 0;
uint8_t  dispStaticEnergy = 0;
bool enhancedModeSeen = false;

// ============================================================
// Parsed payload model
// ============================================================
struct ParsedData {
  bool ok = false;
  bool enhanced = false;
  uint8_t type = 0;
  uint8_t status = 0xFF; // 0..3

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
// Strict parser state
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

StreamParser cfgParser, dataParser;

// ============================================================
// Fallback ring buffer
// ============================================================
const size_t RAW_RING_MAX = 1024;
uint8_t rawRing[RAW_RING_MAX];
size_t rawLen = 0;

// ============================================================
// Helpers
// ============================================================
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
const char* targetKindString(TargetKind tk) {
  switch (tk) {
    case TK_NONE: return "NONE";
    case TK_MOVING: return "MOVING";
    case TK_STATIC: return "STATIC";
    case TK_BOTH: return "BOTH";
    default: return "UNKNOWN";
  }
}

unsigned long computeRawPatternMetric(const uint8_t* buf, size_t n) {
  // telemetry only
  unsigned long score = 0;
  for (size_t i = 0; i < n; i++) {
    uint8_t b = buf[i];
    if (b == 0xC0 || b == 0xC6) score += 2;
    else if (b == 0xFE || b == 0xF8) score += 2;
    else if (b == 0x3E || b == 0x38 || b == 0x0E) score += 1;
  }
  return score;
}

// ============================================================
// Payload parser (MyLD2410-like)
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
// Parsed target update (authoritative)
// ============================================================
void updateTargetFromParsed(const ParsedData &pd) {
  parsedStatusValid = true;
  parsedStatus = pd.status;

  dispMovingDist = pd.mDist;
  dispStaticDist = pd.sDist;
  dispDetectDist = pd.dist;
  dispMovingEnergy = pd.mSig;
  dispStaticEnergy = pd.sSig;
  enhancedModeSeen = pd.enhanced;

  switch (pd.status) {
    case 0: targetKind = TK_NONE; break;
    case 1: targetKind = TK_MOVING; break;
    case 2: targetKind = TK_STATIC; break;
    case 3: targetKind = TK_BOTH; break;
    default: targetKind = TK_UNKNOWN; break;
  }

  Serial.printf("\n[TARGET] %s st=%u mD=%u sD=%u d=%u mE=%u sE=%u\n",
                targetKindString(targetKind), pd.status,
                pd.mDist, pd.sDist, pd.dist, pd.mSig, pd.sSig);
}

// ============================================================
// Frame handlers
// ============================================================
void onCfgFrame(const uint8_t *payload, uint16_t len) {
  parsedAckFrames++;
  Serial.printf("\n[ACK] len=%u:", (unsigned)len);
  for (uint16_t i = 0; i < len; ++i) Serial.printf(" %02X", payload[i]);
  Serial.println();
}

void onDataPayload(const uint8_t *payload, uint16_t len) {
  ParsedData p;
  if (parseDataPayload(payload, len, p)) {
    gData = p;
    parsedDataFrames++;
    lastParsedMs = millis();
    updateTargetFromParsed(p);
  } else {
    parseFailCount++;
  }
}

void onDataFrame(const uint8_t *payload, uint16_t len) {
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
      if (ps.readCount < ps.expectedLen) ps.payload[ps.readCount] = b;
      else if (ps.readCount < ps.expectedLen + 4) ps.tail[ps.readCount - ps.expectedLen] = b;
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
// Fallback extractor (for recovery only)
// ============================================================
void feedByteToFallbackExtractor(uint8_t b) {
  if (rawLen < RAW_RING_MAX) rawRing[rawLen++] = b;
  else {
    memmove(rawRing, rawRing + 1, RAW_RING_MAX - 1);
    rawRing[RAW_RING_MAX - 1] = b;
    rawLen = RAW_RING_MAX;
  }

  // raw metric telemetry only
  if (rawLen >= 64) {
    rawPatternMetric = computeRawPatternMetric(rawRing + (rawLen - 64), 64);
    static unsigned long lastDbg = 0;
    if (millis() - lastDbg > 700) {
      lastDbg = millis();
      Serial.printf("\n[FALLBACK-RAW] metric=%lu (telemetry only)\n", rawPatternMetric);
    }
  }

  // recover packets by pattern [type=01/02][AA] ... [55][00]
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
          fallbackExtractHits++;
          onDataPayload(tmp, (uint16_t)pktLen);

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
// UI
// ============================================================
void drawUI() {
  canvas.fillSprite(BLACK);
  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);

  canvas.setCursor(4, 4);
  canvas.printf("LD2410 Parsed-Only Target");

  canvas.setCursor(4, 16);
  canvas.printf("FB mode:%s parsedValid:%s", fallbackMode ? "ON":"off", parsedStatusValid ? "Y":"N");

  canvas.setCursor(4, 28);
  canvas.printf("Parsed:%lu Ack:%lu Fail:%lu", parsedDataFrames, parsedAckFrames, parseFailCount);

  canvas.setCursor(4, 40);
  canvas.printf("FB recover:%lu rawM:%lu", fallbackExtractHits, rawPatternMetric);

  canvas.setTextSize(2);
  canvas.setCursor(4, 68);
  if (targetKind == TK_NONE) canvas.setTextColor(GREEN);
  else if (targetKind == TK_UNKNOWN) canvas.setTextColor(YELLOW);
  else canvas.setTextColor(RED);
  canvas.printf("Target:%s", targetKindString(targetKind));

  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);
  canvas.setCursor(4, 96);
  if (parsedStatusValid) {
    canvas.printf("st:%u mD:%u sD:%u d:%u", parsedStatus, dispMovingDist, dispStaticDist, dispDetectDist);
    canvas.setCursor(4, 108);
    canvas.printf("mE:%u sE:%u eng:%s", dispMovingEnergy, dispStaticEnergy, enhancedModeSeen ? "Y":"N");
  } else {
    canvas.printf("No parsed status yet");
    canvas.setCursor(4, 108);
    canvas.printf("Distances/Energies unavailable");
  }

  canvas.setCursor(4, canvas.height() - 14);
  canvas.printf("%s", statusMessage.c_str());

  canvas.pushSprite(2, 2);
}

// ============================================================
// Setup / loop
// ============================================================
void setup() {
  Serial.begin(LOG_BAUD);
  Serial.println("\n=== CardPuter LD2410 Parsed-Only ===");

  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);
  M5Cardputer.Display.setRotation(1);

  uint16_t w = M5Cardputer.Display.width();
  uint16_t h = M5Cardputer.Display.height();
  canvas.createSprite(w - 4, h - 4);

  ld2410.begin(SENSOR_BAUD, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);

  resetParser(cfgParser);
  resetParser(dataParser);
  rawLen = 0;

  targetKind = TK_UNKNOWN;
  parsedStatusValid = false;
  parsedStatus = 0xFF;
  lastParsedMs = millis();

  statusMessage = "Keys: E/N/R Esc";
  Serial.println("[INIT] keys: E engON, N engOFF, R sensor reboot, Esc reboot");
}

void loop() {
  M5Cardputer.update();

  while (ld2410.available()) {
    uint8_t b = (uint8_t)ld2410.read();

    rawByteCount++;
    anyDataReceived = true;
    lastDataMs = millis();

    // raw stream debug
    if (b < 16) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ");
    if (rawByteCount % 32 == 0) Serial.println();

    // always parse ACK path
    feedParserByte(cfgParser, b, CFG_HEAD, CFG_TAIL, onCfgFrame);

    // strict parse first; keep fallback extractor warm in parallel
    if (!fallbackMode) {
      feedParserByte(dataParser, b, DATA_HEAD, DATA_TAIL, onDataFrame);
      feedByteToFallbackExtractor(b);
    } else {
      feedByteToFallbackExtractor(b);
    }
  }

  // auto enable fallback only for recovery
  if (!fallbackMode && (millis() - lastParsedMs > FALLBACK_TIMEOUT_MS)) {
    fallbackMode = true;
    statusMessage = "Fallback extractor ON";
    Serial.println("\n[PARSER] strict starved -> fallback ON");
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
      else if (k == '\x1B') esp_restart();
    }
  }

  drawUI();
}
