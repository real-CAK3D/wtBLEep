// Biscuit C5 node that BROADCASTS to Flying Flocks — now DUAL-FUNCTION.
//
// It speaks the public Biscuit BLE/GATT protocol, so Flying Flocks connects to it
// like any fleet board and its results flow onto the wardrive map with no app changes.
// The SAME line protocol is mirrored over USB-CDC serial (DATA:/STATUS: out, CMD: in),
// so the app can also drive it over a USB cable with no BLE.
// One board, three jobs per sweep (the ESP32 Wi-Fi radio and the CC1101 are separate,
// so they don't fight each other):
//   1. Dual-band Wi-Fi scan  -> DATA:AP   (feeds 5 GHz APs the other boards can't see)
//   2. Sub-GHz RSSI sweep     -> DATA:RF   (315/433/868/915 band activity for the map)
//   3. Sub-GHz raw decode     -> DATA:TPMS / DATA:WEATHER / DATA:KEYFOB  (tire sensors,
//                                433 weather stations, keyfobs/remotes)
// Plus, when compiled with ALLOW_TX, a gated replay: "CMD:tx:<freq>,<bits>" transmits a
// captured OOK burst back on-air — YOUR OWN devices only (the app confirms ownership first).
//
// Protocol (from the app's BiscuitGatt):
//   service   4fafc201-1fb5-459e-8fcc-c5c9c331914b   (advertised; name "wtBLEep")
//   CMD   w   beb5483e-36e1-4688-b7f5-ea07361b26a8   central -> device  "CMD:scanall:" etc.
//   RESP  n   beb5483e-36e1-4688-b7f5-ea07361b26a9   device -> central  "DATA:..\n"
//   STAT  r   beb5483e-36e1-4688-b7f5-ea07361b26aa   JSON health
//   AP line:     DATA:AP:<SSID>,<BSSID>,<Channel>,<RSSI>,<Security>\n
//   RF line:     DATA:RF:<freqMHz>,<rssi>\n
//   TPMS line:   DATA:TPMS:<freqMHz>,<id>,<kPa>,<tempC>,<rawhex>\n
//   KEYFOB line: DATA:KEYFOB:<freqMHz>,<pulses>,<bits>\n
//
// Wiring — CC1101 4-wire SPI + ONE data wire for decode (all on front XIAO C5 pads):
//   CSN=GPIO1/D0  SCK=GPIO7/D3  MOSI=GPIO10/D10  MISO=GPIO8/D8  VCC=3V3  GND=GND
//   GDO0 (E07 pin 3) -> GPIO25/D2   <-- raw demodulated data, required for TPMS/keyfob decode.
//                                       (Without it, jobs 1 and 2 still work; 3 stays quiet.)
//   MOSI and GDO0 moved off GPIO2/GPIO4 (the C5's underside JTAG pads, not solderable)
//   onto the front D10/D2 pads.
//
// Honest status on job 3: it frames and Manchester-decodes bursts and emits the raw hex plus a
// best-effort TPMS parse. Pressure scaling / CRCs are per-manufacturer, so the first run near a
// car is a tuning pass — the raw hex lets us lock each sensor's format down (cf. rtl_433 / URH).

#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>

#define SVC_UUID  "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CMD_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define RESP_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define STAT_UUID "beb5483e-36e1-4688-b7f5-ea07361b26aa"

#define PIN_SCK  7      // D3
#define PIN_MISO 8      // D8
#define PIN_MOSI 10     // D10 (was GPIO2 = underside MTMS pad)
#define PIN_CSN  1      // D0
// *** SET THIS to whatever free GPIO you solder GDO0 (E07 pin 3) to. ***
#define GDO0_PIN 25     // D2  (was GPIO4 = underside MTCK pad)

const float SUBGHZ_FREQS[] = {315.00f, 433.92f, 868.00f, 915.00f};  // RSSI sweep bands
const int   SUBGHZ_N = 4;
const int   RF_ACTIVITY_DBM = -92;
const float RF_DECODE_HZ[] = {433.92f, 315.00f};                    // raw-decode bands (TPMS/keyfob)
const int   RF_DECODE_N = 2;

// --- Sub-GHz TRANSMIT gate -------------------------------------------------
// TX is OFF unless YOU compile with ALLOW_TX 1. It replays a captured OOK burst
// (e.g. your own garage/keyfob) when the app sends "CMD:tx:<freq>,<bits>".
// LEGAL/SAFETY: only ever transmit on frequencies and codes for devices you own
// and are authorized to operate. The app already gates each transmit behind an
// "I own this" confirmation; this flag is a second, firmware-level gate so a
// board can never transmit unless its operator deliberately built it to.
#define ALLOW_TX 1
// Replay symbol lengths: each received bit toggles the OOK carrier; a '1' half-
// symbol is long, a '0' short — reconstructing the captured envelope approximately.
const uint32_t TX_SHORT_US = 500;
const uint32_t TX_LONG_US  = 1000;
const int      TX_REPEATS  = 4;      // most OOK remotes send the frame several times

// GDO0 edge-capture ring buffer (only drained during the raw-decode window).
static const int CAP_MAX = 1024;
volatile uint32_t capBuf[CAP_MAX];
volatile uint16_t capHead = 0;
uint16_t          capTail = 0;
portMUX_TYPE capMux = portMUX_INITIALIZER_UNLOCKED;

BLECharacteristic *respChar = nullptr;
volatile bool connected = false;
volatile bool scanning = false;
bool cc1101ok = false;

// Pending transmit request, handed from the BLE write callback to loop() (radio
// ops must not run inside the BLE callback). Empty bits == nothing pending.
volatile bool txPending = false;
float  txFreq = 433.92f;
String txBits = "";

void IRAM_ATTR gdo0Isr() {
  uint32_t t = micros();
  portENTER_CRITICAL_ISR(&capMux);
  uint16_t next = (capHead + 1) % CAP_MAX;
  if (next != capTail) { capBuf[capHead] = t; capHead = next; }
  portEXIT_CRITICAL_ISR(&capMux);
}

// True once a USB host has sent us a CMD over Serial — lets the board run its sweep
// jobs when driven over USB-CDC with no BLE central connected.
bool serialActive = false;

void notifyLine(const String &line) {
  // Always mirror protocol lines to USB-CDC so a serial host (Flying Flocks over USB,
  // or a plain terminal) sees the same DATA:/STATUS: stream a BLE central would.
  Serial.print(line);
  if (respChar && connected) {
    respChar->setValue((uint8_t *)line.c_str(), line.length());
    respChar->notify();
    delay(6);  // let each notification flush before the next
  }
}

const char *authStr(wifi_auth_mode_t a) {
  switch (a) {
    case WIFI_AUTH_OPEN: return "OPEN";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA";
    case WIFI_AUTH_WPA2_PSK: return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
    case WIFI_AUTH_WPA3_PSK: return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
    default: return "WPA2";
  }
}

// One command path shared by the BLE characteristic and the USB-serial bridge.
void handleCommand(String v) {
  v.trim();
  if (v.startsWith("CMD:stopscan")) scanning = false;
  else if (v.startsWith("CMD:scan") || v.startsWith("CMD:sniff")) scanning = true;
  else if (v.startsWith("CMD:tx:")) {
    // CMD:tx:<freqMHz>,<bits>  — replay a captured OOK burst (your own devices only).
    String p = v.substring(7);           // after "CMD:tx:"
    int comma = p.indexOf(',');
    if (comma > 0) {
      txFreq = p.substring(0, comma).toFloat();
      txBits = p.substring(comma + 1);
      txBits.trim();
      if (txFreq > 250 && txFreq < 960 && txBits.length() > 0) txPending = true;
    }
  }
}

class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override { handleCommand(c->getValue().c_str()); }
};

// Read newline-delimited CMD lines from the USB host (Flying Flocks over USB-CDC).
void pollSerialCommands() {
  static String sbuf;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (sbuf.length()) {
        String l = sbuf; sbuf = "";
        l.trim();
        if (l.startsWith("CMD:")) { serialActive = true; handleCommand(l); }
      }
    } else if (sbuf.length() < 160) {
      sbuf += ch;
    } else {
      sbuf = "";  // runaway line without a newline; drop it
    }
  }
}

class SrvCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override { connected = true; }
  void onDisconnect(BLEServer *s) override {
    connected = false; scanning = false;
    BLEDevice::startAdvertising();  // be findable again
  }
};

bool initCC1101() {
  ELECHOUSE_cc1101.setSpiPin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  ELECHOUSE_cc1101.Init();
  if (!ELECHOUSE_cc1101.getCC1101()) return false;
  ELECHOUSE_cc1101.setMHZ(433.92);
  ELECHOUSE_cc1101.SetRx();
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1200);
  Serial.println("\n=== Biscuit C5 -> Flying Flocks (Wi-Fi + sub-GHz RSSI + TPMS/weather/keyfob"
#if ALLOW_TX
                 " + gated TX"
#endif
                 ") ===");

  cc1101ok = initCC1101();
  Serial.println(cc1101ok ? "[sub] CC1101 detected." : "[sub] CC1101 not wired yet.");

  pinMode(GDO0_PIN, INPUT);   // interrupt is attached only during the raw-decode window

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);

  BLEDevice::init("wtBLEep");
  BLEDevice::setMTU(517);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new SrvCallbacks());
  BLEService *svc = server->createService(SVC_UUID);

  BLECharacteristic *cmd = svc->createCharacteristic(
      CMD_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  cmd->setCallbacks(new CmdCallbacks());

  respChar = svc->createCharacteristic(RESP_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  respChar->addDescriptor(new BLE2902());

  BLECharacteristic *stat = svc->createCharacteristic(STAT_UUID, BLECharacteristic::PROPERTY_READ);
  stat->setValue("{\"status\":0,\"battery\":100,\"op\":\"idle\",\"model\":\"wtBLEep-C5\"}");

  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("[ble] advertising as \"wtBLEep\" — connect from Flying Flocks and Start a wardrive.");
}

// ---- Job 1: Wi-Fi ----------------------------------------------------------
// The C5 is SINGLE-CORE, so a blocking dual-band WiFi scan (several seconds)
// starves the BLE host task and the phone's link times out mid-sweep — the
// board "connects then disappears". Run the scan ASYNC and poll it with short
// delay()s so the RTOS keeps servicing BLE the whole time the radio is busy.
void wifiScanBroadcast() {
  WiFi.scanNetworks(/*async=*/true, /*show_hidden=*/true);
  uint32_t t0 = millis();
  while (WiFi.scanComplete() == WIFI_SCAN_RUNNING) {
    delay(30);                      // yields to the BLE/RTOS tasks — keeps the link alive
    if (millis() - t0 > 15000) { WiFi.scanDelete(); return; }  // safety cap
  }
  int n = WiFi.scanComplete();
  if (n < 0) { WiFi.scanDelete(); return; }
  for (int i = 0; i < n; i++) {
    if (!connected) break;          // bail out the moment the phone drops
    String ssid = WiFi.SSID(i);
    String line = "DATA:AP:" + ssid + "," + WiFi.BSSIDstr(i) + "," +
                  String(WiFi.channel(i)) + "," + String(WiFi.RSSI(i)) + "," +
                  authStr(WiFi.encryptionType(i)) + "\n";
    notifyLine(line);
  }
  Serial.printf("[ble] sent %d APs\n", n);
  WiFi.scanDelete();
}

// ---- Job 2: sub-GHz RSSI sweep (normal RX mode) ----------------------------
void subghzRssiSweep() {
  if (!cc1101ok) { cc1101ok = initCC1101(); if (!cc1101ok) return; }
  ELECHOUSE_cc1101.setCCMode(1);          // normal packet/RSSI mode (GDO back to default)
  ELECHOUSE_cc1101.setModulation(0);
  for (int i = 0; i < SUBGHZ_N; i++) {
    ELECHOUSE_cc1101.setMHZ(SUBGHZ_FREQS[i]);
    ELECHOUSE_cc1101.SetRx();
    delay(60);
    int rssi = ELECHOUSE_cc1101.getRssi();
    if (rssi > RF_ACTIVITY_DBM)
      notifyLine("DATA:RF:" + String(SUBGHZ_FREQS[i], 2) + "," + String(rssi) + "\n");
  }
}

// ---- Job 3: sub-GHz raw decode (async RX mode, GDO0) ------------------------
static const uint32_t GAP_US = 5000;    // 5 ms idle ends a burst

void applyRadioRaw(float mhz, bool ook) {
  ELECHOUSE_cc1101.setCCMode(0);                   // raw/async: GDO0 streams demodulated data
  ELECHOUSE_cc1101.setModulation(ook ? 2 : 0);     // 2 = ASK/OOK (keyfob), 0 = 2-FSK (TPMS)
  ELECHOUSE_cc1101.setMHZ(mhz);
  if (ook) { ELECHOUSE_cc1101.setDRate(5); ELECHOUSE_cc1101.setRxBW(203); }
  else { ELECHOUSE_cc1101.setDeviation(38); ELECHOUSE_cc1101.setDRate(19.2); ELECHOUSE_cc1101.setRxBW(203); }
  ELECHOUSE_cc1101.SetRx();
}

int drainBurst(uint16_t *out, int maxOut) {
  uint16_t head, tail;
  portENTER_CRITICAL(&capMux); head = capHead; tail = capTail; portEXIT_CRITICAL(&capMux);
  if (head == tail) return 0;
  uint32_t lastT; uint16_t prev = (head + CAP_MAX - 1) % CAP_MAX;
  portENTER_CRITICAL(&capMux); lastT = capBuf[prev]; portEXIT_CRITICAL(&capMux);
  if ((micros() - lastT) < GAP_US) return 0;       // burst not finished yet

  int n = 0; uint32_t prevT = 0; bool first = true;
  while (tail != head && n < maxOut) {
    uint32_t t; portENTER_CRITICAL(&capMux); t = capBuf[tail]; tail = (tail + 1) % CAP_MAX; portEXIT_CRITICAL(&capMux);
    if (!first) { uint32_t d = t - prevT; out[n++] = (d > 65535) ? 65535 : (uint16_t)d; }
    prevT = t; first = false;
  }
  portENTER_CRITICAL(&capMux); capTail = tail; portEXIT_CRITICAL(&capMux);
  return n;
}

int manchester(const uint16_t *iv, int n, uint32_t bitUs, uint8_t *bytes, int maxBytes) {
  static uint8_t half[2048];
  int hn = 0, level = 0;
  for (int i = 0; i < n && hn < 2046; i++) {
    int reps = (iv[i] > (bitUs + bitUs / 2)) ? 2 : 1;   // ~2T -> two half-bits, else one
    for (int r = 0; r < reps && hn < 2046; r++) half[hn++] = level;
    level ^= 1;
  }
  int outBits = 0; uint8_t b = 0;
  for (int i = 0; i + 1 < hn; i += 2) {
    int a = half[i], c = half[i + 1];
    if (a == c) return 0;                            // not a valid Manchester pair
    b = (b << 1) | ((a == 1 && c == 0) ? 1 : 0);
    if (++outBits % 8 == 0) { if (outBits / 8 <= maxBytes) bytes[outBits / 8 - 1] = b; b = 0; }
  }
  return outBits / 8;
}

String toHex(const uint8_t *b, int n) {
  String s; char t[3];
  for (int i = 0; i < n; i++) { sprintf(t, "%02X", b[i]); s += t; }
  return s;
}

void emitTpms(float mhz, const uint8_t *b, int n) {
  if (n < 7) return;
  String id = toHex(b, 4);
  float kpa = b[5] * 2.5f;        // approximate scaling; refine per sensor
  int tempC = b[6] - 50;          // common offset; refine per sensor
  notifyLine("DATA:TPMS:" + String(mhz, 2) + "," + id + "," +
             String(kpa, 0) + "," + String(tempC) + "," + toHex(b, n) + "\n");
}

// ---- Weather / 433 sensor decode (Nexus / Sencor-style PWM-OOK) ------------
// Very common cheap 433.92 MHz temp/humidity stations (Nexus, Sencor SWS, many
// Chinese re-brands). 36 bits, PWM: each bit is a short (~500us) carrier pulse
// followed by a gap — a short gap (~1000us) is 0, a long gap (~2000us) is 1;
// a very long gap (>3800us) separates repeated frames. We read the gaps from the
// edge-interval buffer (pulses are skipped as the short intervals).
//   [8 id][1 batt][1 0][2 chan][12 temp signed /10 degC][4 0xF][8 humidity]
// Returns true and fills out-params on a valid frame.
bool decodeNexus(const uint16_t *iv, int n, uint8_t &id, int &tempC10, int &hum, int &chan, int &batt) {
  uint8_t bits[40]; int nb = 0;
  for (int i = 0; i < n; i++) {
    uint16_t d = iv[i];
    if (d > 3800) { nb = 0; continue; }          // frame gap -> start fresh
    if (d < 750) continue;                        // the ~500us pulse -> skip
    if (nb < 40) bits[nb++] = (d > 1500) ? 1 : 0; // short gap=0, long gap=1
    if (nb == 36) break;
  }
  if (nb < 36) return false;
  // bit 9 must be 0 and the 0xF marker nibble must be present — cheap integrity check.
  int marker = (bits[24] << 3) | (bits[25] << 2) | (bits[26] << 1) | bits[27];
  if (marker != 0xF || bits[9] != 0) return false;
  id = 0; for (int i = 0; i < 8; i++) id = (id << 1) | bits[i];
  batt = bits[8];
  chan = (bits[10] << 1) | bits[11];
  int t = 0; for (int i = 12; i < 24; i++) t = (t << 1) | bits[i];
  if (t & 0x800) t -= 0x1000;                     // sign-extend 12-bit
  tempC10 = t;                                    // tenths of a degree C
  hum = 0; for (int i = 28; i < 36; i++) hum = (hum << 1) | bits[i];
  if (hum < 1 || hum > 100) return false;
  if (tempC10 < -400 || tempC10 > 700) return false;
  return true;
}

bool emitWeather(float mhz, const uint16_t *iv, int n) {
  uint8_t id; int t10, hum, ch, batt;
  if (!decodeNexus(iv, n, id, t10, hum, ch, batt)) return false;
  char ids[8]; sprintf(ids, "%02X%d", id, ch);    // station id + channel
  notifyLine("DATA:WEATHER:" + String(mhz, 2) + "," + String(ids) + "," +
             String(t10 / 10.0f, 1) + "," + String(hum) + "\n");
  return true;
}

void processBurst(float mhz, bool ook, const uint16_t *iv, int n) {
  if (ook) {
    if (emitWeather(mhz, iv, n)) return;          // a weather station? emit and stop.
    // Anything else on the OOK path is a raw burst: keyfob, remote, doorbell/garage — OR an
    // OOK-based TPMS, which reads identically here. We tag it KEYFOB (the app shows it as an
    // "OOK burst", not definitively a keyfob); positive TPMS ID happens on the FSK path below.
    String bits;
    for (int i = 0; i < n && bits.length() < 96; i++) bits += (iv[i] > 500) ? '1' : '0';
    notifyLine("DATA:KEYFOB:" + String(mhz, 2) + "," + String(n) + "," + bits + "\n");
  } else {
    uint8_t bytes[32];
    int nb = manchester(iv, n, 52, bytes, sizeof(bytes));
    if (nb < 7) nb = manchester(iv, n, 26, bytes, sizeof(bytes));
    if (nb >= 7) emitTpms(mhz, bytes, nb < 20 ? nb : 20);
  }
}

// Listen in raw async mode across the decode bands/modulations for a short window each.
void rfDecodeWindow() {
  if (!cc1101ok) return;
  static uint16_t iv[CAP_MAX];
  attachInterrupt(digitalPinToInterrupt(GDO0_PIN), gdo0Isr, CHANGE);
  for (int band = 0; band < RF_DECODE_N; band++) {
    for (int mod = 0; mod < 2; mod++) {              // FSK (TPMS) then OOK (keyfob)
      applyRadioRaw(RF_DECODE_HZ[band], mod == 1);
      portENTER_CRITICAL(&capMux); capHead = 0; capTail = 0; portEXIT_CRITICAL(&capMux);
      uint32_t t0 = millis();
      while (millis() - t0 < 400 && connected && scanning) {
        int n = drainBurst(iv, CAP_MAX);
        if (n >= 16) processBurst(RF_DECODE_HZ[band], mod == 1, iv, n);
        delay(2);
      }
    }
  }
  detachInterrupt(digitalPinToInterrupt(GDO0_PIN));
}

#if ALLOW_TX
// Best-effort OOK replay of a captured burst. Each received bit toggles the OOK
// carrier for one half-symbol, reconstructing the captured envelope approximately.
// Untested on-air until you bench it against your OWN device — treat as a first pass.
void doTransmit(float mhz, const String &bits) {
  if (!cc1101ok) { cc1101ok = initCC1101(); if (!cc1101ok) return; }
  detachInterrupt(digitalPinToInterrupt(GDO0_PIN));   // capture ISR must be off while we key the pin
  ELECHOUSE_cc1101.setCCMode(0);                       // async serial mode
  ELECHOUSE_cc1101.setModulation(2);                   // ASK/OOK
  ELECHOUSE_cc1101.setMHZ(mhz);
  ELECHOUSE_cc1101.SetTx();
  pinMode(GDO0_PIN, OUTPUT);                           // in async TX, GDO0 is the data line INTO the CC1101
  for (int rep = 0; rep < TX_REPEATS; rep++) {
    int level = 1;                                     // first symbol is a mark (carrier on)
    for (unsigned i = 0; i < bits.length(); i++) {
      digitalWrite(GDO0_PIN, level);
      delayMicroseconds(bits[i] == '1' ? TX_LONG_US : TX_SHORT_US);
      level ^= 1;
    }
    digitalWrite(GDO0_PIN, LOW);
    delay(8);                                          // inter-frame gap
  }
  ELECHOUSE_cc1101.SetRx();
  pinMode(GDO0_PIN, INPUT);
  notifyLine("DATA:TXDONE:" + String(mhz, 2) + "," + String(bits.length()) + "\n");
  Serial.printf("[tx] replayed %u symbols @ %.2f MHz x%d\n", bits.length(), mhz, TX_REPEATS);
}
#endif

void loop() {
  pollSerialCommands();    // accept CMD: lines from a USB-CDC host
#if ALLOW_TX
  if (txPending) { txPending = false; doTransmit(txFreq, txBits); }
#endif
  // Run the sweep jobs when driven over BLE OR over USB serial.
  bool host = connected || serialActive;
  if (host && scanning) {
    wifiScanBroadcast();   // job 1: DATA:AP (both bands)
    delay(40);             // let the links breathe between heavy jobs
    if (scanning) subghzRssiSweep();  // job 2: DATA:RF (band activity)
    delay(40);
    if (scanning) rfDecodeWindow();   // job 3: DATA:TPMS / DATA:WEATHER / DATA:KEYFOB
  } else {
    delay(200);
  }
}
