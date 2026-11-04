// ============================================================================
//  Marg-Darshak Receiver  (ESP32 WROOM-32, Arduino-ESP32 core 3.x)
//
//  BLE scan -> decode 5-byte payload -> route_db.h lookup -> per-route EMA RSSI
//  -> threshold + hysteresis -> sentence builder -> W25Q128 raw flash -> I2S.
//
//  Needs in the same sketch folder:   audio_map.h   (from pack_audio.py)
//                                     route_db.h    (Step 2)
//  Flash audio_data.bin to the W25Q128 at address 0x000000 first.
//
//  Structure:
//    * BLE scanning runs in its own FreeRTOS task (core 0).
//    * loop() never blocks: buttons, route evaluation, vibrator and the
//      audio state machine all run as short ticks.
// ============================================================================
#include <Arduino.h>
#include <SPI.h>
#include <driver/i2s.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include "audio_map.h"
#include "route_db.h"

// ---------- PINS ----------
#define FLASH_CS        5
#define FLASH_SCK       18
#define FLASH_MISO      19
#define FLASH_MOSI      23

#define I2S_BCLK        26
#define I2S_LRC         25
#define I2S_DOUT        22
#define I2S_PORT        I2S_NUM_0

#define BUTTON_POWER    32
#define BUTTON_LANGUAGE 33

#define VIBRATOR_PIN        27
#define VIBRATOR_PWM_FREQ   200
#define VIBRATOR_RESOLUTION 8

// ---------- BLE PAYLOAD (must match the Transmitter) ----------
#define MFG_ID_LO   0x4C
#define MFG_ID_HI   0x01
#define MD_MAGIC    0xA5
// Payload as returned by getManufacturerData():
//   [0]=4C [1]=01 [2]=A5 [3]=0x80|(route>>7) [4]=0x80|(route&0x7F)

// ---------- EMA / TRIGGER TUNING ----------
#define EMA_ALPHA           0.25f   // 0.2-0.3: lower = smoother but slower
#define RSSI_THRESHOLD      -70     // dBm: announce when EMA >= this
#define RSSI_HYSTERESIS     5       // dB: EMA must drop below THRESHOLD-this to re-arm
#define MIN_SAMPLES         3       // packets needed before a route may trigger
#define STALE_MS            3000    // route forgotten if not heard for this long
#define ANNOUNCE_COOLDOWN   8000    // ms between repeats while bus stays close
#define MAX_ANNOUNCEMENTS   3       // per approach
#define EVAL_PERIOD_MS      50

// ---------- AUDIO TUNING ----------
#define AUDIO_GAIN          10      // same as the tested sketch (clipping-protected)
#define GAP_CLIP_MS         80      // silence between clips
#define GAP_LANG_MS         700     // silence between languages
#define ANNOUNCE_ALL_LANGUAGES 0    // 0 = selected language only, 1 = EN, HI, MR in turn
#define CHUNK_BYTES         512

// ---------- VIBRATOR TUNING ----------
#define VIB_MIN_DUTY        60      // tune: lowest duty at which the motor really spins
#define VIB_MAX_DUTY        255

// ============================================================================
//  GLOBAL STATE
// ============================================================================
volatile bool deviceOn = false;
volatile uint8_t selectedLanguage = 0;      // 0=EN, 1=HI, 2=MR
const char* LANG_NAMES[] = { "EN", "HI", "MR" };

SPISettings flashSPI(10000000, MSBFIRST, SPI_MODE0);
BLEScan* pBLEScan;

// ---------- per-route EMA state (parallel to ROUTE_DB) ----------
struct RouteState {
  bool     valid = false;
  float    ema = -100.0f;
  uint8_t  samples = 0;
  uint32_t lastSeenMs = 0;
  uint32_t lastAnnounceMs = 0;
  uint8_t  announceCount = 0;
};
RouteState routeState[ROUTE_COUNT];
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

// ---------- audio engine state ----------
enum AudioState : uint8_t { AUD_IDLE, AUD_PLAYING, AUD_GAP };
AudioState audioState = AUD_IDLE;

struct PlayItem { uint8_t lang; uint8_t clip; };
#define MAX_PLAYLIST 40
PlayItem playlist[MAX_PLAYLIST];
uint8_t  plLen = 0, plPos = 0;

uint32_t clipAddr = 0, clipLeft = 0;
uint8_t  chunk[CHUNK_BYTES];
size_t   chunkLen = 0, chunkOff = 0;
uint32_t gapUntil = 0;

// ============================================================================
//  NOR FLASH
// ============================================================================
void flashRead(uint32_t address, uint8_t* buffer, size_t length) {
  SPI.beginTransaction(flashSPI);
  digitalWrite(FLASH_CS, LOW);
  SPI.transfer(0x03);
  SPI.transfer((address >> 16) & 0xFF);
  SPI.transfer((address >> 8) & 0xFF);
  SPI.transfer(address & 0xFF);
  for (size_t i = 0; i < length; i++) buffer[i] = SPI.transfer(0x00);
  digitalWrite(FLASH_CS, HIGH);
  SPI.endTransaction();
}

void flashCheckId() {
  uint8_t id[3];
  SPI.beginTransaction(flashSPI);
  digitalWrite(FLASH_CS, LOW);
  SPI.transfer(0x9F);
  for (int i = 0; i < 3; i++) id[i] = SPI.transfer(0x00);
  digitalWrite(FLASH_CS, HIGH);
  SPI.endTransaction();
  Serial.printf("Flash JEDEC ID: %02X %02X %02X (W25Q128 = EF 40 18)\n", id[0], id[1], id[2]);
  if (id[0] == 0x00 || id[0] == 0xFF) Serial.println("WARNING: flash not responding - check SPI wiring.");
}

// ============================================================================
//  I2S
// ============================================================================
void setupI2S() {
  i2s_config_t config = {};
  config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  config.sample_rate = AUDIO_SAMPLE_RATE;
  config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  config.dma_buf_count = 8;
  config.dma_buf_len = 256;
  config.use_apll = false;
  config.tx_desc_auto_clear = true;
  config.fixed_mclk = 0;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = I2S_BCLK;
  pins.ws_io_num = I2S_LRC;
  pins.data_out_num = I2S_DOUT;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  if (i2s_driver_install(I2S_PORT, &config, 0, nullptr) != ESP_OK) {
    Serial.println("I2S install failed");
    while (true) delay(1000);
  }
  if (i2s_set_pin(I2S_PORT, &pins) != ESP_OK) {
    Serial.println("I2S pin setup failed");
    while (true) delay(1000);
  }
  i2s_zero_dma_buffer(I2S_PORT);
}

// ============================================================================
//  VIBRATOR
// ============================================================================
void setVibrator(uint8_t duty) { ledcWrite(VIBRATOR_PIN, duty); }

// ============================================================================
//  SENTENCE BUILDER + AUDIO ENGINE (non-blocking state machine)
// ============================================================================
void plAdd(uint8_t lang, uint8_t clip) {
  if (plLen < MAX_PLAYLIST) playlist[plLen++] = { lang, clip };
}

// BUS + [route digits] + source + TO + destination + ARRIVING
void buildSentence(const BusRoute* r, uint8_t lang) {
  plAdd(lang, CLIP_BUS);

  uint8_t digits[5];
  int n = 0;
  uint16_t v = r->routeNo;
  do { digits[n++] = v % 10; v /= 10; } while (v && n < 5);
  while (n > 0) plAdd(lang, CLIP_DIGIT(digits[--n]));   // no leading zeros

  plAdd(lang, r->source);
  plAdd(lang, CLIP_TO);
  plAdd(lang, r->destination);
  plAdd(lang, CLIP_ARRIVING);
}

void loadClip(const PlayItem& item) {
  const AudioClip& c = AUDIO_MAP[item.lang][item.clip];
  clipAddr = c.addr;
  clipLeft = c.len & ~1UL;      // whole 16-bit samples only
  chunkLen = chunkOff = 0;
}

void audioStart() {
  if (plLen == 0) return;
  plPos = 0;
  loadClip(playlist[0]);
  audioState = AUD_PLAYING;
}

void audioStop() {
  audioState = AUD_IDLE;
  plLen = plPos = 0;
  chunkLen = chunkOff = 0;
  clipLeft = 0;
  i2s_zero_dma_buffer(I2S_PORT);
}

// Call every loop pass. Does at most one flash read or one I2S write; never waits.
void audioTick() {
  if (audioState == AUD_GAP) {
    if ((int32_t)(millis() - gapUntil) >= 0) {
      loadClip(playlist[plPos]);
      audioState = AUD_PLAYING;
    }
    return;
  }
  if (audioState != AUD_PLAYING) return;

  // Need a new chunk?
  if (chunkOff >= chunkLen) {
    if (clipLeft == 0) {                       // current clip finished
      uint8_t curLang = playlist[plPos].lang;
      plPos++;
      if (plPos >= plLen) {                    // whole sentence finished
        audioState = AUD_IDLE;
        plLen = plPos = 0;
        return;                                // let the DMA tail play out
      }
      gapUntil = millis() + (playlist[plPos].lang != curLang ? GAP_LANG_MS : GAP_CLIP_MS);
      audioState = AUD_GAP;
      return;
    }

    size_t n = clipLeft > CHUNK_BYTES ? CHUNK_BYTES : clipLeft;
    flashRead(clipAddr, chunk, n);

    for (size_t i = 0; i < n; i += 2) {        // gain with clipping protection
      int16_t s = (int16_t)((uint16_t)chunk[i] | ((uint16_t)chunk[i + 1] << 8));
      int32_t a = (int32_t)s * AUDIO_GAIN;
      if (a > 32767) a = 32767;
      if (a < -32768) a = -32768;
      chunk[i]     = (uint8_t)(a & 0xFF);
      chunk[i + 1] = (uint8_t)(((uint16_t)a >> 8) & 0xFF);
    }
    clipAddr += n;
    clipLeft -= n;
    chunkLen = n;
    chunkOff = 0;
  }

  // Push as much as the DMA buffer accepts right now (timeout 0 = don't wait)
  size_t written = 0;
  i2s_write(I2S_PORT, chunk + chunkOff, chunkLen - chunkOff, &written, 0);
  chunkOff += written;
}

// ============================================================================
//  BLE: payload decode, route lookup, per-route EMA
// ============================================================================
bool decodePayload(const String& m, uint16_t& routeNo) {
  if (m.length() != 5) return false;
  if ((uint8_t)m[0] != MFG_ID_LO || (uint8_t)m[1] != MFG_ID_HI) return false;
  if ((uint8_t)m[2] != MD_MAGIC) return false;
  uint8_t hi = (uint8_t)m[3], lo = (uint8_t)m[4];
  if (!(hi & 0x80) || !(lo & 0x80)) return false;
  routeNo = ((uint16_t)(hi & 0x7F) << 7) | (lo & 0x7F);
  return true;
}

void updateEma(size_t idx, int rssi) {
  uint32_t now = millis();
  portENTER_CRITICAL(&stateMux);
  RouteState& s = routeState[idx];
  if (!s.valid) {                    // first sample seeds the filter
    s = RouteState();
    s.valid = true;
    s.ema = (float)rssi;
    s.samples = 1;
  } else {
    s.ema = EMA_ALPHA * (float)rssi + (1.0f - EMA_ALPHA) * s.ema;
    if (s.samples < 255) s.samples++;
  }
  s.lastSeenMs = now;
  portEXIT_CRITICAL(&stateMux);
}

void resetRouteStates() {
  portENTER_CRITICAL(&stateMux);
  for (size_t i = 0; i < ROUTE_COUNT; i++) routeState[i] = RouteState();
  portEXIT_CRITICAL(&stateMux);
}

class BusBeaconCallback : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice device) {
    if (!deviceOn) return;
    if (!device.haveManufacturerData()) return;

    String m = device.getManufacturerData();
    uint16_t routeNo;
    if (!decodePayload(m, routeNo)) return;       // not one of our beacons

    const BusRoute* r = findRoute(routeNo);
    if (!r) {                                     // valid beacon, route not in DB
      static uint32_t lastWarn = 0;
      if (millis() - lastWarn > 2000) {
        lastWarn = millis();
        Serial.printf("Beacon for route %u ignored (not in route DB)\n", routeNo);
      }
      return;
    }
    updateEma((size_t)(r - ROUTE_DB), device.getRSSI());
  }
};
BusBeaconCallback beaconCallback;

// Continuous scanning in its own task so loop() is never blocked.
void scanTask(void*) {
  for (;;) {
    if (!deviceOn) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
    pBLEScan->start(1, false);     // 1 s scan window
    pBLEScan->clearResults();      // free the result list
    vTaskDelay(1);
  }
}

// ============================================================================
//  ROUTE EVALUATION: staleness, hysteresis, trigger, vibrator
// ============================================================================
void evaluateRoutes(uint32_t now) {
  int   bestIdx = -1, annIdx = -1;
  float bestEma = -127.0f, annEma = -127.0f;
  bool  audioBusy = (audioState != AUD_IDLE);

  portENTER_CRITICAL(&stateMux);
  for (size_t i = 0; i < ROUTE_COUNT; i++) {
    RouteState& s = routeState[i];
    if (!s.valid) continue;

    if ((int32_t)(now - s.lastSeenMs) > STALE_MS) {   // bus left: forget it
      s = RouteState();
      continue;
    }
    if (s.ema < RSSI_THRESHOLD - RSSI_HYSTERESIS) s.announceCount = 0;   // re-arm

    if (s.ema > bestEma) { bestEma = s.ema; bestIdx = (int)i; }

    if (!audioBusy &&
        s.samples >= MIN_SAMPLES &&
        s.ema >= RSSI_THRESHOLD &&
        s.announceCount < MAX_ANNOUNCEMENTS &&
        (s.announceCount == 0 || (int32_t)(now - s.lastAnnounceMs) >= ANNOUNCE_COOLDOWN) &&
        s.ema > annEma) {
      annEma = s.ema;
      annIdx = (int)i;
    }
  }
  if (annIdx >= 0) {
    routeState[annIdx].announceCount++;
    routeState[annIdx].lastAnnounceMs = now;
  }
  portEXIT_CRITICAL(&stateMux);

  // Vibrator scales with the strongest route's EMA (0 below threshold)
  uint8_t duty = 0;
  if (bestIdx >= 0 && bestEma >= RSSI_THRESHOLD) {
    long d = map((long)bestEma, -90, -50, VIB_MIN_DUTY, VIB_MAX_DUTY);
    duty = (uint8_t)constrain(d, 0L, 255L);
  }
  setVibrator(duty);

  // Start an announcement
  if (annIdx >= 0) {
    const BusRoute* r = &ROUTE_DB[annIdx];
    plLen = 0;
#if ANNOUNCE_ALL_LANGUAGES
    for (uint8_t l = 0; l < AUDIO_NUM_LANGS; l++) buildSentence(r, l);
#else
    buildSentence(r, selectedLanguage);
#endif
    Serial.printf("ANNOUNCE route %u | lang %s | EMA %.1f | clips %u\n",
                  r->routeNo, LANG_NAMES[selectedLanguage], annEma, plLen);
    audioStart();
  }
}

// ============================================================================
//  BUTTONS (debounced, same logic as the tested sketch)
// ============================================================================
bool lastPowerReading = HIGH, lastLanguageReading = HIGH;
bool stablePowerState = HIGH, stableLanguageState = HIGH;
uint32_t powerChangeTime = 0, languageChangeTime = 0;
const uint32_t DEBOUNCE_MS = 50;

void handleButtons() {
  uint32_t now = millis();

  bool p = digitalRead(BUTTON_POWER);
  if (p != lastPowerReading) { powerChangeTime = now; lastPowerReading = p; }
  if (now - powerChangeTime >= DEBOUNCE_MS && p != stablePowerState) {
    stablePowerState = p;
    if (stablePowerState == LOW) {
      deviceOn = !deviceOn;
      resetRouteStates();
      if (deviceOn) {
        Serial.println("Device ON - scanning");
      } else {
        Serial.println("Device OFF");
        setVibrator(0);
        audioStop();
      }
    }
  }

  bool l = digitalRead(BUTTON_LANGUAGE);
  if (l != lastLanguageReading) { languageChangeTime = now; lastLanguageReading = l; }
  if (now - languageChangeTime >= DEBOUNCE_MS && l != stableLanguageState) {
    stableLanguageState = l;
    if (stableLanguageState == LOW) {
      selectedLanguage = (selectedLanguage + 1) % 3;
      Serial.printf("Language: %s\n", LANG_NAMES[selectedLanguage]);
    }
  }
}

// ============================================================================
//  SETUP / LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(BUTTON_POWER, INPUT_PULLUP);
  pinMode(BUTTON_LANGUAGE, INPUT_PULLUP);

  if (!ledcAttach(VIBRATOR_PIN, VIBRATOR_PWM_FREQ, VIBRATOR_RESOLUTION)) {
    Serial.println("Vibrator PWM setup failed.");
    while (true) delay(1000);
  }
  setVibrator(0);

  pinMode(FLASH_CS, OUTPUT);
  digitalWrite(FLASH_CS, HIGH);
  SPI.begin(FLASH_SCK, FLASH_MISO, FLASH_MOSI, FLASH_CS);
  flashCheckId();

  setupI2S();

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(&beaconCallback, true);  // true = report every packet (needed for EMA)
  pBLEScan->setActiveScan(false);   // beacon data is in the main advertisement
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  xTaskCreatePinnedToCore(scanTask, "bleScan", 6144, nullptr, 1, nullptr, 0);

  Serial.printf("Margdarshak Receiver ready | %u routes in DB | language: %s\n",
                (unsigned)ROUTE_COUNT, LANG_NAMES[selectedLanguage]);
  Serial.println("Press the power button (GPIO 32) to start.");
}

void loop() {
  static uint32_t lastEval = 0, lastPrint = 0;
  handleButtons();

  uint32_t now = millis();
  if (deviceOn) {
    if (now - lastEval >= EVAL_PERIOD_MS) {
      lastEval = now;
      evaluateRoutes(now);
    }
    audioTick();

    if (now - lastPrint >= 1000) {
      lastPrint = now;
      portENTER_CRITICAL(&stateMux);
      RouteState snap[ROUTE_COUNT];
      memcpy(snap, routeState, sizeof(snap));
      portEXIT_CRITICAL(&stateMux);
      for (size_t i = 0; i < ROUTE_COUNT; i++) {
        if (snap[i].valid)
          Serial.printf("Route %u | EMA %.1f dBm | n=%u | ann=%u | lang %s\n",
                        ROUTE_DB[i].routeNo, snap[i].ema, snap[i].samples,
                        snap[i].announceCount, LANG_NAMES[selectedLanguage]);
      }
    }
  }
  delay(1);
}
