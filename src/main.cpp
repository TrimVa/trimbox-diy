#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <LSM6DS3.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>
#include <bluefruit.h>
#include <nrf_soc.h>

// --- stockage sur la flash QSPI 2 Mo embarquée (P25Q16H) ---
#include <Adafruit_SPIFlash.h>
#include <SPI.h>

// ============================================================================
// --- USER CUSTOMIZATION  ---
// ============================================================================
#define DEVICE_NICKNAME "Buggy 1"

#define SERIAL_NUM "0123456789"
#define BRAND       "TrimBox DIY"
#define DEVICE_NAME "TrimBox " DEVICE_NICKNAME
#define MODEL_STRING BRAND
#define HARDWARE_VER "1.0"
#define MANUFACTURER BRAND
#define FIRMWARE_VER "1.0"

#define BUILD_STAMP __DATE__ " " __TIME__

// --- GPS Performance ---
#define MAX_NAVIGATION_RATE 25
#define GPS_BAUD 115200

#define SYSTEM_RATE_REPORT_MS 5000

// --- Power & Efficiency ---
#define EXPECT_BATTERY true
#define GPS_HOT_TIMEOUT_MS 900000
#define DEEP_SLEEP_DAYS 1
#define ENABLE_DEEP_SLEEP false
#define FAST_ADV_INTERVAL 160
#define ECO_ADV_INTERVAL 4000
#define SLEEP_WHILE_CHARGING true
#define LOW_POWER_BT_TX_POWER -4

// --- GNSS Constellation Toggle ---
#define ENABLE_GNSS_GPS
#define ENABLE_GNSS_GALILEO

// --- Hardware Version ---
#define PCB_VERSION

// ============================================================================
// ---  HARDWARE MAPPINGS ---
// ============================================================================
#define GPS_EN_PIN D1
#define PIN_VBAT_ENABLE 14
#define PIN_HICHG 22
#define PIN_CHG 23
#define ACCEL_INT_PIN PIN_LSM6DS3TR_C_INT1

// ============================================================================
// ---  GLOBAL SYSTEM STATE ---
// ============================================================================
SFE_UBLOX_GNSS myGNSS;
LSM6DS3 IMU(I2C_MODE, 0x6A);

bool deviceConnected = false;
bool gpsEnabled = false;
bool imuEnabled = false;
bool pendingConfig = false;
bool lastPluggedInState = false;
bool batteryConnected = true;
uint8_t currentBatteryPercentage = 100;
const float batteryMultiplier = 3.0;

bool isCritical = false;
bool isNoBatteryMode = false;
int GPSFixType = 0;

unsigned long lastDisconnectTime = 0;
unsigned long lastActivityTime = 0;
unsigned long lastGpsRateCheckTime = 0;
unsigned int gpsUpdateCount = 0;
unsigned int gnssUpdateCount = 0;

float imu_ax = 0, imu_ay = 0, imu_az = 0;
float imu_gx = 0, imu_gy = 0, imu_gz = 0;

const uint8_t RACEBOX_SERVICE_UUID[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E};
const uint8_t RACEBOX_TX_UUID[]      = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E};
const uint8_t RACEBOX_RX_UUID[]      = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E};
const uint8_t RACEBOX_GNSS_UUID[]    = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x04, 0x00, 0x40, 0x6E};

#define ENABLE_OTA_DFU 1
#if ENABLE_OTA_DFU
BLEDfu bledfu;
#endif

BLEService rbService(RACEBOX_SERVICE_UUID);
BLECharacteristic rbTx(RACEBOX_TX_UUID);
BLECharacteristic rbRx(RACEBOX_RX_UUID);
BLECharacteristic rbGnss(RACEBOX_GNSS_UUID);

BLEService disService(UUID16_SVC_DEVICE_INFORMATION);
BLECharacteristic disModel(UUID16_CHR_MODEL_NUMBER_STRING);
BLECharacteristic disSerial(UUID16_CHR_SERIAL_NUMBER_STRING);
BLECharacteristic disFirmware(UUID16_CHR_FIRMWARE_REVISION_STRING);
BLECharacteristic disHardware(UUID16_CHR_HARDWARE_REVISION_STRING);
BLECharacteristic disManuf(UUID16_CHR_MANUFACTURER_NAME_STRING);

const int OnboardledPin = LED_BLUE;

// --- PROTOTYPES DES FONCTIONS ---
void enterDeepSleep();
bool configureGPS();
bool isCharging();
bool isPluggedIn();
float getBatteryVoltage();
void setupAdvertising(int8_t power, uint16_t interval);
void enableGPS();
void disableGPS();
void enableIMU();
void disableIMU();

// --- Flash QSPI ---
#define USE_EXPLICIT_FLASH_DEVICE 1

#if USE_EXPLICIT_FLASH_DEVICE
static const SPIFlash_Device_t P25Q16H_DEVICE = {
    .total_size = (1UL << 21),
    .start_up_time_us = 10000,
    .manufacturer_id = 0x85,
    .memory_type = 0x60,
    .capacity = 0x15,
    .max_clock_speed_mhz = 55,
    .quad_enable_bit_mask = 0x02,
    .has_sector_protection = 1,
    .supports_fast_read = 1,
    .supports_qspi = 1,
    .supports_qspi_writes = 1,
    .write_status_register_split = 1,
    .single_status_byte = 0,
    .is_fram = 0,
};
#endif

#if defined(EXTERNAL_FLASH_USE_QSPI)
Adafruit_FlashTransport_QSPI flashTransport;
#elif defined(EXTERNAL_FLASH_USE_SPI)
Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS, EXTERNAL_FLASH_USE_SPI);
#else
SPIClass SPI_QSPI(NRF_SPIM2, PIN_QSPI_IO1, PIN_QSPI_SCK, PIN_QSPI_IO0);
Adafruit_FlashTransport_SPI flashTransport(PIN_QSPI_CS, SPI_QSPI);
#endif
Adafruit_SPIFlash flash(&flashTransport);

#define REC_MAGIC 0x534D4252UL
#define REC_CFG_VERSION 2
#define META_A 0
#define META_B 4096
#define DATA_START 8192
#define SLOT_SIZE 81

static bool flashReady = false;
static uint32_t flashTotalBytes = 0;
static uint32_t totalSlots = 0;
static uint32_t usedSlots = 0;

#define REC_TYPE_DATA 0x21
#define REC_TYPE_STATE 0x26
#define REC_TYPE_EMPTY 0xFF

#define REC_STATE_OFF 0
#define REC_STATE_RUNNING 1
#define REC_STATE_PAUSED 2

#define RECFLAG_WAIT_FIX 0x01
#define RECFLAG_STATIONARY 0x02
#define RECFLAG_NOFIX 0x04
#define RECFLAG_AUTOSHUTDOWN 0x08
#define RECFLAG_WAIT_DATA 0x10

struct __attribute__((packed)) RecConfig {
  uint32_t magic;
  uint8_t version;
  uint8_t enabled;
  uint8_t dataRate;
  uint8_t flags;
  uint16_t statSpeed;
  uint16_t statInterval;
  uint16_t noFixInterval;
  uint16_t autoOffInterval;
  uint8_t gnssDynModel;
  uint8_t gnss3dSpeed;
  uint8_t gnssMinAcc;
  uint8_t reserved;
  uint32_t seq;
  uint32_t crc;
};

static RecConfig recCfg;
static uint8_t recState = REC_STATE_OFF;
static bool memoryFull = false;

static unsigned long slowSinceMs = 0;
static unsigned long noFixSinceMs = 0;
static unsigned long lastStoredDataMs = 0;
static bool anyDataSinceEnable = false;

#define TXQ_SIZE 2048
static uint8_t txq[TXQ_SIZE];
static uint16_t txqHead = 0, txqTail = 0;
static uint16_t negotiatedMtu = 23;

#define RXRING_SIZE 512
static uint8_t rxRing[RXRING_SIZE];
static volatile uint16_t rxHead = 0, rxTail = 0;
static uint8_t asmBuf[560];
static uint16_t asmLen = 0;
static volatile uint32_t rxByteCount = 0;
#define RX_DEBUG 0

static bool downloadActive = false;
static uint32_t downloadCursor = 0;
static uint32_t downloadEnd = 0;
static uint32_t downloadSkipped = 0;
static bool eraseActive = false;
static uint32_t eraseBlock = 0;
static uint32_t eraseBlockCount = 0;
static int lastErasePct = -1;

// --- Helper Utilities ---
template <typename T>
void writeLittleEndian(uint8_t *buffer, int offset, T value) {
  memcpy(buffer + offset, &value, sizeof(T));
}

void calculateChecksum(uint8_t *payload, uint16_t len, uint8_t cls, uint8_t id,
                       uint8_t *ckA, uint8_t *ckB) {
  *ckA = *ckB = 0;
  *ckA += cls;
  *ckB += *ckA;
  *ckA += id;
  *ckB += *ckA;
  *ckA += len & 0xFF;
  *ckB += *ckA;
  *ckA += len >> 8;
  *ckB += *ckA;
  for (uint16_t i = 0; i < len; i++) {
    *ckA += payload[i];
    *ckB += *ckA;
  }
}

static inline uint16_t txqUsed() {
  return (uint16_t)((txqHead - txqTail) & (TXQ_SIZE - 1));
}
static inline uint16_t txqFree() { return TXQ_SIZE - 1 - txqUsed(); }

static bool txqPush(const uint8_t *d, uint16_t n) {
  if (txqFree() < n)
    return false;
  for (uint16_t i = 0; i < n; i++) {
    txq[txqHead] = d[i];
    txqHead = (txqHead + 1) & (TXQ_SIZE - 1);
  }
  return true;
}

static void txqService() {
  if (!deviceConnected)
    return;

  uint16_t maxChunk = negotiatedMtu > 3 ? negotiatedMtu - 3 : 20;
  if (maxChunk > 244) maxChunk = 244;

  static uint8_t chunk[244];
  uint16_t n = txqUsed();
  while (n && n <= maxChunk) {
    for (uint16_t i = 0; i < n; i++)
      chunk[i] = txq[(txqTail + i) & (TXQ_SIZE - 1)];
    if (!rbTx.notify(chunk, n))
      return;
    txqTail = (txqTail + n) & (TXQ_SIZE - 1);
    n = txqUsed();
  }
}

static bool sendUbx(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  static uint8_t pkt[8 + 256];
  if (len > 256)
    return false;
  pkt[0] = 0xB5;
  pkt[1] = 0x62;
  pkt[2] = cls;
  pkt[3] = id;
  pkt[4] = len & 0xFF;
  pkt[5] = len >> 8;
  if (len && payload)
    memcpy(pkt + 6, payload, len);
  uint8_t ckA, ckB;
  calculateChecksum((uint8_t *)payload, len, cls, id, &ckA, &ckB);
  pkt[6 + len] = ckA;
  pkt[7 + len] = ckB;
  return txqPush(pkt, len + 8);
}

static void sendAck(uint8_t cls, uint8_t id) {
  uint8_t p[2] = {cls, id};
  sendUbx(0xFF, 0x02, p, 2);
}

static void sendNack(uint8_t cls, uint8_t id) {
  uint8_t p[2] = {cls, id};
  sendUbx(0xFF, 0x03, p, 2);
}

static inline uint32_t slotAddr(uint32_t i) {
  return DATA_START + i * (uint32_t)SLOT_SIZE;
}

static uint8_t slotType(uint32_t i) {
  uint8_t t = REC_TYPE_EMPTY;
  flash.readBuffer(slotAddr(i), &t, 1);
  return t;
}

static uint8_t lastMetaSlot = 1;

static uint32_t cfgCrc(const void *cfg) {
  const RecConfig *c = (const RecConfig *)cfg;
  const uint8_t *p = (const uint8_t *)c;
  uint32_t s = 0x1234ABCDUL;
  for (size_t i = 0; i < sizeof(RecConfig) - 4; i++)
    s = (s * 31u) + p[i];
  return s;
}

static bool cfgValid(const void *cfg) {
  const RecConfig *c = (const RecConfig *)cfg;
  return c->magic == REC_MAGIC && c->version == REC_CFG_VERSION && c->crc == cfgCrc(c);
}

static void saveConfig() {
  recCfg.magic = REC_MAGIC;
  recCfg.version = REC_CFG_VERSION;
  recCfg.seq++;
  recCfg.crc = cfgCrc(&recCfg);

  uint8_t target = lastMetaSlot ? 0 : 1;
  uint32_t addr = target ? META_B : META_A;
  flash.eraseSector(addr / 4096);
  flash.waitUntilReady();
  flash.writeBuffer(addr, (uint8_t *)&recCfg, sizeof(RecConfig));
  flash.waitUntilReady();

  RecConfig back;
  flash.readBuffer(addr, (uint8_t *)&back, sizeof(back));
  if (cfgValid(&back))
    lastMetaSlot = target;
  else
    Serial.println("⚠️ Écriture de configuration non confirmée.");
}

static bool loadConfig() {
  RecConfig a, b;
  flash.readBuffer(META_A, (uint8_t *)&a, sizeof(a));
  flash.readBuffer(META_B, (uint8_t *)&b, sizeof(b));
  bool va = cfgValid(&a), vb = cfgValid(&b);

  if (va && vb) {
    bool bIsNewer = (int32_t)(b.seq - a.seq) > 0;
    recCfg = bIsNewer ? b : a;
    lastMetaSlot = bIsNewer ? 1 : 0;
  } else if (va) {
    recCfg = a;
    lastMetaSlot = 0;
    Serial.println("⚠️ Exemplaire B illisible : reprise sur A.");
  } else if (vb) {
    recCfg = b;
    lastMetaSlot = 1;
    Serial.println("⚠️ Exemplaire A illisible : reprise sur B.");
  } else {
    return false;
  }
  return true;
}

static void defaultConfig() {
  uint32_t keepSeq = recCfg.seq;
  memset(&recCfg, 0, sizeof(recCfg));
  recCfg.magic = REC_MAGIC;
  recCfg.version = REC_CFG_VERSION;
  recCfg.seq = keepSeq;
  recCfg.enabled = 0;
  recCfg.dataRate = 0;
  recCfg.flags = 0x1F;
  recCfg.statSpeed = 1389;
  recCfg.statInterval = 30;
  recCfg.noFixInterval = 30;
  recCfg.autoOffInterval = 300;
  recCfg.gnssDynModel = 7;
  recCfg.gnss3dSpeed = 0;
  recCfg.gnssMinAcc = 3;
}

static void fullEraseBlocking() {
  Serial.println("🧹 Formatage initial de la mémoire flash...");
  uint32_t blocks = flashTotalBytes / 65536;
  for (uint32_t b = 0; b < blocks; b++) {
    flash.eraseBlock(b);
    flash.waitUntilReady();
    if ((b % 8) == 0)
      Serial.printf("   ... %lu%%\n", (unsigned long)(b * 100 / blocks));
  }
  usedSlots = 0;
  memoryFull = false;
  Serial.println("✅ Mémoire prête.");
}

static void locateWritePointer() {
  uint32_t lo = 0, hi = totalSlots;
  while (lo < hi) {
    uint32_t mid = lo + (hi - lo) / 2;
    if (slotType(mid) == REC_TYPE_EMPTY)
      hi = mid;
    else
      lo = mid + 1;
  }
  usedSlots = lo;
  memoryFull = (usedSlots >= totalSlots);
}

static bool storeRecord(uint8_t type, const uint8_t *payload80) {
  if (!flashReady)
    return false;
  if (usedSlots >= totalSlots) {
    if (!memoryFull)
      Serial.println("⚠️ Mémoire pleine : enregistrement interrompu.");
    memoryFull = true;
    return false;
  }
  uint8_t buf[SLOT_SIZE];
  buf[0] = type;
  memcpy(buf + 1, payload80, 80);
  flash.writeBuffer(slotAddr(usedSlots), buf, SLOT_SIZE);
  usedSlots++;
  return true;
}

static void buildStatePayload(uint8_t *p, uint8_t newState) {
  memset(p, 0, 80);
  p[0] = newState;
  p[1] = 0;
  p[2] = recCfg.dataRate;
  p[3] = recCfg.flags;
  writeLittleEndian(p, 4, recCfg.statSpeed);
  writeLittleEndian(p, 6, recCfg.statInterval);
  writeLittleEndian(p, 8, recCfg.noFixInterval);
  writeLittleEndian(p, 10, recCfg.autoOffInterval);
}

static void announceStateChange(uint8_t newState, bool alsoStore) {
  uint8_t p[80];
  buildStatePayload(p, newState);
  if (alsoStore)
    storeRecord(REC_TYPE_STATE, p);
  if (deviceConnected)
    sendUbx(0xFF, 0x26, p, 12);
}

static uint8_t rateToHz(uint8_t r) {
  switch (r) {
  case 1: return 10;
  case 2: return 5;
  case 3: return 1;
  case 4: return 20;
  default: return 25;
  }
}

static void applyNavRate() {
  if (!gpsEnabled)
    return;
  uint8_t hz = recCfg.enabled ? rateToHz(recCfg.dataRate) : 25;
  myGNSS.setNavigationFrequency(hz);
  Serial.printf("🛰️ Fréquence GNSS : %u Hz\n", hz);
}

static void startRecording() {
  if (memoryFull) {
    Serial.println("⚠️ Impossible de démarrer : mémoire pleine.");
    return;
  }
  recCfg.enabled = 1;
  recState = REC_STATE_RUNNING;
  anyDataSinceEnable = false;
  slowSinceMs = 0;
  noFixSinceMs = 0;
  lastStoredDataMs = millis();
  saveConfig();
  applyNavRate();
  announceStateChange(REC_STATE_RUNNING, true);
  Serial.println("⏺️ Enregistrement autonome DÉMARRÉ.");
}

static void stopRecording() {
  if (recState != REC_STATE_OFF)
    announceStateChange(REC_STATE_OFF, true);
  recCfg.enabled = 0;
  recState = REC_STATE_OFF;
  saveConfig();
  applyNavRate();
  Serial.println("⏹️ Enregistrement autonome ARRÊTÉ.");
}

static void recordTick(const uint8_t *payload80) {
  if (recState == REC_STATE_OFF || !flashReady)
    return;

  bool haveFix = (myGNSS.packetUBXNAVPVT != NULL && myGNSS.packetUBXNAVPVT->data.fixType == 3);
  int32_t gSpeed = (myGNSS.packetUBXNAVPVT != NULL) ? myGNSS.packetUBXNAVPVT->data.gSpeed : 0;
  unsigned long now = millis();

  if ((recCfg.flags & RECFLAG_WAIT_FIX) && !haveFix && !anyDataSinceEnable)
    return;

  bool shouldPause = false;

  if (recCfg.flags & RECFLAG_NOFIX) {
    if (!haveFix) {
      if (noFixSinceMs == 0)
        noFixSinceMs = now;
      if ((now - noFixSinceMs) >= (unsigned long)recCfg.noFixInterval * 1000UL)
        shouldPause = true;
    } else {
      noFixSinceMs = 0;
    }
  }

  if ((recCfg.flags & RECFLAG_STATIONARY) && haveFix) {
    if (gSpeed < (int32_t)recCfg.statSpeed) {
      if (slowSinceMs == 0)
        slowSinceMs = now;
      if ((now - slowSinceMs) >= (unsigned long)recCfg.statInterval * 1000UL)
        shouldPause = true;
    } else {
      slowSinceMs = 0;
    }
  }

  if (shouldPause) {
    if (recState == REC_STATE_RUNNING) {
      recState = REC_STATE_PAUSED;
      announceStateChange(REC_STATE_PAUSED, true);
      Serial.println("⏸️ Enregistrement en pause (filtre actif).");
    }
    return;
  }

  if (recState == REC_STATE_PAUSED) {
    recState = REC_STATE_RUNNING;
    announceStateChange(REC_STATE_RUNNING, false);
    Serial.println("▶️ Reprise de l'enregistrement.");
  }

  if (storeRecord(REC_TYPE_DATA, payload80)) {
    anyDataSinceEnable = true;
    lastStoredDataMs = now;
  }
}

static void serviceAutoShutdown() {
  if (!(recCfg.flags & RECFLAG_AUTOSHUTDOWN) || recState == REC_STATE_OFF)
    return;
  if (deviceConnected || downloadActive || eraseActive)
    return;
  if (isPluggedIn() || isCharging())
    return;
  if (millis() < 120000UL)
    return;
  if (!anyDataSinceEnable)
    return;
  if (!(recCfg.flags & (RECFLAG_WAIT_FIX | RECFLAG_STATIONARY | RECFLAG_NOFIX)))
    return;
  if ((recCfg.flags & RECFLAG_WAIT_DATA) && !anyDataSinceEnable)
    return;
  if ((millis() - lastStoredDataMs) > (unsigned long)recCfg.autoOffInterval * 1000UL) {
    Serial.println("⌛ Auto-Shutdown : aucune donnée depuis un moment.");
    enterDeepSleep();
  }
}

static void startDownload() {
  downloadActive = true;
  downloadSkipped = 0;
  downloadCursor = 0;
  downloadEnd = usedSlots;
  uint8_t p[4];
  writeLittleEndian(p, 0, (uint32_t)downloadEnd);
  sendUbx(0xFF, 0x23, p, 4);
  Serial.printf("⬇️ Téléchargement de %lu enregistrements...\n", (unsigned long)downloadEnd);
}

static bool recordLooksValid(const uint8_t *p) {
  int32_t lat, lon, gSpeed;
  memcpy(&lon, p + 24, 4);
  memcpy(&lat, p + 28, 4);
  memcpy(&gSpeed, p + 48, 4);
  if (p[20] > 5) return false;
  if (p[23] > 60) return false;
  if (lat > 900000000L || lat < -900000000L) return false;
  if (lon > 1800000000L || lon < -1800000000L) return false;
  if (gSpeed < 0 || gSpeed > 140000000L) return false;
  return true;
}

static void serviceDownload() {
  if (!downloadActive)
    return;
  while (downloadCursor < downloadEnd && txqFree() >= 96) {
    uint8_t slot[SLOT_SIZE];
    flash.readBuffer(slotAddr(downloadCursor), slot, SLOT_SIZE);
    if (slot[0] == REC_TYPE_DATA) {
      if (recordLooksValid(slot + 1)) {
        if (!sendUbx(0xFF, 0x21, slot + 1, 80))
          break;
      } else {
        downloadSkipped++;
      }
    } else if (slot[0] == REC_TYPE_STATE) {
      if (!sendUbx(0xFF, 0x26, slot + 1, 12))
        break;
    }
    downloadCursor++;
  }
  if (downloadCursor >= downloadEnd && txqUsed() == 0) {
    downloadActive = false;
    sendAck(0xFF, 0x23);
    Serial.printf("✅ Téléchargement terminé (%lu enregistrement(s) corrompu(s) écarté(s)).\n", (unsigned long)downloadSkipped);
  }
}

static void startErase() {
  eraseActive = true;
  eraseBlock = 0;
  uint32_t usedBytes = DATA_START + usedSlots * (uint32_t)SLOT_SIZE;
  eraseBlockCount = (usedBytes + 65535) / 65536;
  if (eraseBlockCount == 0)
    eraseBlockCount = 1;
  lastErasePct = -1;
  Serial.println("🧹 Effacement de la mémoire...");
}

static void serviceErase() {
  if (!eraseActive)
    return;
  flash.eraseBlock(eraseBlock);
  flash.waitUntilReady();
  eraseBlock++;

  int pct = (int)((eraseBlock * 100UL) / eraseBlockCount);
  if (pct > 100)
    pct = 100;
  if (pct != lastErasePct) {
    lastErasePct = pct;
    uint8_t p = (uint8_t)pct;
    sendUbx(0xFF, 0x24, &p, 1);
  }

  if (eraseBlock >= eraseBlockCount) {
    eraseActive = false;
    usedSlots = 0;
    memoryFull = false;
    saveConfig();
    sendAck(0xFF, 0x24);
    Serial.println("✅ Mémoire effacée.");
  }
}

static void handleCommand(uint8_t cls, uint8_t id, uint8_t *pl, uint16_t len) {
  Serial.printf("📨 CMD 0x%02X 0x%02X (%u o)", cls, id, len);
  for (uint16_t i = 0; i < len && i < 16; i++)
    Serial.printf(" %02X", pl[i]);
  Serial.println();

  if (cls != 0xFF) {
    if (gpsEnabled) {
      uint8_t hdr[6] = {0xB5, 0x62, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
      uint8_t ckA, ckB;
      calculateChecksum(pl, len, cls, id, &ckA, &ckB);
      Serial1.write(hdr, 6);
      if (len)
        Serial1.write(pl, len);
      Serial1.write(ckA);
      Serial1.write(ckB);
    }
    return;
  }

  switch (id) {
  case 0x22: {
    uint8_t p[12];
    memset(p, 0, 12);
    p[0] = (recState != REC_STATE_OFF) ? 1 : 0;
    p[1] = totalSlots ? (uint8_t)((usedSlots * 100UL) / totalSlots) : 0;
    p[2] = 0x00;
    p[3] = 0;
    writeLittleEndian(p, 4, (uint32_t)usedSlots);
    writeLittleEndian(p, 8, (uint32_t)totalSlots);
    sendUbx(0xFF, 0x22, p, 12);
    break;
  }
  case 0x25: {
    if (len == 0) {
      uint8_t p[12];
      memset(p, 0, 12);
      p[0] = recCfg.enabled;
      p[1] = recCfg.dataRate;
      p[2] = recCfg.flags;
      p[3] = 0;
      writeLittleEndian(p, 4, recCfg.statSpeed);
      writeLittleEndian(p, 6, recCfg.statInterval);
      writeLittleEndian(p, 8, recCfg.noFixInterval);
      writeLittleEndian(p, 10, recCfg.autoOffInterval);
      sendUbx(0xFF, 0x25, p, 12);
      break;
    }
    if (len < 10) {
      sendNack(0xFF, 0x25);
      break;
    }
    if (pl[0] == 0) {
      stopRecording();
      sendAck(0xFF, 0x25);
      break;
    }
    if (memoryFull) {
      sendNack(0xFF, 0x25);
      break;
    }
    if (pl[1] > 4) {
      sendNack(0xFF, 0x25);
      break;
    }
    recCfg.dataRate = pl[1];
    recCfg.flags = pl[2];
    memcpy(&recCfg.statSpeed, pl + 4, 2);
    memcpy(&recCfg.statInterval, pl + 6, 2);
    memcpy(&recCfg.noFixInterval, pl + 8, 2);
    if (len >= 12)
      memcpy(&recCfg.autoOffInterval, pl + 10, 2);
    sendAck(0xFF, 0x25);
    startRecording();
    break;
  }
  case 0x23: {
    if (len >= 1) {
      if (downloadActive) {
        downloadCursor = downloadEnd;
        Serial.println("⏹️ Téléchargement annulé.");
      }
      break;
    }
    if (downloadActive || eraseActive) {
      sendNack(0xFF, 0x23);
      break;
    }
    startDownload();
    break;
  }
  case 0x24: {
    if (len >= 1) {
      Serial.println("ℹ️ Annulation ignorée : effacement mené à son terme.");
      break;
    }
    if (downloadActive || eraseActive) {
      sendNack(0xFF, 0x24);
      break;
    }
    if (recState != REC_STATE_OFF)
      stopRecording();
    startErase();
    break;
  }
  case 0x30:
    sendAck(0xFF, 0x30);
    break;
  case 0x27: {
    if (len == 0) {
      uint8_t p[3] = {recCfg.gnssDynModel, recCfg.gnss3dSpeed, recCfg.gnssMinAcc};
      sendUbx(0xFF, 0x27, p, 3);
      break;
    }
    if (len < 3 || pl[0] > 8) {
      sendNack(0xFF, 0x27);
      break;
    }
    recCfg.gnssDynModel = pl[0];
    recCfg.gnss3dSpeed = pl[1] ? 1 : 0;
    recCfg.gnssMinAcc = pl[2];
    if (gpsEnabled)
      myGNSS.setDynamicModel((dynModel)recCfg.gnssDynModel);
    saveConfig();
    sendAck(0xFF, 0x27);
    break;
  }
  case 0xF0: {
    char info[128];
    int n = snprintf(info, sizeof(info), "%s|%s|%s|%u|%s", MODEL_STRING, FIRMWARE_VER, BUILD_STAMP, 16u, DEVICE_NICKNAME);
    if (n < 0) n = 0;
    if (n > (int)sizeof(info)) n = sizeof(info);
    sendUbx(0xFF, 0xF0, (const uint8_t *)info, (uint16_t)n);
    break;
  }
  case 0x01:
    break;
  default:
    sendNack(0xFF, id);
    break;
  }
}

static void serviceRxParser() {
#if RX_DEBUG
  static uint32_t lastReported = 0;
  if (rxByteCount != lastReported) {
    uint16_t pending = (uint16_t)((rxHead - rxTail) & (RXRING_SIZE - 1));
    Serial.printf("🔻 RX brut : %lu octets au total, %u en attente :", (unsigned long)rxByteCount, pending);
    for (uint16_t i = 0; i < pending && i < 24; i++)
      Serial.printf(" %02X", rxRing[(rxTail + i) & (RXRING_SIZE - 1)]);
    Serial.println();
    lastReported = rxByteCount;
  }
#endif

  while (rxTail != rxHead) {
    uint8_t b = rxRing[rxTail];
    rxTail = (rxTail + 1) & (RXRING_SIZE - 1);

    if (asmLen == 0) {
      if (b == 0xB5)
        asmBuf[asmLen++] = b;
      continue;
    }
    if (asmLen == 1) {
      if (b == 0x62)
        asmBuf[asmLen++] = b;
      else
        asmLen = (b == 0xB5) ? 1 : 0;
      continue;
    }
    if (asmLen < sizeof(asmBuf))
      asmBuf[asmLen++] = b;
    else {
      asmLen = 0;
      continue;
    }

    if (asmLen >= 6) {
      uint16_t plen = asmBuf[4] | ((uint16_t)asmBuf[5] << 8);
      if (plen > 512) {
        asmLen = 0;
        continue;
      }
      if (asmLen == plen + 8) {
        uint8_t ckA, ckB;
        calculateChecksum(asmBuf + 6, plen, asmBuf[2], asmBuf[3], &ckA, &ckB);
        if (ckA == asmBuf[6 + plen] && ckB == asmBuf[7 + plen])
          handleCommand(asmBuf[2], asmBuf[3], asmBuf + 6, plen);
        else
          Serial.println("⚠️ Checksum invalide sur une commande reçue.");
        asmLen = 0;
      }
    }
  }
}

static void setupStorage() {
  bool ok = flash.begin();
#if USE_EXPLICIT_FLASH_DEVICE
  if (!ok)
    ok = flash.begin(&P25Q16H_DEVICE, 1);
#endif
  if (!ok) {
    Serial.println("❌ Flash QSPI non détectée : enregistrement désactivé.");
    flashReady = false;
    return;
  }
  flashReady = true;
  flashTotalBytes = flash.size();
  if (flashTotalBytes == 0)
    flashTotalBytes = (1UL << 21);
  totalSlots = (flashTotalBytes - DATA_START) / SLOT_SIZE;
  Serial.printf("💾 Flash %lu Ko | capacité : %lu enregistrements\n", (unsigned long)(flashTotalBytes / 1024), (unsigned long)totalSlots);

  if (!loadConfig()) {
    Serial.println("ℹ️ Aucune configuration valide en mémoire.");
    bool alreadyBlank = true;
    uint8_t probe[64];
    for (uint32_t addr = 0; addr < flashTotalBytes && alreadyBlank; addr += 65536) {
      flash.readBuffer(addr, probe, sizeof(probe));
      for (uint8_t i = 0; i < sizeof(probe); i++)
        if (probe[i] != 0xFF) {
          alreadyBlank = false;
          break;
        }
    }
    if (alreadyBlank) {
      Serial.println("✅ Mémoire déjà vierge, formatage inutile.");
      usedSlots = 0;
      memoryFull = false;
    } else {
      fullEraseBlocking();
    }
    defaultConfig();
    saveConfig();
  } else {
    locateWritePointer();
  }
  Serial.printf("💾 Occupation : %lu / %lu (%lu%%)\n", (unsigned long)usedSlots, (unsigned long)totalSlots, (unsigned long)(totalSlots ? usedSlots * 100 / totalSlots : 0));

  uint32_t freeSlots = totalSlots - usedSlots;
  Serial.printf("💾 Reste ~%lu min à %u Hz\n", (unsigned long)(freeSlots / (60UL * rateToHz(recCfg.dataRate))), rateToHz(recCfg.dataRate));
}

bool isCharging() { return digitalRead(PIN_CHG) == LOW; }

struct VoltagePoint {
  float voltage;
  uint8_t percentage;
};

const VoltagePoint batteryMap[] = {
    {4.13, 100}, {4.12, 98}, {4.10, 95}, {4.05, 92}, {4.00, 88}, {3.98, 85},
    {3.96, 82},  {3.94, 79}, {3.92, 75}, {3.90, 72}, {3.88, 68}, {3.85, 65},
    {3.82, 62},  {3.78, 55}, {3.72, 45}, {3.68, 35}, {3.63, 25}, {3.58, 18},
    {3.50, 10},  {3.35, 5},  {3.20, 0}};

const uint8_t mapSize = sizeof(batteryMap) / sizeof(VoltagePoint);

float percentageFromVoltage(float v) {
  if (v >= batteryMap[0].voltage)
    return 100.0f;
  if (v <= batteryMap[mapSize - 1].voltage)
    return 0.0f;

  for (int i = 0; i < mapSize - 1; i++) {
    float vHigh = batteryMap[i].voltage;
    float vLow = batteryMap[i + 1].voltage;
    if (v <= vHigh && v > vLow) {
      uint8_t pHigh = batteryMap[i].percentage;
      uint8_t pLow = batteryMap[i + 1].percentage;
      return pLow + (v - vLow) * (pHigh - pLow) / (vHigh - vLow);
    }
  }
  return 0.0f;
}

void updateBatteryState() {
  static float filteredPct = -1.0;
  bool pluggedIn = isCharging();
  float currentV = getBatteryVoltage();
  float rawPct = percentageFromVoltage(currentV);

  if (filteredPct < 0) {
    filteredPct = rawPct;
    currentBatteryPercentage = (uint8_t)rawPct;
  }

  filteredPct = (rawPct * 15 + filteredPct * 85) / 100.0f;
  uint8_t rounded = (uint8_t)(filteredPct + 0.5);

  isCritical = (currentV < 3.35);

  if (currentBatteryPercentage == 100 && !pluggedIn && rounded > 95) {
    rounded = 100;
  }

  int diff = (int)rounded - (int)currentBatteryPercentage;
  if (diff < 0) diff = -diff;
  if (diff >= 2 || rounded == 100 || rounded == 0) {
    if (pluggedIn && rounded > currentBatteryPercentage)
      currentBatteryPercentage = rounded;
    else if (!pluggedIn && rounded < currentBatteryPercentage) {
      currentBatteryPercentage = rounded;
      if (rawPct > currentBatteryPercentage + 5.0f || currentBatteryPercentage > rawPct + 20.0f) {
        currentBatteryPercentage = (uint8_t)rawPct;
        filteredPct = rawPct;
      }
    }
  }
}

bool isPluggedIn() {
  return NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk;
}

float getBatteryVoltage() {
  digitalWrite(PIN_VBAT_ENABLE, LOW);
  delay(1);

  uint32_t sum = 0;
  for (int i = 0; i < 4; i++) {
    sum += analogRead(PIN_VBAT);
    delayMicroseconds(50);
  }
  float voltage = batteryMultiplier * 3.6f * sum / (4096.0f * 4.0f);

  if (gpsEnabled && !isPluggedIn()) {
    voltage += 0.01f;
  }

  if (!isCharging() && !isPluggedIn()) {
    digitalWrite(PIN_VBAT_ENABLE, HIGH);
  }
  return voltage;
}

bool buildDataPayload(uint8_t *payload) {
  if (myGNSS.packetUBXNAVPVT == NULL)
    return false;

  memset(payload, 0, 80);
  auto *data = &myGNSS.packetUBXNAVPVT->data;

  payload[0] = data->iTOW & 0xFF; payload[1] = (data->iTOW >> 8) & 0xFF;
  payload[2] = (data->iTOW >> 16) & 0xFF; payload[3] = (data->iTOW >> 24) & 0xFF;
  payload[4] = data->year & 0xFF; payload[5] = (data->year >> 8) & 0xFF;
  payload[6] = data->month & 0xFF; payload[7] = data->day & 0xFF;
  payload[8] = data->hour & 0xFF; payload[9] = data->min & 0xFF;
  payload[10] = data->sec & 0xFF;

  uint8_t validFlags = data->valid.bits.validDate | (data->valid.bits.validTime << 1) | (data->valid.bits.fullyResolved << 2);
  payload[11] = validFlags;
  payload[12] = data->tAcc & 0xFF; payload[13] = (data->tAcc >> 8) & 0xFF;
  payload[14] = (data->tAcc >> 16) & 0xFF; payload[15] = (data->tAcc >> 24) & 0xFF;
  payload[16] = data->nano & 0xFF; payload[17] = (data->nano >> 8) & 0xFF;
  payload[18] = (data->nano >> 16) & 0xFF; payload[19] = (data->nano >> 24) & 0xFF;
  payload[20] = data->fixType & 0xFF;

  uint8_t fixFlags = (data->fixType == 3) | (myGNSS.getHeadVehValid() ? 1 << 5 : 0);
  payload[21] = fixFlags;

  uint8_t dtFlags = (data->valid.bits.validTime ? 1 << 5 : 0) | (data->valid.bits.validDate ? 1 << 6 : 0) | ((data->valid.bits.validTime && data->valid.bits.fullyResolved) ? 1 << 7 : 0);
  payload[22] = dtFlags;
  payload[23] = data->numSV & 0xFF;
  payload[24] = data->lon & 0xFF; payload[25] = (data->lon >> 8) & 0xFF;
  payload[26] = (data->lon >> 16) & 0xFF; payload[27] = (data->lon >> 24) & 0xFF;
  payload[28] = data->lat & 0xFF; payload[29] = (data->lat >> 8) & 0xFF;
  payload[30] = (data->lat >> 16) & 0xFF; payload[31] = (data->lat >> 24) & 0xFF;
  payload[32] = data->height & 0xFF; payload[33] = (data->height >> 8) & 0xFF;
  payload[34] = (data->height >> 16) & 0xFF; payload[35] = (data->height >> 24) & 0xFF;
  payload[36] = data->hMSL & 0xFF; payload[37] = (data->hMSL >> 8) & 0xFF;
  payload[38] = (data->hMSL >> 16) & 0xFF; payload[39] = (data->hMSL >> 24) & 0xFF;
  payload[40] = data->hAcc & 0xFF; payload[41] = (data->hAcc >> 8) & 0xFF;
  payload[42] = (data->hAcc >> 16) & 0xFF; payload[43] = (data->hAcc >> 24) & 0xFF;
  payload[44] = data->vAcc & 0xFF; payload[45] = (data->vAcc >> 8) & 0xFF;
  payload[46] = (data->vAcc >> 16) & 0xFF; payload[47] = (data->vAcc >> 24) & 0xFF;
  payload[48] = data->gSpeed & 0xFF; payload[49] = (data->gSpeed >> 8) & 0xFF;
  payload[50] = (data->gSpeed >> 16) & 0xFF; payload[51] = (data->gSpeed >> 24) & 0xFF;
  payload[52] = data->headMot & 0xFF; payload[53] = (data->headMot >> 8) & 0xFF;
  payload[54] = (data->headMot >> 16) & 0xFF; payload[55] = (data->headMot >> 24) & 0xFF;
  payload[56] = data->sAcc & 0xFF; payload[57] = (data->sAcc >> 8) & 0xFF;
  payload[58] = (data->sAcc >> 16) & 0xFF; payload[59] = (data->sAcc >> 24) & 0xFF;
  payload[60] = data->headAcc & 0xFF; payload[61] = (data->headAcc >> 8) & 0xFF;
  payload[62] = (data->headAcc >> 16) & 0xFF; payload[63] = (data->headAcc >> 24) & 0xFF;
  payload[64] = data->pDOP & 0xFF; payload[65] = (data->pDOP >> 8) & 0xFF;

  if (data->fixType < 2)
    payload[66] = 1;

  payload[67] = currentBatteryPercentage & 0x7F | (isCharging() ? 0x80 : 0);

  payload[68] = (uint8_t)((imu_ax + 512) / 10); // ~9% error
  payload[69] = (uint8_t)((imu_ay + 512) / 10);
  payload[70] = (uint8_t)((imu_az + 512) / 10);
  payload[71] = (uint8_t)((imu_gx + 512) / 10);
  payload[72] = (uint8_t)((imu_gy + 512) / 10);
  payload[73] = (uint8_t)((imu_gz + 512) / 10);

  return true;
}

void sendLivePacket(const uint8_t *payload) {
  if (!deviceConnected)
    return;
  if (downloadActive || eraseActive)
    return;

  static uint8_t packet[88] = {0};
  packet[0] = 0xB5;
  packet[1] = 0x62;
  packet[2] = 0xFF;
  packet[3] = 0x01;
  packet[4] = 80;
  packet[5] = 0;
  memcpy(packet + 6, payload, 80);

  uint8_t ckA, ckB;
  calculateChecksum((uint8_t *)payload, 80, 0xFF, 0x01, &ckA, &ckB);
  packet[86] = ckA;
  packet[87] = ckB;

  if (txqFree() < 88 + 256)
    return;
  if (txqPush(packet, 88))
    gpsUpdateCount++;
}

void processGNSS() {
  if (!gpsEnabled)
    return;

  static unsigned long lastValidData = 0;
  static uint32_t lastITOW = 0;

  if (myGNSS.getPVT()) {
    lastValidData = millis();
    auto *data = &myGNSS.packetUBXNAVPVT->data;

    if (data->iTOW != lastITOW) {
      lastITOW = data->iTOW;
      gnssUpdateCount++;

      uint8_t payload[80];
      if (buildDataPayload(payload)) {
        recordTick(payload);
        sendLivePacket(payload);
      }
    }
  } else if (deviceConnected && millis() - lastValidData > 2000) {
    myGNSS.checkUblox();
  }
}

void processIMU() {
  if (!imuEnabled)
    return;

  uint16_t ax, ay, az, gx, gy, gz;
  IMU.readRawAccel(&ax, &ay, &az);
  IMU.readRawGyro(&gx, &gy, &gz);
  imu_ax = (float)ax / 32768.0f * 16.0f;
  imu_ay = (float)ay / 32768.0f * 16.0f;
  imu_az = (float)az / 32768.0f * 16.0f;
  imu_gx = (float)gx / 32768.0f * 500.0f;
  imu_gy = (float)gy / 32768.0f * 500.0f;
  imu_gz = (float)gz / 32768.0f * 500.0f;
}

bool resetGpsBaudRate() {
  Serial.println("🔍 Deep Scanning for GNSS activity...");
  long bauds[] = {9600, 38400, 115200, 57600};

  for (int b = 0; b < 4; b++) {
    Serial.print("Checking ");
    Serial.print(bauds[b]);
    Serial.print(" baud: ");

    Serial1.end();
    delay(100);
    Serial1.begin(bauds[b]);

    unsigned long sniffStart = millis();
    bool activity = false;
    while (millis() - sniffStart < 1500) {
      if (Serial1.available()) {
        activity = true;
        break;
      }
    }

    if (activity) {
      Serial.print("RAW DATA DETECTED! ");
      delay(200);
      while (Serial1.available())
        Serial1.read();

      if (myGNSS.begin(Serial1)) {
        Serial.println("✅ UBX Protocol Synced!");

        if (bauds[b] != GPS_BAUD) {
          Serial.print("Elevating to ");
          Serial.print(GPS_BAUD);
          Serial.println(" baud...");
          myGNSS.setSerialRate(GPS_BAUD);
          delay(100);
          Serial1.end();
          delay(100);
          Serial1.begin(GPS_BAUD);
        }

        myGNSS.saveConfiguration();
        return true;
      } else {
        Serial.println("❌ Bytes received, but u-blox library could not sync (Check protocol/clones).");
      }
    } else {
      Serial.println("Silent.");
    }

    Serial1.end();
  }

  Serial.println("❌ GNSS not detected. Check VCC voltage or TX/RX wiring.");
  return false;
}

bool configureGPS() {
  if (!pendingConfig)
    return false;
  Serial.println("⚙️ Syncing GPS Settings...");
  Serial1.begin(GPS_BAUD);

  bool detected = false;
  for (int i = 0; i < 3; i++) {
    if (myGNSS.begin(Serial1)) {
      detected = true;
      break;
    }
    delay(20);
  }

  if (!detected) {
    return resetGpsBaudRate();
  }

  myGNSS.setPortOutput(COM_PORT_UART1, COM_TYPE_UBX);
  myGNSS.setAutoPVT(true);
  myGNSS.setDynamicModel((dynModel)recCfg.gnssDynModel);

  myGNSS.setVal8(0x10220001, 0);
  myGNSS.setVal8(0x10220002, 0);
  myGNSS.setVal8(0x20250038, 0);
  myGNSS.setVal8(0x201100D5, 1);

#ifdef ENABLE_GNSS_GPS
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS);
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GPS);
#endif

#ifdef ENABLE_GNSS_GALILEO
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO);
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GALILEO);
#endif

#ifdef ENABLE_GNSS_GLONASS
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS);
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GLONASS);
#endif

#ifdef ENABLE_GNSS_BEIDOU
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU);
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_BEIDOU);
#endif

#ifdef ENABLE_GNSS_SBAS
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_SBAS);
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_SBAS);
#endif

#ifdef ENABLE_GNSS_QZSS
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_QZSS);
#else
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_QZSS);
#endif

  {
    uint8_t navHz = recCfg.enabled ? rateToHz(recCfg.dataRate) : MAX_NAVIGATION_RATE;
    myGNSS.setNavigationFrequency(navHz);
  }

  pendingConfig = false;
  Serial.println("✅ Configuration complete.");
  return true;
}

void setupAdvertising(int8_t power, uint16_t interval) {
  if (deviceConnected)
    return;

  Bluefruit.Advertising.stop();
  Bluefruit.setTxPower(power);
  Bluefruit.Advertising.setInterval(interval, interval + 200);

  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  Bluefruit.Advertising.addService(rbService);
  Bluefruit.Advertising.addService(disService);

  Bluefruit.ScanResponse.clearData();
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.start(0);
}

void enableGPS() {
  if (gpsEnabled)
    return;
#ifdef PCB_VERSION
  digitalWrite(GPS_EN_PIN, LOW);
#else
  digitalWrite(GPS_EN_PIN, HIGH);
#endif
  gpsEnabled = true;
  delay(100);
  if (!deviceConnected) {
    setupAdvertising(0, FAST_ADV_INTERVAL);
  }
}

void disableGPS() {
  GPSFixType = 0;
  pendingConfig = true;
#ifdef PCB_VERSION
  digitalWrite(GPS_EN_PIN, HIGH);
#else
  digitalWrite(GPS_EN_PIN, LOW);
#endif
  gpsEnabled = false;

  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  if (!deviceConnected) {
    setupAdvertising(LOW_POWER_BT_TX_POWER, ECO_ADV_INTERVAL);
  }
}

void enableIMU() {
  if (imuEnabled)
    return;
  IMU.settings.accelSampleRate = 1660;
  IMU.settings.gyroSampleRate = 1660;
  IMU.settings.accelRange = 16;
  IMU.settings.gyroRange = 500;

  if (IMU.begin() != 0)
    return;

  IMU.writeRegister(0x13, 0x02);
  IMU.writeRegister(0x15, 0x01);
  IMU.writeRegister(0x17, 0x21);

  imuEnabled = true;
}

void disableIMU() {
  if (!imuEnabled)
    return;
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x00);
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x00);
  imuEnabled = false;
}

void powerDownSensors() {
  if (gpsEnabled)
    disableGPS();
  if (imuEnabled)
    disableIMU();

#ifdef PCB_VERSION
  digitalWrite(GPS_EN_PIN, HIGH);
#else
  digitalWrite(GPS_EN_PIN, LOW);
#endif
}

void managePower() {
  if (isNoBatteryMode) {
    if (!gpsEnabled)
      enableGPS();
    configureGPS();
    if (!imuEnabled)
      enableIMU();
    return;
  }

  bool currentlyPluggedIn = isPluggedIn();

  if (!currentlyPluggedIn && !deviceConnected && currentBatteryPercentage == 0) {
    enterDeepSleep();
  }

  bool shouldBeActive = deviceConnected || (currentlyPluggedIn && !SLEEP_WHILE_CHARGING) || (recState != REC_STATE_OFF);

  if (shouldBeActive) {
    lastActivityTime = millis();
    lastDisconnectTime = millis();
    if (!gpsEnabled) {
      enableGPS();
    }
    configureGPS();
    enableIMU();
  } else {
    if (imuEnabled)
      disableIMU();
  }

  if (!deviceConnected && gpsEnabled && SLEEP_WHILE_CHARGING && recState == REC_STATE_OFF) {
    if (millis() - lastDisconnectTime > GPS_HOT_TIMEOUT_MS) {
      Serial.printf("⏰ GPS Hot Timeout Reached. Rebooting to clear hardware state...\n");
      Serial.flush();
      delay(10);
      NVIC_SystemReset();
    }
  }

  if (ENABLE_DEEP_SLEEP && !deviceConnected && !currentlyPluggedIn) {
    if (millis() - lastActivityTime > (DEEP_SLEEP_DAYS * 86400000UL))
      enterDeepSleep();
  }

  if (lastPluggedInState && !currentlyPluggedIn) {
    lastActivityTime = millis();
    if (!deviceConnected)
      lastDisconnectTime = millis();
  }
  lastPluggedInState = currentlyPluggedIn;
}

void manageBatterySampling() {
  if (isNoBatteryMode) {
    currentBatteryPercentage = 100;
    isCritical = false;
    batteryConnected = false;
    return;
  }

  static unsigned long lastBatteryUpdate = 0;
  const unsigned long batteryInterval = 30000;

  bool charging = isCharging();

  static bool lastChargingStatus = false;
  bool stateChanged = (charging != lastChargingStatus);

  batteryConnected = true;

  if (millis() - lastBatteryUpdate >= batteryInterval || stateChanged || lastBatteryUpdate == 0) {
    lastBatteryUpdate = millis();
    lastChargingStatus = charging;
    updateBatteryState();
  }
}

void reportSystemStats() {
  if (millis() - lastGpsRateCheckTime < SYSTEM_RATE_REPORT_MS)
    return;

  static float cachedBatVoltage = 0.0f;
  if (!cachedBatVoltage || millis() - lastGpsRateCheckTime > 1000) {
    cachedBatVoltage = getBatteryVoltage();
  }
  float batVoltage = cachedBatVoltage;
  Serial.printf("POWER   | Bat: %d%% (%0.2fV) \n", currentBatteryPercentage, batVoltage);
  Serial.printf("STATE   | Charging: %s | USB: %s | BLE: %s | BAT: %s\n",
                isCharging() ? "YES ⚡" : "NO 🔋",
                isPluggedIn() ? "CONNECTED" : "DISCONNECTED",
                deviceConnected ? "CONNECTED" : "IDLE",
                batteryConnected ? "PRESENT" : "MISSING");
  if (gpsEnabled && myGNSS.packetUBXNAVPVT) {
    Serial.printf("GNSS    | BLE: %.2f Hz | GNSS: %.2f Hz | SVs: %u | Fix: %u\n", bleRate, gnssRate, myGNSS.packetUBXNAVPVT->data.numSV, myGNSS.packetUBXNAVPVT->data.fixType);
  }

  Serial.printf("BUILD   | %s « %s » · %s · %s · accel ±16 g\n", BRAND, DEVICE_NICKNAME, FIRMWARE_VER, BUILD_STAMP);
  if (!flashReady) {
    Serial.println("MEMORY  | ❌ Flash QSPI NON détectée (enregistrement off)");
  } else {
    const char *st = (recState == REC_STATE_RUNNING) ? "EN COURS" : (recState == REC_STATE_PAUSED) ? "EN PAUSE" : "arrêté";
    uint32_t freeSlots = totalSlots - usedSlots;
    uint32_t minLeft = freeSlots / (60UL * recHz);
  Serial.printf("MEMORY  | %lu/%lu (%u%%) | REC: %s @%uHz | reste ~%lu min\n",
                  usedSlots, totalSlots,
                  totalSlots ? (usedSlots * 100UL / totalSlots) : 0,
                  recStateStr, recHz,
                  minLeft);
  }
  Serial.println("--------------------------------------------------");

  gpsUpdateCount = 0;
  gnssUpdateCount = 0;
  lastGpsRateCheckTime = millis();
}

void updateLEDs(uint8_t fixType) {
  static unsigned long lastBlink = 0;
  static bool ledState = false;

  if (!deviceConnected) {
    uint32_t phase = millis() % 3000;
    bool on = (memoryFull && recCfg.enabled && ((phase < 100) || (phase >= 200 && phase < 300)))
             || (recState == REC_STATE_RUNNING && phase < 80)
             || (recState == REC_STATE_PAUSED && ((phase < 80) || (phase >= 1500 && phase < 1580)));
    digitalWrite(OnboardledPin, on ? LOW : HIGH);
  }

  if (isCritical && !isCharging()) {
    if (millis() - lastBlink >= 500) {
      lastBlink = millis();
      ledState ^= 1;
      digitalWrite(LED_RED, ledState);
      digitalWrite(LED_GREEN, HIGH);
    }
    return;
  }

  if (!gpsEnabled) {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    return;
  }

  switch (fixType) {
  case 3: case 4:
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    break;
  case 1: case 2:
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    break;
  default:
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
    break;
  }
}

void connect_callback(uint16_t conn_handle) {
  deviceConnected = true;
  lastActivityTime = millis();
  lastDisconnectTime = millis();

  digitalWrite(OnboardledPin, LOW);
  Serial.println("✅ Client connected!");

  Bluefruit.setTxPower(0);
  Bluefruit.Connection(conn_handle)->requestPHY(BLE_GAP_PHY_2MBPS);
  Bluefruit.Connection(conn_handle)->requestMtuExchange(247);
  Bluefruit.Connection(conn_handle)->requestConnectionParameter(24, 0, 400);
  delay(400);
  uint16_t mtu = Bluefruit.Connection(conn_handle)->getMtu();
  Serial.printf(">> Negotiated MTU = %u (need >= 91 to fit 88-byte notify)\n", mtu);

  negotiatedMtu = mtu;
  asmLen = 0;
  rxTail = rxHead;
  txqTail = txqHead;
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  deviceConnected = false;
  lastDisconnectTime = millis();
  lastActivityTime = millis();

  digitalWrite(OnboardledPin, HIGH);

  if (downloadActive) {
    downloadActive = false;
    Serial.println("⏹️ Téléchargement interrompu (déconnexion).");
  }
  txqTail = txqHead;
  asmLen = 0;

  Serial.println("❌ BLE Client disconnected.");
  Serial.printf("🛰️ GPS staying hot for %d minutes...\n", (GPS_HOT_TIMEOUT_MS / 60000));
}

void write_callback(uint16_t conn_handle, BLECharacteristic *chr, uint8_t *data, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    uint16_t next = (rxHead + 1) & (RXRING_SIZE - 1);
    if (next == rxTail)
      break;
    rxRing[rxHead] = data[i];
    rxHead = next;
  }
  rxByteCount += len;
}

void setIMUForSleep() {
  IMU.settings.gyroEnabled = 0;
  IMU.settings.accelEnabled = 0;
  IMU.begin();
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x30);
  IMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x8E);
  IMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, 0x8C);
  IMU.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2, 0x20);
  IMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x80);
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL6_G, 0x10);
  IMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x08);
  pinMode(PIN_LSM6DS3TR_C_INT1, INPUT_PULLDOWN_SENSE);
}

void enterDeepSleep() {
  Serial.println("💤 Entering Deep Sleep (Shake to Wake)...");
  Bluefruit.autoConnLed(false);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);

  disableGPS();

  setIMUForSleep();
  pinMode(PIN_CHG, INPUT_PULLUP_SENSE);

  delay(100);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);

  Serial.flush();
  NRF_POWER->SYSTEMOFF = 1;
}

bool detectNoBatteryAtBoot() { return !EXPECT_BATTERY; }

void setupHardware() {
  pinMode(GPS_EN_PIN, OUTPUT);
#ifdef PCB_VERSION
  digitalWrite(GPS_EN_PIN, LOW);
#else
  digitalWrite(GPS_EN_PIN, HIGH);
#endif

  pinMode(PIN_VBAT, INPUT);
  pinMode(PIN_VBAT_ENABLE, OUTPUT);
  digitalWrite(PIN_VBAT_ENABLE, LOW);
  pinMode(PIN_HICHG, OUTPUT);
  digitalWrite(PIN_HICHG, LOW);
  pinMode(PIN_CHG, INPUT_PULLUP);

  Wire.setClock(400000);
  analogReference(AR_DEFAULT);
  analogReadResolution(12);

  NRF_POWER->DCDCEN = 1;
  if (NRF_POWER->MAINREGSTATUS & (POWER_MAINREGSTATUS_MAINREGSTATUS_High << POWER_MAINREGSTATUS_MAINREGSTATUS_Pos)) {
    NRF_POWER->DCDCEN0 = 1;
  }

  pinMode(PIN_QSPI_CS, OUTPUT);
  digitalWrite(PIN_QSPI_CS, HIGH);

  pinMode(OnboardledPin, OUTPUT);
  digitalWrite(OnboardledPin, HIGH);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
}

void setupBLE() {
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.autoConnLed(false);
  Bluefruit.setTxPower(LOW_POWER_BT_TX_POWER);
  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  Bluefruit.Periph.setConnInterval(12, 24);

#if ENABLE_OTA_DFU
  bledfu.begin();
#endif

  disService.begin();
  disModel.setProperties(CHR_PROPS_READ);
  disModel.begin();
  disModel.write(MODEL_STRING);
  disSerial.setProperties(CHR_PROPS_READ);
  disSerial.begin();
  disSerial.write(SERIAL_NUM);
  disFirmware.setProperties(CHR_PROPS_READ);
  disFirmware.begin();
  disFirmware.write(FIRMWARE_VER);
  disHardware.setProperties(CHR_PROPS_READ);
  disHardware.begin();
  disHardware.write(HARDWARE_VER);
  disManuf.setProperties(CHR_PROPS_READ);
  disManuf.begin();
  disManuf.write(MANUFACTURER);

  rbService.begin();
  rbTx.setProperties(CHR_PROPS_NOTIFY);
  rbTx.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  rbTx.setMaxLen(244);
  rbTx.setCccdWriteCallback([](uint16_t conn, BLECharacteristic* c, uint16_t value){
    Serial.printf(">> CCCD write on rbTx: 0x%04X (notify %s)\n", value, (value & 0x0001) ? "ENABLED" : "disabled");
  });
  rbTx.begin();

  rbRx.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  rbRx.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  rbRx.setMaxLen(247);
  rbRx.setWriteCallback(write_callback);
  rbRx.begin();

  rbGnss.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  rbGnss.begin();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n🚀 SYSTEM STARTUP");
  Serial.printf("   %s « %s » · firmware %s · compilé le %s\n", BRAND, DEVICE_NICKNAME, FIRMWARE_VER, BUILD_STAMP);
  Serial.println("   Commandes série : s=stop  i=état  z=config par défaut  ?=aide");

  setupHardware();
  setupStorage();

  Serial.println("🔍 Checking power source...");
  isNoBatteryMode = detectNoBatteryAtBoot();
  if (isNoBatteryMode) {
    batteryConnected = false;
    Serial.println("⚠️ NO BATTERY MODE ENABLED");
  } else {
    updateBatteryState();
  }

  setupBLE();
  setupAdvertising(LOW_POWER_BT_TX_POWER, ECO_ADV_INTERVAL);
  enableGPS();

  lastDisconnectTime = millis();
  lastActivityTime = millis();
  Serial.println("✅ Hardware initialized successfully.");
}

void loop() {
  txqService();
  serviceRxParser();
  manageBatterySampling();
  managePower();

  processGNSS();
  if (imuEnabled)
    processIMU();

  serviceDownload();
  serviceErase();
  serviceAutoShutdown();

  updateLEDs(GPSFixType);
  reportSystemStats();

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') {
      stopRecording();
    } else if (c == 'i') {
      Serial.printf("📊 Enregistrement : %s | Remplissage : %lu/%lu\n",
                    recState == REC_STATE_RUNNING ? "EN COURS" :
                    recState == REC_STATE_PAUSED ? "PAUSE" : "ARRÊTÉ",
                    usedSlots, totalSlots);
    } else if (c == 'z') {
      defaultConfig();
      saveConfig();
      Serial.println("✅ Configuration remise par défaut.");
    } else if (c == '?') {
      Serial.println("💡 Aide : s=stop | i=status | z=default");
    }
  }

  delay(2);
}