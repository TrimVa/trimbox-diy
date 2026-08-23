#include <Adafruit_TinyUSB.h>
#include <LSM6DS3.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>
#include <bluefruit.h>
#include <nrf_soc.h>

// --- AJOUT : stockage sur la flash QSPI 2 Mo embarquée (P25Q16H) ---
#include <Adafruit_SPIFlash.h>
#include <SPI.h>

// ============================================================================
// --- USER CUSTOMIZATION  ---
// ============================================================================

// --- BLE Branding ---
#define SERIAL_NUM "2230456789" // The unique 10-digit serial

// (DO NOT CHANGE THESE: Required for RaceBox Application compatibility)
// --- MODIFIÉ : on se présente comme un RaceBox Mini S (= modèle avec
//     enregistrement autonome en mémoire interne). Le préfixe du nom ET la
//     chaîne du Model Characteristic doivent correspondre exactement à ce
//     que la doc du protocole RaceBox impose, sinon l'app ne proposera pas
//     les fonctions de mémoire.
#define DEVICE_NAME "RaceBox Mini S " SERIAL_NUM // Auto-synced Name
#define MODEL_STRING "RaceBox Mini S"            // <-- lu par l'app RaceBox
#define HARDWARE_VER "1.0"
#define MANUFACTURER "RaceBox"
#define FIRMWARE_VER "3.3"

// --- AJOUT : empreinte de compilation. Le bootloader UF2 redémarre la carte
//     dès le dernier bloc écrit, ce qui provoque une erreur de déconnexion
//     côté ordinateur ou téléphone : celle-ci est normale et n'indique pas si
//     le firmware a réellement été pris. Cette empreinte, affichée au
//     démarrage ET dans le rapport périodique, lève le doute.
#define BUILD_STAMP __DATE__ " " __TIME__

// --- GPS Performance ---
#define MAX_NAVIGATION_RATE 25 // 25Hz: Max rate for RaceBox Mini protocol
#define GPS_BAUD 115200        // High speed for 25Hz data
#define FACTORY_GPS_BAUD 9600  // Default for cold modules

#define SYSTEM_RATE_REPORT_MS 5000 // Interval for Serial stats reporting

// --- Power & Efficiency ---
#define EXPECT_BATTERY true // Set to false to force USB-only No-Battery mode
#define GPS_HOT_TIMEOUT_MS 900000 // 15 Minutes (Stay powered after disconnect)
#define DEEP_SLEEP_DAYS 1         // Safety net before absolute shutdown
#define ENABLE_DEEP_SLEEP false   // Usually false for standard RaceBox usage
#define FAST_ADV_INTERVAL 160 // 100ms: Fast discovery for apps (160 * 0.625ms)
#define ECO_ADV_INTERVAL                                                       \
  4000 // 2500ms: Extremely low power (4000 * 0.625ms = 2.5s latency to connect)
#define LOOP_SLEEP 2500 // 2500ms delay for the main loop iteration while idle
#define SLEEP_WHILE_CHARGING                                                   \
  true // Allow Light Sleep even when plugged in /Set false to force high power
       // mode when plugged in
#define LOW_POWER_BT_TX_POWER -4 // dBm for low power consumption

// --- GNSS Constellation Toggle ---
#define ENABLE_GNSS_GPS
#define ENABLE_GNSS_GALILEO
// #define ENABLE_GNSS_GLONASS
// #define ENABLE_GNSS_BEIDOU
// #define ENABLE_GNSS_SBAS
// #define ENABLE_GNSS_QZSS

// --- Hardware Version ---
// Uncomment the line below if using the custom PCB version where
// the GPS_EN_PIN logic is inverted. (For PCB: LOW = ON, HIGH = OFF)
#define PCB_VERSION

// ============================================================================
// ---  HARDWARE MAPPINGS ---
// ============================================================================

#define GPS_EN_PIN D1      // GPS Power Enable Rail
#define PIN_VBAT_ENABLE 14 // Battery Read Enable
#define PIN_HICHG 22       // Charge Speed (LOW=100mA)
#define PIN_CHG 23         // Charge Indicator (LOW=Charging)
#define ACCEL_INT_PIN PIN_LSM6DS3TR_C_INT1

// ============================================================================
// ---  GLOBAL SYSTEM STATE ---
// ============================================================================

SFE_UBLOX_GNSS myGNSS;
LSM6DS3 IMU(I2C_MODE, 0x6A);

// System Flags
bool deviceConnected = false;
bool gpsEnabled = false;
bool imuEnabled = false;
bool pendingConfig = false;
bool lastChargingState = false;
bool lastPluggedInState = false;
bool batteryConnected = true;
uint8_t currentBatteryPercentage = 100;
float batteryMultiplier = 3.0; // Voltage divider 1/3

// Global State
bool isCritical = false;
bool isNoBatteryMode = false;
int GPSFixType = 0;

// Timing Trackers
unsigned long lastDisconnectTime = 0;
unsigned long lastActivityTime = 0;
unsigned long lastGpsRateCheckTime = 0;
unsigned int gpsUpdateCount = 0;
unsigned int gnssUpdateCount = 0;

// Filter/IMU State (Now using hardware filtering)
float imu_ax = 0, imu_ay = 0, imu_az = 0;
float imu_gx = 0, imu_gy = 0, imu_gz = 0;

// BLE Core Objects
const uint8_t RACEBOX_SERVICE_UUID[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5,
                                        0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
                                        0x01, 0x00, 0x40, 0x6E};
const uint8_t RACEBOX_TX_UUID[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5,
                                   0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
                                   0x03, 0x00, 0x40, 0x6E};
const uint8_t RACEBOX_RX_UUID[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5,
                                   0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
                                   0x02, 0x00, 0x40, 0x6E};
const uint8_t RACEBOX_GNSS_UUID[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5,
                                     0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
                                     0x04, 0x00, 0x40, 0x6E};

// --- AJOUT : mise à jour du firmware par Bluetooth (OTA DFU) ---
// Mettre à 0 pour retirer complètement le service, par exemple si le service
// n'existe pas dans votre version du core, ou pour empêcher toute
// reprogrammation sans fil de l'appareil.
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

// ############################################################################
// ###  AJOUT : ÉMULATION RACEBOX MINI S — ENREGISTREMENT AUTONOME          ###
// ###                                                                      ###
// ###  Implémente la section "Standalone Recording" de la documentation    ###
// ###  officielle du protocole BLE RaceBox (révision 8) :                  ###
// ###    0xFF 0x21  History Data Message                                   ###
// ###    0xFF 0x22  Standalone Recording Status                            ###
// ###    0xFF 0x23  Recorded Data Download / Cancel                        ###
// ###    0xFF 0x24  Recorded Data Erase / Cancel (+ progression)           ###
// ###    0xFF 0x25  Standalone Recording Configuration                     ###
// ###    0xFF 0x26  Recording State Change Message                         ###
// ###    0xFF 0x27  GNSS Receiver Configuration                            ###
// ###    0xFF 0x30  Unlock Memory                                          ###
// ###    0xFF 0x02 / 0x03  ACK / NACK                                      ###
// ############################################################################

// --- Flash QSPI embarquée (Seeed XIAO nRF52840 : P25Q16H, 2 Mo) ---
// IMPORTANT : utiliser la version d'Adafruit_SPIFlash FOURNIE AVEC LE CORE
// Seeeduino. Si une copie est installée dans Documents\Arduino\libraries,
// elle prend la priorité et entraîne un conflit de type 'File' dans
// bonding.cpp (Bluefruit). Dans ce cas, renommez le dossier en doublon.
//
// Le core connaît normalement déjà la P25Q16H. La description explicite
// ci-dessous ne sert que de secours si la détection automatique échoue.
// Si elle refuse de compiler (par ex. champ 'is_fram' inconnu sur une
// version ancienne), passez USE_EXPLICIT_FLASH_DEVICE à 0.
#define USE_EXPLICIT_FLASH_DEVICE 1

#if USE_EXPLICIT_FLASH_DEVICE
static const SPIFlash_Device_t P25Q16H_DEVICE = {
    .total_size = (1UL << 21), // 2 Mo
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
Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS,
                                           EXTERNAL_FLASH_USE_SPI);
#else
// Repli : bit-bang SPI sur les broches QSPI.
// NB : on utilise SPIM2 et surtout PAS SPIM0, qui partage son bloc matériel
// avec TWIM0 (le bus Wire de l'IMU) sur le nRF52840.
SPIClass SPI_QSPI(NRF_SPIM2, PIN_QSPI_IO1, PIN_QSPI_SCK, PIN_QSPI_IO0);
Adafruit_FlashTransport_SPI flashTransport(PIN_QSPI_CS, SPI_QSPI);
#endif
Adafruit_SPIFlash flash(&flashTransport);

// --- Découpage de la mémoire ---
#define REC_MAGIC 0x534D4252UL // "RBMS"
#define REC_CFG_VERSION 2
// --- SÉCURITÉ : la configuration est écrite en DOUBLE EXEMPLAIRE, dans deux
//     secteurs distincts et en alternance. Sauvegarder implique d'effacer puis
//     de réécrire un secteur ; si le courant est coupé pendant cette fenêtre,
//     l'exemplaire en cours d'écriture est perdu — mais l'autre, intact, prend
//     le relais au démarrage suivant. Un numéro de séquence désigne le plus
//     récent, une somme de contrôle écarte celui qui serait tronqué.
#define META_A 0               // secteur 0 : exemplaire A
#define META_B 4096            // secteur 1 : exemplaire B
#define DATA_START 8192        // les enregistrements commencent après les deux
#define SLOT_SIZE 81           // 1 octet de type + 80 octets de payload

// Reprise automatique d'une session interrompue par une coupure de courant.
// Laissé à 0 volontairement : reprendre seul rallume le GPS et peut rendre
// l'appareil difficile à reprendre en main. Les données déjà enregistrées ne
// sont jamais perdues, seule la poursuite automatique est désactivée.
#define AUTO_RESUME_RECORDING 0

static bool flashReady = false;
static uint32_t flashTotalBytes = 0;
static uint32_t totalSlots = 0; // capacité en enregistrements
static uint32_t usedSlots = 0;  // pointeur d'écriture

// --- Types d'enregistrement stockés (= ID du message RaceBox) ---
#define REC_TYPE_DATA 0x21
#define REC_TYPE_STATE 0x26
#define REC_TYPE_EMPTY 0xFF

// --- États d'enregistrement (doc RaceBox) ---
#define REC_STATE_OFF 0
#define REC_STATE_RUNNING 1
#define REC_STATE_PAUSED 2

// --- Bits du champ "Filters and features flags" ---
#define RECFLAG_WAIT_FIX 0x01
#define RECFLAG_STATIONARY 0x02
#define RECFLAG_NOFIX 0x04
#define RECFLAG_AUTOSHUTDOWN 0x08
#define RECFLAG_WAIT_DATA 0x10

// --- Configuration persistante ---
struct __attribute__((packed)) RecConfig {
  uint32_t magic;
  uint8_t version;
  uint8_t enabled;         // 1 = enregistrement autonome actif
  uint8_t dataRate;        // 0=25Hz 1=10Hz 2=5Hz 3=1Hz 4=20Hz
  uint8_t flags;           // bitmask des filtres
  uint16_t statSpeed;      // seuil du filtre "stationnaire" en mm/s
  uint16_t statInterval;   // en secondes
  uint16_t noFixInterval;  // en secondes
  uint16_t autoOffInterval;// en secondes
  uint8_t gnssDynModel;    // 0xFF 0x27 : modèle dynamique u-blox
  uint8_t gnss3dSpeed;     // 0xFF 0x27 : vitesse 3D au lieu de vitesse sol
  uint8_t gnssMinAcc;      // 0xFF 0x27 : précision horizontale mini (m)
  uint8_t reserved;
  uint32_t seq;            // numéro de séquence : le plus grand fait foi
  uint32_t crc;            // somme de contrôle des champs précédents
};

static RecConfig recCfg;
static uint8_t recState = REC_STATE_OFF;
static bool memoryFull = false;

// --- Suivi des filtres ---
static unsigned long slowSinceMs = 0;
static unsigned long noFixSinceMs = 0;
static unsigned long lastStoredDataMs = 0;
static bool anyDataSinceEnable = false;

// --- File d'émission BLE (flux d'octets, pour saturer la bande passante) ---
#define TXQ_SIZE 2048
static uint8_t txq[TXQ_SIZE];
static uint16_t txqHead = 0, txqTail = 0;
static uint16_t negotiatedMtu = 23;

// --- Réception BLE (anneau rempli dans le callback, traité dans loop) ---
#define RXRING_SIZE 512
static uint8_t rxRing[RXRING_SIZE];
static volatile uint16_t rxHead = 0, rxTail = 0;
static uint8_t asmBuf[560];
static uint16_t asmLen = 0;
// Diagnostic : nombre total d'octets écrits par le client sur la RX.
static volatile uint32_t rxByteCount = 0;
#define RX_DEBUG 1 // 1 = affiche les octets bruts reçus (mettre 0 en usage réel)

// --- Opérations longues (téléchargement / effacement) ---
static bool downloadActive = false;
static uint32_t downloadCursor = 0;
static uint32_t downloadEnd = 0;
static uint32_t downloadSkipped = 0;
static bool eraseActive = false;
static uint32_t eraseBlock = 0;
static uint32_t eraseBlockCount = 0;
static int lastErasePct = -1;

// Déclarations anticipées (fonctions définies plus bas dans le fichier)
void enterDeepSleep();
bool configureGPS();

// ---------------------------------------------------------------- file TX ---
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

// Vide la file par paquets de la taille maximale que le MTU autorise.
static void txqService() {
  if (!deviceConnected)
    return;
  uint16_t maxChunk = (negotiatedMtu > 3) ? (negotiatedMtu - 3) : 20;
  if (maxChunk > 244)
    maxChunk = 244;

  uint8_t chunk[244];
  // Plusieurs notifications par tour de boucle tant que la pile BLE accepte.
  // La file transporte désormais aussi les 25 Hz de données live, il faut donc
  // pouvoir la vider plus vite qu'elle ne se remplit.
  for (uint8_t burst = 0; burst < 12; burst++) {
    uint16_t n = txqUsed();
    if (n == 0)
      return;
    if (n > maxChunk)
      n = maxChunk;
    for (uint16_t i = 0; i < n; i++)
      chunk[i] = txq[(txqTail + i) & (TXQ_SIZE - 1)];
    if (!rbTx.notify(chunk, n))
      return; // tampon SoftDevice plein : on réessaiera au tour suivant
    txqTail = (txqTail + n) & (TXQ_SIZE - 1);
  }
}

// Encapsule un message dans une trame UBX et l'empile pour émission.
static bool sendUbx(uint8_t cls, uint8_t id, const uint8_t *payload,
                    uint16_t len) {
  // Statique : appelé uniquement depuis la boucle principale, et évite de
  // consommer inutilement la pile de la tâche Arduino.
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

// ------------------------------------------------------------- stockage ---
static inline uint32_t slotAddr(uint32_t i) {
  return DATA_START + i * (uint32_t)SLOT_SIZE;
}

static uint8_t slotType(uint32_t i) {
  uint8_t t = REC_TYPE_EMPTY;
  flash.readBuffer(slotAddr(i), &t, 1);
  return t;
}

static uint8_t lastMetaSlot = 1; // dernier exemplaire écrit (0 = A, 1 = B)

// NB : ces deux fonctions prennent un 'const void *' et non un
// 'const RecConfig *'. L'IDE Arduino génère automatiquement les prototypes et
// les insère AVANT la déclaration de la structure : une signature typée
// provoquerait alors « 'RecConfig' does not name a type ».
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
  return c->magic == REC_MAGIC && c->version == REC_CFG_VERSION &&
         c->crc == cfgCrc(c);
}

// Écrit la configuration dans l'exemplaire NON utilisé la dernière fois.
// L'autre reste donc toujours valide pendant toute l'opération.
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

  // Relecture de contrôle : si l'écriture s'est mal passée, on n'adopte pas
  // ce nouvel exemplaire comme référence.
  RecConfig back;
  flash.readBuffer(addr, (uint8_t *)&back, sizeof(back));
  if (cfgValid(&back))
    lastMetaSlot = target;
  else
    Serial.println("⚠️ Écriture de configuration non confirmée.");
}

// Relit les deux exemplaires et retient le plus récent qui soit intact.
static bool loadConfig() {
  RecConfig a, b;
  flash.readBuffer(META_A, (uint8_t *)&a, sizeof(a));
  flash.readBuffer(META_B, (uint8_t *)&b, sizeof(b));
  bool va = cfgValid(&a), vb = cfgValid(&b);

  if (va && vb) {
    // Comparaison tolérante au rebouclage du compteur.
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
  recCfg.dataRate = 0;          // 25 Hz
  recCfg.flags = 0x1F;          // tous les filtres, réglage recommandé RaceBox
  recCfg.statSpeed = 1389;      // ~5 km/h
  recCfg.statInterval = 30;
  recCfg.noFixInterval = 30;
  recCfg.autoOffInterval = 300; // 5 minutes
  // 7 = DYN_MODEL_AIRBORNE2g : conserve le réglage "piste" d'origine.
  // L'app RaceBox propose en général 4 (automobile) ; elle peut l'imposer.
  recCfg.gnssDynModel = 7;
  recCfg.gnss3dSpeed = 0;
  recCfg.gnssMinAcc = 3;
}

// Effacement complet, bloquant (utilisé seulement au tout premier démarrage).
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

// Recherche dichotomique du premier emplacement libre.
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

// Construit le payload d'un message 0xFF 0x26 (changement d'état).
// Le tampon fourni doit faire 80 octets : les 12 premiers portent
// l'information, le reste est mis à zéro pour occuper un emplacement mémoire
// de taille identique à celle d'un enregistrement de données.
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

// Enregistre le changement d'état en mémoire ET le notifie au client connecté.
static void announceStateChange(uint8_t newState, bool alsoStore) {
  uint8_t p[80];
  buildStatePayload(p, newState);
  if (alsoStore)
    storeRecord(REC_TYPE_STATE, p);
  if (deviceConnected)
    sendUbx(0xFF, 0x26, p, 12);
}

// Traduit le champ "Data Rate" du protocole en fréquence de navigation GNSS.
static uint8_t rateToHz(uint8_t r) {
  switch (r) {
  case 1:
    return 10;
  case 2:
    return 5;
  case 3:
    return 1;
  case 4:
    return 20;
  default:
    return 25;
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

// Appelé à chaque nouveau point GNSS : applique les filtres puis stocke.
static void recordTick(const uint8_t *payload80) {
  if (recState == REC_STATE_OFF || !flashReady)
    return;

  bool haveFix = (myGNSS.packetUBXNAVPVT != NULL &&
                  myGNSS.packetUBXNAVPVT->data.fixType == 3);
  int32_t gSpeed =
      (myGNSS.packetUBXNAVPVT != NULL) ? myGNSS.packetUBXNAVPVT->data.gSpeed : 0;
  unsigned long now = millis();

  // "Wait for GNSS fix" : rien n'est stocké tant qu'il n'y a pas de fix 3D.
  if ((recCfg.flags & RECFLAG_WAIT_FIX) && !haveFix && !anyDataSinceEnable)
    return;

  bool shouldPause = false;

  // Filtre "No-Fix"
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

  // Filtre "Stationnaire" (uniquement si le fix est valide)
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

  // Reprise : la doc précise que le passage pause -> actif n'est PAS stocké,
  // mais qu'il est bien notifié au client connecté.
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

// Extinction automatique après inactivité prolongée.
static void serviceAutoShutdown() {
  if (!(recCfg.flags & RECFLAG_AUTOSHUTDOWN) || recState == REC_STATE_OFF)
    return;
  if (deviceConnected || downloadActive || eraseActive)
    return;
  // --- CORRIGÉ : ne JAMAIS s'éteindre tant que l'USB est branché. Sans cette
  //     garde, un module en enregistrement mais sans fix partait en veille
  //     profonde au bout du délai configuré : le port USB disparaissait et la
  //     carte devenait impossible à reflasher sans double-clic sur RESET.
  if (isPluggedIn() || isCharging())
    return;
  // Fenêtre de grâce après démarrage : laisse toujours le temps de reprendre
  // la main sur l'appareil avant toute extinction automatique.
  if (millis() < 120000UL)
    return;
  // --- SÉCURITÉ : ne jamais s'éteindre si la session n'a jamais rien
  //     enregistré. Sans cette garde, un module qui n'accroche aucun satellite
  //     se met hors tension tout seul et devient injoignable.
  if (!anyDataSinceEnable)
    return;
  // Sans au moins un filtre de pause, l'appareil enregistrerait en continu.
  if (!(recCfg.flags & (RECFLAG_WAIT_FIX | RECFLAG_STATIONARY | RECFLAG_NOFIX)))
    return;
  if ((recCfg.flags & RECFLAG_WAIT_DATA) && !anyDataSinceEnable)
    return;
  if ((millis() - lastStoredDataMs) >
      (unsigned long)recCfg.autoOffInterval * 1000UL) {
    Serial.println("⌛ Auto-Shutdown : aucune donnée depuis un moment.");
    enterDeepSleep();
  }
}

// --------------------------------------------------- téléchargement/erase ---
static void startDownload() {
  downloadActive = true;
  downloadSkipped = 0;
  downloadCursor = 0;
  downloadEnd = usedSlots; // instantané : on n'envoie pas ce qui s'ajoute après
  uint8_t p[4];
  writeLittleEndian(p, 0, (uint32_t)downloadEnd);
  sendUbx(0xFF, 0x23, p, 4);
  Serial.printf("⬇️ Téléchargement de %lu enregistrements...\n",
                (unsigned long)downloadEnd);
}

// Un enregistrement dont l'écriture a été interrompue par une coupure de
// courant contient des octets restés à 0xFF. Transmis tel quel, il produit des
// positions et des vitesses absurdes côté application. On le contrôle donc
// avant de l'envoyer.
static bool recordLooksValid(const uint8_t *p) {
  int32_t lat, lon, gSpeed;
  memcpy(&lon, p + 24, 4);
  memcpy(&lat, p + 28, 4);
  memcpy(&gSpeed, p + 48, 4);
  if (p[20] > 5)          return false;   // type de fix
  if (p[23] > 60)         return false;   // nombre de satellites
  if (lat >  900000000L || lat < -900000000L) return false;
  if (lon > 1800000000L || lon < -1800000000L) return false;
  if (gSpeed < 0 || gSpeed > 140000000L)      return false; // ~500 km/h
  return true;
}

static void serviceDownload() {
  if (!downloadActive)
    return;
  // On remplit la file tant qu'il reste de la place pour un paquet complet.
  while (downloadCursor < downloadEnd && txqFree() >= 96) {
    uint8_t slot[SLOT_SIZE];
    flash.readBuffer(slotAddr(downloadCursor), slot, SLOT_SIZE);
    if (slot[0] == REC_TYPE_DATA) {
      if (recordLooksValid(slot + 1)) {
        if (!sendUbx(0xFF, 0x21, slot + 1, 80))
          break;                       // file pleine : on réessaiera, sans
                                       // avancer, pour ne rien perdre
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
    Serial.printf("✅ Téléchargement terminé (%lu enregistrement(s) corrompu(s) "
                  "écarté(s)).\n", (unsigned long)downloadSkipped);
  }
}

static void startErase() {
  eraseActive = true;
  eraseBlock = 0;
  // On n'efface que la zone réellement écrite (bien plus rapide).
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
    saveConfig(); // le secteur 0 vient d'être effacé, on le réécrit
    sendAck(0xFF, 0x24);
    Serial.println("✅ Mémoire effacée.");
  }
}

// ------------------------------------------------------ traitement des cmd ---
static void handleCommand(uint8_t cls, uint8_t id, uint8_t *pl, uint16_t len) {
  // Trace utile au diagnostic : montre ce que l'application demande réellement.
  Serial.printf("📨 CMD 0x%02X 0x%02X (%u o)", cls, id, len);
  for (uint16_t i = 0; i < len && i < 16; i++)
    Serial.printf(" %02X", pl[i]);
  Serial.println();

  if (cls != 0xFF) {
    // Passerelle transparente vers le récepteur u-blox (ex. données AssistNow).
    if (gpsEnabled) {
      uint8_t hdr[6] = {0xB5, 0x62, cls, id, (uint8_t)(len & 0xFF),
                        (uint8_t)(len >> 8)};
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

  // --- 0xFF 0x22 : état de l'enregistrement autonome ---
  case 0x22: {
    uint8_t p[12];
    memset(p, 0, 12);
    p[0] = (recState != REC_STATE_OFF) ? 1 : 0;
    p[1] = totalSlots ? (uint8_t)((usedSlots * 100UL) / totalSlots) : 0;
    p[2] = 0x00; // sécurité mémoire désactivée
    p[3] = 0;
    writeLittleEndian(p, 4, (uint32_t)usedSlots);
    writeLittleEndian(p, 8, (uint32_t)totalSlots);
    sendUbx(0xFF, 0x22, p, 12);
    break;
  }

  // --- 0xFF 0x25 : configuration de l'enregistrement autonome ---
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
      sendNack(0xFF, 0x25); // plus assez de mémoire libre
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

  // --- 0xFF 0x23 : téléchargement des données enregistrées ---
  case 0x23: {
    if (len >= 1) { // annulation
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

  // --- 0xFF 0x24 : effacement de la mémoire ---
  case 0x24: {
    if (len >= 1) {
      // Annulation demandée. La doc l'autorise (effacement du haut vers le
      // bas), mais ici le journal est un simple ajout linéaire : s'arrêter à
      // mi-chemin laisserait des secteurs non effacés devant le pointeur
      // d'écriture, sur lesquels il serait impossible d'écrire ensuite.
      // On laisse donc l'effacement se terminer — il ne porte que sur la zone
      // réellement utilisée, donc il est court — et l'ACK partira à la fin.
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

  // --- 0xFF 0x30 : déverrouillage mémoire (sécurité non activée ici) ---
  case 0x30:
    sendAck(0xFF, 0x30);
    break;

  // --- 0xFF 0x27 : configuration du récepteur GNSS ---
  case 0x27: {
    if (len == 0) {
      uint8_t p[3] = {recCfg.gnssDynModel, recCfg.gnss3dSpeed,
                      recCfg.gnssMinAcc};
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

  case 0x01: // message de données live : rien à faire s'il nous revient
    break;

  default:
    sendNack(0xFF, id);
    break;
  }
}

// Assemble les octets reçus en trames UBX complètes (une notification BLE ne
// contient pas forcément un message entier, ni un seul).
static void serviceRxParser() {
#if RX_DEBUG
  // Trace brute : distingue « rien n'arrive » de « ça arrive mais ne parse pas ».
  static uint32_t lastReported = 0;
  if (rxByteCount != lastReported) {
    uint16_t pending = (uint16_t)((rxHead - rxTail) & (RXRING_SIZE - 1));
    Serial.printf("🔻 RX brut : %lu octets au total, %u en attente :",
                  (unsigned long)rxByteCount, pending);
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

// Initialisation de la flash et restauration de la configuration.
static void setupStorage() {
  // 1) détection automatique via la liste de composants du core
  bool ok = flash.begin();
#if USE_EXPLICIT_FLASH_DEVICE
  // 2) secours : on impose la description de la P25Q16H
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
  Serial.printf("💾 Flash %lu Ko | capacité : %lu enregistrements\n",
                (unsigned long)(flashTotalBytes / 1024),
                (unsigned long)totalSlots);

  if (!loadConfig()) {
    Serial.println("ℹ️ Aucune configuration valide en mémoire.");
    // --- AJOUT : ne reformater que si la puce contient réellement quelque
    //     chose. Sur une flash déjà vierge, un effacement complet coûterait
    //     30 secondes pendant lesquelles ni le BLE ni les messages série ne
    //     seraient disponibles — au point de faire croire à un plantage.
    bool alreadyBlank = true;
    uint8_t probe[64];
    for (uint32_t addr = 0; addr < flashTotalBytes && alreadyBlank;
         addr += 65536) {
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
  Serial.printf("💾 Occupation : %lu / %lu (%lu%%)\n", (unsigned long)usedSlots,
                (unsigned long)totalSlots,
                (unsigned long)(totalSlots ? usedSlots * 100 / totalSlots : 0));

  // Estimation de l'autonomie restante à la cadence configurée.
  uint32_t freeSlots = totalSlots - usedSlots;
  Serial.printf("💾 Reste ~%lu min à %u Hz\n",
                (unsigned long)(freeSlots / (60UL * rateToHz(recCfg.dataRate))),
                rateToHz(recCfg.dataRate));
}

bool isCharging() { return digitalRead(PIN_CHG) == LOW; }

// Calibration Table
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

// Lookup Function
float getRawPercentage() {
  float v = getBatteryVoltage(); // Your 16-sample average function
  if (v >= batteryMap[0].voltage)
    return 100.0;
  if (v <= batteryMap[mapSize - 1].voltage)
    return 0.0;

  for (int i = 0; i < mapSize - 1; i++) {
    if (v <= batteryMap[i].voltage && v > batteryMap[i + 1].voltage) {
      float vHigh = batteryMap[i].voltage;
      float vLow = batteryMap[i + 1].voltage;
      uint8_t pHigh = batteryMap[i].percentage;
      uint8_t pLow = batteryMap[i + 1].percentage;
      return pLow + (v - vLow) * (pHigh - pLow) / (vHigh - vLow);
    }
  }
  return 0.0;
}

// State Update
void updateBatteryState() {
  static float filteredPct = -1.0;
  bool pluggedIn = isCharging();
  float rawPct = getRawPercentage();
  float currentV = getBatteryVoltage();

  // Initial Sync
  if (filteredPct < 0) {
    filteredPct = rawPct;
    currentBatteryPercentage = (uint8_t)rawPct;
  }

  // Heavy Filter (Adjusted for 30s interval to prevent lag)
  filteredPct = (rawPct * 0.15) + (filteredPct * 0.85);
  uint8_t rounded = (uint8_t)(filteredPct + 0.5);

  // Set Critical Flag (e.g., below 3.35V / 10%)
  isCritical = (currentV < 3.35);

  // --- STICKY 100% LATCH ---
  // If we were at 100%, don't drop the display until the filtered value
  // hits 95%. This prevents the 5V boost-regulator sag from killing
  // the "Full" status immediately upon power-on.
  if (currentBatteryPercentage == 100 && !pluggedIn && rounded > 95) {
    rounded = 100;
  }

  if (abs((int)rounded - (int)currentBatteryPercentage) >= 2 ||
      rounded == 100 || rounded == 0) {
    if (pluggedIn) {
      if (rounded > currentBatteryPercentage)
        currentBatteryPercentage = rounded;
    } else {
      if (rounded < currentBatteryPercentage)
        currentBatteryPercentage = rounded;

      // Boot/Recovery Sync: If raw is significantly out of sync, force update.
      // This prevents the percentage from getting stuck if updates were missed.
      if (rawPct > currentBatteryPercentage + 5.0 ||
          currentBatteryPercentage > rawPct + 20.0) {
        currentBatteryPercentage = (uint8_t)rawPct;
        filteredPct = rawPct;
      }
    }
  }
}

bool isPluggedIn() {
  // Check if USB power is detected
  return NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk;
}

float getBatteryVoltage() {
  digitalWrite(PIN_VBAT_ENABLE, LOW); // Enable divider
  delay(1);

  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(PIN_VBAT);
    delayMicroseconds(50);
  }
  float adcCount = (float)sum / 8.0;
  float voltage = (batteryMultiplier * 3.6 * adcCount / 4096);

  // --- LOAD COMPENSATION ---
  // If the GPS is running (30mA draw), the battery voltage sags.
  // We add an offset to compensate so the percentage doesn't drop just
  // because the sensors turned on.
  if (gpsEnabled && !isPluggedIn()) {
    voltage += 0.010; // Approx compensation for 30mA on a 1000mAh pack
  }

  if (!isCharging() && !isPluggedIn()) {
    digitalWrite(PIN_VBAT_ENABLE, HIGH);
  }
  return voltage;
}

// ============================================================================
// ---  SENSOR PROCESSING MODULES ---
// ============================================================================

// Assemble and transmit the proprietary RaceBox Mini protocol packet

// --- MODIFIÉ : la construction du payload est isolée pour être réutilisée
//     à la fois par l'émission live (0xFF 0x01) et par l'enregistrement
//     autonome en mémoire (0xFF 0x21). Le payload est strictement identique
//     dans les deux cas, comme sur un vrai Mini S.
bool buildDataPayload(uint8_t *payload) {
  if (myGNSS.packetUBXNAVPVT == NULL)
    return false;

  memset(payload, 0, 80);
  auto *data = &myGNSS.packetUBXNAVPVT->data;

  // Time and Resolution
  writeLittleEndian(payload, 0, data->iTOW);
  writeLittleEndian(payload, 4, data->year);
  writeLittleEndian(payload, 6, data->month);
  writeLittleEndian(payload, 7, data->day);
  writeLittleEndian(payload, 8, data->hour);
  writeLittleEndian(payload, 9, data->min);
  writeLittleEndian(payload, 10, data->sec);

  // Status and Accuracy
  uint8_t val = 0;
  if (data->valid.bits.validDate)
    val |= (1 << 0);
  if (data->valid.bits.validTime)
    val |= (1 << 1);
  if (data->valid.bits.fullyResolved)
    val |= (1 << 2);
  writeLittleEndian(payload, 11, val);
  writeLittleEndian(payload, 12, data->tAcc);
  writeLittleEndian(payload, 16, data->nano);
  writeLittleEndian(payload, 20, data->fixType);

  // Fix and Info Flags
  uint8_t fixFlags = 0;
  if (data->fixType == 3)
    fixFlags |= (1 << 0);
  if (myGNSS.getHeadVehValid())
    fixFlags |= (1 << 5);
  writeLittleEndian(payload, 21, fixFlags);

  uint8_t dtFlags = 0;
  if (data->valid.bits.validTime)
    dtFlags |= (1 << 5);
  if (data->valid.bits.validDate)
    dtFlags |= (1 << 6);
  if (data->valid.bits.validTime && data->valid.bits.fullyResolved)
    dtFlags |= (1 << 7);
  writeLittleEndian(payload, 22, dtFlags);

  // Position, Speed, and Heading
  writeLittleEndian(payload, 23, data->numSV);
  writeLittleEndian(payload, 24, (int32_t)data->lon);
  writeLittleEndian(payload, 28, (int32_t)data->lat);
  writeLittleEndian(payload, 32, (int32_t)data->height);
  writeLittleEndian(payload, 36, (int32_t)data->hMSL);
  writeLittleEndian(payload, 40, (uint32_t)data->hAcc);
  writeLittleEndian(payload, 44, (uint32_t)data->vAcc);
  writeLittleEndian(payload, 48, (int32_t)data->gSpeed);
  writeLittleEndian(payload, 52, (int32_t)data->headMot);
  writeLittleEndian(payload, 56, (uint32_t)data->sAcc);
  writeLittleEndian(payload, 60, (uint32_t)data->headAcc);
  writeLittleEndian(payload, 64, (uint16_t)data->pDOP);

  // Fix Quality and Misc
  if (data->fixType < 2)
    writeLittleEndian(payload, 66, (uint8_t)(1 << 0));

  uint8_t batPct = currentBatteryPercentage & 0x7F;
  if (isCharging())
    batPct |= 0x80;
  writeLittleEndian(payload, 67, batPct);

  // Physical Sensors (Hardware Filtered)
  writeLittleEndian(payload, 68, (int16_t)(imu_ax * 1000.0));
  writeLittleEndian(payload, 70, (int16_t)(imu_ay * 1000.0));
  writeLittleEndian(payload, 72, (int16_t)(imu_az * 1000.0));
  writeLittleEndian(payload, 74, (int16_t)(imu_gx * 100.0));
  writeLittleEndian(payload, 76, (int16_t)(imu_gy * 100.0));
  writeLittleEndian(payload, 78, (int16_t)(imu_gz * 100.0));

  return true;
}

// Émission du message live 0xFF 0x01 vers le client connecté.
void sendLivePacket(const uint8_t *payload) {
  if (!deviceConnected)
    return;
  // La doc impose de couper les données live pendant un dump mémoire ou un
  // effacement, pour libérer toute la bande passante BLE.
  if (downloadActive || eraseActive)
    return;

  uint8_t packet[88] = {0};
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

  // --- CORRIGÉ : tout passe par la file d'émission. Auparavant les paquets
  //     live partaient directement par notify() tandis que les réponses aux
  //     commandes transitaient par la file : un paquet live pouvait donc
  //     s'insérer au milieu d'une réponse fragmentée et corrompre le flux
  //     d'octets côté client, qui rejetait alors la réponse.
  //     Les données live sont sacrifiables : on garde toujours une réserve
  //     libre pour qu'une réponse de commande ne soit jamais perdue faute de
  //     place dans la file.
  if (txqFree() < 88 + 256)
    return;
  if (txqPush(packet, 88))
    gpsUpdateCount++;
}

// Background polling and data routing for the u-blox module
void processGNSS() {
  if (!gpsEnabled)
    return;
  myGNSS.checkUblox();

  if (myGNSS.getPVT()) {
    static uint32_t lastITOW = 0;
    uint32_t currentITOW = myGNSS.packetUBXNAVPVT->data.iTOW;
    GPSFixType = myGNSS.packetUBXNAVPVT->data.fixType;
    if (currentITOW != lastITOW) {
      lastITOW = currentITOW;
      gnssUpdateCount++;

      // --- MODIFIÉ : un seul payload construit par point GNSS, utilisé
      //     pour l'enregistrement local ET pour la diffusion BLE.
      uint8_t payload[80];
      if (buildDataPayload(payload)) {
        recordTick(payload);
        sendLivePacket(payload);
      }
    }
  }

  // Backup Recovery: Ensure background processing during data stalls
  static unsigned long lastValidData = 0;
  if (myGNSS.getPVT())
    lastValidData = millis();
  if (deviceConnected && (millis() - lastValidData > 2000))
    myGNSS.checkUblox();
}

// IMU Sampling (Hardware filters handle smoothing)
void processIMU() {
  if (!imuEnabled)
    return;

  imu_ax = IMU.readFloatAccelX();
  imu_ay = IMU.readFloatAccelY();
  imu_az = IMU.readFloatAccelZ();
  imu_gx = IMU.readFloatGyroX();
  imu_gy = IMU.readFloatGyroY();
  imu_gz = IMU.readFloatGyroZ();
}

// ============================================================================
// --- 🔋 POWER & SYSTEM MANAGEMENT ---
// ============================================================================
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

    // Sniff for raw activity first (for 1.5 seconds)
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

      // Clear the buffer of NMEA messages
      delay(200);
      while (Serial1.available())
        Serial1.read();

      // Try the library sync
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
        Serial.println("❌ Bytes received, but u-blox library could not sync "
                       "(Check protocol/clones).");
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

  // High-Performance Track Dynamics Configuration
  // Used Airborne2g since passenger track cars (GT3/GT4) easily exceed 1g
  // braking/cornering and we do not want the navigation engine to "smooth" or
  // reject these forces as anomalies.
  // --- MODIFIÉ : valeur par défaut inchangée (Airborne2g, choix volontaire de
  //     l'auteur pour la piste), mais l'app peut la changer via 0xFF 0x27.
  myGNSS.setDynamicModel((dynModel)recCfg.gnssDynModel);

  // Disable low-pass filters to prevent time-domain lag on braking and
  // acceleration telemetry
  myGNSS.setVal8(0x10220001,
                 0); // CFG-ODO-OUTLPVEL: Disable speed (3D) low-pass filter
  myGNSS.setVal8(
      0x10220002,
      0); // CFG-ODO-OUTLPCOG: Disable course over ground low-pass filter

  // Ensure static hold is explicitly disabled so slow pit-lane creeping isn't
  // frozen at 0mph
  myGNSS.setVal8(0x20250038,
                 0); // CFG-MOT-GNSSSPEED_THRS: Static hold threshold = 0

  // Enable Super-S (Automatic Mode) to compensate for any signal attenuation
  // through the ABS enclosure
  myGNSS.setVal8(0x201100D5, 1); // CFG-NAVSPG-SIGATTCOMP: 1 = Automatic

  // For SAM-M10Q 25Hz, we MUST explicitly disable other constellations FIRST
  // before increasing the navigation frequency, otherwise the module rejects
  // it.
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

  // Now that extraneous constellations are OFF, we can safely request 25Hz
  // --- MODIFIÉ : si une session est configurée à 10/5/1 Hz, il ne faut pas
  //     que cette reconfiguration la ramène de force à 25 Hz.
  {
    uint8_t navHz =
        recCfg.enabled ? rateToHz(recCfg.dataRate) : MAX_NAVIGATION_RATE;
    myGNSS.setNavigationFrequency(navHz);
  }

  pendingConfig = false;
  Serial.println("✅ Configuration complete.");
  return true;
}

// Re-configure advertising with specific power and interval
// Standard Adafruit/Seeed Bluefruit requires re-adding data to update the
// packet-reported TX power
void setupAdvertising(int8_t power, uint16_t interval) {
  if (deviceConnected)
    return;

  Bluefruit.Advertising.stop();
  Bluefruit.setTxPower(power);
  Bluefruit.Advertising.setInterval(interval, interval + 200);

  // Clear and Rebuild Advertising Data
  // This ensures the TX Power field in the packet matches the new hardware
  // power
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // NOTE: We keep the Service UUIDs in the primary advertisement for
  // compatibility with the RaceBox application.
  Bluefruit.Advertising.addService(rbService);
  Bluefruit.Advertising.addService(disService);

  // Scan Response only contains the Name to keep it simple
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
  // Serial1.end(); // Dont uncomment this, it causes a 4mA power leak
  // pinMode(PIN_SERIAL1_TX, INPUT_PULLDOWN);
  // pinMode(PIN_SERIAL1_RX, INPUT_PULLDOWN);

  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  if (!deviceConnected) {
    setupAdvertising(LOW_POWER_BT_TX_POWER, ECO_ADV_INTERVAL);
  }
}

void enableIMU() {
  if (imuEnabled)
    return;
  IMU.settings.accelSampleRate = 1660; // 1.6kHz for hardware filtering
  IMU.settings.gyroSampleRate = 1660;
  // --- MODIFIÉ : ±16 g au lieu de ±8 g. Sur une voiture RC, les chocs
  //     d'atterrissage de saut saturaient l'échelle ±8 g : les relevés
  //     plafonnaient alors à 7,99 g sur les trois axes simultanément, ce qui
  //     masquait la valeur réelle. La contrepartie est une résolution deux
  //     fois moins fine (0,488 mg par pas au lieu de 0,244), négligeable
  //     devant l'amplitude mesurée ici.
  IMU.settings.accelRange = 16;
  IMU.settings.gyroRange = 500;

  if (IMU.begin() != 0)
    return;

  // --- HARDWARE FILTER CONFIGURATION ---
  // Gyroscope: Enable LPF1 (Register 0x13, Bit 1)
  IMU.writeRegister(0x13, 0x02);
  // Gyroscope: Set LPF1 Bandwidth (Register 0x15, Bits 1:0 = 01)
  IMU.writeRegister(0x15, 0x01);
  // Accelerometer: Enable LPF2 (Register 0x17, Bit 0) and Set HPCF (Bits 6:5 =
  // 01 for ODR/50)
  IMU.writeRegister(0x17, 0x21);

  imuEnabled = true;
}

void disableIMU() {
  if (!imuEnabled)
    return;
  // Explicitly power down the sensor registers
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x00);
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x00);
  imuEnabled = false;
}

void powerDownSensors() {
  // Soft disable
  if (gpsEnabled)
    disableGPS();
  if (imuEnabled)
    disableIMU();

  // Enforce hardware shutdown state to prevent parasitic leaks or pin sync
  // issues
#ifdef PCB_VERSION
  digitalWrite(GPS_EN_PIN, HIGH);
#else
  digitalWrite(GPS_EN_PIN, LOW);
#endif
}

// Manage Charging, Disconnect Timeouts, and Deep Sleep
void managePower() {
  if (isNoBatteryMode) {
    // In No-Battery mode, everything stays hot always.
    if (!gpsEnabled)
      enableGPS();
    configureGPS();
    if (!imuEnabled)
      enableIMU();
    return;
  }

  bool currentlyPluggedIn = isPluggedIn();

  if (!currentlyPluggedIn && !deviceConnected &&
      currentBatteryPercentage == 0) {
    enterDeepSleep();
  }

  // Determine if sensors should be active
  // --- MODIFIÉ : un enregistrement autonome doit continuer même sans client
  //     BLE connecté — c'est tout l'intérêt du mode Mini S.
  bool shouldBeActive = deviceConnected ||
                        (currentlyPluggedIn && !SLEEP_WHILE_CHARGING) ||
                        (recState != REC_STATE_OFF);

  if (shouldBeActive) {
    lastActivityTime = millis();
    lastDisconnectTime = millis();
    if (!gpsEnabled) {
      enableGPS();
    }
    // Keep trying to configure until pendingConfig is false
    configureGPS();
    enableIMU();
  } else {
    if (imuEnabled)
      disableIMU();
  }

  // Hot-State Timeout (Keep GPS active for a window after usage)
  // --- MODIFIÉ : ne jamais redémarrer le MCU pendant un enregistrement,
  //     cela couperait la session en cours.
  if (!deviceConnected && gpsEnabled && SLEEP_WHILE_CHARGING &&
      recState == REC_STATE_OFF) {
    if (millis() - lastDisconnectTime > GPS_HOT_TIMEOUT_MS) {
      Serial.printf(
          "⏰ GPS Hot Timeout Reached. Rebooting to clear hardware state...\n");
      Serial.flush();
      delay(10);

      // Hard reset the MCU instead of trying to manually power down bugged
      // peripherals. Setup() will natively drop the board back into its 40uA
      // state.
      NVIC_SystemReset();
    }
  }

  // Deep Sleep Safety Net
  if (ENABLE_DEEP_SLEEP && !deviceConnected && !currentlyPluggedIn) {
    if (millis() - lastActivityTime > (DEEP_SLEEP_DAYS * 86400000UL))
      enterDeepSleep();
  }

  // Reset charging state trackers
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
    return; // Skip battery processing completely in No-Battery Mode
  }

  static unsigned long lastBatteryUpdate = 0;
  const unsigned long batteryInterval = 30000; // 30 Seconds

  bool charging = isCharging();

  // Force an update if the power state just changed (Plugged in or Unplugged)
  // This ensures the Serial report and LEDs react instantly to the cable.
  static bool lastChargingStatus = false;
  bool stateChanged = (charging != lastChargingStatus);

  batteryConnected = true;

  if (millis() - lastBatteryUpdate >= batteryInterval || stateChanged ||
      lastBatteryUpdate == 0) {
    lastBatteryUpdate = millis();
    lastChargingStatus = charging;

    updateBatteryState();
  }
}

// Periodic Status Feed to the Computer
void reportSystemStats() {
  if (millis() - lastGpsRateCheckTime < SYSTEM_RATE_REPORT_MS)
    return;

  float bleRate = gpsUpdateCount / (SYSTEM_RATE_REPORT_MS / 1000.0);
  float gnssRate = gnssUpdateCount / (SYSTEM_RATE_REPORT_MS / 1000.0);
  Serial.println("--------------------------------------------------");
  Serial.printf("POWER   | Bat: %d%% (%0.2fV) \n", currentBatteryPercentage,
                getBatteryVoltage());
  Serial.printf("STATE   | Charging: %s | USB: %s | BLE: %s | BAT: %s\n",
                isCharging() ? "YES ⚡" : "NO 🔋",
                isPluggedIn() ? "CONNECTED" : "DISCONNECTED",
                deviceConnected ? "CONNECTED" : "IDLE",
                batteryConnected ? "PRESENT" : "MISSING");
  if (gpsEnabled && myGNSS.packetUBXNAVPVT) {
    Serial.printf(
        "GNSS    | BLE: %.2f Hz | GNSS: %.2f Hz | SVs: %u | Fix: %u\n", bleRate,
        gnssRate, myGNSS.packetUBXNAVPVT->data.numSV,
        myGNSS.packetUBXNAVPVT->data.fixType);
  }

  // --- AJOUT : état du stockage, visible en permanence. Évite de dépendre
  //     des messages de setup(), souvent manqués car le moniteur série se
  //     reconnecte après la ré-énumération USB qui suit le reset.
  Serial.printf("BUILD   | %s · %s · accel ±16 g\n", FIRMWARE_VER, BUILD_STAMP);
  if (!flashReady) {
    Serial.println("MEMORY  | ❌ Flash QSPI NON détectée (enregistrement off)");
  } else {
    const char *st = (recState == REC_STATE_RUNNING)   ? "EN COURS"
                     : (recState == REC_STATE_PAUSED)  ? "EN PAUSE"
                                                       : "arrêté";
    uint32_t freeSlots = totalSlots - usedSlots;
    Serial.printf("MEMORY  | %lu/%lu (%lu%%) | REC: %s @%uHz | reste ~%lu min\n",
                  (unsigned long)usedSlots, (unsigned long)totalSlots,
                  (unsigned long)(totalSlots ? usedSlots * 100 / totalSlots : 0),
                  st, rateToHz(recCfg.dataRate),
                  (unsigned long)(freeSlots /
                                  (60UL * rateToHz(recCfg.dataRate))));
  }
  Serial.println("--------------------------------------------------");

  gpsUpdateCount = 0;
  gnssUpdateCount = 0;
  lastGpsRateCheckTime = millis();
}

// --- LED Status ---
void updateLEDs(uint8_t fixType) {
  static unsigned long lastBlink = 0;
  static bool ledState = false;

  // --- AJOUT : témoin d'enregistrement autonome. Sans client connecté, la
  //     LED bleue est libre : un flash bref toutes les 3 s indique que la
  //     session tourne, un double flash rapide que la mémoire est pleine.
  if (!deviceConnected) {
    static unsigned long recBlinkRef = 0;
    unsigned long phase = millis() % 3000;
    bool on = false;
    if (memoryFull && recCfg.enabled)
      on = (phase < 100) || (phase >= 200 && phase < 300);
    else if (recState == REC_STATE_RUNNING)
      on = (phase < 80);
    else if (recState == REC_STATE_PAUSED)
      on = (phase < 80) || (phase >= 1500 && phase < 1580);
    digitalWrite(OnboardledPin, on ? LOW : HIGH);
    (void)recBlinkRef;
  }

  // 1. HIGHEST PRIORITY: Critical Battery Alert
  if (isCritical && !isCharging()) {
    if (millis() - lastBlink >= 500) {
      lastBlink = millis();
      ledState = !ledState;

      // Blink Red, keep Green off during critical alert
      digitalWrite(LED_RED, ledState);
      digitalWrite(LED_GREEN, HIGH);
    }
    return; // Exit early so GPS logic doesn't overrule the blink
  }

  // 2. SECOND PRIORITY: GPS Disabled
  if (!gpsEnabled) {
    digitalWrite(LED_RED, HIGH);   // OFF
    digitalWrite(LED_GREEN, HIGH); // OFF
    return;
  }

  // 3. LOWEST PRIORITY: Standard GPS Status
  switch (fixType) {
  case 3:                         // 3D Fix
  case 4:                         // GNSS + DR
    digitalWrite(LED_RED, HIGH);  // Red OFF
    digitalWrite(LED_GREEN, LOW); // Green ON
    break;

  case 1:                         // DR only
  case 2:                         // 2D Fix
    digitalWrite(LED_RED, LOW);   // Red ON
    digitalWrite(LED_GREEN, LOW); // Green ON -> Orange/Yellow
    break;

  default:                         // No fix
    digitalWrite(LED_RED, LOW);    // Red ON
    digitalWrite(LED_GREEN, HIGH); // Green OFF
    break;
  }
}

// BLE Connection Callbacks
void connect_callback(uint16_t conn_handle) {
  deviceConnected = true;
  lastActivityTime = millis();
  lastDisconnectTime = millis();

  digitalWrite(OnboardledPin, LOW); // Solid Blue ON when connected
  Serial.println("✅ Client connected!");

  // Bumping TX power back to 0 dBm for a stable, high-range connection
  Bluefruit.setTxPower(0);

  Bluefruit.Connection(conn_handle)->requestPHY(); // try 2M, harmless if rejected
  Bluefruit.Connection(conn_handle)->requestMtuExchange(247);
  Bluefruit.Connection(conn_handle)->requestConnectionParameter(24, 0, 400); // 30ms, 4s sup timeout
  delay(400);
  uint16_t mtu = Bluefruit.Connection(conn_handle)->getMtu();
  Serial.printf(">> Negotiated MTU = %u (need >= 91 to fit 88-byte notify)\n", mtu);

  // --- AJOUT : le MTU sert à dimensionner les notifications du dump mémoire.
  negotiatedMtu = mtu;
  // La doc précise que le verrou mémoire est réinitialisé à chaque connexion.
  asmLen = 0;
  rxTail = rxHead;
  txqTail = txqHead;
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  deviceConnected = false;
  lastDisconnectTime = millis(); // Start GPS hot timeout timer immediately
  lastActivityTime = millis();   // Count disconnection as activity

  // Turn off Blue LED immediately on disconnect
  digitalWrite(OnboardledPin, HIGH);

  // --- AJOUT : un dump en cours n'a plus de destinataire.
  if (downloadActive) {
    downloadActive = false;
    Serial.println("⏹️ Téléchargement interrompu (déconnexion).");
  }
  txqTail = txqHead;
  asmLen = 0;

  Serial.println("❌ BLE Client disconnected.");
  Serial.printf("🛰️ GPS staying hot for %d minutes...\n",
                (GPS_HOT_TIMEOUT_MS / 60000));
}

void write_callback(uint16_t conn_handle, BLECharacteristic *chr, uint8_t *data,
                    uint16_t len) {
  // --- MODIFIÉ : on ne fait qu'empiler les octets ici. Ce callback tourne
  //     dans le contexte du SoftDevice : aucun accès flash ni traitement long
  //     ne doit y être fait. L'assemblage des trames se fait dans loop().
  for (uint16_t i = 0; i < len; i++) {
    uint16_t next = (rxHead + 1) & (RXRING_SIZE - 1);
    if (next == rxTail)
      break; // anneau plein : on jette le surplus
    rxRing[rxHead] = data[i];
    rxHead = next;
  }
  rxByteCount += len;
}

void setIMUForSleep() {
  IMU.settings.gyroEnabled = 0;
  IMU.settings.accelEnabled = 0;
  IMU.begin();
  // 52Hz, ±2g
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x30);
  // Enable tap detection
  IMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x8E);
  IMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, 0x8C);
  IMU.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2, 0x20);
  // Enable single+double tap
  IMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x80);
  // Low power accel
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL6_G, 0x10);
  // Route to INT1
  IMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x08);
  // Enable wake pin
  pinMode(PIN_LSM6DS3TR_C_INT1, INPUT_PULLDOWN_SENSE);
}

void enterDeepSleep() {
  Serial.println("💤 Entering Deep Sleep (Shake to Wake)...");
  Bluefruit.autoConnLed(false);

  // Turn off all LEDs
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);

  // Ensure GPS is off
  disableGPS();

  // Configure Triggers for Wake-up
  setIMUForSleep(); // Trigger 1: Shake (IMU INT pin)
  pinMode(PIN_CHG,
          INPUT_PULLUP_SENSE); // Trigger 2: Plug-in (Charge pin goes LOW)

  delay(100); // Small delay for I2C to finish and IMU to settle
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);

  Serial.flush(); // Ensure serial message is sent before power cut
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
  digitalWrite(PIN_VBAT_ENABLE, LOW); // Start LOW & Stay LOW (Safe & Stable)
  pinMode(PIN_HICHG, OUTPUT);
  digitalWrite(PIN_HICHG, LOW);
  pinMode(PIN_CHG, INPUT_PULLUP); // Prevent float current leakage

  Wire.setClock(400000);
  analogReference(AR_DEFAULT);
  analogReadResolution(12);

  NRF_POWER->DCDCEN = 1; // Enable DC-DC converter (Saves ~30% radio current)
  // Enable REG0 DC-DC if using VDDH (High Voltage Mode)
  if (NRF_POWER->MAINREGSTATUS & (POWER_MAINREGSTATUS_MAINREGSTATUS_High
                                  << POWER_MAINREGSTATUS_MAINREGSTATUS_Pos)) {
    NRF_POWER->DCDCEN0 = 1;
  }

  // Safety: Ensure QSPI Flash CS is High (Deselected) to prevent floating
  // inputs
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

  // --- AJOUT : service de mise à jour sans fil (OTA DFU).
  //     Permet de reflasher l'appareil depuis un téléphone Android avec
  //     l'application « nRF Device Firmware Update », sans câble ni bouton.
  //     Adafruit impose que ce service soit déclaré EN PREMIER.
#if ENABLE_OTA_DFU
  bledfu.begin();
#endif

  // Service Setup
  disService.begin();
  disModel.setProperties(CHR_PROPS_READ);
  disModel.begin();
  // --- CORRIGÉ : la doc RaceBox impose que cette caractéristique contienne
  //     exactement "RaceBox Mini", "RaceBox Mini S" ou "RaceBox Micro" —
  //     PAS le nom complet avec le numéro de série. C'est sur cette chaîne
  //     que l'app décide d'afficher ou non les fonctions de mémoire.
  disModel.write(MODEL_STRING);
  disSerial.setProperties(CHR_PROPS_READ);
  disSerial.begin();
  disSerial.write(SERIAL_NUM);
  disFirmware.setProperties(CHR_PROPS_READ);
  disFirmware.begin();
  disFirmware.write(FIRMWARE_VER);
  // --- AJOUT : disHardware était déclaré mais jamais initialisé.
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
    Serial.printf(">> CCCD write on rbTx: 0x%04X (notify %s)\n",
      value, (value & 0x0001) ? "ENABLED" : "disabled");
  });
  rbTx.begin();

  rbRx.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  rbRx.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  // --- CORRIGÉ : sans setMaxLen(), Bluefruit limite la caractéristique à
  //     20 octets. Toute écriture plus longue est rejetée par le SoftDevice
  //     SANS déclencher le callback : les commandes de l'app passaient donc
  //     totalement inaperçues.
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
  Serial.printf("   Firmware %s · compilé le %s\n", FIRMWARE_VER, BUILD_STAMP);
  Serial.println("   Commandes série : s=stop  i=état  z=config par défaut  ?=aide");

  setupHardware();

  // --- AJOUT : la flash doit être prête avant tout le reste, car la config
  //     d'enregistrement qui y est stockée conditionne l'état de démarrage.
  setupStorage();

  Serial.println("🔍 Checking power source...");
  isNoBatteryMode = detectNoBatteryAtBoot();
  if (isNoBatteryMode) {
    batteryConnected = false;
    Serial.println("⚠️ NO BATTERY DETECTED! Booting in USB Always-On mode.");
  } else {
    Serial.println("🔋 Battery detected! Booting in standard/Eco mode.");
  }

  updateBatteryState();
  if (currentBatteryPercentage == 0 && !isNoBatteryMode && !isCharging()) {
    // Flash RED LED for 5 seconds
    // (10 cycles of 250ms ON + 250ms OFF = 5 seconds)
    for (int i = 0; i < 10; i++) {
      digitalWrite(LED_RED, LOW); // ON
      delay(250);
      digitalWrite(LED_RED, HIGH); // OFF
      delay(250);
    }
    // Ensure everything is off before sleep
    digitalWrite(LED_RED, HIGH);
    // Enter Deep Sleep
    enterDeepSleep();
  }

  if (IMU.begin() != 0) {
    Serial.println("❌ IMU Init Failed");
  } else {
    imuEnabled = true;
    if (!isNoBatteryMode)
      disableIMU();
  }

  setupBLE();

  // Initial Advertising Setup
  if (isNoBatteryMode) {
    setupAdvertising(0, FAST_ADV_INTERVAL);
    enableGPS(); // Keep it hot from the start
    Serial.println("📡 BLE Broadcast Started (FAST - Always On).");
  } else {
    setupAdvertising(LOW_POWER_BT_TX_POWER, ECO_ADV_INTERVAL);
    disableGPS();
    Serial.println("📡 BLE Broadcast Started (ECO).");
  }

  // Initialize Activity Trackers to current time to prevent immediate timeouts
  // after long boot/standby durations.
  lastActivityTime = millis();
  lastDisconnectTime = millis();
  lastGpsRateCheckTime = millis();

  // --- AJOUT : comme un vrai Mini S / Micro, la configuration survit à une
  //     coupure d'alimentation. Si l'enregistrement était actif, on reprend
  //     tout seul, sans avoir besoin du téléphone.
  // --- SÉCURITÉ : une configuration marquée « active » au démarrage signifie
  //     que la session précédente ne s'est pas terminée proprement — coupure
  //     d'alimentation, batterie arrachée, redémarrage. Reprendre seul
  //     rallumerait le GPS et rendrait l'appareil difficile à reprendre en
  //     main. On repart donc à l'arrêt, sans jamais toucher aux données déjà
  //     enregistrées, qui restent intégralement téléchargeables.
  if (flashReady && recCfg.enabled) {
#if AUTO_RESUME_RECORDING
    if (!memoryFull) {
      Serial.println("⏺️ Reprise de l'enregistrement autonome.");
      recState = REC_STATE_RUNNING;
      anyDataSinceEnable = false;
      slowSinceMs = 0;
      noFixSinceMs = 0;
      lastStoredDataMs = millis();
      pendingConfig = true;
      enableGPS();
      enableIMU();
      announceStateChange(REC_STATE_RUNNING, true);
    } else {
      Serial.println("⚠️ Mémoire pleine : enregistrement non repris.");
      recCfg.enabled = 0;
      saveConfig();
    }
#else
    Serial.println("⚠️ Session précédente interrompue anormalement.");
    Serial.println("   Enregistrement laissé à l'arrêt par sécurité.");
    Serial.printf("   %lu points conservés et téléchargeables.\n",
                  (unsigned long)usedSlots);
    recCfg.enabled = 0;
    recState = REC_STATE_OFF;
    saveConfig();
#endif
  }

  // Flash GREEN LED 5 times to indicate successful startup
  Serial.println("Startup Complete.");
  for (int i = 0; i < 1; i++) {
    digitalWrite(LED_GREEN, LOW); // ON
    delay(500);
    digitalWrite(LED_GREEN, HIGH); // OFF
    delay(500);
  }
}

// --- SÉCURITÉ : commandes de secours au clavier, dans le moniteur série.
//     Permettent de reprendre la main sur un appareil parti en enregistrement
//     sans avoir à le reflasher.
static void serviceSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r' || c == ' ')
      continue;
    switch (c) {
    case 's': // stop d'urgence
      Serial.println("🛑 Arrêt d'urgence demandé.");
      if (recState != REC_STATE_OFF)
        stopRecording();
      else {
        recCfg.enabled = 0;
        saveConfig();
        Serial.println("   (aucun enregistrement en cours)");
      }
      break;
    case 'i': // état complet
      Serial.printf("ÉTAT | flash %s | %lu/%lu points | REC %s @%uHz | "
                    "config seq %lu (exemplaire %c)\n",
                    flashReady ? "OK" : "absente", (unsigned long)usedSlots,
                    (unsigned long)totalSlots,
                    recState == REC_STATE_RUNNING   ? "en cours"
                    : recState == REC_STATE_PAUSED  ? "en pause"
                                                    : "arrêté",
                    rateToHz(recCfg.dataRate), (unsigned long)recCfg.seq,
                    lastMetaSlot ? 'B' : 'A');
      break;
    case 'z': // remise à zéro de la configuration
      Serial.println("♻️ Configuration réinitialisée (données conservées).");
      if (recState != REC_STATE_OFF)
        stopRecording();
      defaultConfig();
      saveConfig();
      break;
    case 'd': // redémarrage en mode mise à jour sans fil
      Serial.println("📡 Redémarrage en mode OTA DFU...");
      Serial.println("   Cherchez « AdaDFU » depuis l'application nRF DFU.");
      Serial.flush();
      delay(100);
      // 0xA8 est le code que le bootloader Adafruit interprète comme
      // « démarrer en mise à jour sans fil ». Avec le SoftDevice actif, le
      // registre doit être écrit par appel système.
      sd_power_gpregret_clr(0, 0xFF);
      sd_power_gpregret_set(0, 0xA8);
      NVIC_SystemReset();
      break;
    case '?':
      Serial.println("Commandes : s=stop d'urgence  i=état  z=config par "
                     "défaut  d=mise à jour sans fil  ?=aide");
      break;
    default:
      break;
    }
  }
}

void loop() {
  serviceSerialCommands(); // secours clavier : toujours en premier
  // --- AJOUT : le protocole passe avant tout, y compris en veille, sinon une
  //     commande envoyée par l'app resterait sans réponse pendant 2,5 s.
  serviceRxParser(); // assemble les trames reçues et exécute les commandes
  serviceErase();    // effacement progressif, avec notifications de %
  serviceDownload(); // dump mémoire vers l'app
  txqService();      // vidange de la file d'émission BLE
  serviceAutoShutdown();

  // --- MODIFIÉ : on ne part jamais en veille pendant un enregistrement,
  //     un téléchargement ou un effacement.
  bool busy = (recState != REC_STATE_OFF) || downloadActive || eraseActive;
  bool idle = !deviceConnected && !gpsEnabled && !imuEnabled && !busy;

  if (idle && !isNoBatteryMode) {
    manageBatterySampling(); // Always track battery to prevent deep sleep death
    if (isPluggedIn()) {
      reportSystemStats();
    }
    managePower();
    powerDownSensors(); // Enforce shutdown state while in light sleep
    // Sleep in small chunks so we can wake up instantly when a BLE connection
    // occurs
    for (int i = 0; i < LOOP_SLEEP; i += 100) {
      if (deviceConnected)
        break;
      delay(100);
    }
    return;
  }

  processGNSS();
  processIMU();
  managePower();
  reportSystemStats();
  manageBatterySampling();
  updateLEDs(GPSFixType);
  delay(1);
}
