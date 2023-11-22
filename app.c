#include "app.h"
#include "app_config.h"

#include "tokens.h"
#include "tokenutil.h"

#include "sensors/rhtemp.h"
#include "sensors/battery_voltage.h"

#include "config/app_properties_config.h"

#include "em_common.h"
#include "em_rmu.h"
#include "em_system.h"
#include "em_msc.h"
#include "em_cmu.h"

#include "app_assert.h"
#include "sl_bluetooth.h"
#include "sl_status.h"
#include "sl_sleeptimer.h"
#include "sl_board_control.h"

#include "nvm3.h"
#include "nvm3_default.h"

#include "gatt_db.h"
#include "mbedtls/aes.h"
#include "mbedtls/sha256.h"

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>

// CONFIG
const uint8_t swVersion = 0x02;

/* Force send a packet this number of seconds since last packet */
const unsigned heartbeatTimebaseDelta = 30 * 60;

// DEFINES
typedef struct {
  bool enabled;
  uint32_t lastMeasurementStarted;
  uint32_t measurementInterval;
} SensorState;


#define MAX_MEASUREMENT_COUNT 10
#if DEBUG_OUT
#define dbg_printf(...) printf(__VA_ARGS__)
#else
#define dbg_printf(...)
#endif

#if ENCRYPTION
static const uint8_t app_packetId[2] = {0xAA,0x06}; /* PacketId AA06 for V2 encrypted packet*/
static const bool app_encryption = true;
#else
static const uint8_t app_packetId[2] = {0xAA,0x56}; /* PacketId AA56 for V2 plaintext packet*/
static const bool app_encryption = false;
#endif

static const uint8_t *app_aesKey = TOKEN_TO_ARRAY(AES_KEY_ADDR);
static int     app_txPower;
static uint8_t app_pktCount;
static uint8_t app_pktCountEvent;
static bool    app_tick = false;
static bool    app_sendPkt = false;

// NVM3 keys
const uint32_t RESET_COUNTER_KEY  = 0x01;


typedef void (*MeasureFunc)(void);
const MeasureFunc measureFunctions[SENSOR_ID_COUNT] = {
  rhtemp_measure,
  NULL
};

typedef struct {
  MeasurementType type;
  uint32_t value;
} Measurement;

/***********************************************************************************************//**
 * Global variables
 **************************************************************************************************/

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

// Large default time base, sensors can set lower values
static uint32_t timeBase           = 120000;
static uint32_t packetCnt          = 0;
static uint32_t lastPacketSentTime = 0;

static uint8_t  relativeHumidity = 0;
static int16_t  temperature      = 0x8000;  // = -128.0

static Measurement measurements[MAX_MEASUREMENT_COUNT];
static size_t      measurementCount = 0;
static SensorState sensorStates[SENSOR_ID_COUNT];


static sl_sleeptimer_timer_handle_t timebaseTimer;


static uint8_t ctr[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

/***********************************************************************************************//**
 * Local functions
 **************************************************************************************************/

// static void advertise(void);
static void timebaseTick(sl_sleeptimer_timer_handle_t *handle, void *data);
static void tick(void);
static void setPayload(void);
static void encryptPayload(void);
static void sendPacket(uint8_t pktCount);
static void initSensors(void);
static void initNonce(void);
static bool initTokens(void);
static void initPacket(void);
static uint32_t htonl(uint32_t h);
static uint16_t htons(uint16_t h);

static void dbg_printArray(const uint8_t* binbuf, unsigned int binbuflen);

typedef struct __attribute__((__packed__)) {
  uint8_t  swVersion;
  uint8_t  relativeHumidity;
  int16_t  temperature;
  uint16_t uptime;
  uint8_t  voltage;
  uint8_t  extraType;
  uint32_t extraValue;
  uint32_t magic; // magic field AA55FF00 (must decrypt correctly = MAC)
} PktPayload;

typedef struct __attribute__((__packed__)) {
  uint8_t flagsLen;     /* Length of the Flags field. */
  uint8_t flagsType;    /* Type of the Flags field. */
  uint8_t flags;        /* Flags field. */
  uint8_t mandataLen;   /* Length of the Manufacturer Data field. */
  uint8_t mandataType;  /* Type of the Manufacturer Data field. */
  uint8_t packetId[2];
  uint8_t payload[sizeof(PktPayload)];
  uint32_t nonce;
} AdvPacket;

static AdvPacket packet;
static PktPayload payload;

// Add call to an init function which registers the sensor here
static void initSensors(void)
{
  uint8_t enable;
  if (getTokenU8(FEATURE_RH_TEMP_ADDR, &enable) && enable){
    if (!rhtemp_init()){
      dbg_printf("RH Temp init failed!\r\n");
    }
  }
  batt_init();
}

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void)
{
  sl_sleeptimer_init();
  app_board_init();
  if (!initTokens()){
    dbg_printf("Tokens are not configured correctly, using defaults!\r\n");
  }

  initSensors();
  initPacket();
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void)
{
  if (app_tick){
    app_tick = false;
    tick();
  }

  if (app_sendPkt) {
    app_sendPkt = false;

    uint32_t uptime = sl_sleeptimer_get_time();
    uint8_t pktCount = app_pktCount;
    if (measurementCount > 0) {
      // Assume all "extra" measurements are important - send 3 times
      pktCount = app_pktCountEvent;
    }
    lastPacketSentTime = uptime;
    setPayload();
    sendPacket(pktCount);
    // // Send more often if there are pending measurements?
    // if (measurementCount > 0) {

    // }
  }
}

void tick(void) {
  static uint32_t initSequenceStage = 0;
  sl_status_t sc;

  if (initSequenceStage != UINT32_MAX){
    // Do init procedure: Dummy send, increment reset counter, proceed with real packet
    if (initSequenceStage == 0){
      // Prepare dummy packet - no encryption yet to not leak any information
      dbg_printf("Sending dummy packet to test for BOD. All data 0xA5.\r\n");
      memset(&packet.payload, 0xA5, sizeof(packet.payload));
      packet.nonce = 0;
      sendPacket(1);
      initSequenceStage++;
      sc = sl_sleeptimer_restart_periodic_timer_ms(&timebaseTimer, 1000,
                                               timebaseTick, NULL,
                                                0, 0);
      app_assert(sc == SL_STATUS_OK, "[E: 0x%04x] Failed to restart timer", sc);
      // Let the advertiser send the packet, be back in a second
      return;
    }
    else if ( initSequenceStage == 1 ){
      // Sending went OK - we can increment reset counter and start encrpyting
      initNonce();
      sc = sl_sleeptimer_restart_periodic_timer_ms(&timebaseTimer, timeBase,
                                               timebaseTick, NULL,
                                                0, 0);
      app_assert(sc == SL_STATUS_OK, "[E: 0x%04x] Failed to restart timer", sc);

      // Init complete
      initSequenceStage = UINT32_MAX;
    }
  }

  uint32_t uptime = sl_sleeptimer_get_time();

  for (size_t i = 0; i < SENSOR_ID_COUNT; i++){
    if (sensorStates[i].enabled){
      uint32_t timeSinceLastMeasurement = uptime - sensorStates[i].lastMeasurementStarted;
      if (timeSinceLastMeasurement >= sensorStates[i].measurementInterval){
        // Start measurement for this sensor
        measureFunctions[i]();
        sensorStates[i].lastMeasurementStarted = uptime;
      }
    }
  }
  
  
  // Measure and send full packet regardless of differential state
  if ( uptime - lastPacketSentTime > heartbeatTimebaseDelta ){
    dbg_printf("FORCE SEND!\r\n");
    // Just register a dummy measurement to force a packet send
    app_registerMeasurement(TemperatureMeasurement, temperature, true);
  }
}


void app_enableSensor(SensorId id, uint32_t interval)
{
  if (id >= SENSOR_ID_COUNT){
    dbg_printf("Invalid sensor ID: %d\r\n", id);
    return;
  }
  sensorStates[id].enabled = true;
  sensorStates[id].measurementInterval = interval;
  sensorStates[id].lastMeasurementStarted = 0;

  if (interval * 1000 < timeBase){
    timeBase = interval * 1000;
  }
}





void app_registerMeasurement(MeasurementType type, uint32_t value, bool send)
{
  dbg_printf("Register measurement: Type: %d, Value: %lu (Send: %d)\r\n", type, value, send);
  if ( type == TemperatureMeasurement ) {
    // Already converted to fit in int16_t
    temperature = value;
  }
  else if ( type == RelHumidityMeasurement ) {
    // Already converted to fit in uint8_t
    relativeHumidity = value;
  }
  else {
    if ( measurementCount < MAX_MEASUREMENT_COUNT ){
      measurements[measurementCount] = (Measurement){type, value};
      measurementCount++;
    }
    else {
      dbg_printf("ERROR: %d > MAX_MEASUREMENT_COUNT (%d)\r\n", measurementCount+1, MAX_MEASUREMENT_COUNT);
    }
  }

  if ( send ){
    app_sendPkt = true;
  }
}

static void timebaseTick(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle;
  (void)data;
  app_tick = true;
}

static void sendPacket(uint8_t numPackets){
  sl_status_t sc;

  // Set custom advertising data.
  sc = sl_bt_advertiser_set_data(advertising_set_handle,
                                 sl_bt_advertiser_advertising_data_packet,
                                 sizeof(packet),
                                 (uint8_t *)(&packet)
                                 );
  app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to set advertiser data\r\n", sc);

  // Set advertising parameters. 20-30 ms advertisement interval.
  sc = sl_bt_advertiser_set_timing(
    advertising_set_handle,
    0x20,     // min. adv. interval (milliseconds * 1.6)
    0x30,     // max. adv. interval (milliseconds * 1.6)
    0,        // adv. duration
    numPackets); // max. num. adv. events
  app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to set advertising timing\r\n", sc);

  // Start advertising in user mode and disable connections.
  sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_user_data,
        advertiser_non_connectable);
  app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to start advertising\r\n", sc);

}

void setPayload(void){

  uint32_t uptime = sl_sleeptimer_get_time();
  if (uptime > 0x7FFF) {
    // After 32700 s (~9 hours) start counting hours instead
    uptime /= 3600;
    // Set high bit to indicate that this is encoded hours
    uptime = 0x8000 | (uptime & 0x7FFF);
  }
  // Convert 3300 mV -> 165 for uint8_5
  float voltage = batt_measureBatteryVoltage() / 20.0f + 0.5f;
  if (voltage > 254) {
    voltage = 254;
  }
  
  payload.temperature = htons(temperature);
  payload.relativeHumidity = relativeHumidity;
  payload.voltage = (uint8_t)voltage;
  payload.uptime = htons(uptime);
  payload.swVersion = swVersion;
  payload.magic = htonl(0xAA5500FF);

  if (measurementCount > 0){
    // Rethink this.. FILO is only good if (only) the last measurment needs to be sent quickly
    payload.extraType = measurements[measurementCount].type;
    payload.extraValue = measurements[measurementCount].value;
    measurementCount--;
  }

  packet.nonce = htonl(packetCnt);

  dbg_printf("\r\nNew Packet\r\n");
  dbg_printf("Temperature: %d (%.2f C)\r\n", temperature, temperature / 100.0f);
  dbg_printf("Humidity: %d (%.2f %%)\r\n", relativeHumidity, relativeHumidity / 2.0f);
  dbg_printf("Voltage: %d (%.2f V)\r\n", (uint8_t)voltage, (voltage - 0.5f) / 50.0f);
  dbg_printf("Uptime: %ld\r\n", uptime);
  dbg_printf("Packet counter: %ld\r\n", packetCnt);

  packetCnt++;

  if (app_encryption){
    encryptPayload();
  }
  else {
    memcpy(&packet.payload, &payload, sizeof(payload));
  }
}

static void encryptPayload(void){

  dbg_printf("Encryption key: 0x");
  dbg_printArray(app_aesKey, 16);

  dbg_printf("Plaintext: 0x");
  dbg_printArray((uint8_t*)&payload, sizeof(payload));

  // Prepare counter value
  memcpy(&ctr[12], &packet.nonce, sizeof(packet.nonce));
  
  dbg_printf("Counter: 0x");
  dbg_printArray(ctr, 16);

  mbedtls_aes_context ctx;
  size_t ncOffset = 0;
  int stat;
  static uint8_t encPkt[sizeof(payload)];
  static uint8_t streamBlock[16];
  memset(streamBlock, 0, sizeof(streamBlock));

  mbedtls_aes_init(&ctx);
  stat = mbedtls_aes_setkey_enc(&ctx, app_aesKey, 128);
  app_assert(stat == SL_STATUS_OK, "AES setkey failed");

  stat = mbedtls_aes_crypt_ctr(&ctx, sizeof(payload), &ncOffset, ctr, streamBlock, (uint8_t*)&payload, encPkt);
  app_assert(stat == SL_STATUS_OK, "AES Encrypt");

  memcpy(&packet.payload, encPkt, sizeof(payload));

  dbg_printf("Ciphertext: 0x");
  dbg_printArray(encPkt, sizeof(encPkt));
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to get Bluetooth address\r\n",
                    (int)sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to write attribute\r\n",
                    (int)sc);


      int16_t set_max_power, set_min_power;
      // Set Transmit Power.
      sc = sl_bt_system_set_tx_power(app_txPower, app_txPower, &set_min_power, &set_max_power);
      app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to set TX power for Bluetooth\r\n",
                    (int)sc);

      int16_t support_max_power, support_min_power, rf_path_gain;
      sc = sl_bt_system_get_tx_power_setting(&support_min_power, &support_max_power, &set_min_power, &set_max_power, &rf_path_gain);
      app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to get max TX power for Bluetooth\r\n",
                    (int)sc);

      dbg_printf("Tx Power min-max:\r\n");
      dbg_printf("Supported: %f-%f dBm\r\n", support_min_power * 0.1f, support_max_power * 0.1f);
      dbg_printf("Set      : %f-%f dBm\r\n", set_min_power * 0.1f, set_max_power * 0.1f);


      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to create advertising set\r\n",
                    (int)sc);

      sc = sl_sleeptimer_start_periodic_timer_ms(&timebaseTimer, timeBase,
                                                 timebaseTick, (void *)NULL,
                                                 0, 0);

      app_assert(sc == SL_STATUS_OK, "[E: 0x%04x] Failed to start timer", (int)sc);

#if DEBUG_OUT
      // Faster startup while debugging
      sl_sleeptimer_restart_periodic_timer_ms(&timebaseTimer, 1000,
                                              timebaseTick, NULL,
                                              0, 0);
#endif
      break;

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

void initNonce(void){
  uint32_t rebootCnt = 0;
  Ecode_t ecode;
  
  // Read/initialize reset counter object
  ecode = nvm3_readCounter(nvm3_defaultHandle, RESET_COUNTER_KEY, &rebootCnt);
  if (ecode != ECODE_NVM3_OK){
    dbg_printf("Reading reset counter from NVM3 failed: %08x\r\n", (int)ecode);
    // Initialize NVM3 token from legacy MFG token
    rebootCnt = TOKEN_TO_UINT32(RESET_COUNT_ADDR);
    nvm3_writeCounter(nvm3_defaultHandle, RESET_COUNTER_KEY, rebootCnt);
  }

  ecode = nvm3_incrementCounter(nvm3_defaultHandle, RESET_COUNTER_KEY, &rebootCnt);
  if (rebootCnt > 1024){
    rebootCnt = 0;
    nvm3_writeCounter(nvm3_defaultHandle, RESET_COUNTER_KEY, rebootCnt);
  }

  dbg_printf("\r\n\r\nReboot count: %d\r\n", (int)rebootCnt);
  packetCnt = (rebootCnt << 22);

  uint64_t uniqueDeviceNumber = SYSTEM_GetUnique();
  uint8_t mac[6];
  mac[0] = (uint8_t)(uniqueDeviceNumber>>56);
  mac[1] = (uint8_t)(uniqueDeviceNumber>>48);
  mac[2] = (uint8_t)(uniqueDeviceNumber>>40);
  mac[3] = (uint8_t)(uniqueDeviceNumber>>16);
  mac[4] = (uint8_t)(uniqueDeviceNumber>>8);
  mac[5] = (uint8_t) uniqueDeviceNumber;

  uint8_t sha[32];

  int stat = mbedtls_sha256(mac, 6, sha, 0);
  app_assert(stat == 0, "SHA256 failed");

  // Use first 12 bytes of Sha for counter
  uint8_t i = 0;
  for (i = 0; i < 12; i++){
    ctr[i] = sha[i];
  }

  dbg_printf("MAC: ");
  dbg_printArray(mac, 6);
}

bool initTokens(void){

  bool success = true;

  if (!getTokenU32(TX_POWER_ADDR, (uint32_t*)&app_txPower)){
    success = false;
    app_txPower = 0;
  }

  if (!getTokenU8(PKT_COUNT_ADDR, &app_pktCount)){
    success = false;
    app_pktCount = 2;
  }

  if (!getTokenU8(PKT_COUNT_EVENT_ADDR, &app_pktCountEvent)){
    success = false;
    app_pktCountEvent = 4;
  }

  return success;
}

static void initPacket(void)
{
  memset(&packet, 0, sizeof(packet));
    /* See Bluetooth 4.0 Core Specification , Volume 3, Appendix C, 18.1 for more details on flags. */
  packet.flagsLen = 2;
  packet.flagsType = 0x01;
  /* Flags: LE General Discoverable Mode, BR/EDR is disabled. */
  packet.flags = 0x04 | 0x02;
  packet.mandataLen = 23;
  packet.mandataType = 0xFF;
  memcpy(&packet.packetId, &app_packetId, sizeof(packet.packetId));
}

static void dbg_printArray(const uint8_t* binbuf, unsigned int binbuflen)
{
#if DEBUG_OUT
  uint8_t i = 0;
  for (i = 0; i < binbuflen; i++)
  {
    printf("%02X", binbuf[i]);
  }
  printf("\r\n");
#else
(void)binbuf;
(void)binbuflen;
#endif
}


static uint32_t htonl(uint32_t h)
{
  uint8_t *data = (uint8_t*)&h;

  return ((uint32_t) data[3] << 0)
        | ((uint32_t) data[2] << 8)
        | ((uint32_t) data[1] << 16)
        | ((uint32_t) data[0] << 24);
}

static uint16_t htons(uint16_t h)
{
  uint8_t *data = (uint8_t*)&h;
  return ((uint16_t) data[1] << 0) | ((uint16_t) data[0] << 8);
}
