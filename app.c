#include "app.h"
#include "app_config.h"

#include "tokens.h"
#include "tokenutil.h"

#include "em_common.h"
#include "em_rtcc.h"
#include "em_rmu.h"
#include "em_system.h"
#include "em_msc.h"
#include "em_cmu.h"

#include "sl_bluetooth.h"
#include "sl_status.h"
#include "sl_app_assert.h"
#include "sl_sleeptimer.h"
#include "sl_i2cspm_instances.h"
#include "sl_si70xx.h"
#include "sl_board_control.h"

#include "nvm3.h"
#include "nvm3_default.h"

#include "gatt_db.h"
#include "mbedtls/aes.h"
#include "mbedtls/sha256.h"


#define UINT16_TO_BYTES(n)            ((uint8_t) (n)), ((uint8_t)((n) >> 8))
#define UINT16_TO_BYTE0(n)            ((uint8_t) (n))
#define UINT16_TO_BYTE1(n)            ((uint8_t) ((n) >> 8))

#define UINT32_TO_BYTE0(n)            ((uint8_t) (n))
#define UINT32_TO_BYTE1(n)            ((uint8_t) ((n) >> 8))
#define UINT32_TO_BYTE2(n)            ((uint8_t) ((n) >> 16))
#define UINT32_TO_BYTE3(n)            ((uint8_t) ((n) >> 24))


#ifdef RMU_RSTCAUSE_EXTRST
#define RSTCAUSE_PIN RMU_RSTCAUSE_EXTRST
#else
#define RSTCAUSE_PIN EMU_RSTCAUSE_PIN
#endif

/***********************************************************************************************//**
 * Application settings
 **************************************************************************************************/
#define TIMER_CLK_FREQ ((uint32_t)32768)

#define DEBUG_OUT           1
#define ENCRYPTION          1

const unsigned TIMEBASE = 30000;

/* Force send a packet this number of minutes since last packet */
const unsigned heartbeatTimebaseDelta = 30;

/* Send packet if temperature has changed more than this (celsius) */
static float tempDiffThreshold;
/* Send packet if humidity has changed more than this (percent) */
static float humDiffThreshold;

// NVM3 key
const uint32_t RESET_COUNTER_KEY = 0x01;


/***********************************************************************************************//**
 * Global variables
 **************************************************************************************************/

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

static uint32_t timeBase;
static uint32_t timeBaseCnt             = 0;
static uint32_t lastPacketTimeBaseCnt   = 0;
static uint32_t packetCnt         = 0;

static uint32_t timRtc          = 0;
static uint64_t timNum64        = 0;

static uint32_t relativeHumidity = 0;
static int32_t  temperature = 0x8000;  // = -128.0
static int32_t  lastTemperature = 0x8000;
static int32_t  lastRelativeHumidity = 0x8000;
static uint16_t batteryVoltage = 0;
static uint32_t uptime = 0;

const uint16_t swVersion = 0x20;

static sl_sleeptimer_timer_handle_t measureTimer;
static sl_sleeptimer_timer_handle_t delayTimer;

const uint8_t *key = TOKEN_TO_ARRAY(AES_KEY_ADDR);

static uint8_t ctr[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

/***********************************************************************************************//**
 * Local functions
 **************************************************************************************************/

static void advertise(sl_sleeptimer_timer_handle_t *handle, void *data);
static void startMeasurement(sl_sleeptimer_timer_handle_t *handle, void *data);
static void setPayload(void);
static void initNonce(void);
static bool initTokens(void);

#if DEBUG_OUT
void printArray(const uint8_t* binbuf, unsigned int binbuflen);
#endif

/* Format:
 *  packetId = AA55
 *  payload =
 *        2-bytes temperature
 *        2-bytes relativeHumidity
 *        2-bytes voltage
 *        4-bytes uptime
 *        2-bytes spare
 *        4-bytes magic field AA55FF00 (must decrypt correctly = MAC)
 */


static struct {
    uint8_t flagsLen;     /* Length of the Flags field. */
    uint8_t flagsType;    /* Type of the Flags field. */
    uint8_t flags;        /* Flags field. */
    uint8_t mandataLen;   /* Length of the Manufacturer Data field. */
    uint8_t mandataType;  /* Type of the Manufacturer Data field. */

    uint8_t packetId[2];
    uint8_t payload[16];  /* Format: */
    uint8_t nonce[4];
  }
  packet
    = {
    /* Flag bits - See Bluetooth 4.0 Core Specification , Volume 3, Appendix C, 18.1 for more details on flags. */
    2,  /* length  */
    0x01, /* type */
    0x04 | 0x02, /* Flags: LE General Discoverable Mode, BR/EDR is disabled. */

    /* Manufacturer specific data */
    23,  /* length of field*/
    0xFF, /* type of field */

  #if ENCRYPTION
    {0xAA,0x05},            /* PacketId AA05 for encrypted packet*/
  #else
  {0xAA,0x55},            /* PacketId AA55 for plaintext packet*/
  #endif
  {0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00}    /* Nonce init */

  };



/***********************************************************************************************//**
 * Main program and loop
 **************************************************************************************************/

/***********************************************************************************************//**
* Main program flow:
*  Sleep 30 s (measureTimer)
*  Power on sensor, sleep 80 ms (delayTimer)
*  Start measurement, sleep 50 ms (delayTimer)
*  Read measurement from sensor and send if required
**************************************************************************************************/
void enableSensor(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle;
  (void)data;
  sl_status_t sc;
  sl_board_enable_sensor(SL_BOARD_SENSOR_RHT);
#if DEBUG_OUT
  printf("Enable\r\n");
#endif

  sc = sl_sleeptimer_start_timer_ms(&delayTimer, 80, startMeasurement, NULL, 0, 0);
  sl_app_assert(sc == SL_STATUS_OK, "[E: 0x%04x] Failed to start timer", sc);
}

void startMeasurement(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  (void)handle;
  (void)data;

#if DEBUG_OUT
  printf("Measure\r\n");
#endif
  sl_status_t sc;
  sc = sl_si70xx_start_no_hold_measure_rh_and_temp(sl_i2cspm_sensor, SI7006_ADDR);
  if ( sc != SL_STATUS_OK ){
    // Something went wrong, wait for the next attempt
    #if DEBUG_OUT
    printf("SI7021 measurement failed, return value: 0x%x\r\n", sc);
    #endif
    return;
  }

  sc = sl_sleeptimer_start_timer_ms(&delayTimer, 50, advertise, NULL, 0, 0);
  sl_app_assert(sc == SL_STATUS_OK, "[E: 0x%04x] Failed to start timer", sc);
}

void advertise(sl_sleeptimer_timer_handle_t *handle, void *data)
{
  /* This function sets up a custom advertisement package if temperature changed, or if the ticker mandates full packet. */
  
  (void)handle;
  (void)data;
  sl_status_t sc;

#if DEBUG_OUT
  printf("Advertise\r\n");
#endif
  
  // Start with a temperature measurement
  bool sending = false;
  sc = sl_si70xx_read_rh_and_temp(sl_i2cspm_sensor, SI7006_ADDR, &relativeHumidity, &temperature);
  sl_board_disable_sensor(SL_BOARD_SENSOR_RHT);
  if ( sc != SL_STATUS_OK ){
    // Something went wrong, wait for the next attempt
    #if DEBUG_OUT
    printf("SI7021 readout failed, return value: 0x%x\r\n", sc);
    #endif
    return;
  }


  // Measure and send full packet regardless of differential state
  if ( timeBaseCnt - lastPacketTimeBaseCnt > heartbeatTimebaseDelta ){
    sending = true;
  }

  // If a value has changed significantly, send anyways
  if ( abs(temperature - lastTemperature) > 100 * tempDiffThreshold || abs(relativeHumidity - lastRelativeHumidity) > 100 * humDiffThreshold){
    sending = true;
  }

  if ( sending ){
    lastTemperature = temperature;
    lastRelativeHumidity = relativeHumidity;
    lastPacketTimeBaseCnt = timeBaseCnt;

    temperature = (temperature + 5) / 10;
    relativeHumidity = (relativeHumidity + 5) / 10;
    if ( relativeHumidity > 10000 ){
      relativeHumidity = 10000;
    }
    batteryVoltage = (app_measureBatteryVoltage() + 5) / 10;

    uint32_t timRtcNow = RTCC_CounterGet();
    timNum64 += (timRtcNow - timRtc);
    timRtc = timRtcNow;
    uptime = (uint32_t)(timNum64 / (TIMER_CLK_FREQ ));

    setPayload();
    packetCnt++;

    // Set custom advertising data.
    sc = sl_bt_advertiser_set_data(advertising_set_handle,
                                   0,
                                   sizeof(packet),
                                   (uint8_t *)(&packet));
    sl_app_assert(sc == SL_STATUS_OK,
                  "[E: 0x%04x] Failed to set advertiser data\n", sc);

    // Set advertising parameters. 100ms advertisement interval.
    sc = sl_bt_advertiser_set_timing(
      advertising_set_handle,
      100,     // min. adv. interval (milliseconds * 1.6)
      200,     // max. adv. interval (milliseconds * 1.6)
      0,       // adv. duration
      1);      // max. num. adv. events
    sl_app_assert(sc == SL_STATUS_OK,
                  "[E: 0x%04x] Failed to set advertising timing\n", sc);

    // Start advertising in user mode and disable connections.
    sc = sl_bt_advertiser_start(
      advertising_set_handle,
      advertiser_user_data,
      advertiser_non_connectable);
    sl_app_assert(sc == SL_STATUS_OK,
                  "[E: 0x%04x] Failed to start advertising\n", sc);

  }
}

void setPayload(void){
    packet.payload[0] = UINT16_TO_BYTE1(temperature);
    packet.payload[1] = UINT16_TO_BYTE0(temperature);
    packet.payload[2] = UINT16_TO_BYTE1(relativeHumidity);
    packet.payload[3] = UINT16_TO_BYTE0(relativeHumidity);
    packet.payload[4] = UINT16_TO_BYTE1(batteryVoltage);
    packet.payload[5] = UINT16_TO_BYTE0(batteryVoltage);
    packet.payload[6] = UINT32_TO_BYTE3(uptime);
    packet.payload[7] = UINT32_TO_BYTE2(uptime);
    packet.payload[8] = UINT32_TO_BYTE1(uptime);
    packet.payload[9] = UINT32_TO_BYTE0(uptime);
    packet.payload[10] = 0x00;
    packet.payload[11] = swVersion;
    packet.payload[12] = 0xAA;
    packet.payload[13] = 0x55;
    packet.payload[14] = 0x00;
    packet.payload[15] = 0xFF;
    packet.nonce[0] = UINT32_TO_BYTE3(packetCnt);
    packet.nonce[1] = UINT32_TO_BYTE2(packetCnt);
    packet.nonce[2] = UINT32_TO_BYTE1(packetCnt);
    packet.nonce[3] = UINT32_TO_BYTE0(packetCnt);

#if DEBUG_OUT
    printf("\r\nNew Packet\r\n");
    printf("Temperature: %d\r\n", temperature);
    printf("Humidity: %d\r\n", relativeHumidity);
    printf("Voltage: %d\r\n", batteryVoltage);
    printf("Uptime: %d\r\n", uptime);
    printf("Packet counter: %d\r\n", packetCnt);
  #endif

#if ENCRYPTION
#if DEBUG_OUT
    printf("Encryption key: 0x");
    printArray(key, 16);

  printf("Plaintext: 0x");
  printArray(packet.payload, 16);

  printf("Counter: 0x");
  printArray(ctr, 16);
#endif

  // Prepare counter value
  ctr[12] = packet.nonce[0];
  ctr[13] = packet.nonce[1];
  ctr[14] = packet.nonce[2];
  ctr[15] = packet.nonce[3];
  mbedtls_aes_context ctx;
  size_t ncOffset = 0;
  int stat;
  static uint8_t encPkt[sizeof(packet.payload)];
  static uint8_t streamBlock[16];
  memset(streamBlock, 0, sizeof(streamBlock));

  mbedtls_aes_init(&ctx);
  stat = mbedtls_aes_setkey_enc(&ctx, key, 128);
  sl_app_assert(stat == SL_STATUS_OK, "AES setkey failed");

  stat = mbedtls_aes_crypt_ctr(&ctx, sizeof(packet.payload), &ncOffset, ctr, streamBlock, packet.payload, encPkt);
  sl_app_assert(stat == SL_STATUS_OK, "AES Encrypt");

  memcpy(packet.payload, encPkt, sizeof(packet.payload));
#if DEBUG_OUT
  printf("Ciphertext: 0x");
  printArray(packet.payload, 16);
#endif
#endif
}




/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  sl_sleeptimer_init();
  app_board_init();
  if (!initTokens()){
    printf("Tokens are not configured correctly, using defaults!\r\n");
  }
  sl_board_enable_sensor(SL_BOARD_SENSOR_RHT);
  sl_si70xx_init(sl_i2cspm_sensor, SI7021_ADDR);
  sl_board_disable_sensor(SL_BOARD_SENSOR_RHT);
  initNonce();
  app_adcInit();
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
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
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to get Bluetooth address\n",
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
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to write attribute\n",
                    (int)sc);


      int16_t ret_power;
      // Set Transmit Power.
      sc = sl_bt_system_set_max_tx_power(APP_TX_POWER, &ret_power);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to set max TX power for Bluetooth\n",
                    (int)sc);

      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to create advertising set\n",
                    (int)sc);

      sc = sl_sleeptimer_start_periodic_timer_ms(&measureTimer, timeBase,
                                                 enableSensor, (void *)NULL,
                                                 0, 0);

      sl_app_assert(sc == SL_STATUS_OK, "[E: 0x%04x] Failed to start timer", sc);

#if DEBUG_OUT
      // Manual immediate measurement
      enableSensor(NULL, NULL);
#endif
      break;


    case sl_bt_evt_advertiser_timeout_id: // Finished with beacons, sleep
      break;

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

void initNonce(void){
  uint32_t rebootCnt = 0;
  Ecode_t ecode = nvm3_readCounter(nvm3_defaultHandle, RESET_COUNTER_KEY, &rebootCnt);
  if (ecode != ECODE_NVM3_OK){
    printf("Reading reset counter from NVM3 failed: %08x\r\n", ecode);
    rebootCnt = TOKEN_TO_UINT32(RESET_COUNT_ADDR);
    if (rebootCnt > 1024){
      rebootCnt = 0;
    }
    nvm3_writeCounter(nvm3_defaultHandle, RESET_COUNTER_KEY, rebootCnt);
  }
  uint32_t resetCause = RMU_ResetCauseGet();
  RMU_ResetCauseClear();
#if DEBUG_OUT
  printf("\r\nReset cause: %08x\r\n", resetCause);
#endif
  if (resetCause == RSTCAUSE_PIN ){
    // Only increment reboot count in case of pin reset (not brownout)
    ecode = nvm3_incrementCounter(nvm3_defaultHandle, RESET_COUNTER_KEY, &rebootCnt);
    if (rebootCnt > 1024){
      rebootCnt = 0;
      nvm3_writeCounter(nvm3_defaultHandle, RESET_COUNTER_KEY, rebootCnt);
    }
#ifdef _SILICON_LABS_32B_SERIES_2_CONFIG_2
    CMU_ClockEnable(cmuClock_MSC, false);
#endif
  }
#if DEBUG_OUT
  printf("\r\n\r\nReboot count: %d\r\n", rebootCnt);
#endif
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

  int stat = mbedtls_sha256_ret(mac, 6, sha, 0);
  sl_app_assert(stat == 0,
                "SHA256 failed");

  // Use first 12 bytes of Sha for counter
  uint8_t i = 0;
  for (i = 0; i < 12; i++){
    ctr[i] = sha[i];
  }

#if DEBUG_OUT
  printf("MAC: ");
  printArray(mac, 6);
#endif
}

bool initTokens(void){
  bool success = true;
  timeBase = TOKEN_TO_UINT32(SENSING_TIME_BASE_ADDR);
  if (timeBase == 0xFFFFFFFF){
      success = false;
      timeBase = 30000;
  }

  tempDiffThreshold = (float)(0.001f * (TOKEN_TO_UINT32(TEMP_DIFF_THRESHOLD_ADDR)));
  humDiffThreshold = (float)(0.01f * (TOKEN_TO_UINT32(RH_DIFF_THRESHOLD_ADDR)));

  return success;
}


#if DEBUG_OUT
void printArray(const uint8_t* binbuf, unsigned int binbuflen)
{
  uint8_t i = 0;
  for (i = 0; i < binbuflen; i++)
  {
    printf("%02X", binbuf[i]);
  }
  printf("\r\n");
}
#endif
