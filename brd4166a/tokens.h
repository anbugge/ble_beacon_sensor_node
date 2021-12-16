#ifndef TOKENS_H_INCLUDED__
#define TOKENS_H_INCLUDED__

#define TOKEN_BOOTLOADER_BASE_ADDRESS  0x0FE10000
#define TOKEN_DEVINFO_BASE_ADDRESS     0x0FE08000
#define TOKEN_LOCKBITS_BASE_ADDRESS    0x0FE04000
#define TOKEN_MAINFLASH_BASE_ADDRESS   0x00000000
#define TOKEN_USERDATA_BASE_ADDRESS    0x0FE00000

#define TOKEN_COUNT 12

#define RESET_COUNT_OFFSET            0x0000
#define RESET_COUNT_ADDR              (TOKEN_USERDATA_BASE_ADDRESS + RESET_COUNT_OFFSET)
#define RESET_COUNT_SIZE              4

#define FEATURE_RH_TEMP_OFFSET        0x0300
#define FEATURE_RH_TEMP_ADDR          (TOKEN_USERDATA_BASE_ADDRESS + FEATURE_RH_TEMP_OFFSET)
#define FEATURE_RH_TEMP_SIZE          1

#define FEATURE_HALL_OFFSET           0x0301
#define FEATURE_HALL_ADDR             (TOKEN_USERDATA_BASE_ADDRESS + FEATURE_HALL_OFFSET)
#define FEATURE_HALL_SIZE             1

#define FEATURE_ALS_OFFSET            0x0302
#define FEATURE_ALS_ADDR              (TOKEN_USERDATA_BASE_ADDRESS + FEATURE_ALS_OFFSET)
#define FEATURE_ALS_SIZE              1

#define FEATURE_BUTTONS_OFFSET        0x0303
#define FEATURE_BUTTONS_ADDR          (TOKEN_USERDATA_BASE_ADDRESS + FEATURE_BUTTONS_OFFSET)
#define FEATURE_BUTTONS_SIZE          1

#define RH_SENSING_TIME_BASE_OFFSET   0x0310
#define RH_SENSING_TIME_BASE_ADDR     (TOKEN_USERDATA_BASE_ADDRESS + RH_SENSING_TIME_BASE_OFFSET)
#define RH_SENSING_TIME_BASE_SIZE     4

#define ALS_SENSING_TIME_BASE_OFFSET  0x0314
#define ALS_SENSING_TIME_BASE_ADDR    (TOKEN_USERDATA_BASE_ADDRESS + ALS_SENSING_TIME_BASE_OFFSET)
#define ALS_SENSING_TIME_BASE_SIZE    4

#define TEMP_DIFF_THRESHOLD_OFFSET    0x0320
#define TEMP_DIFF_THRESHOLD_ADDR      (TOKEN_USERDATA_BASE_ADDRESS + TEMP_DIFF_THRESHOLD_OFFSET)
#define TEMP_DIFF_THRESHOLD_SIZE      4

#define RH_DIFF_THRESHOLD_OFFSET      0x0324
#define RH_DIFF_THRESHOLD_ADDR        (TOKEN_USERDATA_BASE_ADDRESS + RH_DIFF_THRESHOLD_OFFSET)
#define RH_DIFF_THRESHOLD_SIZE        4

#define TX_POWER_OFFSET               0x0330
#define TX_POWER_ADDR                 (TOKEN_USERDATA_BASE_ADDRESS + TX_POWER_OFFSET)
#define TX_POWER_SIZE                 2

#define HALL_THRESHOLD_OFFSET         0x0328
#define HALL_THRESHOLD_ADDR           (TOKEN_USERDATA_BASE_ADDRESS + HALL_THRESHOLD_OFFSET)
#define HALL_THRESHOLD_SIZE           4

#define AES_KEY_OFFSET                0x0460
#define AES_KEY_ADDR                  (TOKEN_LOCKBITS_BASE_ADDRESS + AES_KEY_OFFSET)
#define AES_KEY_SIZE                  16


#endif
