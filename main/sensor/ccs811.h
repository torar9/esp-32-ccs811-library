#ifndef CCS811_H
#define CCS811_H

#include <inttypes.h>
#include "driver/i2c_master.h"

#define CCS811_ADDRESS (0x5A)
#define CCS811_ADDRESS_READ ((CCS811_ADDRESS << 1) + 1U)
#define CCS811_ADDRESS_WRITE (CCS811_ADDRESS << 1)

typedef struct {
    uint16_t eCO2;
    uint16_t TVOC;
    uint8_t status;
    uint8_t error_id;
    uint16_t raw_data;
} CCS811_MEASUREMENT;

typedef union
{
    struct 
    {
        uint8_t error: 1;
        uint8_t reserved: 2;
        uint8_t data_ready: 1;
        uint8_t app_valid: 1;
        uint8_t reserved2: 2;
        uint8_t fw_mode: 1;
    };
    uint8_t flags;
} CCS811_STATUS;

typedef union
{
    struct 
    {
        uint8_t reserved: 2;
        uint8_t thresh: 1;
        uint8_t interrupt: 1;
        uint8_t drive_mode: 3;
        uint8_t reserved2: 1;
    };
    uint8_t flags;
} CCS811_MEAS_MODE;

typedef enum
{
    WRITE_REG_INVALID = 0,
    READ_REG_INVALID = 1,
    MEASMODE_INVALID = 2,
    MAX_RESISTANCE = 3,
    HEATER_FAULT = 4,
    HEATER_SUPPLY = 5,
    RESERVE_1 = 6,
    RESERVE_2 = 7
}CCS811_ERR_TYPE;

/**
  * @brief  Initialize CCS811
  * @param  i2c - i2c handler
  * @retval uint8_t - return result 0 means OK 1 means fail
  */
uint8_t ccs811_init(i2c_master_dev_handle_t* i2c);

/**
  * @brief  Starts reading of measurements
  * @param  data - return parameter of measured data
  * @retval uint8_t - return result 0 means OK 1 means fail
  */
uint8_t ccs811_read_measurement(CCS811_MEASUREMENT* data);

/**
  * @brief  Gathers status registers from sensor
  * @param  status - return parameter of status gathered from sensor
  * @retval uint8_t - return result 0 means OK 1 means fail
  */
uint8_t ccs811_get_status(CCS811_STATUS* status);

/**
  * @brief  Gathers status registers from sensor
  * @param  mode - return parameter of mode gathered from sensor
  * @retval uint8_t - return result 0 means OK 1 means fail
  */
uint8_t ccs811_get_mode(CCS811_MEAS_MODE* mode);

/**
  * @brief  Set measuring mode - see datasheet for more details
  * @param  data - measurement mode which needs to be set
  * @retval uint8_t - return result 0 means OK 1 means fail
  */
uint8_t ccs811_set_mode(const CCS811_MEAS_MODE mode);

/**
  * @brief  Get HW version
  * @param  data - measurement mode which needs to be set
  * @retval uint8_t - return result 0 means OK 1 means fail
  */
uint8_t ccs811_get_hw_ver(uint8_t* hw_ver);

/**
  * @brief  Get HW id
  * @param  data - measurement mode which needs to be set
  * @retval uint8_t - return result 0 means OK 1 means fail
  */
uint8_t ccs811_get_hw_id(uint8_t* hw_id);

/**
  * @brief  Get error status - see datasheet for more details
  * @param  data - measurement mode which needs to be set
  * @retval uint8_t - return result 0 means OK 1 means fail
  */
uint8_t ccs811_get_err_stat(CCS811_ERR_TYPE* err_stat);

/**
  * @brief  Start application - exit bootloader
  * @param  data - measurement mode which needs to be set
  * @retval uint8_t - return result 0 means OK 1 means fail
  */
uint8_t ccs811_app_start();

/**
  * @brief  Reset device and return to bootloader
  * @retval uint8_t - return result 0 means OK 1 means fail
  */
uint8_t ccs811_sw_reset();

/**
  * @brief  Get bootloader version
  * @param  boot_ver - return parameter containing bootloader version
  * @retval uint8_t - return result 0 means OK 1 means fail
  */
uint8_t ccs811_get_bootloader_version(uint16_t* boot_ver);

/**
  * @brief  Get app version
  * @param  app_ver - return parameter containing app version
  * @retval uint8_t - return result 0 means OK 1 means fail
  */
uint8_t ccs811_get_app_version(uint16_t* app_ver);

/**
  * @brief  Get raw ADC measurement data for resistance and current source used.
  * @param  raw_data - raw current and ADC data. 5:0 current, 9:0 ADC reading
  * @retval uint8_t - return result 0 means OK 1 means fail
  */
uint8_t ccs811_get_raw_data(uint16_t* raw_data);

/**
  * @brief  Write environment data such as humidity and temp in order to adjust sensor accuracy
  * @param  env_data - raw current and ADC data. 5:0 current, 9:0 ADC reading
  * @retval uint8_t - return result 0 means OK 1 means fail
  */
uint8_t ccs811_set_env_data(uint16_t* env_data);

#endif