#include "ccs811.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CCS811_TIMEOUT (255)
#define CCS811_STATUS_ADDR (0x00)
#define CCS811_MODE_ADDR (0x01)
#define CCS811_RESULT_ADDR (0x02)
#define CCS811_RAW_DATA_ADDR (0x03)
#define CCS811_ENV_DATA_ADDR (0x05)
#define CCS811_HW_VER_ADDR (0x20)
#define CCS811_HW_ID_ADDR (0x21)
#define CCS811_ERR_STAT_ADDR (0xE0)
#define CCS811_APP_START_ADDR (0xF4)
#define CCS811_SW_RESET_ADDR (0xFF)
#define CCS811_FW_BOOT_VER_ADDR (0x23)
#define CCS811_FW_APP_VER_ADDR (0x23)
#define CCS811_COM_DELAY (0)

static i2c_master_dev_handle_t* hi2c2 = NULL;

uint8_t ccs811_init(i2c_master_dev_handle_t* i2c)
{
    uint8_t result = 0U;

    if(i2c == NULL)
    {
        result = 1;
    }
    else
    {
        hi2c2 = i2c;
    }

    return result;
}

uint8_t ccs811_read_measurement(CCS811_MEASUREMENT* data)
{
    uint8_t result = 0U;
    uint8_t comStatus = 0U;
    uint8_t buffer[8] = {CCS811_RESULT_ADDR, 0, 0, 0, 0, 0, 0, 0};

    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_transmit(*hi2c2, buffer, 1, CCS811_TIMEOUT);
    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_receive(*hi2c2, buffer, 8, CCS811_TIMEOUT);

    if(0U != comStatus)
    {
        result = 1;
    }
    else
    {
        data->eCO2 = (uint16_t)buffer[0] << 8U;
        data->eCO2 |= (uint16_t)buffer[1];

        data->TVOC = (uint16_t)buffer[2] << 8U;
        data->TVOC |= (uint16_t)buffer[3];

        data->status = (uint16_t)buffer[4];

        data->error_id = (uint16_t)buffer[5];

        data->raw_data = (uint16_t)buffer[6] << 8U;
        data->raw_data |= (uint16_t)buffer[7];
    }

    return result;
}

uint8_t ccs811_get_status(CCS811_STATUS* status)
{
    uint8_t result = 0U;
    uint8_t buffer[1] = {CCS811_STATUS_ADDR};
    uint8_t comStatus = 0U;

    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_transmit(*hi2c2, buffer, 1, CCS811_TIMEOUT);
    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_receive(*hi2c2, buffer, 1, CCS811_TIMEOUT);

    if(0U != comStatus)
    {
        result = 1;
    }
    else
    {
        status->flags = buffer[0];
    }

    return result;
}

uint8_t ccs811_get_mode(CCS811_MEAS_MODE* mode)
{
    uint8_t result = 0U;
    uint8_t buffer[1] = {CCS811_MODE_ADDR};
    uint8_t comStatus = 0U;

    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_transmit(*hi2c2, buffer, 1, CCS811_TIMEOUT);
    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_receive(*hi2c2, buffer, 1, CCS811_TIMEOUT);

    if(0U != comStatus)
    {
        result = 1;
    }
    else
    {
        mode->flags = buffer[0];
    }

    return result;
}

uint8_t ccs811_set_mode(const CCS811_MEAS_MODE mode)
{
    uint8_t result = 0U;
    uint8_t buffer[2] = {CCS811_MODE_ADDR, mode.flags};
    uint8_t comStatus = 0U;

    vTaskDelay(5);
    comStatus |= i2c_master_transmit(*hi2c2, buffer, 2, CCS811_TIMEOUT);

    if(0U != comStatus)
    {
        result = 1;
    }

    return result;
}

uint8_t ccs811_get_hw_ver(uint8_t* hw_ver)
{
    uint8_t result = 0U;
    uint8_t buffer[1] = {CCS811_HW_ID_ADDR};
    uint8_t comStatus = 0U;

    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_transmit(*hi2c2, buffer, 1, CCS811_TIMEOUT);
    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_receive(*hi2c2, buffer, 1, CCS811_TIMEOUT);

    if(0U != comStatus)
    {
        result = 1;
    }
    else
    {
        *hw_ver = buffer[0];
    }

    return result;
}

uint8_t ccs811_get_hw_id(uint8_t* hw_id)
{
    uint8_t result = 0U;
    uint8_t buffer[1] = {CCS811_HW_VER_ADDR};
    uint8_t comStatus = 0U;

    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_transmit(*hi2c2, buffer, 1, CCS811_TIMEOUT);
    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_receive(*hi2c2, buffer, 1, CCS811_TIMEOUT);

    if(0U != comStatus)
    {
        result = 1;
    }
    else
    {
        *hw_id = buffer[0];
    }

    return result;
}

uint8_t ccs811_get_err_stat(CCS811_ERR_TYPE* err_stat)
{
    uint8_t result = 0U;
    uint8_t buffer[1] = {CCS811_ERR_STAT_ADDR};
    uint8_t comStatus = 0U;

    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_transmit(*hi2c2, buffer, 1, CCS811_TIMEOUT);
    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_receive(*hi2c2, buffer, 1, CCS811_TIMEOUT);

    if(0U != comStatus)
    {
        result = 1;
    }
    else
    {
        *err_stat = buffer[0];
    }

    return result;
}

uint8_t ccs811_app_start()
{
    uint8_t result = 0U;
    uint8_t buffer[1] = {CCS811_APP_START_ADDR};
    uint8_t comStatus = 0U;

    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_transmit(*hi2c2, buffer, 1, CCS811_TIMEOUT);

    if(0U != comStatus)
    {
        result = 1;
    }

    return result;
}

/**
  * @brief  Reset device and return to bootloader
  * @retval uint8_t - return result 0 means OK 1 means fail
  */
uint8_t ccs811_sw_reset()
{
    uint8_t result = 0U;
    uint8_t buffer[5] = {CCS811_SW_RESET_ADDR, 0x11, 0xE5, 0x72, 0x8A};
    uint8_t comStatus = 0U;

    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_transmit(*hi2c2, buffer, 5, CCS811_TIMEOUT);

    if(0U != comStatus)
    {
        result = 1;
    }

    return result;
}

uint8_t ccs811_get_bootloader_version(uint16_t* boot_ver)
{
    uint8_t result = 0U;
    uint8_t buffer[2] = {CCS811_FW_BOOT_VER_ADDR, 0x00};
    uint8_t comStatus = 0U;

    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_transmit(*hi2c2, buffer, 1, CCS811_TIMEOUT);
    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_receive(*hi2c2, buffer, 2, CCS811_TIMEOUT);

    if(0U != comStatus)
    {
        result = 1;
    }
    else
    {
        *boot_ver = buffer[0]<< 8U;
        *boot_ver |= buffer[1];
    }

    return result;
}

uint8_t ccs811_get_app_version(uint16_t* app_ver)
{
    uint8_t result = 0U;
    uint8_t buffer[2] = {CCS811_FW_APP_VER_ADDR, 0x00};
    uint8_t comStatus = 0U;

    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_transmit(*hi2c2, buffer, 1, CCS811_TIMEOUT);
    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_receive(*hi2c2, buffer, 2, CCS811_TIMEOUT);

    if(0U != comStatus)
    {
        result = 1;
    }
    else
    {
        *app_ver = buffer[0]<< 8U;
        *app_ver |= buffer[1];
    }

    return result;
}

/**
  * @brief  Get raw ADC measurement data for resistance and current source used.
  * @param  raw_data - raw current and ADC data. 5:0 current, 9:0 ADC reading
  * @retval uint8_t - return result 0 means OK 1 means fail
  */
uint8_t ccs811_get_raw_data(uint16_t* raw_data)
{
    uint8_t result = 0U;
    uint8_t buffer[2] = {CCS811_RAW_DATA_ADDR, 0x00};
    uint8_t comStatus = 0U;

    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_transmit(*hi2c2, buffer, 1, CCS811_TIMEOUT);
    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_receive(*hi2c2, buffer, 2, CCS811_TIMEOUT);

    if(0U != comStatus)
    {
        result = 1;
    }
    else
    {
        *raw_data = buffer[0];
        *raw_data |= buffer[1] << 8U;
    }

    return result;
}

/**
  * @brief  Write environment data such as humidity and temp in order to adjust sensor accuracy
  * @param  env_data - env data, see datasheet for more details
  * @retval uint8_t - return result 0 means OK 1 means fail
  */
uint8_t ccs811_set_env_data(uint16_t* env_data)
{
    uint8_t result = 0U;
    uint8_t buffer[3] = {CCS811_ENV_DATA_ADDR, (uint8_t)(*env_data & 0x00FF), (uint8_t)((*env_data & 0xFF00) >> 8U)};
    uint8_t comStatus = 0U;

    vTaskDelay(CCS811_COM_DELAY);
    comStatus |= i2c_master_transmit(*hi2c2, buffer, 3, CCS811_TIMEOUT);

    if(0U != comStatus)
    {
        result = 1;
    }

    return result;
}