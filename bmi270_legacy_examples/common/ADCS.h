/**
 * Copyright (C) 2022 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    adcs.h
 * @brief   This file contains ADCS layer function prototypes, variable declarations and Macro definitions
 *
 */
/*!
 * @addtogroup adcs_api
 * @{*/

#ifndef ADCS_H_
#define ADCS_H_

#if !defined(ADCS_VERSION)
#define ADCS_VERSION  "v2.7.0"
#endif

/* C++ Guard macro - To prevent name mangling by C++ compiler */
#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************************/
/* Pico-specific defines */
/**********************************************************************************/
#define PIN_SDA 4
#define PIN_SCL 5
#define SUCCESS 0

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/
/*! Streaming states */
#define ADCS_STREAMING_START            1
#define ADCS_STREAMING_STOP             0

/*! ADCS USB buffer max size */
#define ADCS_DATA_BUF_SIZE              (1024)

/*! maximum no of sensor support */
#define ADCS_MIN_SENSOR_ID              1
#define ADCS_MAX_SENSOR_ID              2
#define ADCS_MAX_SENSOR_COUNT           (ADCS_MAX_SENSOR_ID + 1)

/*! adcs stream response buffer size */
#define ADCS_STREAM_RSP_BUF_SIZE        1048576

/*! adcs success code */
#define ADCS_SUCCESS                    0

/*! adcs error code - failure */
#define ADCS_E_FAILURE                  -1

/*! adcs error code - IO error */
#define ADCS_E_COMM_IO_ERROR            -2

/*! adcs error code - Init failure */
#define ADCS_E_COMM_INIT_FAILED         -3

/*! adcs error code - failure to open device */
#define ADCS_E_UNABLE_OPEN_DEVICE       -4

/*! adcs error code - Device not found */
#define ADCS_E_DEVICE_NOT_FOUND         -5

/*! adcs error code - failure to claim interface */
#define ADCS_E_UNABLE_CLAIM_INTF        -6

/*! adcs error code - failure to allocate memory */
#define ADCS_E_MEMORY_ALLOCATION        -7

/*! adcs error code - Feature not supported */
#define ADCS_E_NOT_SUPPORTED            -8

/*! adcs error code - Null pointer */
#define ADCS_E_NULL_PTR                 -9

/*! adcs error code - Wrong response */
#define ADCS_E_COMM_WRONG_RESPONSE      -10

/*! adcs error code - Not configured */
#define ADCS_E_SPI16BIT_NOT_CONFIGURED  -11

/*! adcs error code - SPI invalid bus interface */
#define ADCS_E_SPI_INVALID_BUS_INTF     -12

/*! adcs error code - SPI instance configured already */
#define ADCS_E_SPI_CONFIG_EXIST         -13

/*! adcs error code - SPI bus not enabled */
#define ADCS_E_SPI_BUS_NOT_ENABLED      -14

/*! adcs error code - SPI instance configuration failed */
#define ADCS_E_SPI_CONFIG_FAILED        -15

/*! adcs error code - I2C invalid bus interface */
#define ADCS_E_I2C_INVALID_BUS_INTF     -16

/*! adcs error code - I2C bus not enabled */
#define ADCS_E_I2C_BUS_NOT_ENABLED      -17

/*! adcs error code - I2C instance configuration failed */
#define ADCS_E_I2C_CONFIG_FAILED        -18

/*! adcs error code - I2C instance configured already */
#define ADCS_E_I2C_CONFIG_EXIST         -19

/*! adcs error code - Timer initialization failed */
#define ADCS_E_TIMER_INIT_FAILED        -20

/*! adcs error code - Invalid timer instance */
#define ADCS_E_TIMER_INVALID_INSTANCE   -21

/*! adcs error code - EEPROM reset failed */
#define ADCS_E_EEPROM_RESET_FAILED      -22

/*! adcs error code - EEPROM read failed */
#define ADCS_E_EEPROM_READ_FAILED       -23

/*! adcs error code - Initialization failed */
#define ADCS_E_INIT_FAILED              -24

#if defined(MCU_APP30)
#include <stdio.h>
extern FILE *bt_w, *bt_r;

/* Simplified content for missing 'dirent.h'
 * in arm-none-eabi-gcc toolchain
 */
struct dirent
{
    size_t d_namlen;
    char d_name[41];
};

typedef struct
{
    struct dirent dd_dir;
} DIR;

DIR *opendir(const char *dirname);
struct dirent *readdir(DIR *dirp);
int closedir(DIR *dirp);

#ifndef ADCS_TDM_BUFFER_SIZE_WORDS
#define ADCS_TDM_BUFFER_SIZE_WORDS  600
#endif

#endif

typedef void (*adcs_tdm_callback)(uint32_t const *data);

/**********************************************************************************/
/* data structure declarations  */
/**********************************************************************************/

/*!
 * @brief communication interface type
 */
enum adcs_comm_intf {
    ADCS_COMM_INTF_USB, /*< communication interface USB */
    ADCS_COMM_INTF_VCOM, /*< communication interface VCOM */
    ADCS_COMM_INTF_BLE /*< communication interface BLE */
};

/*!
 * @brief MULTIO Pin direction
 */
enum adcs_pin_direction {
    ADCS_PIN_DIRECTION_IN = 0, /*< PIN direction IN */
    ADCS_PIN_DIRECTION_OUT /*< PIN direction OUT */
};

/*!
 * @brief MULTIO Pin value
 */
enum adcs_pin_value {
    ADCS_PIN_VALUE_LOW = 0, /*< PIN value LOW */
    ADCS_PIN_VALUE_HIGH /*< PIN value HIGH */
};

enum adcs_pin_polarity {
    ADCS_PIN_INT_POLARITY_LOW_TO_HIGH = 1UL, /*< PIN interrupt trigger polarity when Low to High */
    ADCS_PIN_INT_POLARITY_HIGH_TO_LOW = 2UL, /*< PIN interrupt trigger polarity when High to Low */
    ADCS_PIN_INT_POLARITY_TOGGLE = 3UL /*< PIN interrupt trigger polarity toggle */
};

/*!
 * @brief I2C speed mode settings
 */
enum adcs_i2c_mode {
    ADCS_I2C_STANDARD_MODE = 0x00, /*< I2C speed in standard mode */
    ADCS_I2C_FAST_MODE = 0x01, /*< I2C speed in fast mode */
    ADCS_I2C_SPEED_3_4_MHZ = 0x02, /*< I2C speed in 3.4 MHz */
    ADCS_I2C_SPEED_1_7_MHZ = 0x03 /*< I2C speed in 1.7 MHz */
};

/*!
 * @brief ADCS sampling unit
 */
enum adcs_sampling_unit {
    ADCS_SAMPLING_TIME_IN_MICRO_SEC = 0x01, /*< sampling unit in micro second */
    ADCS_SAMPLING_TIME_IN_MILLI_SEC = 0x02 /*< sampling unit in milli second */
};

/*!
 * @brief SPI mode settings
 *
 * > Note - don't change the values
 */
enum adcs_spi_speed {
    ADCS_SPI_SPEED_10_MHZ = 6, /*< 10 MHz */
    ADCS_SPI_SPEED_7_5_MHZ = 8, /*< 7.5 MHz */
    ADCS_SPI_SPEED_6_MHZ = 10, /*< 6 MHz */
    ADCS_SPI_SPEED_5_MHZ = 12, /*< 5 MHz */
    ADCS_SPI_SPEED_3_75_MHZ = 16, /*< 3.75 MHz */
    ADCS_SPI_SPEED_3_MHZ = 20, /*< 3 MHz */
    ADCS_SPI_SPEED_2_5_MHZ = 24, /*< 2.5 MHz */
    ADCS_SPI_SPEED_2_MHZ = 30, /*< 2 MHz */
    ADCS_SPI_SPEED_1_5_MHZ = 40, /*< 1.5 MHz */
    ADCS_SPI_SPEED_1_25_MHZ = 48, /*< 1.25 MHz */
    ADCS_SPI_SPEED_1_2_MHZ = 50, /*< 1.2 MHz */
    ADCS_SPI_SPEED_1_MHZ = 60, /*< 1 MHz */
    ADCS_SPI_SPEED_750_KHZ = 80, /*< 750 kHz */
    ADCS_SPI_SPEED_600_KHZ = 100, /*< 600 kHz */
    ADCS_SPI_SPEED_500_KHZ = 120, /*< 500 kHz */
    ADCS_SPI_SPEED_400_KHZ = 150, /*< 400 kHz */
    ADCS_SPI_SPEED_300_KHZ = 200, /*< 300 kHz */
    ADCS_SPI_SPEED_250_KHZ = 240 /*< 250 kHz */
};

/*!
 * @brief ble transmit power
 *
 * > Note - don't change the values
 */
enum adcs_tx_power {
    ADCS_TX_POWER_MINUS_40_DBM = -40, /*< -40 dBm */
    ADCS_TX_POWER_MINUS_20_DBM = -20, /*< -20 dBm */
    ADCS_TX_POWER_MINUS_16_DBM = -16, /*< -16 dBm */
    ADCS_TX_POWER_MINUS_12_DBM = -12, /*< -12 dBm */
    ADCS_TX_POWER_MINUS_8_DBM = -8, /*< -8 dBm */
    ADCS_TX_POWER_MINUS_4_DBM = -4, /*< -4 dBm */
    ADCS_TX_POWER_0_DBM = 0, /*< 0 dBm */
    ADCS_TX_POWER_2_DBM = 2, /*< 2 dBm */
    ADCS_TX_POWER_3_DBM = 3, /*< 3 dBm */
    ADCS_TX_POWER_4_DBM = 4, /*< 4 dBm */
    ADCS_TX_POWER_5_DBM = 5, /*< 5 dBm */
    ADCS_TX_POWER_6_DBM = 6, /*< 6 dBm */
    ADCS_TX_POWER_7_DBM = 7, /*< 7 dBm */
    ADCS_TX_POWER_8_DBM = 8, /*< 8 dBm */
};

/*!
 *  @brief user configurable Shuttle board pin description
 */
enum adcs_multi_io_pin {
    ADCS_SHUTTLE_PIN_7 = 0x09, /*<  CS pin*/
    ADCS_SHUTTLE_PIN_8 = 0x05, /*<  Multi-IO 5*/
    ADCS_SHUTTLE_PIN_9 = 0x00, /*<  Multi-IO 0*/
    ADCS_SHUTTLE_PIN_14 = 0x01, /*<  Multi-IO 1*/
    ADCS_SHUTTLE_PIN_15 = 0x02, /*<  Multi-IO 2*/
    ADCS_SHUTTLE_PIN_16 = 0x03, /*<  Multi-IO 3*/
    ADCS_SHUTTLE_PIN_19 = 0x08, /*<  Multi-IO 8*/
    ADCS_SHUTTLE_PIN_20 = 0x06, /*<  Multi-IO 6*/
    ADCS_SHUTTLE_PIN_21 = 0x07, /*<  Multi-IO 7*/
    ADCS_SHUTTLE_PIN_22 = 0x04, /*<  Multi-IO 4*/

#if !defined(MCU_APP20)
    ADCS_MINI_SHUTTLE_PIN_1_4 = 0x10, /*<  GPIO0 */
    ADCS_MINI_SHUTTLE_PIN_1_5 = 0x11, /*<  GPIO1 */
    ADCS_MINI_SHUTTLE_PIN_1_6 = 0x12, /*<  GPIO2/INT1 */
    ADCS_MINI_SHUTTLE_PIN_1_7 = 0x13, /*<  GPIO3/INT2 */
    ADCS_MINI_SHUTTLE_PIN_2_5 = 0x14, /*<  GPIO4 */
    ADCS_MINI_SHUTTLE_PIN_2_6 = 0x15, /*<  GPIO5 */
    ADCS_MINI_SHUTTLE_PIN_2_1 = 0x16, /*<  CS */
    ADCS_MINI_SHUTTLE_PIN_2_3 = 0x17, /*<  SDO */
    ADCS_MINI_SHUTTLE_PIN_2_7 = 0x1D, /*<  GPIO6 */
    ADCS_MINI_SHUTTLE_PIN_2_8 = 0x1E, /*<  GPIO7 */
#endif

#if !((defined(MCU_APP20)) || (defined(PC)))
    ADCS_APP30_LED_R = 0x18, /*<  red LED */
    ADCS_APP30_LED_G = 0x19, /*<  green LED */
    ADCS_APP30_LED_B = 0x1A, /*<  blue LED */
    ADCS_APP30_BUTTON_1 = 0x1B, /*< button 1 */
    ADCS_APP30_BUTTON_2 = 0x1C, /*< button 2 */
#endif
    ADCS_SHUTTLE_PIN_SDO = 0x1F,
    ADCS_SHUTTLE_PIN_MAX = 0x20
};

/*!
 * @brief SPI mode settings
 */
enum adcs_spi_mode {
    ADCS_SPI_MODE0 = 0x00, /*< SPI Mode 0: CPOL=0; CPHA=0 */
    ADCS_SPI_MODE1 = 0x01, /*< SPI Mode 1: CPOL=0; CPHA=1 */
    ADCS_SPI_MODE2 = 0x02, /*< SPI Mode 2: CPOL=1; CPHA=0 */
    ADCS_SPI_MODE3 = 0x03 /*< SPI Mode 3: CPOL=1; CPHA=1 */
};

/*!
 * @brief SPI Number of bits per transfer
 */
enum adcs_spi_transfer_bits {
    ADCS_SPI_TRANSFER_8BIT = 8, /**< Transfer 8 bit */

    /* Before selecting the below 16 bit, ensure that,
     * the intended sensor supports 16bit register read/write
     */
    ADCS_SPI_TRANSFER_16BIT = 16 /**< Transfer 16 bit */
};

/*!
 * @brief interface type
 */
enum adcs_sensor_intf {
    ADCS_SENSOR_INTF_SPI, /*< SPI Interface */
    ADCS_SENSOR_INTF_I2C /*< I2C Interface */
};

/*!
 * @brief i2c bus
 */
enum adcs_i2c_bus {
    ADCS_I2C_BUS_0, /*< I2C bus 0 */
    ADCS_I2C_BUS_1, /*< I2C bus 1 */
    ADCS_I2C_BUS_MAX
};

/*!
 * @brief spi bus
 */
enum adcs_spi_bus {
    ADCS_SPI_BUS_0, /*< SPI bus 0 - Sensor interface  */
    ADCS_SPI_BUS_1, /*< SPI bus 1 - OIS interface     */
    ADCS_SPI_BUS_MAX
};

/*!
 * @brief timer instance
 */
enum adcs_timer_instance {
    ADCS_TIMER_INSTANCE_0, /*< instance 0 */
    ADCS_TIMER_INSTANCE_1, /*< instance 1 */
    ADCS_TIMER_INSTANCE_2, /*< instance 2 */
    ADCS_TIMER_INSTANCE_MAX
};

/*!
 * @brief timer configuration
 */
enum adcs_timer_config {
    ADCS_TIMER_STOP, /*< TIMER Stop */
    ADCS_TIMER_START, /*< TIMER Start */
    ADCS_TIMER_RESET /*< TIMER Reset */
};

/*!
 * @brief times stamp config
 */
enum adcs_time_stamp_config {
    ADCS_TIMESTAMP_ENABLE = 0x03, /*< TIMESTAMP Enable */
    ADCS_TIMESTAMP_DISABLE = 0x04 /*< TIMESTAMP Disable */
};

/*!
 * @brief adcs streaming mode
 */
enum adcs_streaming_mode {
    ADCS_STREAMING_MODE_POLLING, /*< Polling mode streaming */
    ADCS_STREAMING_MODE_INTERRUPT /*< Interrupt mode streaming */
};

/*!
 * @brief adcs led state
 */
enum adcs_led_state {
    ADCS_LED_STATE_ON = 0, /*< Led state ON*/
    ADCS_LED_STATE_OFF /*< Led state OFF */
};

/*!
 * @brief adcs led
 */
enum adcs_led {
    ADCS_LED_RED, /*< Red Led */
    ADCS_LED_GREEN, /*< Green Led */
    ADCS_LED_BLUE /*< Blue Led */
};

/*!
 * @brief Structure to store the board related information
 */
struct adcs_board_info
{
    uint16_t hardware_id; /*< Board hardware ID */
    uint16_t software_id; /*< Board software ID */
    uint8_t board; /*< Type of the board like APP2.0, Arduino Due */
    uint16_t shuttle_id; /*< Shuttle ID of the sensor connected */
};

/*!
 * @brief streaming address block
 */
struct adcs_streaming_blocks
{
    uint16_t no_of_blocks; /*< Number of blocks */
    uint8_t reg_start_addr[10]; /*< Register start address */
    uint8_t no_of_data_bytes[10]; /*< Number of data bytes */
};

/*!
 * @brief streaming config settings
 */
struct adcs_streaming_config
{
    enum adcs_sensor_intf intf; /*< Sensor Interface */
    enum adcs_i2c_bus i2c_bus; /*< I2C bus */
    enum adcs_spi_bus spi_bus; /*< SPI bus */
    uint8_t dev_addr; /*< I2C -Device address */
    uint8_t cs_pin; /*< Chip select */
    uint16_t sampling_time; /*< Sampling time */
    enum adcs_sampling_unit sampling_units; /*< micro second / milli second - Sampling unit */
    enum adcs_multi_io_pin int_pin; /*< Interrupt pin */
    uint8_t int_timestamp; /*< 1- enable /0- disable time stamp for corresponding sensor */
};

/*!
 * @brief Structure to store the ble configuration (name and power)
 */
struct adcs_ble_config
{
    char * name; /*< ble device name */
    enum adcs_tx_power tx_power; /*<  Radio transmit power in dBm. */
};

/*!
 * @brief Pin interrupt modes
 */
enum adcs_pin_interrupt_mode {
    ADCS_PIN_INTERRUPT_CHANGE, /*< Trigger interrupt on pin state change */
    ADCS_PIN_INTERRUPT_RISING_EDGE, /*< Trigger interrupt when pin changes from low to high */
    ADCS_PIN_INTERRUPT_FALLING_EDGE /*< Trigger interrupt when pin changes from high to low */
};

/**********************************************************************************/
/* function prototype declarations */
/**********************************************************************************/

/*!
 * @brief This API is used to initialize the communication according to interface type.
 *
 * @param[in] intf_type : Type of interface(USB, COM, or BLE).
 * @param[in] arg       : Void pointer not used currently but for future use,
 *                          e.g. to input additional details like COM port, Baud rate, etc.
 *
 * @return Result of API execution status
 * @retval Zero -> Success
 * @retval Negative -> Error
 */
int16_t adcs_open_comm_intf(enum adcs_comm_intf intf_type, void *arg);

/*!
 * @brief This API is used to close the active communication(USB,COM or BLE).
 *
 * @param[in] intf_type : Type of interface(USB, COM, or BLE).
 * @param[in] arg       : Void pointer not used currently but for future use,
 *                          e.g. to input additional details like COM port, Baud rate, etc.
 *
 * @return Result of API execution status
 * @retval Zero -> Success
 * @retval Negative -> Error
 */
int16_t adcs_close_comm_intf(enum adcs_comm_intf intf_type, void *arg);

/*!
 *  @brief This API is used to get the board information.
 *
 *  @param[out] data  : Board information details.
 *
 *  @return Result of API execution status.
 *  @retval 0 -> Success.
 *  @retval Any non zero value -> Fail.
 *
 */

int16_t adcs_get_board_info(struct adcs_board_info *data);

/*!
 *  @brief This API is used to configure the pin(MULTIIO/SPI/I2C in shuttle board).
 *
 *  @param[in] pin_number : pin to be configured.
 *  @param[in] direction : pin direction information(ADCS_PIN_DIRECTION_IN or ADCS_PIN_DIRECTION_OUT) *
 *  @param[in] pin_value : pin value information(ADCS_PIN_VALUE_LOW or ADCS_PIN_VALUE_HIGH)
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t adcs_set_pin_config(enum adcs_multi_io_pin pin_number,
                              enum adcs_pin_direction direction,
                              enum adcs_pin_value pin_value);

/*!
 *  @brief This API function is used to get the pin direction and pin state.
 *
 *  @param[in] pin_number : pin number for getting the status.
 *  @param[out] pin_direction : pin direction information(ADCS_PIN_DIRECTION_IN or ADCS_PIN_DIRECTION_OUT)
 *  @param[out] pin_value : pin value information(ADCS_PIN_VALUE_LOW or ADCS_PIN_VALUE_HIGH)*
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t adcs_get_pin_config(enum adcs_multi_io_pin pin_number,
                              enum adcs_pin_direction *pin_direction,
                              enum adcs_pin_value *pin_value);

/*!
 *  @brief This API is used to configure the VDD and VDDIO of the sensor.
 *
 *  @param[in] vdd_millivolt     : VDD voltage to be set in sensor.
 *  @param[in] vddio_millivolt   : VDDIO voltage to be set in sensor.
 *
 *  @note In APP2.0 board, voltage level of 0 or 3300mV is supported.
 *        In APP3.0 board, voltage levels of 0, 1800mV and 2800mV are supported.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t adcs_set_shuttleboard_vdd_vddio_config(uint16_t vdd_millivolt, uint16_t vddio_millivolt);

/*!
 *  @brief This API is used to configure the SPI bus
 *
 *  @param[in] bus         : bus
 *  @param[in] spi_speed   : SPI speed
 *  @param[in] spi_mode    : SPI mode
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t adcs_config_spi_bus(enum adcs_spi_bus bus, enum adcs_spi_speed spi_speed, enum adcs_spi_mode spi_mode);

/*!
 *  @brief This API is used to de-configure the SPI bus
 *
 *  @param[in] bus         : bus
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t adcs_deconfig_spi_bus(enum adcs_spi_bus bus);

/*!
 *  @brief This API is used to configure the SPI bus as either 8 bit or 16 bit length
 *
 *  @param[in] bus         : bus
 *  @param[in] spi_speed   : spi speed
 *  @param[in] spi_mode    : spi mode
 *  @param[in] spi_transfer_bits    : bits to transfer
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t adcs_config_word_spi_bus(enum adcs_spi_bus bus,
                                   enum adcs_spi_speed spi_speed,
                                   enum adcs_spi_mode spi_mode,
                                   enum adcs_spi_transfer_bits spi_transfer_bits);

/*!
 *  @brief This API is used to configure the I2C bus
 *
 *  @param[in] bus : i2c bus
 *  @param[in] i2c_mode   : i2c_mode
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t adcs_config_i2c_bus(enum adcs_i2c_bus bus, enum adcs_i2c_mode i2c_mode);

/*!
 *  @brief This API is used to de-configure the I2C bus
 *
 *  @param[in] bus : i2c bus
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t adcs_deconfig_i2c_bus(enum adcs_i2c_bus bus);

/*!
 *  @brief This API is used to write 8-bit register data on the I2C device.
 *
 *  @param[in] bus      : i2c bus.
 *  @param[in] dev_addr : Device address for I2C write.
 *  @param[in] reg_addr : Starting address for writing the data.
 *  @param[in] reg_data : Data to be written.
 *  @param[in] count    : Number of bytes to write.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t adcs_write_i2c(enum adcs_i2c_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count);

/*!
 *  @brief This API is used to read 8-bit register data from the I2C device.
 *
 *  @param[in] bus      : i2c bus.
 *  @param[in] dev_addr  : Device address for I2C read.
 *  @param[in] reg_addr  : Starting address for reading the data.
 *  @param[out] reg_data : Data read from the sensor.
 *  @param[in] count     : Number of bytes to read.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t adcs_read_i2c(enum adcs_i2c_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count);

/*!
 *  @brief This API is used to write 16-bit register data on the SPI device.
 *
 *  @param[in] bus      : spi bus.
 *  @param[in] cs       : Chip select pin number for SPI write.
 *  @param[in] reg_addr : Starting address for writing the data.
 *  @param[in] reg_data : Data to be written.
 *  @param[in] count    : Number of bytes to write.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t adcs_write_16bit_spi(enum adcs_spi_bus bus, uint8_t cs, uint16_t reg_addr, void *reg_data, uint16_t count);

/*!
 *  @brief This API is used to write 8-bit register data on the SPI device.
 *
 *  @param[in] bus      : spi bus.
 *  @param[in] dev_addr : Chip select pin number for SPI write.
 *  @param[in] reg_addr : Starting address for writing the data.
 *  @param[in] reg_data : Data to be written.
 *  @param[in] count    : Number of bytes to write.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t adcs_write_spi(enum adcs_spi_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count);

/*!
 *  @brief This API is used to read 16-bit register data from the SPI device.
 *
 *  @param[in] bus       : spi bus.
 *  @param[in] cs        : Chip select pin number for SPI read.
 *  @param[in] reg_addr  : Starting address for reading the data.
 *  @param[out] reg_data : Data read from the sensor.
 *  @param[in] count     : Number of bytes to read.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t adcs_read_16bit_spi(enum adcs_spi_bus bus, uint8_t cs, uint16_t reg_addr, void *reg_data, uint16_t count);

/*!
 *  @brief This API is used to read the data in SPI communication.
 *
 *  @param[in] bus      : spi bus.
 *  @param[in] dev_addr : Chip select pin number for SPI read.
 *  @param[in] reg_addr : Starting address for reading the data.
 *  @param[out] reg_data : Data read from the sensor.
 *  @param[in] count    : Number of bytes to read.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t adcs_read_spi(enum adcs_spi_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count);

/*!
 *  @brief This API is used for introducing a delay in milliseconds
 *
 *  @param[in] delay_ms   :  delay in milliseconds.
 *
 *  @return void
 */
void adcs_delay_msec(uint32_t delay_ms);

/*!
 *  @brief This API is used for introducing a delay in microseconds
 *
 *  @param[in] delay_us   :  delay in microseconds.
 *
 *  @return void
 */
void adcs_delay_usec(uint32_t delay_us);

/*!
 * @brief This API is used to send the streaming settings to the board.
 *
 * @param[in] channel_id    :  channel identifier (Possible values - 1,2)
 * @param[in] stream_config :  stream_config
 * @param[in] data_blocks   :  data_blocks
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t adcs_config_streaming(uint8_t channel_id,
                                struct adcs_streaming_config *stream_config,
                                struct adcs_streaming_blocks *data_blocks);

/*!
 * @brief This API is used to start or stop the streaming.
 *
 * @param[in] stream_mode :  stream_mode
 * @param[in] samples     :  samples
 * @param[in] start_stop  :  start or stop steaming
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t adcs_start_stop_streaming(enum adcs_streaming_mode stream_mode, uint8_t start_stop);

/*!
 * @brief This API is used to read the streaming sensor data.
 *
 * @param[in] sensor_id             :  Sensor Identifier.
 * @param[in] number_of_samples     :  Number of samples to be read.
 * @param[out] data                 :  Buffer to retrieve the sensor data.
 * @param[out] valid_samples_count  :  Count of valid samples available.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t adcs_read_stream_sensor_data(uint8_t sensor_id,
                                       uint32_t number_of_samples,
                                       uint8_t *data,
                                       uint32_t *valid_samples_count);

/*!
 * @brief This API is used to configure the hardware timer
 *
 * @param[in] instance : timer instance
 * @param[in] handler : callback to be called when timer expires.
 *                      use the function pointer for timer event handler as per the TARGET
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t adcs_timer_config(enum adcs_timer_instance instance, void* handler);

/*!
 * @brief This API is used to start the configured hardware timer
 *
 * @param[in] instance : timer instance
 * @param[in] timeout : in microsecond
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t adcs_timer_start(enum adcs_timer_instance instance, uint32_t timeout);

/*!
 * @brief This API is used to stop the hardware timer
 *
 * @param[in] instance : timer instance
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t adcs_timer_stop(enum adcs_timer_instance instance);

/*!
 * @brief This API is used to trigger the timer in firmware and enable or disable system time stamp
 *
 * @param[in] tmr_cfg : timer config value
 * @param[in] ts_cfg : timer stamp cfg value
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t adcs_trigger_timer(enum adcs_timer_config tmr_cfg, enum adcs_time_stamp_config ts_cfg);

/*!
 * @brief This API returns the number of milliseconds passed since the program started
 *
 * @return Time in milliseconds
 */
uint32_t adcs_get_millis();

/*!
 * @brief This API returns the number of microseconds passed since the program started
 *
 * @return Time in microseconds
 */
uint64_t adcs_get_micro_sec();

/*!
 * @brief This API is used to introduce delay based on high precision RTC(LFCLK crystal)
 * with the resolution of 30.517 usec
 *
 * @param[in]   : required delay in microseconds
 * @return      : None
 */
void adcs_delay_realtime_usec(uint32_t period);

/*!
 * @brief This API is used to get the current counter(RTC) reference time in usec
 *
 * @param[in]   : None
 * @return      : counter(RTC) reference time in usec
 * */
uint32_t adcs_get_realtime_usec(void);

/*!
 * @brief Attaches a interrupt to a Multi-IO pin
 *
 * @param[in] pin_number : Multi-IO pin
 * @param[in] callback : Name of the function to be called on detection of interrupt
 * @param[in] mode : Trigger modes - change,rising edge,falling edge
 *
 * @return void
 */
void adcs_attach_interrupt(enum adcs_multi_io_pin pin_number,
                             void (*callback)(uint32_t, uint32_t),                
                             enum adcs_pin_interrupt_mode int_mode);

/*!
 * @brief Detaches a interrupt from a Multi-IO pin
 *
 * @param[in] pin_number : Multi-IO pin
 *
 * @return void
 */
void adcs_detach_interrupt(enum adcs_multi_io_pin pin_number);

/*!
 * @brief Get ADCS library version
 *
 * @return pointer to version string
 */
char* adcs_get_version();

/*!
 * @brief Return the number of bytes available in the read buffer of the interface
 *
 * @param[in] intf : Type of interface(USB, COM, or BLE).
 *
 * @return number of bytes in the read buffer
 */
uint16_t adcs_intf_available(enum adcs_comm_intf intf);

/*!
 * @brief Check if the interface is connected
 *
 * @param[in] intf : Type of interface(USB, COM, or BLE).
 *
 * @return true if connected, false otherwise
 */
bool adcs_intf_connected(enum adcs_comm_intf intf);

/*!
 * @brief Read data over the specified interface
 *
 * @param[in] intf    : Type of interface(USB, COM, or BLE).
 * @param[out] buffer : Pointer to the buffer to store the data
 * @param[in] len     : Length of the buffer
 *
 * @return number of bytes read
 */
uint16_t adcs_read_intf(enum adcs_comm_intf intf, void *buffer, uint16_t len);

/*!
 * @brief Write data over the specified interface
 *
 * @param[in] intf    : Type of interface(USB, COM, or BLE).
 * @param[out] buffer : Pointer to the buffer storing the data
 * @param[in] len     : Length of the buffer
 *
 * @return void
 */
void adcs_write_intf(enum adcs_comm_intf intf, void *buffer, uint16_t len);

/*!
 * @brief This API is used to read the temperature sensor data.
 *
 * @param[out] temp_conv_data       :  Buffer to retrieve the sensor data in degC.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t adcs_read_temp_data(float *temp_data);

/*!
 * @brief This API is used to read the battery status .
 *
 * @param[out] bat_status_mv            :  Buffer to retrieve the battery status in millivolt.
 *
 * @param[out] bat_status_percent       :  Buffer to retrieve the battery status in %.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t adcs_read_bat_status(uint16_t *bat_status_mv, uint8_t *bat_status_percent);

/*!
 * @brief Resets the device
 *
 * @note  After reset device jumps to the address specified in makefile (APP_START_ADDRESS).
 *
 * @return void
 */
void adcs_soft_reset(void);

/*!
 *  @brief This API is used to write the data in I2C communication.
 *
 *  @param[in] bus      : i2c bus.
 *  @param[in] dev_addr : Device address for I2C write.
 *  @param[in] data     : Data to be written.
 *  @param[in] count    : Number of bytes to write.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t adcs_i2c_set(enum adcs_i2c_bus bus, uint8_t dev_addr, uint8_t *data, uint8_t count);

/*!
 *  @brief This API is used to read the data in I2C communication.
 *
 *  @param[in] bus      : i2c bus.
 *  @param[in] dev_addr : Device address for I2C read.
 *  @param[out] data    : Data read from the sensor.
 *  @param[in] count    : Number of bytes to read.
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int8_t adcs_i2c_get(enum adcs_i2c_bus bus, uint8_t dev_addr, uint8_t *data, uint8_t count);

/*!
 *  @brief This API is used to configure BLE name and power.This API should be called
 *         before calling adcs_open_comm_intf().
 *
 *  @param[in] ble_config : structure holding ble name and power details
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int16_t adcs_ble_config(struct adcs_ble_config *ble_config);

/*!
 *  @brief This API is used to set led state(on or off).
 *
 *  @param[in] led             : led to which the state has to be set
 *  @param[in] led_state       : state to be set to the given led
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t adcs_set_led(enum adcs_led led, enum adcs_led_state led_state);

/*!
 * @brief Flush the write buffer
 *
 * @param[in] intf    : Type of interface(USB, COM, or BLE).
 *
 * @return void
 */
void adcs_flush_intf(enum adcs_comm_intf intf);

#if defined(MCU_APP30)

/**
 * @brief This API can be defined to perform a task when yielded from an ongoing blocking call
 */
void adcs_yield(void);

/**
 * @brief This API is used to configure the I2S bus to match the TDM configuration
 *
 * @param[in] data_words : number of words to use in the buffer. Max is set at ADCS_TDM_BUFFER_SIZE_WORDS
 * @param[in] callback   : register a callback to be called to process and copy the data over
 *
 * @return Results of API execution status.
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t adcs_config_i2s_bus(uint16_t data_words, adcs_tdm_callback callback);

/**
 * @brief This API is used to stop the I2S/TDM interface from reading data from the sensor
 */
void adcs_deconfig_i2s_bus(void);

#endif

#ifdef __cplusplus
}
#endif

#endif /* ADCS_H_ */

/** @}*/
