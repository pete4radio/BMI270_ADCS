/**\
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include "bmi270_legacy.h"
#include "common.h"

/******************************************************************************/
/*!                  Macros                                                   */

/*! Buffer size allocated to store raw FIFO data. */
#define BMI270_LEGACY_FIFO_RAW_DATA_BUFFER_SIZE  UINT16_C(2048)

/*! Length of data to be read from FIFO. */
#define BMI270_LEGACY_FIFO_RAW_DATA_USER_LENGTH  UINT16_C(2048)

/*! Number of accel frames to be extracted from FIFO. */

/*! Calculation for frame count: Total frame count = Fifo buffer size(2048)/ Total frames(6 Accel, 6 Gyro and 1 header,
 * totaling to 13) which equals to 157.
 *
 * Extra frames to parse sensortime data
 */
#define BMI2_FIFO_ACCEL_FRAME_COUNT              UINT8_C(185)

/*! Number of gyro frames to be extracted from FIFO. */
#define BMI2_FIFO_GYRO_FRAME_COUNT               UINT8_C(185)

/*! Macro to read sensortime byte in FIFO. */
#define SENSORTIME_OVERHEAD_BYTE                 UINT8_C(220)

/******************************************************************************/
/*!                        Global Variables                                   */

/* To read sensortime, extra bytes are added to fifo buffer. */
uint16_t fifo_buffer_size = BMI270_LEGACY_FIFO_RAW_DATA_BUFFER_SIZE + SENSORTIME_OVERHEAD_BYTE;

/* Number of bytes of FIFO data
 * NOTE : Dummy byte (for SPI Interface) required for FIFO data read must be given as part of array size
 * Array size same as fifo_buffer_size
 */
uint8_t fifo_data[BMI270_LEGACY_FIFO_RAW_DATA_BUFFER_SIZE + SENSORTIME_OVERHEAD_BYTE];

/* Array of accelerometer frames -> Total bytes =
 * 157 * (6 axes + 1 header bytes) = 1099 bytes */
struct bmi2_sens_axes_data fifo_accel_data[BMI2_FIFO_ACCEL_FRAME_COUNT] = { { 0 } };

/* Array of gyro frames -> Total bytes =
 * 157 * (6 axes + 1 header bytes) = 1099 bytes */
struct bmi2_sens_axes_data fifo_gyro_data[BMI2_FIFO_GYRO_FRAME_COUNT] = { { 0 } };

/******************************************************************************/
/*!           Static Function Declaration                                     */

/*!
 *  @brief This internal API is used to set configurations for accel and gyro.
 *  @param[in] dev       : Structure instance of bmi2_dev.
 *  @return Status of execution.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *dev);

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program. */
int main(void)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    uint16_t index = 0;
    uint16_t fifo_length = 0;
    uint16_t config = 0;

    uint8_t try = 1;

    /* Variable to get fifo full interrupt status. */
    uint16_t int_status = 0;

    uint16_t accel_frame_length = BMI2_FIFO_ACCEL_FRAME_COUNT;

    uint16_t gyro_frame_length = BMI2_FIFO_GYRO_FRAME_COUNT;

    /* Sensor initialization configuration. */
    struct bmi2_dev dev;

    /* Initialize FIFO frame structure. */
    struct bmi2_fifo_frame fifoframe = { 0 };

    /* Accel and gyro sensor are listed in array. */
    uint8_t sensor_sel[2] = { BMI2_ACCEL, BMI2_GYRO };

    /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    rslt = bmi2_interface_init(&dev, BMI2_I2C_INTF);
    bmi2_error_codes_print_result(rslt);

    /* Initialize bmi270_legacy. */
    rslt = bmi270_legacy_init(&dev);
    bmi2_error_codes_print_result(rslt);

    /* Configuration settings for accel and gyro. */
    rslt = set_accel_gyro_config(&dev);
    bmi2_error_codes_print_result(rslt);

    /* NOTE:
     * Accel and Gyro enable must be done after setting configurations
     */
    rslt = bmi270_legacy_sensor_enable(sensor_sel, 2, &dev);
    bmi2_error_codes_print_result(rslt);

    /* Before setting FIFO, disable the advance power save mode. */
    rslt = bmi2_set_adv_power_save(BMI2_DISABLE, &dev);
    bmi2_error_codes_print_result(rslt);

    /* Initially disable all configurations in fifo. */
    rslt = bmi2_get_fifo_config(&config, &dev);
    bmi2_error_codes_print_result(rslt);

    /* Initially disable all configurations in fifo. */
    rslt = bmi2_set_fifo_config(BMI2_FIFO_ALL_EN, BMI2_DISABLE, &dev);
    bmi2_error_codes_print_result(rslt);

    /* Update FIFO structure. */
    /* Mapping the buffer to store the fifo data. */
    fifoframe.data = fifo_data;

    /* Length of FIFO frame. */
    /* To read sensortime, extra bytes are added to fifo user length. */
    fifoframe.length = BMI270_LEGACY_FIFO_RAW_DATA_USER_LENGTH + SENSORTIME_OVERHEAD_BYTE;

    /* Set FIFO configuration by enabling accel, gyro and timestamp.
     * NOTE 1: The header mode is enabled by default.
     * NOTE 2: By default the FIFO operating mode is in FIFO mode.
     * NOTE 3: Sensortime is enabled by default */
    printf("FIFO is configured in header mode\n");
    rslt = bmi2_set_fifo_config(BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN, BMI2_ENABLE, &dev);
    bmi2_error_codes_print_result(rslt);

    /* Map FIFO full interrupt. */
    fifoframe.data_int_map = BMI2_FFULL_INT;
    rslt = bmi2_map_data_int(fifoframe.data_int_map, BMI2_INT1, &dev);
    bmi2_error_codes_print_result(rslt);

    while (try <= 10)
    {
        /* Read FIFO data on interrupt. */
        rslt = bmi2_get_int_status(&int_status, &dev);
        bmi2_error_codes_print_result(rslt);

        if ((rslt == BMI2_OK) && (int_status & BMI2_FFULL_INT_STATUS_MASK))
        {
            printf("\nIteration : %d\n", try);

            accel_frame_length = BMI2_FIFO_ACCEL_FRAME_COUNT;

            gyro_frame_length = BMI2_FIFO_GYRO_FRAME_COUNT;

            rslt = bmi2_get_fifo_length(&fifo_length, &dev);
            bmi2_error_codes_print_result(rslt);

            /* Updating FIFO length to be read based on available length and dummy byte updation */
            fifoframe.length = fifo_length + SENSORTIME_OVERHEAD_BYTE + dev.dummy_byte;

            printf("\nFIFO data bytes available : %d \n", fifo_length);
            printf("\nFIFO data bytes requested : %d \n", fifoframe.length);

            /* Read FIFO data. */
            rslt = bmi2_read_fifo_data(&fifoframe, &dev);
            bmi2_error_codes_print_result(rslt);

            /* Read FIFO data on interrupt. */
            rslt = bmi2_get_int_status(&int_status, &dev);
            bmi2_error_codes_print_result(rslt);

            if (rslt == BMI2_OK)
            {
                printf("\nFIFO accel frames requested : %d \n", accel_frame_length);

                /* Parse the FIFO data to extract accelerometer data from the FIFO buffer. */
                (void)bmi2_extract_accel(fifo_accel_data, &accel_frame_length, &fifoframe, &dev);
                printf("\nFIFO accel frames extracted : %d \n", accel_frame_length);

                printf("\nFIFO gyro frames requested : %d \n", gyro_frame_length);

                /* Parse the FIFO data to extract gyro data from the FIFO buffer. */
                (void)bmi2_extract_gyro(fifo_gyro_data, &gyro_frame_length, &fifoframe, &dev);
                printf("\nFIFO gyro frames extracted : %d \n", gyro_frame_length);

                printf("\nExtracted accel frames\n");

                printf("ACCEL_DATA, X, Y, Z\n");

                /* Print the parsed accelerometer data from the FIFO buffer. */
                for (index = 0; index < accel_frame_length; index++)
                {
                    printf("%d, %d, %d, %d\n",
                           index,
                           fifo_accel_data[index].x,
                           fifo_accel_data[index].y,
                           fifo_accel_data[index].z);
                }

                printf("\nExtracted gyro frames\n");

                printf("GYRO_DATA, X, Y, Z\n");

                /* Print the parsed gyro data from the FIFO buffer. */
                for (index = 0; index < gyro_frame_length; index++)
                {
                    printf("%d, %d, %d, %d\n", index, fifo_gyro_data[index].x, fifo_gyro_data[index].y,
                           fifo_gyro_data[index].z);
                }

                /* Print control frames like sensor time and skipped frame count. */
                printf("\nSkipped frame count = %d\n", fifoframe.skipped_frame_count);

                printf("Sensor time(in seconds) = %.4lf  s\n", (fifoframe.sensor_time * BMI2_SENSORTIME_RESOLUTION));
            }

            try++;
        }
    }

    bmi2_adcs_deinit();

    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for accel and gyro.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *dev)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accel and gyro configurations. */
    struct bmi2_sens_config config[2];

    /* Configure the type of feature. */
    config[0].type = BMI2_ACCEL;
    config[1].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi270_legacy_get_sensor_config(config, 2, dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameters according to their requirement. */
        /* Accel configuration settings. */
        /* Output Data Rate */
        config[0].cfg.acc.odr = BMI2_ACC_ODR_50HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* Gyro configuration settings. */
        /* Output data Rate */
        config[1].cfg.gyr.odr = BMI2_GYR_ODR_50HZ;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;

        /* Gyroscope Bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set new configurations. */
        rslt = bmi270_legacy_set_sensor_config(config, 2, dev);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}
