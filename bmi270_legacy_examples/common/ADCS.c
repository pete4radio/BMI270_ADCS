/* ADCS.c
The pico-sdk compatible I2C interface
Interrupts are not supported.
SPI is not supported
Closing the communication interface is not supported
The Shuttle board and its temperature and battery is not supported
Sensor Vdd adjustment is not supported
Streaming is not supported
Pete*/

#include "ADCS.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/spi.h"

// from logger.h
#include "pico/printf.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/types.h"

int16_t adcs_open_comm_intf(enum adcs_comm_intf intf_type, void *arg) { return 0;}
int16_t adcs_close_comm_intf(enum adcs_comm_intf intf_type, void *arg) { return 0;}
int16_t adcs_get_board_info(struct adcs_board_info *data) { return 0;}
int16_t adcs_set_pin_config(enum adcs_multi_io_pin pin_number,
    enum adcs_pin_direction direction,
    enum adcs_pin_value pin_value) { return 0;}
int16_t adcs_get_pin_config(enum adcs_multi_io_pin pin_number,
    enum adcs_pin_direction *pin_direction,
    enum adcs_pin_value *pin_value) { return 0;}
int16_t adcs_set_shuttleboard_vdd_vddio_config(uint16_t vdd_millivolt, uint16_t vddio_millivolt) { return 0;}
int16_t adcs_config_spi_bus(enum adcs_spi_bus bus, enum adcs_spi_speed spi_speed, enum adcs_spi_mode spi_mode) { return 0;}
int16_t adcs_deconfig_spi_bus(enum adcs_spi_bus bus) { return 0;}
int16_t adcs_config_word_spi_bus(enum adcs_spi_bus bus,
    enum adcs_spi_speed spi_speed,
    enum adcs_spi_mode spi_mode,
    enum adcs_spi_transfer_bits spi_transfer_bits) { return 0;}

    //I2C Init
int16_t adcs_config_i2c_bus(enum adcs_i2c_bus bus, enum adcs_i2c_mode i2c_mode) {
        i2c_init(i2c0, 400 * 1000);  //Initialize I2C 0 port at 400 kHz
        gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
        gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
        return SUCCESS;
}

int16_t adcs_deconfig_i2c_bus(enum adcs_i2c_bus bus)  { return 0;}

//I2C write
int8_t adcs_write_i2c(enum adcs_i2c_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count) {
    int num_bytes_read = 0;
    uint8_t msg[count + 1];
// Check to make sure caller is sending 1 or more byte
    if (count < 1) { return 0; }
// Append register address to front of data packet
    msg[0] = reg_addr;
    for (int i = 0; i < count; i++) {  msg[i + 1] = reg_data[i]; }
// Write data to register(s) over I2C
    i2c_write_blocking(i2c0, dev_addr, msg, (count + 1), false);
    return num_bytes_read; 
}
//I2C read
int8_t adcs_read_i2c(enum adcs_i2c_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count) {
    int num_bytes_read = 0;
// Check to make sure caller is asking for 1 or more bytes
   if (count < 1) {  return 0;  }
// Read data from register(s) over I2C
	i2c_write_blocking(i2c0, dev_addr, &reg_addr, 1, true);
	num_bytes_read = i2c_read_blocking(i2c0, reg_addr, reg_data, count, false);
	return num_bytes_read;
}

int8_t adcs_write_16bit_spi(enum adcs_spi_bus bus, uint8_t cs, uint16_t reg_addr, void *reg_data, uint16_t count)  { return 0; }
int8_t adcs_write_spi(enum adcs_spi_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count) { return 0; }
int8_t adcs_read_16bit_spi(enum adcs_spi_bus bus, uint8_t cs, uint16_t reg_addr, void *reg_data, uint16_t count) { return 0; }
int8_t adcs_read_spi(enum adcs_spi_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count) { return 0; }

// delay in ms
void adcs_delay_msec(uint32_t delay_ms) {sleep_ms(delay_ms); return;  }

// delay in us
void adcs_delay_usec(uint32_t delay_us) {sleep_us(delay_us); return; }

int16_t adcs_start_stop_streaming(enum adcs_streaming_mode stream_mode, uint8_t start_stop) { return 0; }
int16_t adcs_read_stream_sensor_data(uint8_t sensor_id,
    uint32_t number_of_samples,
    uint8_t *data,
    uint32_t *valid_samples_count) { return 0; }
// Configure Hardware timer
int16_t adcs_timer_config(enum adcs_timer_instance instance, void* handler) { return 0; }
// Start hardware timer
int16_t adcs_timer_start(enum adcs_timer_instance instance, uint32_t timeout) { return 0; }
//  Stop hardware timer
int16_t adcs_timer_stop(enum adcs_timer_instance instance) { return 0; }
// Trigger hardware timer
int16_t adcs_trigger_timer(enum adcs_timer_config tmr_cfg, enum adcs_time_stamp_config ts_cfg) { return 0; }

// get time since program started
uint32_t adcs_get_millis() {  return to_ms_since_boot(get_absolute_time());  }

// get time since program started in us
uint64_t adcs_get_micro_sec() {return to_us_since_boot(get_absolute_time()); }

// introduce a very accurate delay in microseconds
void adcs_delay_realtime_usec(uint32_t period) { sleep_us(period); }

// current reference time in us
uint32_t adcs_get_realtime_usec(void) { return to_us_since_boot(get_absolute_time()); }

void adcs_attach_interrupt(enum adcs_multi_io_pin pin_number,
    void (*callback)(uint32_t, uint32_t),                
    enum adcs_pin_interrupt_mode int_mode);
    void adcs_detach_interrupt(enum adcs_multi_io_pin pin_number) { return ; }
// get program version
char* adcs_get_version() { return "V1.0";  }
uint16_t adcs_intf_available(enum adcs_comm_intf intf) { return 0; }
bool adcs_intf_connected(enum adcs_comm_intf intf) { return 0; }
//  Number of bytes read
uint16_t adcs_read_intf(enum adcs_comm_intf intf, void *buffer, uint16_t len);
// Write data over selected interface
void adcs_write_intf(enum adcs_comm_intf intf, void *buffer, uint16_t len);
int16_t adcs_read_temp_data(float *temp_data) { return 0; }
int16_t adcs_read_bat_status(uint16_t *bat_status_mv, uint8_t *bat_status_percent) { return 0; }

//  Reboot the controller board
void adcs_soft_reset(void) { return; }

//  Set I2C bus parameters
int8_t adcs_i2c_set(enum adcs_i2c_bus bus, uint8_t dev_addr, uint8_t *data, uint8_t count);
// Get I2C bus parameters
int8_t adcs_i2c_get(enum adcs_i2c_bus bus, uint8_t dev_addr, uint8_t *data, uint8_t count);
int16_t adcs_ble_config(struct adcs_ble_config *ble_config);
//  Turn the LED on or off
int16_t adcs_set_led(enum adcs_led led, enum adcs_led_state led_state);
void adcs_flush_intf(enum adcs_comm_intf intf) { return; }
