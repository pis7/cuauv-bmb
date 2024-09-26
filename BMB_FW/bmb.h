#ifndef bmb_h
#define bmb_h

#include <auv_serial.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <framework.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include "twi_master_driver.h"
#include "twi_master_driver.c"

// Serial Definitions
#define HEARTBEAT_PERIOD 250 // in ms
const uint32_t serial_update_period = 100;

// TWI (I2C) Configuration
#define CPU_SPEED 32000000
#define BAUDRATE 100000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)

// I2C Addresses
#define BQ_ADDR 0x0B                 // SMBus address
#define BQ_VOLTAGE_ADDR 0x09         // in mV
#define BQ_CURRENT_ADDR 0x0A         // in mA, max 32A
#define BQ_TEMP_ADDR 0x08            // in 0.1 degree K
#define BQ_CV1_ADDR 0x3F             // in 0.1 degree K
#define BQ_CV2_ADDR 0x3E             // in 0.1 degree K
#define BQ_CV3_ADDR 0x3D             // in 0.1 degree K
#define BQ_CV4_ADDR 0x3C             // in 0.1 degree K
#define BQ_CAPACITY_PCT_ADDR 0x0D    // as %
#define BQ_CAPACITY_VAL_ADDR 0x0F    // in mAh
#define BQ_FULL_CHARGE_CAP_ADDR 0x10 // in mAh
#define BQ_BATT_STATUS_ADDR 0x16     // Section 12.23 in TRM for bit info
#define BQ_SAFETY_STATUS_ADDR 0x51   // Section 12.1.37 in TRM for bit info
#define BQ_PF_STATUS_ADDR 0x53       // Section 12.1.39 in TRM for bit info
#define BQ_SOH_ADDR 0x4F             // in %
#define BQ_AVG_EMPTY_ADDR 0x12       // in mins
#define BQ_AVG_FULL_ADDR 0x13        // in mins
#define BQ_OP_STATUS_ADDR 0x54       // Section 12.1.40 in TRM for bit info
#define BQ_DA_STATUS_1_ADDR 0x71     // Section 12.1.51 in TRM for bit info
#define BQ_DA_STATUS_2_ADDR 0x72     // Section 12.1.52 in TRM for bit info

#define OLED_ADDR 0x3C // SMBus address

// I2C Master Controller
TWI_Master_t twi_master;

// I2C Transmission Status
volatile uint8_t transmission_status;

// Counters
volatile uint8_t heartbeat_counter;
volatile uint8_t serial_counter;

// Serial times
volatile uint32_t current_time;
volatile uint32_t prev_serial_update;

// MAC
uint8_t prev_do_mac_command = 0;

// OLED Control
uint8_t oled_enter_on = 0;
uint8_t oled_enter_off = 1;

// Local BQ values - to be updated to serial variables on serial tick interval
float local_tos_voltage;
float local_cell_1_voltage;
float local_cell_2_voltage;
float local_cell_3_voltage;
float local_cell_4_voltage;
float local_batt_plus_voltage;
float local_pack_plus_voltage;
float local_current;
uint16_t local_rem_cap_pct;
uint16_t local_rem_cap_mAh;
uint16_t local_full_chg_cap_mAh;
float local_batt_temp;
float local_cell1_temp;
float local_cell2_temp;
float local_cell3_temp;
float local_cell4_temp;
float local_fet_temp;
uint16_t local_batt_status;
uint8_t local_is_charging;
uint8_t local_fully_charged;
uint8_t local_fully_discharged;
uint16_t local_safety_status_high;
uint16_t local_safety_status_low;
uint16_t local_time_to_full;
uint16_t local_time_to_empty;
uint16_t local_soh;
uint16_t local_op_status_high;
uint16_t local_op_status_low;
uint16_t local_pf_status_high;
uint16_t local_pf_status_low;
uint8_t local_chg_fet_on;
uint8_t local_dsg_fet_on;
uint8_t local_sys_pres;
uint8_t local_cell_balancing_en;
uint8_t local_terminate_charge_alarm;
uint8_t local_terminate_discharge_alarm;
uint8_t local_remaining_capacity_alarm;
uint8_t local_remaining_time_alarm;

// Funtions ----------------------------------------------------------------

// Initializations ---

/**
 * @brief Initialize MCU GPIO inputs and outputs
 *
 */
void gpio_init();
/**
 * @brief Initialize SMBus/I2C module and interrupts
 *
 */
void smbus_init();

// SMBus/I2C Transmission ---

/**
 * @brief Wait for transmission to be complete, indicated by the
 * transmission_status flag being set to 0 by the interrupt handler
 *
 */
void wait_transmission();

/**
 * @brief Read a specified 16-bit register from the BQ40Z50
 *
 * @param reg the register to read (8 bits)
 * @return uint16_t the 16-bit value read from the register
 */
uint16_t read_16_bit_bq(uint8_t reg);

/**
 * @brief Read the current() register from the BQ40Z50
 *
 * @return float the signed value of the current
 */
float read_current();

/**
 * @brief Write an 8-bit manufacturer access (MAC) command to the
 * BQ40Z50
 *
 * @param c the 8-bit MAC command to write
 */
void write_mac_command(uint8_t c);

/**
 * @brief Read a 16-bit word from a MAC register
 *
 * @param reg the MAC register to read from
 * @param bytes_to_read the number of bytes the MAC register contains (found in TRM)
 * @param byte_offset the offset of the first of the two bytes to read
 * @return uint16_t the 16-bit value contained within the specified word of the register
 */
uint16_t read_mac_get_word(uint8_t reg, uint8_t bytes_to_read, uint8_t byte_offset);

// OLED Control ---

/**
 * @brief Initialize the OLED screen via a control sequence specified in
 * the datasheet.
 *
 */
void init_oled();

/**
 * @brief Clear all contents of the screen with a specified control sequence.
 *
 */
void oled_clear_screen();

/**
 * @brief Write the row containing voltage and current on the screen
 *
 */
void write_oled_v_and_c();

/**
 * @brief Write the remaining time until empty value to the screen
 * if discharging/relaxing.
 *
 */
void write_oled_time_to_empty();

/**
 * @brief Write the remaining time until full value to the screen
 * if charging.
 *
 */
void write_oled_time_to_full();

/**
 * @brief Write the remaining battery capacity in percent and mAh
 * to the screen.
 *
 */
void write_oled_capacity();

/**
 * @brief Write the battery and FET temperatures to the screen.
 *
 */
void write_oled_temp();

/**
 * @brief Write the safety status code to the screen if present.
 *
 */
void write_oled_safety_status();

/**
 * @brief Calls all above as a wrapper function to write all
 * values to the screen at once based on current state of BMB.
 *
 */
void update_oled();

// GPIO Handlers ---

/**
 * @brief Obtains the reed switch status. 1 means the magnet has
 * not been applied, 0 means the magnet has been applied.
 *
 * @return uint8_t a 1 or0 representing the reed switch status.
 */
uint8_t reed_status();

/**
 * @brief Turns on the safety alert LED if there is any bit set
 * in the safety status registers.
 *
 */
void handle_safety_led();

// Heartbeat ---

/**
 * @brief Initialize a 1ms timer for the heartbeat LED
 *
 */
void counter_init();

/**
 * @brief Toggle the heatbeat LED if the specified time has elapsed.
 *
 */
void heartbeat();

// Serial ---

/**
 * @brief Updates the serial variables from the local values
 * which are only updated on a valid SMBus/I2C transaction.
 *
 */
void local_serial_handler();

/**
 * @brief Updates the local variable with the new value if
 * the SMBus/I2C transaction performed was successful (1).
 * Otherwise, the local value stays the same.
 *
 * @param new_value the value to possibly write to the local variable.
 * @param old_local the pointer to the local variable.
 */
void verify_smbus_float(float new_value, float *old_local);

/**
 * @brief Updates the local variable with the new value if
 * the SMBus/I2C transaction performed was successful (1).
 * Otherwise, the local value stays the same.
 *
 * @param new_value the value to possibly write to the local variable.
 * @param old_local the pointer to the local variable.
 */
void verify_smbus_uint_16(uint16_t new_value, uint16_t *old_local);

#endif /* bmb_h */
