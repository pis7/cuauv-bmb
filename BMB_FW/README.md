# High-Level Overview

The BMB MCU firmware serves as the intermediary to read data from the BQ40Z50 BMS chip over SMBus and output it to the OLED screen over I2C and RS232 serial connection to serial board. Additional features include being able to detect when the screen should be turned on via an external GPIO signal, writing MAC (manufacturer access) commands to the BQ40Z50, and custom I2C/SMBus error checking to ensure no erroneous values are written.

# Usage

Two project versions of this code exist: `BMB_A` and `BMB_B`. The only difference is that that former is meant for PODA battery pods and the latter is meant for PODB batteries, differentiated by the board name in the .toml file which is needed by the software team to recognize and read values from both BMB's simultaneously while in the sub.

# Main Loop

All initialization functions are first called before entering the infinite while loop. Inside the while loop, the voltages, current, temperatures, capacity values, etc. are all read from the BQ40Z50 and converted to appropriate units. The values are also verified with their corresponding verification functions before possibly updating the local variables if the transmission was successful.

The battery status, safety status, operation status, and permanent fail status registers are then read, verified, and each bit extracted and set to the appropriate local variable.

Next, the reed switch input is checked to see if the OLED screen should be turned on or not. There are also flags used to control operations that should be done to the screen once upon entering the on and off states such as calling the initialization function and engaging the reset signal, respectively. Note that the screen is only updated with values when it is on.

Finally, MAC commands are handled using flags to determine how to send the data. The user should enter MAC commands as decimal versions of their respective hexadecimal representations to the `mac_command` variable. To send the command, the `do_mac_command` variable needs to be set to 1. To send a new MAC command, `do_mac_command` should be set back to 0, a new MAC command entered to `mac_command` and `do_mac_command` set back to a 1 (the new MAC command can be entered first before cycling `do_mac_command` - the order does not matter). This setup ensures the MAC command is only sent to the BQ40Z50 once instead of continuously.

# Functions

### `void gpio_init()`
Initializes PORTB Pin 0 as an input for the detection of the reed switch signal, PORTA Pin 6 as an output for the safety status LED, and PORTA Pin 7 as an output for the OLED screen reset signal.

--------------------------------------------------------------------------------------------------------

### `void smbus_init()`
Initializes the MCU as a TWI master via the external TWI library on bus E with low level interrupts and the specified baud rate in the header file.

--------------------------------------------------------------------------------------------------------

### `void wait_transmission()`
Waits for an SMBus/I2C transmission to complete by waiting for the `transmission_status` variable to become something other than a 0.

--------------------------------------------------------------------------------------------------------

### `uint16_t read_16_bit_bq(uint8_t reg)`
Reads the specified 8-bit `reg` register from the BQ40Z50, waits for the transmission to complete, and returns the 16 bit data.

--------------------------------------------------------------------------------------------------------

### `float read_current`
Reads the 16-bit current value from the BQ40Z50 and converts it from an unsigned number to signed number between roughly -64 and +64 A, accounting for the 2x scaling factor set in the BQ chip firmware.

--------------------------------------------------------------------------------------------------------

### `void write_mac_command(uint8_t c)`
Writes the specified 8-bit `c` MAC command to the BQ40Z50 and waits for the transmission to complete. The data sequenece is defined in section 16.1 of the [BQ40Z50-R5 TRM](https://www.ti.com/lit/ug/sluucn4a/sluucn4a.pdf?ts=1715232050394&ref_url=https%253A%252F%252Fwww.google.com%252F).

--------------------------------------------------------------------------------------------------------

### `void init_oled()`
Initializes the OLED screen using the byte sequence defined in the [EA OLEDM204 Datasheet](https://www.lcd-module.de/fileadmin/html-seiten/eng/pdf/doma/oledm204-ae.pdf). Also disengages the reset signal.

--------------------------------------------------------------------------------------------------------

### `void oled_clear_screen()`
Clears the contents currently on the OLED screen according to the byte sequence in the datasheet.

--------------------------------------------------------------------------------------------------------

### `void write_oled_v_and_c()`
Extracts the whole, fractional, and sign parts of the float voltage and current values from the BQ40Z50 and translates it into a string format with the `sprintf` function. The data is formatted so that the length of the string is always exactly 20 characters (+ 1 for the null terminator) as the screen row is exactly 20 characters long. This string is then written to the OLED screen to print according to the byte sequence specified in the datasheet.

--------------------------------------------------------------------------------------------------------

### `void write_oled_time_to_empty()`
Writes the time to empty from the BQ40Z50 to the OLED screen with appropriate formatting and row character length as described previously.

--------------------------------------------------------------------------------------------------------

### `void write_oled_time_to_full()`
Writes the time to full from the BQ40Z50 to the OLED screen with appropriate formatting and row character length as described previously.

--------------------------------------------------------------------------------------------------------

### `void write_oled_capacity()`
Writes the remaining capacity value and percent from the BQ40Z50 to the OLED screen with appropriate formatting and row character length as described previously.

--------------------------------------------------------------------------------------------------------

### `void write_oled_temp()`
Writes the battery temperature and FET temperature values from the BQ40Z50 to the OLED screen with appropriate formatting and row character length as described previously.

--------------------------------------------------------------------------------------------------------

### `void write_oled_safety_status()`
Determines which bits of the safety_status register are set and prints out a message in readable text as to what the error code is as defined in section 16.1.38 of the TRM. If the code is not preprogrammed, the 32-bit hexadecimal number is written out to the screen to be compared against the bitmask definitions in the TRM directly.

--------------------------------------------------------------------------------------------------------

### `void update_oled()`
Calls all of the above `write_oled` functions as a wrapper. Uses the charging bit from the BQ40Z50 to determine whether to write remaining time to full or empty on the second row, as well as the OR of all safety status bits to determine if to write the safety status code or the temperatures on the fourth row.

--------------------------------------------------------------------------------------------------------

### `void reed_status()`
Obtains the binary value representing whether the reed switch is turned on or not. 0 represents the magent is applied and 1 represents the magent is not applied (active-low signal).

--------------------------------------------------------------------------------------------------------

### `void handle_safety_led()`
Turns on the safety status LED if there is a bit set in the safety status register. Turns it off otherwise.

--------------------------------------------------------------------------------------------------------

### `void counter_init()`
Initializes the 16MHz external oscillator and the 1 ms timer for the heartbeat LED.

--------------------------------------------------------------------------------------------------------

### `void heartbeat()`
Toggles the heartbeat LED every `HEARTBEAT_PERIOD`.

--------------------------------------------------------------------------------------------------------

### `void local_serial_handler()`
Updates the `serial_counter` on every timer interrupt and updates the serial variables with the local values if `serial_update_period` has been reached. Also calls the `serial_handle_messages()` funcion from the AUV serial library.

--------------------------------------------------------------------------------------------------------

### `void verify_smbus_float(float new_value, float *old_local)`
Sets the local variable referenced by `*old_local` to the `new_value` only if the previous SMBus/I2C transmission was a success (1). Does nothing if it was a fail (2). Also resets the `transmission_status` flag back to 0.

--------------------------------------------------------------------------------------------------------

### `void verify_smbus_uint_16(uint16_t new_value, uint16_t *old_local)`
Same as for the float version but for unsigned 16-bit integers.

# Interrupts

### `ISR(TCC0_OVF_vect)`
Performs a serial tick and calls `local_serial_handler()` every 1 ms. Also calls `heartbeat()` if enabled.

--------------------------------------------------------------------------------------------------------

### `ISR(TWIE_TWIM_vect)`
Calls the TWI library's master interrupt handler upon an I2C interrupt on bus E. Also sets the `transmission_status` flag based on the result of the operation. A 0 indicates the transmission is in progress, a 1 indicates the transmission was successful, and a 2 indicates the transmission failed. Note that a delay of 1 ms is used to prevent the SMBus/I2C line from getting too busy and messing up the data.

# Future Improvements
The code is incredibly robust as it is, but more serial variables may be added in the future for better debugging without having to connect the BQ40Z50 directly to BQStudio. In addition, it may be fun to add animations to the screen if possible such as the AUV prop or sponsor logos.