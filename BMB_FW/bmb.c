#include "bmb.h"

//******************************************************************************
// Configure GPIO Ports and I2C (Setup)

void gpio_init()
{
	// PA0 = REED_MCU (input)
	PORTB.DIRCLR = PIN0_bm;

	// PA6 = Safety Status LED (output)
	// PA7 = OLED_RES (output)
	PORTA.DIRSET = PIN6_bm | PIN7_bm;
}

void smbus_init()
{
	TWI_MasterInit(&twi_master, &TWIE, TWI_MASTER_INTLVL_LO_gc, TWI_BAUDSETTING);
	PMIC.CTRL |= PMIC_LOLVLEN_bm; // Low level interrupts enabled
}

//******************************************************************************
// BQ SMBus (I2C) Commands (R/W)

void wait_transmission()
{
	// Wait for transmission to complete
	while (transmission_status == 0)
		;
}

uint16_t read_16_bit_bq(uint8_t reg)
{
	TWI_MasterWriteRead(&twi_master, BQ_ADDR, &reg, 1, 2);
	wait_transmission();
	return ((twi_master.readData[1] << 8) | twi_master.readData[0]);
}

float read_current()
{
	uint16_t data_16 = read_16_bit_bq(BQ_CURRENT_ADDR);

	// divide by 1000 to get A, multiply by 2 to account for BQ scaling factor
	if (data_16 >= 32768)
		return (data_16 - 65536) / 500.0;
	else
		return data_16 / 500.0;
}

void write_mac_command(uint8_t c)
{
	uint8_t data[] = {0x44, 0x02, c, 0x00};
	TWI_MasterWrite(&twi_master, BQ_ADDR, data, 4);
	wait_transmission();
}

uint16_t read_mac_get_word(uint8_t reg, uint8_t bytes_to_read, uint8_t byte_offset)
{
	// Byte sequence specified in section 12.1 of BQ40Z50 TRM
	uint8_t data[] = {0x44, 0x02, reg, 0x00};
	TWI_MasterWrite(&twi_master, BQ_ADDR, data, 4);
	wait_transmission();
	transmission_status = 0;
	TWI_MasterWriteRead(&twi_master, BQ_ADDR, &data[0], 1, bytes_to_read + 3);
	wait_transmission();
	return ((twi_master.readData[byte_offset + 4] << 8) | twi_master.readData[byte_offset + 3]);
}

//******************************************************************************
// OLED I2C Commands (R/W)

void init_oled()
{
	PORTA.OUTSET = PIN7_bm; // Disengage reset signal

	// Byte sequence specified in EA OLEDM204-GGA datasheet for initialization
	uint8_t rom[] = {0x40, 0x00};
	uint8_t init_seq[] = {0x3A, 0x09, 0x05, 0x38, 0x3A, 0x72, 0x38, 0x0C, 0x01};

	for (int i = 0; i < 9; i++)
	{
		uint8_t command[] = {0x80, init_seq[i]};
		TWI_MasterWrite(&twi_master, OLED_ADDR, command, 2);
		wait_transmission();
		transmission_status = 0;
		if (i == 5)
		{
			TWI_MasterWrite(&twi_master, OLED_ADDR, rom, 2);
			wait_transmission();
			transmission_status = 0;
		}
	}
}

void oled_clear_screen()
{
	// Byte sequence specified in EA OLEDM204-GGA datasheet for initialization
	uint8_t screen_clear[] = {0x80, 0x01};
	TWI_MasterWrite(&twi_master, OLED_ADDR, screen_clear, 2);
	wait_transmission();
	transmission_status = 0;
}

void write_oled_v_and_c()
{
	char write_voltage[11];
	int whole_v = (int)abs(local_tos_voltage);
	int frac_v = (int)((fabs(local_tos_voltage) - whole_v) * 100.0);
	char sign_char_v = local_tos_voltage < 0 ? '-' : ' ';
	sprintf(write_voltage, "V:%c%02u.%02uV ", sign_char_v, whole_v, frac_v);

	char write_current[11];
	int whole_i = (int)abs(local_current);
	int frac_i = (int)((fabs(local_current) - whole_i) * 100.0);
	char sign_char_i = local_current < 0 ? '-' : ' ';
	sprintf(write_current, " C:%c%02u.%02uA", sign_char_i, whole_i, frac_i);

	char write_v_and_c[21];
	sprintf(write_v_and_c, "%s%s", write_voltage, write_current);
	for (int i = 0; i < (int)sizeof(write_v_and_c) - 1; i++)
	{
		uint8_t send_data[] = {0x40, (uint8_t)(write_v_and_c[i])};
		TWI_MasterWrite(&twi_master, OLED_ADDR, send_data, 2);
		wait_transmission();
		transmission_status = 0;
	}
}

void write_oled_time_to_empty()
{
	char write_time_to_empty[21];
	sprintf(write_time_to_empty, "(DSG)-RT: %5u Mins", local_time_to_empty);
	for (int i = 0; i < (int)sizeof(write_time_to_empty) - 1; i++)
	{
		uint8_t send_data[] = {0x40, (uint8_t)(write_time_to_empty[i])};
		TWI_MasterWrite(&twi_master, OLED_ADDR, send_data, 2);
		wait_transmission();
		transmission_status = 0;
	}
}

void write_oled_time_to_full()
{
	char write_time_to_full[21];
	sprintf(write_time_to_full, "(CHG)-RT: %5u Mins", local_time_to_full);
	for (int i = 0; i < (int)sizeof(write_time_to_full) - 1; i++)
	{
		uint8_t send_data[] = {0x40, (uint8_t)(write_time_to_full[i])};
		TWI_MasterWrite(&twi_master, OLED_ADDR, send_data, 2);
		wait_transmission();
		transmission_status = 0;
	}
}

void write_oled_capacity()
{
	char write_capacity[21];
	sprintf(write_capacity, "CAP-R: %5umAh %3u%%", local_rem_cap_mAh, (uint16_t)(local_rem_cap_pct));
	for (int i = 0; i < (int)sizeof(write_capacity) - 1; i++)
	{
		uint8_t send_data[] = {0x40, (uint8_t)(write_capacity[i])};
		TWI_MasterWrite(&twi_master, OLED_ADDR, send_data, 2);
		wait_transmission();
		transmission_status = 0;
	}
}

void write_oled_temp()
{
	char write_batt_temp[11];
	int whole_t_batt = (int)abs(local_batt_temp);
	int frac_t_batt = (int)((fabs(local_batt_temp) - whole_t_batt) * 100.0);
	char sign_char_t_batt = local_batt_temp < 0 ? '-' : ' ';
	sprintf(write_batt_temp, "B:%c%02u.%02uC ", sign_char_t_batt, whole_t_batt, frac_t_batt);

	char write_fet_temp[11];
	int whole_t_fet = (int)abs(local_fet_temp);
	int frac_t_fet = (int)((fabs(local_fet_temp) - whole_t_fet) * 100.0);
	char sign_char_t_fet = local_fet_temp < 0 ? '-' : ' ';
	sprintf(write_fet_temp, " F:%c%02u.%02uC", sign_char_t_fet, whole_t_fet, frac_t_fet);

	char write_temp[21];
	sprintf(write_temp, "%s%s", write_batt_temp, write_fet_temp);
	for (int i = 0; i < (int)sizeof(write_temp) - 1; i++)
	{
		uint8_t send_data[] = {0x40, (uint8_t)(write_temp[i])};
		TWI_MasterWrite(&twi_master, OLED_ADDR, send_data, 2);
		wait_transmission();
		transmission_status = 0;
	}
}

void write_oled_safety_status()
{
	char write_safety[21];
	if ((local_safety_status_low & 0x0001) == 1)
		sprintf(write_safety, "!ALRT: CELL UNDRVLT!");
	else if ((local_safety_status_low & 0x0002) >> 1 == 1)
		sprintf(write_safety, "!ALRT: CELL OVRVOLT!");
	else if ((local_safety_status_low & 0x0010) >> 4 == 1 || (local_safety_status_low & 0x0020) >> 5 == 1)
		sprintf(write_safety, "!ALRT: OVERCURR DSG!");
	else if ((local_safety_status_high & 0x0040) >> 6 == 1)
		sprintf(write_safety, "!  ALRT:  OVERCHG  !");
	else if ((local_safety_status_low & 0x1000) >> 12 == 1 || (local_safety_status_low & 0x2000) >> 13 == 1)
		sprintf(write_safety, "!ALRT: CELL OVERTMP!");
	else if ((local_safety_status_high & 0x0800) >> 11 == 1 || (local_safety_status_high & 0x0400) >> 10 == 1)
		sprintf(write_safety, "!ALRT: THRM DISCONN!");
	else if ((local_safety_status_high & 0x0001) == 1)
		sprintf(write_safety, "!ALRT:  FET OVERTMP!");
	else if ((local_safety_status_low & 0x0040) >> 6 == 1 || (local_safety_status_low & 0x0080) >> 7 == 1)
		sprintf(write_safety, "! ALRT:  OVRLD DSG !");
	else if ((local_safety_status_low & 0x0800) >> 11 == 1 || (local_safety_status_low & 0x0200) >> 9 == 1)
		sprintf(write_safety, "!ALRT:  SHORT CIRCT!");
	else
		sprintf(write_safety, "!  ALRT: %04X%04X  !", local_safety_status_high, local_safety_status_low);
	for (int i = 0; i < (int)sizeof(write_safety) - 1; i++)
	{
		uint8_t send_data[] = {0x40, (uint8_t)(write_safety[i])};
		TWI_MasterWrite(&twi_master, OLED_ADDR, send_data, 2);
		wait_transmission();
		transmission_status = 0;
	}
}

void update_oled()
{

	// oled_screen_clear();

	uint8_t next_row[] = {0x80, 0x80};
	TWI_MasterWrite(&twi_master, OLED_ADDR, next_row, 2);
	wait_transmission();
	transmission_status = 0;
	write_oled_v_and_c();

	// Rows: 0x00, 0x02, 0x04, 0x06, AND with 0x7F and OR that with 0x80
	next_row[1] = 0xA0;
	TWI_MasterWrite(&twi_master, OLED_ADDR, next_row, 2);
	wait_transmission();
	transmission_status = 0;
	if (local_is_charging == 1)
		write_oled_time_to_full();
	else
		write_oled_time_to_empty();

	next_row[1] = 0xC0;
	TWI_MasterWrite(&twi_master, OLED_ADDR, next_row, 2);
	wait_transmission();
	transmission_status = 0;
	write_oled_capacity();

	next_row[1] = 0xE0;
	TWI_MasterWrite(&twi_master, OLED_ADDR, next_row, 2);
	wait_transmission();
	transmission_status = 0;
	if (local_safety_status_high == 0 && local_safety_status_low == 0)
		write_oled_temp(); // No safety statuses triggered
	else
		write_oled_safety_status();
}

//******************************************************************************
// BQ GPIO Commands (R/W)

uint8_t reed_status()
{
	uint8_t adc_read = PORTB.IN & PIN0_bm;
	return adc_read;
}

void handle_safety_led()
{
	if (!(local_safety_status_high == 0 && local_safety_status_low == 0))
		PORTA.OUTSET = PIN6_bm; // Safety status triggered
	else
		PORTA.OUTCLR = PIN6_bm; // No safety status triggered
}

//******************************************************************************
// 1 ms Heartbeat

void counter_init()
{

	INIT_EXT_32MHz_CLK(); // initialize 16MHz external oscillator

	TCC0.CTRLA = TC_CLKSEL_DIV64_gc;		// 64x prescaler
	TCC0.CTRLB = TC_WGMODE_NORMAL_gc;		// Normal operation
	TCC0.INTCTRLA = TC_OVFINTLVL_HI_gc; // Disable error interrupt, enable overflow interrtup (LOW)
	TCC0.PER = 500;											// 500 ticks per interrupt (32MHz/64/500 = 1kHz)
	PORTD.DIRSET |= PIN4_bm;						// set PD4 to be an output (LED Heartbeat pin)
	PMIC.CTRL |= PMIC_HILVLEN_bm;				// enable high level interrupts
}

void heartbeat()
{
	heartbeat_counter++;
	if (heartbeat_counter > HEARTBEAT_PERIOD)
	{
		PORTD.OUTTGL = PIN4_bm; // toggle the LED every HEARTBEAT_PERIOD
		heartbeat_counter = 0;
	}
}

//******************************************************************************
// Local serial handler
void local_serial_handler()
{
	serial_counter++;
	serial_handle_messages();

	// Update values only after update period
	if (serial_counter > serial_update_period)
	{
		a_tos_voltage_V = local_tos_voltage;
		a_current_A = local_current;
		a_rem_cap_pct = local_rem_cap_pct;
		a_rem_cap_mAh = local_rem_cap_mAh;
		a_batt_temp_C = local_batt_temp;
		a_fet_temp_C = local_fet_temp;
		a_time_to_full_mins = local_time_to_full;
		a_time_to_empty_mins = local_time_to_empty;
		a_safety_status_high_BM = local_safety_status_high;
		a_safety_status_low_BM = local_safety_status_low;
		a_terminate_charge_alarm = local_terminate_charge_alarm;
		a_terminate_discharge_alarm = local_terminate_discharge_alarm;
		a_remaining_capacity_alarm = local_remaining_capacity_alarm;
		a_remaining_time_alarm = local_remaining_time_alarm;
		cell_1_voltage_V = local_cell_1_voltage;
		cell_2_voltage_V = local_cell_2_voltage;
		cell_3_voltage_V = local_cell_3_voltage;
		cell_4_voltage_V = local_cell_4_voltage;
		batt_plus_voltage_V = local_batt_plus_voltage;
		pack_plus_voltage_V = local_pack_plus_voltage;
		full_chg_cap_mAh = local_full_chg_cap_mAh;
		cell1_temp_C = local_cell1_temp;
		cell2_temp_C = local_cell2_temp;
		cell3_temp_C = local_cell3_temp;
		cell4_temp_C = local_cell4_temp;
		is_charging = local_is_charging;
		fully_charged = local_fully_charged;
		fully_discharged = local_fully_discharged;
		soh_pct = local_soh;
		op_status_high_BM = local_op_status_high;
		op_status_low_BM = local_op_status_low;
		pf_status_high_BM = local_pf_status_high;
		pf_status_low_BM = local_pf_status_low;
		batt_status_BM = local_batt_status;
		chg_fet_on = local_chg_fet_on;
		dsg_fet_on = local_dsg_fet_on;
		sys_pres = local_sys_pres;
		cell_balancing_en = local_cell_balancing_en;
		serial_counter = 0;
	}
}

void verify_smbus_float(float new_value, float *old_local)
{
	if (transmission_status == 1)
		*old_local = new_value;
	transmission_status = 0;
}

void verify_smbus_uint_16(uint16_t new_value, uint16_t *old_local)
{
	if (transmission_status == 1)
		*old_local = new_value;
	transmission_status = 0;
}

//******************************************************************************
int main()
{

	serial_init();
	gpio_init();
	counter_init();
	smbus_init();
	sei(); // Enable global interrupts
	init_oled();

	while (1)
	{
		// "Analog" values from BMS
		verify_smbus_float(read_16_bit_bq(BQ_VOLTAGE_ADDR) / 1000.0, &local_tos_voltage);
		verify_smbus_float(read_16_bit_bq(BQ_CV1_ADDR) / 1000.0, &local_cell_1_voltage);
		verify_smbus_float(read_16_bit_bq(BQ_CV2_ADDR) / 1000.0, &local_cell_2_voltage);
		verify_smbus_float(read_16_bit_bq(BQ_CV3_ADDR) / 1000.0, &local_cell_3_voltage);
		verify_smbus_float(read_16_bit_bq(BQ_CV4_ADDR) / 1000.0, &local_cell_4_voltage);
		verify_smbus_float(read_mac_get_word(BQ_DA_STATUS_1_ADDR, 32, 8) / 1000.0, &local_batt_plus_voltage);
		verify_smbus_float(read_mac_get_word(BQ_DA_STATUS_1_ADDR, 32, 10) / 1000.0, &local_pack_plus_voltage);
		verify_smbus_float(read_current(), &local_current);
		verify_smbus_uint_16(read_16_bit_bq(BQ_CAPACITY_PCT_ADDR), &local_rem_cap_pct);
		verify_smbus_uint_16(read_16_bit_bq(BQ_CAPACITY_VAL_ADDR) * 2, &local_rem_cap_mAh);
		verify_smbus_uint_16(read_16_bit_bq(BQ_FULL_CHARGE_CAP_ADDR) * 2, &local_full_chg_cap_mAh);
		verify_smbus_float(read_16_bit_bq(BQ_TEMP_ADDR) / 10.0 - 273.15, &local_batt_temp);
		verify_smbus_float(read_mac_get_word(BQ_DA_STATUS_2_ADDR, 18, 2) / 10.0 - 273.15, &local_cell1_temp);
		verify_smbus_float(read_mac_get_word(BQ_DA_STATUS_2_ADDR, 18, 4) / 10.0 - 273.15, &local_cell2_temp);
		verify_smbus_float(read_mac_get_word(BQ_DA_STATUS_2_ADDR, 18, 6) / 10.0 - 273.15, &local_cell3_temp);
		verify_smbus_float(read_mac_get_word(BQ_DA_STATUS_2_ADDR, 18, 8) / 10.0 - 273.15, &local_cell4_temp);
		verify_smbus_float(read_mac_get_word(BQ_DA_STATUS_2_ADDR, 18, 12) / 10.0 - 273.15, &local_fet_temp);
		verify_smbus_uint_16(read_16_bit_bq(BQ_AVG_FULL_ADDR), &local_time_to_full);
		verify_smbus_uint_16(read_16_bit_bq(BQ_AVG_EMPTY_ADDR), &local_time_to_empty);
		verify_smbus_uint_16(read_16_bit_bq(BQ_SOH_ADDR), &local_soh);

		// Battery Status
		verify_smbus_uint_16(read_16_bit_bq(BQ_BATT_STATUS_ADDR), &local_batt_status);
		local_is_charging = ~((local_batt_status & 0x40) >> 6) & 0x0001; // bit 7
		local_fully_charged = (local_batt_status & 0x20) >> 5;					 // bit 6
		local_fully_discharged = (local_batt_status & 0x10) >> 4;				 // bit 5
		local_terminate_charge_alarm = (local_batt_status & 0x4000) >> 14;
		local_terminate_discharge_alarm = (local_batt_status & 0x0800) >> 11;
		local_remaining_capacity_alarm = (local_batt_status & 0x0200) >> 9;
		local_remaining_time_alarm = (local_batt_status & 0x0100) >> 8;

		// Safety Status
		verify_smbus_uint_16(read_mac_get_word(BQ_SAFETY_STATUS_ADDR, 4, 2), &local_safety_status_high);
		verify_smbus_uint_16(read_mac_get_word(BQ_SAFETY_STATUS_ADDR, 4, 0), &local_safety_status_low);
		handle_safety_led();

		// Operation Status
		verify_smbus_uint_16(read_mac_get_word(BQ_OP_STATUS_ADDR, 4, 2), &local_op_status_high);
		verify_smbus_uint_16(read_mac_get_word(BQ_OP_STATUS_ADDR, 4, 0), &local_op_status_low);
		local_cell_balancing_en = (local_op_status_high & 0x1000) >> 12;
		local_chg_fet_on = (local_op_status_low & 0x0004) >> 2;
		local_dsg_fet_on = (local_op_status_low & 0x0002) >> 1;
		local_sys_pres = local_op_status_low & 0x0001;

		// Permanent Fail Status
		verify_smbus_uint_16(read_mac_get_word(BQ_PF_STATUS_ADDR, 4, 2), &local_pf_status_high);
		verify_smbus_uint_16(read_mac_get_word(BQ_OP_STATUS_ADDR, 4, 2), &local_pf_status_low);

		// OLED screen control + update
		if (reed_status() == 0)
		{
			if (oled_enter_on == 1)
			{
				init_oled();
				oled_enter_on = 0;
				oled_enter_off = 1;
			}
			update_oled();
		}
		else
		{
			if (oled_enter_off == 1)
			{
				PORTA.OUTCLR = PIN7_bm; // Engage reset signal
				oled_enter_off = 0;
				oled_enter_on = 1;
			}
		}

		// MAC commands from serial
		if (do_mac_command & ~prev_do_mac_command)
		{
			write_mac_command(mac_command);
			prev_do_mac_command = 1;
		}
		else if (!do_mac_command)
			prev_do_mac_command = 0;
	}
}

//******************************************************************************
// Interrupts

// 1ms Timer Handler
ISR(TCC0_OVF_vect)
{
	serial_tick();
	// heartbeat(); // Disabled to save power normally, can re-enable for debugging
	local_serial_handler();
}

// TWIE Handler
ISR(TWIE_TWIM_vect)
{
	// transmission_status:
	// 0 = transmission in progress
	// 1 = transmission successful
	// 2 = transmission failed

	TWI_MasterInterruptHandler(&twi_master);
	_delay_ms(1);
	if (twi_master.status == TWIM_STATUS_READY)
	{
		if (twi_master.result == TWIM_RESULT_OK)
			transmission_status = 1;
		else
			transmission_status = 2;
	}
}
