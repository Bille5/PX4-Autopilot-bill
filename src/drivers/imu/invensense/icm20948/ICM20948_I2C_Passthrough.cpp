/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ICM20948_I2C_Passthrough.hpp"

#include <mathlib/mathlib.h>

using namespace time_literals;

namespace
{
// Sample the live data registers at 200 Hz. The chip produces data at
// about 225 Hz with its low-pass filters enabled and a divider of 4.
constexpr uint32_t motion_interval_us = 5_ms;
constexpr uint8_t sample_rate_divider = 4;
constexpr uint8_t filter_setting = Bit4 | Bit3;
constexpr uint8_t accel_config = filter_setting | ACCEL_FS_SEL_16G | ACCEL_FCHOICE;
constexpr uint8_t gyro_config = filter_setting | GYRO_FS_SEL_2000_DPS | GYRO_FCHOICE;
constexpr float accel_range = 16.f * CONSTANTS_ONE_G;
constexpr float gyro_range = math::radians(2000.f);
}

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

ICM20948_I2C_Passthrough::ICM20948_I2C_Passthrough(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_publish_motion(config.custom1 == 1),
	_rotation(config.rotation)
{
}

ICM20948_I2C_Passthrough::MotionPublishers::MotionPublishers(uint32_t device_id, enum Rotation rotation) :
	accel(device_id, rotation),
	gyro(device_id, rotation)
{
	accel.set_scale(CONSTANTS_ONE_G / 2048.f);
	accel.set_range(accel_range);
	gyro.set_scale(math::radians(1.f / 16.4f));
	gyro.set_range(gyro_range);
}

ICM20948_I2C_Passthrough::~ICM20948_I2C_Passthrough()
{
	delete _motion;
	perf_free(_reset_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
}

int ICM20948_I2C_Passthrough::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	if (_publish_motion) {
		_motion = new MotionPublishers(get_device_id(), _rotation);

		if (_motion == nullptr) {
			return -ENOMEM;
		}
	}

	return Reset() ? 0 : -1;
}

bool ICM20948_I2C_Passthrough::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ICM20948_I2C_Passthrough::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO_RAW("temperature: %.1f degC", (double)_temperature);

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
}

int ICM20948_I2C_Passthrough::probe()
{
	// 3 retries
	for (int i = 0; i < 3; i++) {

		const uint8_t WHO_AM_I = RegisterRead(Register::BANK_0::WHO_AM_I);

		if (WHO_AM_I == WHOAMI) {
			return PX4_OK;

		} else {
			DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", WHO_AM_I);

			uint8_t reg_bank_sel = RegisterRead(Register::BANK_0::REG_BANK_SEL);
			int bank = reg_bank_sel >> 4;

			if (bank >= 1 && bank <= 3) {
				DEVICE_DEBUG("incorrect register bank for WHO_AM_I REG_BANK_SEL:0x%02x, bank:%d", reg_bank_sel, bank);
				// force bank selection and retry
				SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0, true);
			}
		}
	}

	return PX4_ERROR;
}

void ICM20948_I2C_Passthrough::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		perf_count(_reset_perf);
		// PWR_MGMT_1: Device Reset
		RegisterWrite(Register::BANK_0::PWR_MGMT_1, PWR_MGMT_1_BIT::DEVICE_RESET);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(1_ms);
		break;

	case STATE::WAIT_FOR_RESET:

		// The reset value is 0x00 for all registers other than the registers below
		if ((RegisterRead(Register::BANK_0::WHO_AM_I) == WHOAMI) && (RegisterRead(Register::BANK_0::PWR_MGMT_1) == 0x41)) {
			// Wakeup
			RegisterSetAndClearBits(Register::BANK_0::PWR_MGMT_1, 0, PWR_MGMT_1_BIT::SLEEP);

			// if reset succeeded then configure
			_state = STATE::CONFIGURE;
			ScheduleDelayed(1_ms);

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 30_s) {
				PX4_ERR("Reset failed, retrying");
				Reset();

			} else {
				PX4_DEBUG("Reset not complete, check again in 10 ms");
				ScheduleDelayed(10_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading
			_state = STATE::READ;
			ScheduleOnInterval(_motion ? motion_interval_us : 500_ms);

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 30_s) {
				PX4_ERR("Configure failed, resetting");
				Reset();

			} else {
				PX4_DEBUG("Configure failed, retrying");
				ScheduleDelayed(10_ms);
			}
		}

		break;

	case STATE::READ: {
			bool success;

			if (_motion) {
				success = ReadMotion(now);

			} else {
				const uint8_t temp_high = RegisterRead(Register::BANK_0::TEMP_OUT_H);
				const uint8_t temp_low = RegisterRead(Register::BANK_0::TEMP_OUT_L);
				success = UpdateTemperature(combine(temp_high, temp_low));
			}

			if (success) {
				if (_failure_count > 0) {
					_failure_count--;
				}

			} else {
				perf_count(_bad_transfer_perf);
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					Reset();
					return;
				}
			}

			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_bank0_cfg[_checked_register_bank0])) {
					_last_config_check_timestamp = now;
					_checked_register_bank0 = (_checked_register_bank0 + 1) % size_register_bank0_cfg;

				} else {
					// register check failed, force reset
					perf_count(_bad_register_perf);
					Reset();
				}
			}
		}

		break;
	}
}

bool ICM20948_I2C_Passthrough::SelectRegisterBank(enum REG_BANK_SEL_BIT bank, bool force)
{
	if (bank != _last_register_bank || force) {
		uint8_t cmd_bank_sel[2];
		cmd_bank_sel[0] = static_cast<uint8_t>(Register::BANK_0::REG_BANK_SEL);
		cmd_bank_sel[1] = bank;

		if (transfer(cmd_bank_sel, sizeof(cmd_bank_sel), nullptr, 0) != PX4_OK) {
			return false;
		}

		_last_register_bank = bank;
	}

	return true;
}

bool ICM20948_I2C_Passthrough::Configure()
{
	_retries = 2;

	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_bank0_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_bank0_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	return success && (!_motion || ConfigureMotion());
}

bool ICM20948_I2C_Passthrough::ConfigureMotion()
{
	using Register::BANK_0;
	using Register::BANK_2;

	// Match the PlatformIO driver's wake sequence and full-scale ranges.
	return RegisterWrite(BANK_0::PWR_MGMT_2, 0)
	       && RegisterWrite(BANK_2::ACCEL_SMPLRT_DIV_1, 0)
	       && RegisterWrite(BANK_2::ACCEL_SMPLRT_DIV_2, sample_rate_divider)
	       && RegisterWrite(BANK_2::GYRO_SMPLRT_DIV, sample_rate_divider)
	       && RegisterWrite(BANK_2::ACCEL_CONFIG, accel_config)
	       && RegisterWrite(BANK_2::GYRO_CONFIG_1, gyro_config)
	       && RegisterRead(BANK_2::ACCEL_SMPLRT_DIV_2) == sample_rate_divider
	       && RegisterRead(BANK_2::GYRO_SMPLRT_DIV) == sample_rate_divider
	       && RegisterRead(BANK_2::ACCEL_CONFIG) == accel_config
	       && RegisterRead(BANK_2::GYRO_CONFIG_1) == gyro_config
	       && SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0);
}

bool ICM20948_I2C_Passthrough::ReadMotion(hrt_abstime timestamp_sample)
{
	const uint8_t start = static_cast<uint8_t>(Register::BANK_0::ACCEL_XOUT_H);
	uint8_t sample[14]{};

	if (transfer(&start, 1, sample, sizeof(sample)) != PX4_OK
	    || !UpdateTemperature(combine(sample[12], sample[13]))) {
		return false;
	}

	const uint64_t errors = perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf);
	_motion->accel.set_error_count(errors);
	_motion->gyro.set_error_count(errors);
	_motion->accel.set_temperature(_temperature);
	_motion->gyro.set_temperature(_temperature);

	// Use the same forward-right-down conversion as the SPI driver.
	_motion->accel.update(timestamp_sample, combine(sample[0], sample[1]),
			      -combine(sample[2], sample[3]), -combine(sample[4], sample[5]));
	_motion->gyro.update(timestamp_sample, combine(sample[6], sample[7]),
			     -combine(sample[8], sample[9]), -combine(sample[10], sample[11]));
	return true;
}

bool ICM20948_I2C_Passthrough::UpdateTemperature(int16_t raw_temperature)
{
	const float temperature = raw_temperature / TEMPERATURE_SENSITIVITY + TEMPERATURE_OFFSET;

	if (temperature < TEMPERATURE_SENSOR_MIN || temperature > TEMPERATURE_SENSOR_MAX) {
		return false;
	}

	_temperature = temperature;
	return true;
}

template <typename T>
bool ICM20948_I2C_Passthrough::RegisterCheck(const T &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

template <typename T>
uint8_t ICM20948_I2C_Passthrough::RegisterRead(T reg)
{
	uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t value = 0;

	if (SelectRegisterBank(reg)) {
		transfer(&cmd, 1, &value, 1);
	}

	return value;
}

template <typename T>
bool ICM20948_I2C_Passthrough::RegisterWrite(T reg, uint8_t value)
{
	uint8_t cmd[2];
	cmd[0] = static_cast<uint8_t>(reg);
	cmd[1] = value;
	return SelectRegisterBank(reg) && transfer(cmd, sizeof(cmd), nullptr, 0) == PX4_OK;
}

template <typename T>
void ICM20948_I2C_Passthrough::RegisterSetAndClearBits(T reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}
