/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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

#include "ELVX.hpp"

using namespace time_literals;

ELVX::ELVX(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
}

ELVX::~ELVX()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_fault_perf);
}

int ELVX::probe()
{
	_retries = 2;

	for (int i = 0; i < 10; i++) {
		// ELVX is read-only: a 4-byte data fetch that ACKs and returns a
		// non-fault status confirms the sensor is present.
		uint8_t data[4] {};

		if (transfer(nullptr, 0, &data[0], sizeof(data)) == PX4_OK) {
			const uint8_t status = (data[0] & 0b1100'0000) >> 6;

			if (status != (uint8_t)STATUS::Fault_Detected) {
				_retries = 1; // enable retries during operation
				return PX4_OK;
			}

			PX4_ERR("probe status: %X", status);
		}

		px4_usleep(2000);
	}

	return PX4_ERROR;
}

int ELVX::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	if (ret == PX4_OK) {
		ScheduleNow();
	}

	return ret;
}

void ELVX::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_fault_perf);
}

void ELVX::RunImpl()
{
	perf_begin(_sample_perf);
	_timestamp_sample = hrt_absolute_time();

	// Read-only 4-byte data fetch (Honeywell HSC/SSC / AllSensors family).
	uint8_t data[4] {};
	int ret = transfer(nullptr, 0, &data[0], sizeof(data));
	perf_end(_sample_perf);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);

	} else {
		// Status bits [7:6]
		const uint8_t status = (data[0] & 0b1100'0000) >> 6;

		// 14-bit pressure: Bridge Data [13:8] (data[0][5:0]) + [7:0] (data[1])
		const uint16_t pressure_raw = ((data[0] & 0b0011'1111) << 8) + data[1];

		// 11-bit temperature: Temperature Data [10:3] (data[2]) + [2:0] (data[3][7:5])
		const int16_t temperature_raw = ((data[2] << 8) + (0b1110'0000 & data[3])) >> 5;

		if (status == (uint8_t)STATUS::Fault_Detected) {
			perf_count(_fault_perf);

		} else if (status == (uint8_t)STATUS::Normal_Operation) {
			// ELVX-L05D: +-5 inH2O, differential offset 8192 counts (mid-scale).
			//   P[Pa] = (raw - 8192) * 1.25 * 2 * 5 / 16384 * 248.84(inH2O->Pa)
			//         = (raw - 8192) * 0.18984
			// Constant 0.18984 cross-checked against AllSensors DLVR transfer
			// function (docs/elvx_driver_research.md sec.A.2; algorithm
			// reference only, no GPL code copied).
			// Sign: bench port-press test showed Hi(+) port pressure -> negative
			// raw delta on this build, so negate to make forward airflow
			// (Hi/total > Lo/static) read POSITIVE per PX4 convention.
			const float diff_press_pa = -(pressure_raw - 8192) * 0.18984f;

			const float temperature_c = ((200.f * temperature_raw) / 2047.f) - 50.f;

			differential_pressure_s differential_pressure{};
			differential_pressure.timestamp_sample = _timestamp_sample;
			differential_pressure.device_id = get_device_id();
			differential_pressure.differential_pressure_pa = diff_press_pa;
			differential_pressure.temperature = temperature_c;
			differential_pressure.error_count = perf_event_count(_comms_errors);
			differential_pressure.timestamp = hrt_absolute_time();
			_differential_pressure_pub.publish(differential_pressure);

		} else {
			// Stale_Data or Reserved: no fresh conversion this cycle, skip.
			PX4_DEBUG("status:%X P:%X T:%X", status, pressure_raw, temperature_raw);
		}
	}

	ScheduleDelayed(20_ms); // 50 Hz
}
