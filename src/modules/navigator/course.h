/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
/**
 * @file course.h
 *
 * Course/Heading mode: maintain constant course or heading, altitude, and airspeed.
 * On activation, tracks course if position is available, otherwise heading.
 * Falls back to heading hold automatically on position loss.
 *
 * Accepts MAV_CMD_DO_CHANGE_ALTITUDE, VEHICLE_CMD_DO_REPOSITION, DO_CHANGE_COURSE, CONDITION_YAW and DO_CHANGE_SPEED commands.
 * - DO_CHANGE_COURSE: sets course (ground track direction, requires valid position).
 * - CONDITION_YAW: sets heading (airspeed direction), switches to heading control.
 */

#pragma once

#include "navigator_mode.h"
#include "mission_block.h"

class Course : public MissionBlock
{
public:
	Course(Navigator *navigator);
	~Course() = default;

	void on_activation() override;
	void on_active() override;

	/**
	 * Set course (ground track direction). Requires valid horizontal velocity.
	 * @param course_rad Course angle in radians, 0 = north
	 * @return true if horizontal velocity available and course set, false otherwise
	 */
	bool set_course(float course_rad);

	/**
	 * Set heading (nose/airspeed direction). position-independent. Switches to heading control.
	 * @param heading_rad Heading angle in radians, 0 = north
	 */
	void set_heading(float heading_rad);

	void set_altitude(float alt_amsl) { _altitude = alt_amsl; update_setpoint_triplet(); }
	void set_airspeed(float airspeed) { _airspeed = airspeed; update_setpoint_triplet(); }

private:
	/**
	 * Update the position setpoint triplet to fly the current course or heading.
	 */
	void update_setpoint_triplet();

	enum class ControlMode { Course, Heading };

	ControlMode _control_mode{ControlMode::Course};
	float _course{0.f};		///< [rad] current course bearing (ground track)
	float _heading{0.f};		///< [rad] current heading setpoint (nose direction)
	float _altitude{0.f};		///< [m] AMSL altitude setpoint
	float _airspeed{-1.f};		///< [m/s] cruising speed setpoint (-1 = default)
};
