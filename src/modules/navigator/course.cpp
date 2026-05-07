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
 * @file course.cpp
 *
 * Course/Heading mode: maintain constant course or heading, altitude, and airspeed.
 */

#include "course.h"
#include "navigator.h"

#include <matrix/math.hpp>

Course::Course(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_COURSE)
{
}

void
Course::on_activation()
{
	const vehicle_local_position_s *lpos = _navigator->get_local_position();

	// Capture current heading (nose direction)
	_heading = lpos->heading;
	_altitude = _navigator->get_global_position()->alt;
	_airspeed = -1.f; // default airspeed

	if (lpos->v_xy_valid) {
		_course = matrix::wrap_2pi(atan2f(lpos->vy, lpos->vx));
		_control_mode = ControlMode::Course;

	} else {
		// No valid velocity estimate: fall back to heading mode (nose direction only).
		_control_mode = ControlMode::Heading;
	}

	_navigator->reset_cruising_speed();

	update_setpoint_triplet();
}

void
Course::on_active()
{
	// If horizontal velocity estimate becomes invalid while in course mode, fall back to heading mode
	if (_control_mode == ControlMode::Course && !_navigator->get_local_position()->v_xy_valid) {
		_heading = _navigator->get_local_position()->heading;
		_control_mode = ControlMode::Heading;
		update_setpoint_triplet();
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Course hold: velocity lost, holding current heading\t");
		events::send(events::ID("navigator_course_gps_lost"), {events::Log::Error, events::LogInternal::Info},
			     "Course hold: velocity estimate lost, switching to heading mode");
	}
}

void
Course::set_heading(float heading_rad)
{
	_heading = heading_rad;
	_control_mode = ControlMode::Heading;
	update_setpoint_triplet();
}

bool
Course::set_course(float course_rad)
{
	if (!_navigator->get_local_position()->v_xy_valid) {
		// No valid velocity estimate - cannot compute or maintain a ground track
		return false;
	}

	_course = course_rad;
	_control_mode = ControlMode::Course;
	update_setpoint_triplet();
	return true;
}

void
Course::update_setpoint_triplet()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	pos_sp_triplet->previous.valid = false;

	pos_sp_triplet->current.valid = true;
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	pos_sp_triplet->current.alt = _altitude;

	if (_control_mode == ControlMode::Heading) {
		// Heading mode: control nose direction (yaw).
		// lat/lon must be finite for the setpoint to be valid;
		// use current position if available, otherwise dummy values (not used by the controller in heading mode).
		if (_navigator->get_local_position()->xy_global) {
			pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
			pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;

		} else {
			pos_sp_triplet->current.lat = 0.0;
			pos_sp_triplet->current.lon = 0.0;
		}

		pos_sp_triplet->current.yaw = _heading;
		pos_sp_triplet->current.course = NAN;

	} else {
		// Course mode: control ground track.
		// lat/lon must be finite for the setpoint to be valid;
		// use current position if available (including during dead-reckoning), otherwise dummy values.
		if (_navigator->get_local_position()->xy_global) {
			pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
			pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;

		} else {
			pos_sp_triplet->current.lat = 0.0;
			pos_sp_triplet->current.lon = 0.0;
		}

		pos_sp_triplet->current.yaw = NAN;
		pos_sp_triplet->current.course = _course;
	}

	pos_sp_triplet->current.cruising_speed = _airspeed;
	pos_sp_triplet->current.cruising_throttle = NAN;
	pos_sp_triplet->current.loiter_radius = NAN;
	pos_sp_triplet->current.acceptance_radius = _navigator->get_acceptance_radius();
	pos_sp_triplet->current.timestamp = hrt_absolute_time();

	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}


