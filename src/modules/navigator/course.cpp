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

Course::Course(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_COURSE)
{
}

void
Course::on_activation()
{
	// Capture current state
	_heading = _navigator->get_local_position()->heading;
	_altitude = _navigator->get_global_position()->alt;
	_airspeed = -1.f; // default airspeed

	// Select best mode based on GPS availability
	if (_navigator->gps_position_valid()) {
		// GPS available: use course mode (ground track control)
		_course = _navigator->get_local_position()->heading;
		_use_heading = false;

	} else {
		// No GPS: use heading mode (nose direction only)
		_use_heading = true;
	}

	_navigator->reset_cruising_speed();

	update_course_setpoint();
}

void
Course::on_active()
{
	// Check for incoming vehicle commands
	// Commands are dispatched from navigator_main, which sets fields on this object
	// before calling on_active(). Nothing to poll here - setpoint updates are
	// triggered by set_course(), set_heading(), set_altitude(), set_airspeed() calls from navigator_main.
}

bool
Course::set_course(float course_rad)
{
	if (!_navigator->gps_position_valid()) {
		// No valid GPS - cannot use course mode
		return false;
	}

	_course = course_rad;
	_use_heading = false;
	update_course_setpoint();
	return true;
}

void
Course::update_course_setpoint()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	pos_sp_triplet->previous.valid = false;

	pos_sp_triplet->current.valid = true;
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	pos_sp_triplet->current.alt = _altitude;

	if (_use_heading) {
		// Heading mode: control nose direction (yaw)
		// Use dummy lat/lon if GPS not available - they're not used for heading control
		// but need to be finite for the setpoint to be considered valid
		if (_navigator->get_local_position()->xy_global) {
			pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
			pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;

		} else {
			// No GPS - use dummy position (not used in heading mode anyway)
			pos_sp_triplet->current.lat = 0.0;
			pos_sp_triplet->current.lon = 0.0;
		}

		pos_sp_triplet->current.yaw = _heading;
		pos_sp_triplet->current.course = NAN;

	} else {
		// Course mode: control ground track (requires GPS, checked by set_course())
		pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
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


