/*
 * GlobalStabilization.cpp
 *
 * Copyright 2023 Mikel Rico Abajo (https://github.com/MRicoIE2CS)

 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at

 * http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "GlobalStabilization.h"

void Control::GlobalStabilization::assoc_config(Configuration::Configs::Control::CMTracking &_config)
{
	config_ = &_config;
}

void Control::GlobalStabilization::assoc_sensors(ForceSensorsManager &_force_sensors_manager, GyroscopeAccelerometerManager &_gyroscope_accelerometer_manager)
{
	gyroacc_sensor_ = &_gyroscope_accelerometer_manager;
	force_sensor_ = &_force_sensors_manager;
}

void Control::GlobalStabilization::set_mode(Mode &_mode)
{
	mode_ = _mode;
}

void Control::GlobalStabilization::reset_trajectory()
{
	is_runnning = false;
}

Vector2d Control::GlobalStabilization::compute_ZMP_action()
{
	
}

Vector2d Control::GlobalStabilization::get_ZMP_action()
{
	
}

