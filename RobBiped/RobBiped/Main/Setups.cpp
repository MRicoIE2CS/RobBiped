/*
 * Setups.cpp
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

#include "Executor.h"

void Executor::associations()
{
	user_input_.assoc_GPIO(config_.user_input_pins);
	//TODO: Associate buttons to servo updater calibration
	//servoUpdater.assocButtons(config.gpio.thinButton1, config.gpio.thinButton2);
	force_sensors_manager_.assoc_config(config_.force_sensors);
	gyroscope_accelerometer_manager_.assoc_config(config_.gyro_acc);
}
