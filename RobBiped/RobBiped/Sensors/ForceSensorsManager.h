/*
 * ForceSensorsManager.h
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

#ifndef _FORCESENSORSMANAGER_h
#define _FORCESENSORSMANAGER_h

#include "arduino.h"

#include "../Main/I_PeriodicTask.h"
#include "../Main/Configs.h"
#include "../UserInput/Command.h"
#include "HX711/multiple_HX711.h"
#include "../Utils/Filters/ExponentialFilter.h"
#include "../Utils/Filters/ExponentialFilterWithPeakRejection.h"
#include "../Utils/LinearAlgebra/ArduinoEigenDense.h"

using Eigen::Vector2d;

using namespace Configuration;

class GlobalKinematics; // Forward declaration

// This class manages the use of multiple HX711 ICs in one object, filtering, adjusting and interpreting the obtained values.
// As the use of multiple HX711 and interpretation of the measured magnitudes hardly depends on the HW setup configured for the robot,
// it is needed to modify this class each time the number of hx711 modules or channels is modified.
// It is considered easier to modify the class than to program a configurable class for all possible configurations.
class ForceSensorsManager : public I_PeriodicTask
{
private:

	// Serial Commands pointer
	Command* command_;

	Multiple_HX711 multiple_hx711_;

	// Readings
	int32_t value_LeftFoot_LeftBack_;
	int32_t value_LeftFoot_LeftFront_;
	int32_t value_LeftFoot_RightBack_;
	int32_t value_LeftFoot_RightFront_;
	int32_t value_RightFoot_LeftBack_;
	int32_t value_RightFoot_LeftFront_;
	int32_t value_RightFoot_RightBack_;
	int32_t value_RightFoot_RightFront_;

	// One filter per each measured magnitude
	ExpFilterPeakReject filter_LeftFoot_LeftBack_;
	ExpFilterPeakReject filter_LeftFoot_LeftFront_;
	ExpFilterPeakReject filter_LeftFoot_RightBack_;
	ExpFilterPeakReject filter_LeftFoot_RightFront_;
	ExpFilterPeakReject filter_RightFoot_LeftBack_;
	ExpFilterPeakReject filter_RightFoot_LeftFront_;
	ExpFilterPeakReject filter_RightFoot_RightBack_;
	ExpFilterPeakReject filter_RightFoot_RightFront_;

	Configuration::Configs::ForceSensors *config_;
	Configuration::Configs::Kinematics *kinematics_config_;

	double *calibration_LeftFoot_LeftFront_cell_;
	double *calibration_LeftFoot_RightFront_cell_;
	double *calibration_LeftFoot_LeftBack_cell_;
	double *calibration_LeftFoot_RightBack_cell_;
	double *calibration_RightFoot_LeftFront_cell_;
	double *calibration_RightFoot_RightFront_cell_;
	double *calibration_RightFoot_LeftBack_cell_;
	double *calibration_RightFoot_RightBack_cell_;

	bool is_tare_left_performed_ = true;
	bool is_tare_right_performed_ = true;

	// Touch detection thresholds (hysteresis), in gr, for detection of ground touch for the feet 
	uint32_t touch_detection_up_threshold_gr_;
	uint32_t touch_detection_down_threshold_gr_;
	// Ground touching flags
	bool is_left_foot_touching_ground_ = false;
	bool is_right_foot_touching_ground_ = false;
	void check_touch_detection();

	int16_t *separation_FrontBack_mm_;
	int16_t *separation_LeftRight_mm_;
	double zmp_left_foot_x_mm_;
	double zmp_left_foot_y_mm_;
	double zmp_right_foot_x_mm_;
	double zmp_right_foot_y_mm_;
	ExpFilter filter_zmp_left_foot_x_mm_;
	ExpFilter filter_zmp_left_foot_y_mm_;
	ExpFilter filter_zmp_right_foot_x_mm_;
	ExpFilter filter_zmp_right_foot_y_mm_;
	void calculate_local_ZMP();
	double global_zmp_x_mm_ = 0.0;
	double global_zmp_y_mm_ = 0.0;
	ExpFilter filter_zmp_x_mm_;
	ExpFilter filter_zmp_y_mm_;

	void print_values();
	void print_ZMP();

public:

	void assoc_config(Configs::ForceSensors &_config);

	void init();

	bool update();

	// Readings obtention in gr.
	int32_t getValue_gr_LeftFoot_LeftFrontSensor();
	int32_t getValue_gr_LeftFoot_RightFrontSensor();
	int32_t getValue_gr_LeftFoot_LeftBackSensor();
	int32_t getValue_gr_LeftFoot_RightBackSensor();
	int32_t getValue_gr_RightFoot_LeftFrontSensor();
	int32_t getValue_gr_RightFoot_RightFrontSensor();
	int32_t getValue_gr_RightFoot_LeftBackSensor();
	int32_t getValue_gr_RightFoot_RightBackSensor();

	uint32_t get_last_elapsed_time_between_readings();

	void tare_LeftFoot();
	void tare_RightFoot();

	bool is_tare_left_performed();
	bool is_tare_right_performed();

	void get_values_ZMP_LeftFoot(double& x_mm, double& y_mm);
	Vector2d get_values_ZMP_LeftFoot();
	void get_values_ZMP_RightFoot(double& x_mm, double& y_mm);
	Vector2d get_values_ZMP_RightFoot();
	
	bool is_left_foot_touching_ground();
	bool is_right_foot_touching_ground();

	void compute_global_ZMP(GlobalKinematics *_global_kinematics);
	void get_global_ZMP(double &_x_mm, double &_y_mm);
	Vector2d get_global_ZMP();
};

#endif
