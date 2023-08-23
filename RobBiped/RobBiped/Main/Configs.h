/*
 * Configs.h
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

#ifndef _CONFIGS_h
#define _CONFIGS_h

#include "Arduino.h"

#include <vector>

namespace Configuration
{

enum class JointsNames {LeftFootRoll = 0, LeftFootPitch = 1, LeftKnee = 2, LeftHipPitch = 3,
						LeftHipRoll = 4, LeftShoulderSagittal = 5, LeftShoulderAmplitude = 6,
						Unused1 = 7, Unused2 = 8,
						RightShoulderAmplitude = 9, RightShoulderSagittal = 10, RightHipRoll = 11,
						RightHipPitch = 12, RightKnee = 13, RightFootPitch = 14, RightFootRoll = 15 };

static const uint8_t hx711_number = 8;	// forceSensors configuration

struct Configs 
{
	struct UserInputPins {
		uint8_t forward_button = 2;
		uint8_t back_button = 4;
		uint8_t potentiometer1 = 36;
		uint8_t potentiometer2 = 39;
		}user_input_pins;

	struct ForceSensors {
		// Changes in the configuration of the load cells and the channels used might need modifications in ForceSensorsManager class.
		struct GPIO {
			uint8_t clock = 27;
			uint8_t dINs[hx711_number] = {33, 16, 32, 17, 34, 19, 35, 18};	// Order is important! Search upper for hx711_number static const
			}gpio;
		struct ActiveChannels {
			bool _Ax128 = false;
			bool _Ax64 = true;
			bool _Bx32 = false;
			}active_channels;
		double filter_time_constant_ms = 40;
		uint16_t filter_threshold_value = 500000;
		double calibration_LeftFoot_LeftFront_cell = 0.048;	// Grams per unit
		double calibration_LeftFoot_RightFront_cell = 0.048;
		double calibration_LeftFoot_LeftBack_cell = 0.048;
		double calibration_LeftFoot_RightBack_cell = 0.048;
		double calibration_RightFoot_LeftFront_cell = 0.048;
		double calibration_RightFoot_RightFront_cell = 0.048;
		double calibration_RightFoot_LeftBack_cell = 0.048;
		double calibration_RightFoot_RightBack_cell = 0.048;
		struct Location {
			// Warning: This dimentions should be the same as the ones defined in Configs::kinematics.feet_dimensions
			int16_t frontBack_separation = 115; //103;
			int16_t leftRight_separation = 60; //37;
			}location_mm;
		uint32_t touch_detection_up_threshold_gr = 175;
		uint32_t touch_detection_down_threshold_gr = 125;
		}force_sensors;		// Dependent on static const Configuration::hx711_number

	struct GyroscpeAccelerometer {
		struct Offsets {
			int16_t ax_o = -174;	// Accelerometer offsets
			int16_t ay_o = -193;
			int16_t az_o = 924;
			int16_t gx_o = -2;		// Gyroscope offsets
			int16_t gy_o = -44;
			int16_t gz_o = -14;
			}offsets;
		uint32_t filter_time_constant_ms = 40;
		double Kf_complement_filter = 0.99;	// Constant of the complement filter for the calculation of the inclination angle [0.0-1.0]
		}gyro_acc;

	struct Kinematics {
		// Body distances:
		double d_hip_width = 90.4;		// Lateral distance between both hip roll joints (lh)
		double height_hip = 85;			// Height between joints of the hip (l1)
		double height_hip_knee = 79;	// Height between knee and the first joint of the hip (thigh) (l2)
		double height_knee_ankle = 63;	// Height between ankle and knee (calf) (l3)
		double height_ankle = 45;		// Height between joints of the ankle (l4)
		double height_foot = 35;		// Height between floor and first joint of the ankle (l5)
		double d_lateral_foot = 13;		// Lateral distance between foot center and the axis of the first ankle joint (a1)
		// CM location :
		double height_CM_from_hip = -50.0;	// Vertical distance from hip joints to estimated CM height
		double Kfilter_CM_location = 0.98;//0.955;	// Constant of the complement filter for the estimation of the CM location [0.0-1.0]
		double Kfilter_CM_velocity = 0.99;	// Constant of the complement filter for the estimation of the CM location [0.0-1.0]
		// Hip height limits
		double limit_up_hip_height = 300;
		double limit_down_hip_height = 235;
		struct FeetDimensions {
			// Warning: This dimentions should be the same as the ones defined in Configs::force_sensors.location_mm
			int16_t frontBack_separation = 115;
			int16_t leftRight_separation = 60;
			}feet_dimensions;
		struct HipRollCompensation {
			double negative_compensation;
			double positive_compensation;
			};
		HipRollCompensation left_hip_roll_compensation = { 0.075, 0.05 };
		HipRollCompensation right_hip_roll_compensation = { -0.05, -0.075 };
		}kinematics;

	struct Control {

		struct CMTracking {
			// WARNING! The limit of memory allocation is 160KB!
			// Keep these files light.
			String CM_path_y_filename = "/y_com_ref.txt";
			String dCM_path_y_filename = "/dy_com_ref.txt";
			String ddCM_path_y_filename = "/ddy_com_ref.txt";
			String dddCM_path_y_filename = "/dddy_com_ref.txt";
			String CM_path_x_filename = "/x_com_ref.txt";
			String dCM_path_x_filename = "/dx_com_ref.txt";
			String ddCM_path_x_filename = "/ddx_com_ref.txt";
			String dddCM_path_x_filename = "/dddx_com_ref.txt";
			uint32_t paths_sampletime_ms = 10;
			double Tra_x = 0.005;	// Rising time of ZMP actuation dynamics
			double Tra_y = 0.005;	// Rising time of ZMP actuation dynamics
			// dx: for Tra_x=0.005 [ 14, 230, 120] [ 90, 300, 200] 
			double d0_x = 90;
			double d1_x = 300;
			double d2_x = 200;
			double d0_y = 90;
			double d1_y = 300;
			double d2_y = 200;
			}cm_tracking;

		struct TorsoPosture {
			// PID constants
			double kp = 0.01;
			double ki = 0.0;
			double kd = 0.0;//0.1;//0.25;
			// Anti-windup constant
			double k_windup = 0.5;
			// Setpoint weighting constants [0.0 - 1.0]
			double proportional_setpoint_weight = 1.0;
			double derivative_setpoint_weight = 0.0;
			// Deadband compensation
			double negative_db_compensation_rad = -0.0;//-0.05;
			double positive_db_compensation_rad = 0.05;//0.05;
			// Derivative filter time constant
			uint32_t derivative_time_constant_ms_ = 40;
			// Saturation limits
			double lower_saturation_degrees = -45;
			double upper_saturation_degrees = 45;
			}torso_posture;

		struct ZMPTracking_x {
			struct FeedforwardCurve {
				std::vector<double> curve_points_x = { -55, -45, 45, 55 };
				std::vector<double> curve_points_y = { 0.2, 0.1, -0.1, -0.2 };//{ 0.2, 0.05, -0.05, -0.1 };//{ 0.3, 0.1, -0.05, -0.2 };//{ 0.1, 0.05, -0.05, -0.1 };
				}feedforward_curve;
			struct DeadbandCompensation {
				// Limiting value at which desired ZMP requires negative or positive DB compensation
				double db_delimiting_value;
				double negative_db_compensation_rad;
				double positive_db_compensation_rad;
				};
			DeadbandCompensation leftfoot_deadband_compensation = { 0.0, 0.05, 0.07 };
			DeadbandCompensation rightfoot_deadband_compensation = { 0.0, 0.05, 0.07 };
			struct PID {
				double kp = 0.0;//0.0005;
				double ki = 0.0;
				double kd = 0.0;
				// Anti-windup constant
				double k_windup = 5;
				// Setpoint weighting constants
				double proportional_setpoint_weight = 1.0;
				double derivative_setpoint_weight = 0.0;
				// Saturation limits
				double lower_saturation_degrees = -20;
				double upper_saturation_degrees = 20;
				}pid;
			}x_zmp_tracking;

		struct ZMPTracking_y {
			struct FeedforwardCurve {
				std::vector<double> curve_points_x;
				std::vector<double> curve_points_y;
				};
			FeedforwardCurve left_feedforward_curve = {{ -30, -10, 20, 30 }, { 0.1, 0.0, -0.02, -0.05 }};
			FeedforwardCurve right_feedforward_curve = {{ -30, -20, 10, 30 }, { 0.05, 0.02, 0.0, -0.1 }};
			struct DeadbandCompensation {
				// Limiting value at which desired ZMP requires negative or positive DB compensation
				double db_delimiting_value;
				double negative_db_compensation_rad;
				double positive_db_compensation_rad;
				};
			DeadbandCompensation leftfoot_deadband_compensation = { -10, 0.08, 0.1 };//{ -10, -0.025, 0.025 };
			DeadbandCompensation rightfoot_deadband_compensation = { 10, -0.1, -0.08 };//{ 10, -0.025, 0.025 };
			struct PID {
				double kp = 0.0012;
				double ki = 0.0;
				double kd = 0.0;
				// Anti-windup constant
				double k_windup = 5;
				// Setpoint weighting constants
				double proportional_setpoint_weight = 1.0;
				double derivative_setpoint_weight = 0.0;
				// Saturation limits
				double lower_saturation_degrees = -10;
				double upper_saturation_degrees = 10;
				}pid;
			}y_zmp_tracking;

		}control;
};

} // End namespace Configuration

#endif
