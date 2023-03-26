/*
 * RobBiped.ino
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

#include <iostream>
#include <vector>

#include "Main/Executor.h"
#include "UserInput/Command.h"

#include "Utils/Kinematics/ForwardKinematics.h"
#include "Utils/Kinematics/GeometricInverseKinematics.h"
#include "Utils/LinearAlgebra/ArduinoEigenDense.h"

using Eigen::Matrix4d;
using Eigen::Vector4d;

using Eigen::IOFormat;

Executor executor;

Command* serial_command = Command::get_instance();

void setup()
{
	Serial.begin(500000);
	
	executor.init();
	
	while (!serial_command->commands.init)
	{
		Serial.println("type in 'init' to initialize");
		serial_command->listen_for_commands();
		delay(1000);
	}
	Serial.println("Initialize execution!");
	
	IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
	
	const double l1 = 85;
	const double l2 = 80;
	const double l3 = 65;
	const double l4 = 45;
	const double l5 = 35;
	const double a4 = 13;
	
	double leg_length_mm = 37;
	double forward_angle_rad = -0.7;
	double lateral_angle_rad = 0.0;	// Not used
	Vector3d desired_position;
	Serial.println("Initial leg position: ");
	Serial.println("leg_length_mm: " + (String)leg_length_mm);
	Serial.println("forward_angle_rad: " + (String)forward_angle_rad);
	Serial.println("lateral_angle_rad: " + (String)lateral_angle_rad);
	
	InverseKinematics::Geometric::get_desired_position_from_length_and_angles(leg_length_mm, forward_angle_rad, lateral_angle_rad, desired_position);
	Serial.println("Desired position: ");
	std::cout << desired_position.format(CleanFmt) << std::endl;

	Vector2d links_lengths;
	links_lengths << l3, l2;
	double target_angle_rad_1;
	double target_angle_rad_2;
	InverseKinematics::Geometric::sagittal_two_links_inverse_kinematics(desired_position, links_lengths, target_angle_rad_1, target_angle_rad_2);
	Serial.println("target joint angles: " + (String)target_angle_rad_1 + ", " + (String)target_angle_rad_2);
	
	double q1 = 0;
	double q2 = 0;
	double q3 = target_angle_rad_2;
	double q4 = target_angle_rad_1;
	double q5 = 0;
	
	// Denavit-Hartenberg table
	std::vector<Vector4d> DH_table;
	Vector4d DH_row_1;
	DH_row_1 << q4, 0, l3, 0;
	Vector4d DH_row_2;
	DH_row_2 << q3, 0, l2, 0;
	DH_table.push_back(DH_row_1);
	DH_table.push_back(DH_row_2);
	
	Matrix4d TM;
	
	ForwardKinematics::get_overall_TM_from_DH_table(DH_table, TM);

	Serial.println("TF: ");
	std::cout << TM.format(CleanFmt) << std::endl;
	
	Vector3d current_position = TM.block<3,1>(0,3);
	Serial.println("current_position: ");
	std::cout << current_position.format(CleanFmt) << std::endl;
	leg_length_mm;
	forward_angle_rad; 
	lateral_angle_rad;
	ForwardKinematics::get_length_and_angles_from_position(current_position, leg_length_mm, forward_angle_rad, lateral_angle_rad);
	Serial.println("Final leg position: ");
	Serial.println("leg_length_mm: " + (String)leg_length_mm);
	Serial.println("forward_angle_rad: " + (String)forward_angle_rad);
	Serial.println("lateral_angle_rad: " + (String)lateral_angle_rad);
}

void loop()
{
	//executor.execution();
	
	//serial_command->listen_for_commands();
}
