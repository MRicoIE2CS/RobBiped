/*
 * StateMachine.cpp
 *
 * Created: 15/04/2023 14:30:36
 *  Author: MRICO
 */

#include "Executor.h"

void Executor::state_machine_switch()
{
	read_commands();

	switch (state_number)
	{
		case 0:
			if (squats_on)
			{
				state_number++;
				break;
			}
			break;
		case 1:
			if (!squats_on)
			{
				state_number--;
				break;
			}
			break;
		case 2:
			break;
	}
}

void Executor::state_machine_execution()
{
	switch (state_number)
	{
		case 0:
			state0_execution();
			break;
		case 1:
			state1_execution();
			break;
		case 2:
			state2_execution();
			break;
	}
}

void Executor::read_commands()
{
}

void Executor::state0_execution()
{
}

void Executor::state1_execution()
{
}

void Executor::state2_execution()
{
	
}
