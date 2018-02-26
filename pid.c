/*
 * Copyright (c) 2018 Gorge Labs Limited.  All rights reserved.
 *
 * Digital PID Controller Revision 1.0
 *
 */

#include "pid.h"

// Static definitions
#define FALSE 0
#define TRUE 1
#define ERROR_VALUE -1.0       // Functions will return this if any error occured
#define WATCHDOG_LIMIT 3       // Number of duty cycles without temperature reading before stopping controller

// PID controller parameters - these values are expected to change rarely
struct controller_params
{
	float Kp;                  // Proportional weight
	float Ki;                  // Integral weight
	float Kd;                  // Derivative weight
	float outputMax;           // Limit on maximum controller output
	float outputMin;           // Limit on minimum controller output
	float setpoint;            // Desired temperature setpoint
};

// PID controller working variables - these values are expected to change regularly
struct controller_values
{
	float actual;              // Current temperature value
	float errorP;              // Current proportional error
	float errorI;              // Cumulative integral error
	float errorD;              // Current derivative error
	float previous;            // Previous temperature value (required to calc derivative error)
	float output;              // Pulse width controller output
	unsigned int watchdog;     // Watchdog counter to detect when actual temperature samples interrupted
	unsigned int initialised;  // Flag set after 1st iteration of the controller (must sample temperature twice to calc derivative error)
};

// For each PID controller there is an instance of the parameters and an instance of the working variables 
struct controller_params params[MAX_CONTROLLERS];  // store in non-volatile memory
struct controller_values values[MAX_CONTROLLERS];  // store in RAM

// Call this function when in Initialisation state
void initParameters()
{
	// for each controller set up some default parameter values
	for (int index = 0; index < MAX_CONTROLLERS; index++)
	{
		params[index].Kp = 1.0f;
		params[index].Ki = 0.0f;
		params[index].Kd = 0.0f;
		params[index].outputMax = 100.0f;
		params[index].outputMin = 0.0f;
		params[index].setpoint = 32.0f;
	}
}

// Call this function when on entry to Operation state
void initControllers()
{
	// for each controller set up some initial working values
	for (int index = 0; index < MAX_CONTROLLERS; index++)
	{
		values[index].actual = 32.0f;
		values[index].errorP = 0.0f;
		values[index].errorP = 0.0f;
		values[index].errorP = 0.0f;
		values[index].previous = values[index].actual;
		values[index].output = params[index].outputMin;
		values[index].watchdog = 0;
		values[index].initialised = FALSE;
	}
}


// Functions to set configuration parameters - should the user change any of them
void setKp(int index, float value)
{
	if (index < MAX_CONTROLLERS)
	{
		if (value > 0.0f && params[index].Kp != value)
		{
			params[index].Kp = value;
		}
	}
}

void setKi(int index, float value)
{
	if (index < MAX_CONTROLLERS)
	{
		if (value > 0.0f && params[index].Ki != value)
		{
			params[index].Ki = value;
		}
	}
}

void setKd(int index, float value)
{
	if (index < MAX_CONTROLLERS)
	{
		if (value > 0.0f && params[index].Ki != value)
		{
			params[index].Ki = value;
		}
	}
}

void setOutputMax(int index, float value)
{
	if (index < MAX_CONTROLLERS)
	{
		if (value > params[index].outputMin && params[index].outputMax != value)
		{
			params[index].outputMax = value;
		}
		if (values[index].errorI > params[index].outputMax)
		{
			values[index].errorI = params[index].outputMax;
		}
	}
}

void setOutputMin(int index, float value)
{
	if (index < MAX_CONTROLLERS)
	{
		if (value < params[index].outputMax && params[index].outputMin != value)
		{
			params[index].outputMin = value;
		}
		if (values[index].errorI < params[index].outputMin)
		{
			values[index].errorI = params[index].outputMin;
		}
	}
}

void setSetpoint(int index, float value)
{
	if (index < MAX_CONTROLLERS)
	{
		if (params[index].setpoint != value)
		{
			params[index].setpoint = value;
		}
	}
}

// Functions to check configuration parameters - check after attempting to set them
float getKp(int index)
{
	float value = ERROR_VALUE;
	if (index < MAX_CONTROLLERS)
	{
		value = params[index].Kp;
	}
	return value;
}

float getKi(int index)
{
	float value = ERROR_VALUE;
	if (index < MAX_CONTROLLERS)
	{
		value = params[index].Ki;
	}
	return value;
}

float getKd(int index)
{
	float value = ERROR_VALUE;
	if (index < MAX_CONTROLLERS)
	{
		value = params[index].Kd;
	}
	return value;
}

float getOutputMax(int index)
{
	float value = ERROR_VALUE;
	if (index < MAX_CONTROLLERS)
	{
		value = params[index].outputMax;
	}
	return value;
}

float getOutputMin(int index)
{
	float value = ERROR_VALUE;
	if (index < MAX_CONTROLLERS)
	{
		value = params[index].outputMin;
	}
	return value;
}

float getSetpoint(int index)
{
	float value = ERROR_VALUE;
	if (index < MAX_CONTROLLERS)
	{
		value = params[index].setpoint;
	}
	return value;
}


// Push the temperature readings from the WTS into the PID controller - call every time a WTS temperature reading is received
void setTemperature(int index, float value)
{
	if (index < MAX_CONTROLLERS)
	{
			values[index].actual = value;
			values[index].watchdog = 0;
	}

}

// Execute the PID controller - call every Duty Cycle period
void updateController(int index)
{
	// check if temperature samples have been received
	if (values[index].watchdog < WATCHDOG_LIMIT)
	{
		// save current temperature value for next execution of the controller
		values[index].previous = values[index].actual;
		
		// check if first execution of the controller
		if (values[index].initialised == FALSE)
		{
			// toggle flag so that controller is executed at the next duty cycle
			values[index].initialised = TRUE;
		}
		// not first execution, so there is a previous temperature reading which enables derivative error to be calculated
		else
		{
			// calculate error terms
			values[index].errorP = params[index].setpoint - values[index].actual;
			values[index].errorI += params[index].Ki * values[index].errorP;
			values[index].errorD = values[index].actual - values[index].previous;

			// check error bounds
			if (values[index].errorI > params[index].outputMax)
			{
				values[index].errorI = params[index].outputMax;
			}
			if (values[index].errorI < params[index].outputMin)
			{
				values[index].errorI = params[index].outputMin;
			}

			// calculate controller output
			values[index].output = (params[index].Kp*values[index].errorP) + values[index].errorI - (params[index].Kd*values[index].errorD);

			// check output bounds
			if (values[index].output > params[index].outputMax)
			{
				values[index].output = params[index].outputMax;
			}
			if (values[index].output < params[index].outputMin)
			{
				values[index].output = params[index].outputMin;
			}
		}
	}
	else
	{
		// no temperature samples, so set output to minimum allowed
		values[index].output = params[index].outputMin;
        // set flag so that when temperature samples start re-arriving controller is re-initialised
		values[index].initialised = FALSE;
	}
	// increment watchdog counter
	values[index].watchdog += 1;
}

// Get the PID controller output - call every Duty Cycle period, then update the PWM module
float getPulseWidth(int index)
{
	return values[index].output;
}
