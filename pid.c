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
#define ERROR_VALUE -1.0  // Functions will return this if any error occured

// PID controller parameters - these values are expected to change rarely
struct controller_params
{
	double Kp;          // Proportional weight
	double Ki;          // Integral weight
	double Kd;          // Derivative weight
	double outputMax;   // Limit on maximum controller output
	double outputMin;   // Limit on minimum controller output
	double setpoint;    // Desired temperature setpoint
};

// PID controller working variables - these values are expected to change regularly
struct controller_values
{
	double actual;      // Current temperature value
	double errorP;      // Current proportional error
	double errorI;      // Cumulative integral error
	double errorD;      // Current derivative error
	double previous;    // Previous temperature value (required to calc derivative error)
	double output;      // Pulse width controller output
	int initialised;    // Flag set after 1st iteration of the controller (must sample temperature twice to calc derivative error)
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
		params[index].Kp = 1.0;
		params[index].Ki = 0.0;
		params[index].Kd = 0.0;
		params[index].outputMax = 100.0;
		params[index].outputMin = 0.0;
		params[index].setpoint = 32.0;
	}
}

// Call this function when on entry to Operation state
void initControllers()
{
	// for each controller set up some initial working values
	for (int index = 0; index < MAX_CONTROLLERS; index++)
	{
		values[index].actual = 32.0;
		values[index].errorP = 0.0;
		values[index].errorP = 0.0;
		values[index].errorP = 0.0;
		values[index].previous = 0.0;
		values[index].output = 0.0;
		values[index].initialised = FALSE;
	}
}


// Functions to set configuration parameters - should the user change any of them
void setKp(int index, double value)
{
	if (index < MAX_CONTROLLERS)
	{
		if (value > 0.0 && params[index].Kp != value)
		{
			params[index].Kp = value;
		}
	}
}

void setKi(int index, double value)
{
	if (index < MAX_CONTROLLERS)
	{
		if (value > 0.0 && params[index].Ki != value)
		{
			params[index].Ki = value;
		}
	}
}

void setKd(int index, double value)
{
	if (index < MAX_CONTROLLERS)
	{
		if (value > 0.0 && params[index].Ki != value)
		{
			params[index].Ki = value;
		}
	}
}

void setOutputMax(int index, double value)
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

void setOutputMin(int index, double value)
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

void setSetpoint(int index, double value)
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
double getKp(int index)
{
	double value = ERROR_VALUE;
	if (index < MAX_CONTROLLERS)
	{
		value = params[index].Kp;
	}
	return value;
}

double getKi(int index)
{
	double value = ERROR_VALUE;
	if (index < MAX_CONTROLLERS)
	{
		value = params[index].Ki;
	}
	return value;
}

double getKd(int index)
{
	double value = ERROR_VALUE;
	if (index < MAX_CONTROLLERS)
	{
		value = params[index].Kd;
	}
	return value;
}

double getOutputMax(int index)
{
	double value = ERROR_VALUE;
	if (index < MAX_CONTROLLERS)
	{
		value = params[index].outputMax;
	}
	return value;
}

double getOutputMin(int index)
{
	double value = ERROR_VALUE;
	if (index < MAX_CONTROLLERS)
	{
		value = params[index].outputMin;
	}
	return value;
}

double getSetpoint(int index)
{
	double value = ERROR_VALUE;
	if (index < MAX_CONTROLLERS)
	{
		value = params[index].setpoint;
	}
	return value;
}


// Push the temperature readings from the WTS into the PID controller - call every time a WTS temperature reading is received
void setTemperature(int index, double value)
{
	if (index < MAX_CONTROLLERS)
	{
			values[index].actual = value;
	}

}

// Execute the PID controller - call every Duty Cycle period
void updateController(int index)
{
	// check if there is a previous temperature reading, so that the derivative error can be calculated
	if (values[index].initialised == TRUE)
	{
		// calculate error terms
		values[index].errorP = params[index].setpoint - values[index].actual;
		values[index].errorI += params[index].Ki * values[index].errorP;
		values[index].errorD = values[index].actual - values[index].previous;

		// check error bounds
		if (values[index].errorI>params[index].outputMax)
		{
			values[index].errorI = params[index].outputMax;
		}
		if (values[index].errorI<params[index].outputMin)
		{
			values[index].errorI = params[index].outputMin;
		}

		// calculate controller output
		values[index].output = (params[index].Kp*values[index].errorP) + values[index].errorI - (params[index].Kd*values[index].errorD);

		// check output bounds
		if (values[index].output>params[index].outputMax)
		{
			values[index].output = params[index].outputMax;
		}
		if (values[index].output<params[index].outputMin)
		{
			values[index].output = params[index].outputMin;
		}
	}
	else
	{
		// first execution of the controller, about to save the temperature value, so will be good to go next time around
		values[index].initialised = TRUE;
	}

	// save current temperature value for next execution of the controller
	values[index].previous = values[index].actual;
}

// Get the PID controller output - call every Duty Cycle period, then update the PWM module
double getPulseWidth(int index)
{
	return values[index].output;
}
