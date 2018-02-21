#include "pid.h"

#define FALSE 0
#define TRUE 1
#define ERROR_VALUE -1.0

struct controller_params
{
	double Kp;
	double Ki;
	double Kd;
	double outputMax;
	double outputMin;
	double setpoint;
};

struct controller_params params[MAX_CONTROLLERS];

struct controller_values
{
	double actual;
	double errorP;
	double errorI;
	double errorD;
	double previous;
	double output;
	int initialised;
};

struct controller_values values[MAX_CONTROLLERS];


void initControllers()
{
	for (int index = 0; index < MAX_CONTROLLERS; index++)
	{
		params[index].Kp = 1.0;
		params[index].Ki = 0.0;
		params[index].Kd = 0.0;
		params[index].outputMax = 100.0;
		params[index].outputMin = 0.0;
		params[index].setpoint = 32.0;

		values[index].actual = 32.0;
		values[index].errorP = 0.0;
		values[index].errorP = 0.0;
		values[index].errorP = 0.0;
		values[index].previous = 0.0;
		values[index].output = 0.0;
		values[index].initialised = FALSE;
	}
}


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

void setTemperature(int index, double value)
{
	if (index < MAX_CONTROLLERS)
	{
			values[index].actual = value;
	}

}

void updateController(int index)
{
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
		values[index].initialised = TRUE;
	}

	// save current process value
	values[index].previous = values[index].actual;

}

double getPulseWidth(int index)
{
	return values[index].output;
}



// debugging functions only
double getErrorP(int index)
{
	return (params[index].Kp * values[index].errorP);
}

double getErrorI(int index)
{
	return values[index].errorI;
}

double getErrorD(int index)
{
	return (params[index].Kd * values[index].errorD);
}

