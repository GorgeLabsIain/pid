/*
 * Copyright (c) 2018 Gorge Labs Limited.  All rights reserved.
 *
 * Digital PID Controller Revision 1.0
 *
*/

// limit number of controllers to 2
#define MAX_CONTROLLERS 2

// Call this function when in Initialisation state
void initParameters();

// Call this function when on entry to Operation state
void initControllers();

// Functions to set configuration parameters - should the user change any of them
void setKp(int index, double value);
void setKi(int index, double value);
void setKd(int index, double value);
void setOutputMax(int index, double value);
void setOutputMin(int index, double value);
void setSetpoint(int index, double value);

// Functions to check configuration parameters - check after attempting to set them
double getKp(int index);
double getKi(int index);
double getKd(int index);
double getOutputMax(int index);
double getOutputMin(int index);
double getSetpoint(int index);

// Push the temperature readings from the WTS into the PID controller - call every time a WTS temperature reading is received
void setTemperature(int index, double value);

// Execute the PID controller - call every Duty Cycle period
void updateController(int index);

// Get the PID controller output - call every Duty Cycle period, then update the PWM module
double getPulseWidth(int index);