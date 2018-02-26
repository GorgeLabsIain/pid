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
void setKp(int index, float value);
void setKi(int index, float value);
void setKd(int index, float value);
void setOutputMax(int index, float value);
void setOutputMin(int index, float value);
void setSetpoint(int index, float value);

// Functions to check configuration parameters - check after attempting to set them
float getKp(int index);
float getKi(int index);
float getKd(int index);
float getOutputMax(int index);
float getOutputMin(int index);
float getSetpoint(int index);

// Push the temperature readings from the WTS into the PID controller - call every time a WTS temperature reading is received
void setTemperature(int index, float value);

// Execute the PID controller - call every Duty Cycle period
void updateController(int index);

// Get the PID controller output - call every Duty Cycle period, then update the PWM module
float getPulseWidth(int index);