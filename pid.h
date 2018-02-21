#pragma once

#define MAX_CONTROLLERS 2

void initControllers();
double getKp(int index);
double getKi(int index);
double getKd(int index);
double getOutputMax(int index);
double getOutputMin(int index);
double getSetpoint(int index);
void setKp(int index, double value);
void setKi(int index, double value);
void setKd(int index, double value); 
void setOutputMax(int index, double value);
void setOutputMin(int index, double value);
void setSetpoint(int index, double value);

void setTemperature(int index, double value);
void updateController(int index);
double getPulseWidth(int index);