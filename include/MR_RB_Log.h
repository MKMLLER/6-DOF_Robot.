#pragma once
#ifndef NAMES_H
#define NAMES_H
#include <Arduino.h>
extern int j4;
#include <AccelStepper.h>

extern AccelStepper stepper1;
extern AccelStepper stepper2;
extern AccelStepper stepper3;
extern AccelStepper stepper4;
extern AccelStepper stepper56_1;
extern AccelStepper stepper56_2;

struct Point {
    int x;
    int y;
};

float AllTurns( uint16_t currentAngle, uint16_t &prevAngle  );
bool readLine(float &out);
int GoaltoSteps( float Goal, float driver, float Reduction);
float GetVectorLength( float X, float Y );
float sq1( float x );
float Get3DVectorLen(float x, float y, float z);
void SetStepAngle(float x, float y, float z);

extern Point points[];
extern Point circleP[];
extern const int pointsCount;



#endif