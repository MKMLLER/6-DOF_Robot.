#pragma once

struct Point2 {
  int16_t x;
  int16_t y;
};

extern volatile bool AngleSet;
extern volatile bool CoordSetDone;
extern volatile bool CoordSetMode;
extern volatile bool AngleGoal;
extern volatile bool Path1;
extern volatile bool Path2;
extern volatile bool line;
extern volatile bool CoordAgree;
extern volatile bool X_move;
extern volatile int X1, Y1, Z1;
extern volatile bool angle_J1;
extern volatile int angle;
extern volatile int angle1;
extern volatile int angle2;
extern Point2 p[1000];



void StartSerialMonitor();