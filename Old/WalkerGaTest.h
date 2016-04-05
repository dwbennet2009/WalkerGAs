#ifndef WalkerGaTest_H
#define WalkerGaTest_H

#include <iostream>
#include <time.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#include "SegmentHex.h"

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

double frame[25];
double angRLA[25], angRLC[25], angRLP[25];
double angRRA[25], angRRC[25], angRRP[25];
double angFLA[25], angFLC[25], angFLP[25];
double angFRA[25], angFRC[25], angFRP[25];

enum _objectCategory
{
   EDGE = 0x0001,
   BODY = 0x0002,
   LEFT = 0x0004,
   RIGHT = 0x0008,
};

class WalkerGaTest: public Test
{
   SegmentHex segLoBL, segLoBR, segUpBL, segUpBR;
   SegmentHex segLoFL, segLoFR, segUpFL, segUpFR;
   SegmentHex segBody;

   struct timespec tv;
   double time_init, time_now;
   
public:
   WalkerGaTest();
   void Step(Settings *settings);
   static Test* Create();
};

#endif
