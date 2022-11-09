
#include <Arduino.h>
#include <SimpleFOC.h>

#include "global.hpp"

// instantiate the commander
Commander command = Commander{};


#ifdef BOARD_VESC
BLDCDriver6PWM g_driver(H1, L1, H2, L2, H3, L3, EN_GATE);
#else
BLDCDriver3PWM g_driver			  = BLDCDriver3PWM(25, 26, 27, 33);

#endif

BLDCDriver * driver = &g_driver;

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(1);
