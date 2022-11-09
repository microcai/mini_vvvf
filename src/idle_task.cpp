
#include <Arduino.h>

#include "board_conf.hpp"

#include "global.hpp"


void idle_iteration()
{
	if (Serial.available())
		command.run(Serial);
    bt_run_command();
	delay(10);

#ifdef HAS_throttle
	read_throttle();
#endif
}
