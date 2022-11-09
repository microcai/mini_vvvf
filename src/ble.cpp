
#include <Arduino.h>

#include "global.hpp"

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)


#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

void setup_bt()
{
	SerialBT.begin("vfd");
	Serial.println("BT started");
}

void bt_run_command()
{
#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)
	if (SerialBT.available())
	{
		command.run(SerialBT);
	}
#endif

}

#else

void setup_bt(){}
void bt_run_command(){}

#endif
