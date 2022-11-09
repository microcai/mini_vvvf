
#pragma once

#include <communication/Commander.h>
#include <common/base_classes/BLDCDriver.h>
#include <BLDCMotor.h>

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)
#include <BluetoothSerial.h>
extern BluetoothSerial SerialBT;
#endif

extern Commander command;
extern BLDCDriver* driver;
extern BLDCMotor motor;

float calc_volt_from_herts(float hertz);
void idle_iteration();

void setup_bt();
void bt_run_command();
void setup_wifi();

template <typename T>
auto circularBufferSum(const T& buf) -> decltype(buf[0])
{
	using index_t = typename T::index_t;
	auto s		  = buf[0];
	for (index_t i = 1; i < buf.size(); i++)
	{
		s += buf[i];
	}
	return s;
}

template <typename T>
T clamp(T input, T min, T max)
{
	if (input >= max)
		return max;
	else if (input <= min)
		return min;
	return input;
}

