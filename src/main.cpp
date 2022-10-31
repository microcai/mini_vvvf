
#include "BluetoothSerial.h"
#include <SimpleFOC.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>
#include <thread>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

#define IOUTA 34
#define IOUTB 35
#define IOUTC 32

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(1);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, 33);

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

LowsideCurrentSense current_sense = LowsideCurrentSense(0.005f, 10, IOUTA, IOUTB, IOUTC);

float pi = 3.14159265358979;
// target variable
float target_hz = 10;

float motor_volt = 220;
float motor_freq = 60;

// instantiate the commander
Commander command = Commander(Serial);
Commander commandBT = Commander(SerialBT);

TaskHandle_t Task_idle;

static void Task_idle_code(void* pvParameters)
{
	for (;;)
	{
		vTaskDelay(10);
		if (Serial.available())
			command.run();
    if (SerialBT.available())
    {
      commandBT.run();
    }
		esp_task_wdt_reset();
		vTaskDelay(10);
	}
}

TaskHandle_t Task_print_status;

static void Task_print_status_code(void* pvParameters)
{
	for (;;)
	{
		vTaskDelay(1000);
		Serial.printf("current is %f \t power req: %f \t volt: %f \n",
			current_sense.getPhaseCurrents(),
			target_hz,
			motor.voltage_limit);
		esp_task_wdt_reset();
	}
}

static void printCurrent(char* cmd, Stream & serial) { serial.printf("current is %f\n", current_sense.getPhaseCurrents().a); }

void doTarget(char* cmd, Commander& commander)
{
	float req_hz = 3;
	commander.scalar(&req_hz, cmd);
	if (req_hz >= 0)
	{
		if (req_hz < 3)
			target_hz = 3;
		else
			target_hz = req_hz;
	}
	else if (req_hz > -3)
	{
		target_hz = -3;
	}
	else
		target_hz = req_hz;
	float volt_hz		= std::abs(req_hz);
	motor.voltage_limit = std::max(12.0f, std::min(driver.voltage_limit, volt_hz / motor_freq * motor_volt * 1.4f));
}

void setup()
{
	Serial.begin(250000);
	SerialBT.begin("mini_VFD"); // Bluetooth device name

	sensor.init();

	// driver config
	// power supply voltage [V]
	driver.voltage_power_supply = 24;
	// limit the maximal dc voltage the driver can set
	// as a protection measure for the low-resistance motors
	// this value is fixed on startup
	driver.voltage_limit = 24;
	driver.pwm_frequency = 12000;
	driver.init();
	current_sense.linkDriver(&driver);

	current_sense.init();

	// link the motor and the driver
	motor.linkDriver(&driver);
	motor.linkCurrentSense(&current_sense);

	// limiting motor movements
	// limit the voltage to be set to the motor
	// start very low for high resistance motors
	// currnet = resistance*voltage, so try to be well under 1Amp
	motor.voltage_limit = 24; // [V]
	motor.current_limit = 1;  // [A]

	// open loop control config
	motor.controller = MotionControlType::velocity_openloop;

	// init motor hardware
	motor.init();

	// add target command T
	command.add('T', [](char* cmd){ doTarget(cmd, command); }, "target hz");
	command.add('C', [](char* cmd){ printCurrent(cmd, Serial);}, "print current");
	commandBT.add('T', [](char* cmd){ doTarget(cmd, commandBT); }, "target hz");
	commandBT.add('C', [](char* cmd){ printCurrent(cmd, SerialBT);}, "print current");

	Serial.println("Motor ready!");
	Serial.println("Set target velocity [rad/s]");

	std::thread().native_handle();

	xTaskCreatePinnedToCore(Task_idle_code, /* Task function. */
		"idle_task",						/* name of task. */
		10000,								/* Stack size of task */
		NULL,								/* parameter of the task */
		2,									/* priority of the task */
		&Task_idle,							/* Task handle to keep track of created task */
		0);									/* pin task to core 0 */

	xTaskCreatePinnedToCore(Task_print_status_code, /* Task function. */
		"Task_print_status",						/* name of task. */
		10000,										/* Stack size of task */
		NULL,										/* parameter of the task */
		1,											/* priority of the task */
		&Task_print_status,									/* Task handle to keep track of created task */
		0);											/* pin task to core 0 */

	_delay(100);
}

void loop()
{
	// open loop velocity movement
	// using motor.voltage_limit and motor.velocity_limit
	motor.move(target_hz * 2 * pi);
	// user communication
}
