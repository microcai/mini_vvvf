
#include <Arduino.h>
#include <CircularBuffer.h>
#include <algorithm>
#include <numeric>

#ifdef BOARD_VESC
#ifdef ENABLE_THREADS
#include <STM32FreeRTOS.h>
#define xTaskCreatePinnedToCore(a, b, c, d, e, f, g) xTaskCreate(a, b, c, d, 0, f)
#endif
#endif
#ifdef CONFIG_ESP_TASK_WDT
#include <esp_task_wdt.h>
#endif
// extern const int adc_channel_io_map[SOC_ADC_PERIPH_NUM][SOC_ADC_MAX_CHANNEL_NUM];

// int IRAM_ATTR _adc_channel_io_map[SOC_ADC_PERIPH_NUM][SOC_ADC_MAX_CHANNEL_NUM];

#include "board_conf.hpp"

#include <SimpleFOC.h>

#include "global.hpp"

// MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// BUG, LowsideCurrentSense crash with WiFi/BT
#if defined(HAS_InlineCurrentSense)
InlineCurrentSense current_sense = InlineCurrentSense(Shunt_Resistor, Sensor_GAIN, IOUTA, IOUTB, IOUTC);
#elif defined(HAS_LowSideCurrentSense)
LowsideCurrentSense current_sense = LowsideCurrentSense(Shunt_Resistor, Sensor_GAIN, IOUTA, IOUTB, IOUTC);
#endif

float pi = 3.14159265358979;
// target variable
#ifdef BOARD_VESC
float target_hz = 0;
#else
float target_hz = 0;
#endif

float motor_volt = 24;
float motor_freq = 40;



float filtered_current = 0.0f;

void print_status(Stream& serial_port)
{
	if (motor.enabled)
	{
		serial_port.printf("current is %dmA \t power freq: %dhz \t volt: %dV\n",
			static_cast<int>(filtered_current * 1000.0f),
			static_cast<int>(target_hz),
			static_cast<int>(motor.voltage_limit));
	}
	else
	{
		serial_port.printf("motor stoped\n");
	}
}

#ifdef ENABLE_THREADS
static TaskHandle_t Task_idle;
static TaskHandle_t Task_print_status;

static void Task_idle_code(void* pvParameters)
{
	for (;;)
	{
		delay(10);
		idle_iteration();
	}
}

static void Task_print_status_code(void* pvParameters)
{
	for (;;)
	{
		delay(10);
		print_status(Serial);
		delay(10);
#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)
		print_status(SerialBT);
#endif
	}
}
#endif

static void set_motor_speed(float req_hz)
{
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
	motor.voltage_limit = calc_volt_from_herts(volt_hz);
}

static void doTarget(char* cmd)
{
	float req_hz = 3;
	command.scalar(&req_hz, cmd);
	set_motor_speed(req_hz);
}

static void doVoltage(char* cmd)
{
	float req_voltage = 0;
	command.scalar(&req_voltage, cmd);
	if (req_voltage <= 0)
	{
		req_voltage = 0;
		motor.disable();
	}
	else
	{
		motor.enable();
		motor.voltage_limit = req_voltage;
	}
}

void setup()
{
	// for (uint8_t i = 0; i < SOC_ADC_PERIPH_NUM; i++)
	// {
	// 	for (uint8_t j = 0; j < SOC_ADC_MAX_CHANNEL_NUM; j++)
	// 	{
	// 		_adc_channel_io_map[i][j] = adc_channel_io_map[i][j];
	// 	}
	// }
#ifdef BOARD_VESC
	SystemClock_Config();
#endif

	Serial.begin(UART_BAUD_RATE);

	Serial.println("starting");

	setup_wifi();

	// driver config
	// power supply voltage [V]
	driver->voltage_power_supply = 32;
	// limit the maximal dc voltage the driver can set
	// as a protection measure for the low-resistance motors
	// this value is fixed on startup
	driver->voltage_limit = 32;
	driver->pwm_frequency = 48000;
	driver->init();
	Serial.println("driver inited");

#ifdef HAS_LowSideCurrentSense
	current_sense.linkDriver(&driver);
#endif

#if HAS_CurrentSense
	current_sense.init(); // current_sense.driverSync(&driver); //已经包含在了init函数中，所以屏蔽。simpleFOC版本v2.2.2
	#ifndef BOARD_VESC
	current_sense.skip_align = true; // 对于ESP32drive可以跳过电流检测，同时增益乘以-1，也就是把这四行都打开，
	current_sense.gain_a *= -1; // 在之前的simpleFOC版本中，是不需要乘以-1的，v2.2.2修改了底层代码，所以必须乘以-1
	current_sense.gain_b *= -1;
	current_sense.gain_c *= -1; // 这四行也可以都屏蔽了，通过代码检测电流增益。但是偶尔会检测错误
	#endif
	motor.linkCurrentSense(&current_sense);
	Serial.println("current_senser linked");
#endif

	motor.linkDriver(driver);

	// limiting motor movements
	// limit the voltage to be set to the motor
	// start very low for high resistance motors
	// currnet = resistance*voltage, so try to be well under 1Amp
	motor.voltage_limit = calc_volt_from_herts(target_hz);
	motor.current_limit = 1; // [A]

	// open loop control config
	motor.controller = MotionControlType::velocity_openloop;

	// init motor hardware;
	motor.init();
	// add target command T
	command.add('T', doTarget, "target hz");
	command.add('V', doVoltage, "target volt");

	Serial.println("Motor ready!");
	Serial.println("Set target velocity [hz/s]");

#ifdef ENABLE_THREADS
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
		&Task_print_status,							/* Task handle to keep track of created task */
		0);											/* pin task to core 0 */
#endif

	delay(30);

	setup_bt();

	// xTaskCreatePinnedToCore([](void*) { SerialBT.begin("vfd"); vTaskDelete() }, /* Task function. */
	// 	"start_bt",												  /* name of task. */
	// 	10000,													  /* Stack size of task */
	// 	NULL,													  /* parameter of the task */
	// 	1,														  /* priority of the task */
	// 	&Task_start_bt,											  /* Task handle to keep track of created task */
	// 	0);														  /* pin task to core 0 */
	// SerialBT.begin("vfd");

//	vTaskStartScheduler();
#ifdef HAS_throttle
	pinMode(Throutle_PIN, INPUT_ANALOG);
#endif
}

#ifdef HAS_throttle
void read_throttle()
{
	int input = analogRead(Throutle_PIN);
	//uart_port.printf("analogRead out %d\n", input);

	if (input <= 5)
	{
		set_motor_speed(0);
		motor.disable();
	}
	else
	{
		set_motor_speed((clamp(input - 250, 0, 760)) / 760 * motor_freq);
		if (!motor.enabled)
			motor.enable();
	}
}
#endif

float calc_volt_from_herts(float hertz)
{
	return std::max(7.0f, std::min(driver->voltage_limit, hertz / motor_freq * motor_volt * 1.4f));
}

void loop()
{
	// open loop velocity movement
	// using motor.voltage_limit and motor.velocity_limit
	motor.loopFOC();
	motor.move(target_hz * 2 * pi);
	static long start_micros = 0;
	auto this_tp			 = micros();

#ifdef HAS_CurrentSense
	CircularBuffer<float, 500> current_samples;

	current_samples.push(current_sense.getDCCurrent());

	filtered_current = circularBufferSum(current_samples) / current_samples.size();
#endif

	// user communication
#ifndef ENABLE_THREADS
	command.run(Serial);

	if ((this_tp - start_micros) >= 1200000)
	{
		print_status(Serial);
		start_micros = this_tp;
	}
#ifdef HAS_throttle
	read_throttle();
#endif

#endif
}
