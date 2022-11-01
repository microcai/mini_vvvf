
#include <Arduino.h>

// #ifdef BOARD_VESC
// #include <STM32FreeRTOS.h>
// #define xTaskCreatePinnedToCore(a,b,c,d,e,f,g) xTaskCreate(a,b,c,d,e,f)
// #endif
#ifdef CONFIG_ESP_TASK_WDT
#include <esp_task_wdt.h>
#endif
// extern const int adc_channel_io_map[SOC_ADC_PERIPH_NUM][SOC_ADC_MAX_CHANNEL_NUM];

// int IRAM_ATTR _adc_channel_io_map[SOC_ADC_PERIPH_NUM][SOC_ADC_MAX_CHANNEL_NUM];

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)
#include <BluetoothSerial.h>
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif
#endif

#ifdef BOARD_VESC
#include "vesc_pins.h"
#else
#include "esp32_pins.h"
#endif

#ifdef BOARD_VESC
//HardwareSerial uart_port(PC6, PC7);
#define uart_port Serial
#else
#define uart_port Serial
#endif

#include <SimpleFOC.h>

float calc_volt_from_herts(float hertz);

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor		  = BLDCMotor(1);
#ifdef BOARD_VESC
BLDCDriver6PWM driver(H1, L1, H2, L2, H3, L3, EN_GATE);
#else
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, 33);
#endif
// MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// BUG, LowsideCurrentSense crash with WiFi/BT
#ifdef HAS_LowCurrentSense
LowsideCurrentSense current_sense = LowsideCurrentSense(0.005f, 10, IOUTA, IOUTB, IOUTC);
#endif

float pi = 3.14159265358979;
// target variable
float target_hz = 7;

float motor_volt = 100;
float motor_freq = 140;

// instantiate the commander
Commander command = Commander{};

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)
BluetoothSerial SerialBT;
#endif

#ifndef BOARD_VESC

static TaskHandle_t Task_idle;
static TaskHandle_t Task_print_status;

static void Task_idle_code(void* pvParameters)
{
	for (;;)
	{
		delay(10);
		if (uart_port.available())
			command.run(Serial);
#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)
		if (SerialBT.available())
		{
			command.run(SerialBT);
		}
#endif
#ifdef CONFIG_ESP_TASK_WDT
		esp_task_wdt_reset();
#endif
		delay(10);
	}
}

static void Task_print_status_code(void* pvParameters)
{
	for (;;)
	{
		delay(10);
		uart_port.printf("current is %fmA \t power freq: %fhz \t volt: %fV \n",
			0.0f, // current_sense.getDCCurrent() * 1000.0f,
			target_hz,
			motor.voltage_limit);
		delay(10);
#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)
		SerialBT.printf("current is %fmA \t power freq: %fhz \t volt: %fV \n",
			0.0f, // current_sense.getDCCurrent() * 1000.0f,
			target_hz,
			motor.voltage_limit);
#endif			
	}
}

#endif

void doTarget(char* cmd)
{
	float req_hz = 3;
	command.scalar(&req_hz, cmd);
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

	uart_port.begin(UART_BAUD_RATE);

	uart_port.println("starting");

	// driver config
	// power supply voltage [V]
	driver.voltage_power_supply = 24;
	// limit the maximal dc voltage the driver can set
	// as a protection measure for the low-resistance motors
	// this value is fixed on startup
	driver.voltage_limit = 24;
	driver.pwm_frequency = 24000;
	driver.init();
	uart_port.println("driver inited");

#ifdef HAS_LowCurrentSense
	current_sense.linkDriver(&driver);
	current_sense.init();
	uart_port.println("current_senser inited");
	// link the motor and the driver
	motor.linkCurrentSense(&current_sense);
	uart_port.println("current_senser linked");
#endif

	motor.linkDriver(&driver);

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

	uart_port.println("Motor ready!");
	uart_port.println("Set target velocity [rad/s]");

#ifndef BOARD_VESC

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

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)
	SerialBT.begin("vfd");
	uart_port.println("BT started");
#endif
	// xTaskCreatePinnedToCore([](void*) { SerialBT.begin("vfd"); vTaskDelete() }, /* Task function. */
	// 	"start_bt",												  /* name of task. */
	// 	10000,													  /* Stack size of task */
	// 	NULL,													  /* parameter of the task */
	// 	1,														  /* priority of the task */
	// 	&Task_start_bt,											  /* Task handle to keep track of created task */
	// 	0);														  /* pin task to core 0 */
																  // SerialBT.begin("vfd");
}

void loop()
{
	// open loop velocity movement
	// using motor.voltage_limit and motor.velocity_limit
	motor.loopFOC();
	motor.move(target_hz * 2 * pi);
	// user communication
#ifdef BOARD_VESC
	command.run(Serial);
#endif
}

float calc_volt_from_herts(float hertz)
{
	return std::max(7.0f, std::min(driver.voltage_limit, hertz / motor_freq * motor_volt * 1.4f));
}
