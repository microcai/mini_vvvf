
#include <BluetoothSerial.h>
#include <SimpleFOC.h>
#include <esp_task_wdt.h>

// extern const int adc_channel_io_map[SOC_ADC_PERIPH_NUM][SOC_ADC_MAX_CHANNEL_NUM];

// int IRAM_ATTR _adc_channel_io_map[SOC_ADC_PERIPH_NUM][SOC_ADC_MAX_CHANNEL_NUM];

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

#define IOUTA 34
#define IOUTB 35
#define IOUTC 32

float calc_volt_from_herts(float hertz);

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor		  = BLDCMotor(1);
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, 33);

// MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// BUG, LowsideCurrentSense crash with WiFi/BT
// LowsideCurrentSense current_sense = LowsideCurrentSense(0.005f, 10, IOUTA, IOUTB, IOUTC);

float pi = 3.14159265358979;
// target variable
float target_hz = 10;

float motor_volt = 200;
float motor_freq = 70;

// instantiate the commander
Commander command = Commander{};
BluetoothSerial SerialBT;

static TaskHandle_t Task_idle;
static TaskHandle_t Task_print_status;
static TaskHandle_t Task_start_bt;

static void Task_idle_code(void* pvParameters)
{
	for (;;)
	{
		vTaskDelay(10);
		if (Serial.available())
			command.run(Serial);
		if (SerialBT.available())
		{
			command.run(SerialBT);
		}
		esp_task_wdt_reset();
		vTaskDelay(10);
	}
}

static void Task_print_status_code(void* pvParameters)
{
	for (;;)
	{
		vTaskDelay(1000);
		Serial.printf("current is %fmA \t power freq: %fhz \t volt: %fV \n",
			0.0f, // current_sense.getDCCurrent() * 1000.0f,
			target_hz,
			motor.voltage_limit);
		esp_task_wdt_reset();
		if (SerialBT.availableForWrite())
			SerialBT.printf("current is %fmA \t power freq: %fhz \t volt: %fV \n",
				0.0f, // current_sense.getDCCurrent() * 1000.0f,
				target_hz,
				motor.voltage_limit);
	}
}

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
	motor.voltage_limit = calc_volt_from_herts(req_hz);
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

	Serial.begin(250000);

	Serial.println("starting");

	// driver config
	// power supply voltage [V]
	driver.voltage_power_supply = 24;
	// limit the maximal dc voltage the driver can set
	// as a protection measure for the low-resistance motors
	// this value is fixed on startup
	driver.voltage_limit = 24;
	driver.pwm_frequency = 12000;
	driver.init();
	Serial.println("driver inited");

	// current_sense.linkDriver(&driver);

	// current_sense.init();
	Serial.println("current_senser inited");

	// link the motor and the driver
	motor.linkDriver(&driver);
	// motor.linkCurrentSense(&current_sense);
	Serial.println("current_senser linked");

	vTaskDelay(30);
	Serial.println("starting BT");
	vTaskDelay(30);
	Serial.println("BT started");

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
	Serial.println("motor inited");

	// add target command T
	command.add('T', doTarget, "target hz");

	Serial.println("Motor ready!");
	Serial.println("Set target velocity [rad/s]");

	Serial.println("BT started");

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

	vTaskDelay(30);

	SerialBT.begin("vfd");

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
	motor.move(target_hz * 2 * pi);
	// user communication
}

float calc_volt_from_herts(float hertz)
{
	return std::max(12.0f, std::min(driver.voltage_limit, hertz / motor_freq * motor_volt * 1.4f));
}
