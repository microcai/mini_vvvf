
#include <thread>
#include <SimpleFOC.h>
#include <esp_task_wdt.h>
#include <esp_sleep.h>

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(2);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(25,26,27,33);

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

float pi = 3.14159265358979;
//target variable
float target_hz = 10;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd)
{
  command.scalar(&target_hz, cmd);
  //motor.voltage_limit = target_hz / 100 * driver.voltage_power_supply;
}

TaskHandle_t Task_idle;

//Task1code: blinks an LED every 1000 ms
static void Task_idle_code( void * pvParameters )
{
  for(;;){
    vTaskDelay(10);
    command.run();
    vTaskDelay(10);
    esp_task_wdt_reset();
  }
}

void setup()
{
  Serial.begin(250000);

  sensor.init();

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = 6;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // currnet = resistance*voltage, so try to be well under 1Amp
  motor.voltage_limit = 3;   // [V]

  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  motor.init();

  // add target command T
  command.add('T', doTarget, "target hz");

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");


  std::thread().native_handle();

  xTaskGetIdleTaskHandle();

  xTaskCreatePinnedToCore(
                    Task_idle_code,   /* Task function. */
                    "idle_task",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task_idle,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */

  _delay(100);
}


void loop()
{
  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  motor.move(target_hz * 2 * pi);
  // user communication
}
