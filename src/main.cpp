
#include <thread>
#include <SimpleFOC.h>
#include <esp_task_wdt.h>
#include <esp_sleep.h>

#define IOUTA 34
#define IOUTB 35
#define IOUTC 32

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(1);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(25,26,27,33);

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

LowsideCurrentSense current_sense = LowsideCurrentSense(0.005f, 10, IOUTA, IOUTB, IOUTC);

float pi = 3.14159265358979;
//target variable
float target_hz = 10;

float motor_volt = 220;
float motor_freq = 60;

// instantiate the commander
Commander command = Commander(Serial);
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
  float volt_hz = std::abs(req_hz);
  motor.voltage_limit = std::max(12.0f, std::min(driver.voltage_limit, volt_hz / motor_freq * motor_volt * 1.4f));
}

TaskHandle_t Task_idle;

static void Task_idle_code( void * pvParameters )
{
  for(;;){
    vTaskDelay(10);
    command.run();
    esp_task_wdt_reset();
    vTaskDelay(10);
  }
}

static void Task_print_status_code( void * pvParameters )
{
  for(;;){
    vTaskDelay(1000);
    Serial.printf("current is %f \t power req: %f \t volt: %f \n", current_sense.getPhaseCurrents(), target_hz, motor.voltage_limit);
    esp_task_wdt_reset();
  }
}

static void printCurrent(char* cmd)
{
  Serial.printf("current is %f\n", current_sense.getPhaseCurrents().a);
}

void setup()
{
  Serial.begin(250000);

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
  motor.voltage_limit = 24;   // [V]
  motor.current_limit = 1; // [A]

  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  motor.init();

  // add target command T
  command.add('T', doTarget, "target hz");
  command.add('C', printCurrent, "print current");

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");


  std::thread().native_handle();

  xTaskCreatePinnedToCore(
                    Task_idle_code,   /* Task function. */
                    "idle_task",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    2,           /* priority of the task */
                    &Task_idle,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */

  xTaskCreatePinnedToCore(
                    Task_print_status_code,   /* Task function. */
                    "Task_print_status",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task_idle,      /* Task handle to keep track of created task */
                    0);            /* pin task to core 0 */

  _delay(100);
}


void loop()
{
  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  motor.move(target_hz * 2 * pi);
  // user communication
}
