/**
 * @file main.cpp
 * @brief STM32 SimpleFOC basic test.
 */

// #define SERIAL_UART_INSTANCE 1 /* Comment out to use the default UART (i.e. ST-Link UART). */

#include <Arduino.h>
#include <SimpleFOC.h>
#define BAUDRATE (115200) /* Serial port baudrate. */

/* Mode */
// #define OPENLOOP

/* Motor selection. */
// #define QM4208
#define AK10_9

/* Motor parameters and Power  */
#if defined(AK10_9)
#define MOTOR_POLE_PAIRS (21)
#define MOTOR_PHASE_RESISTANCE (0.090) /* Unit in ohm. */
#define MOTOR_KV (100)                 /* Unit in rpm/V. */
#define PID_P (0.2)
#define PID_I (20)
#define PID_D (0)

/*Power*/
#define VOLTAGE_SUPPLY (22) /* Unit in V. */
#define CURRENT_LIMIT (1)   /* Unit in A. */

#elif defined(QM4208)
#define MOTOR_POLE_PAIRS (7)
#define MOTOR_PHASE_RESISTANCE (0.101) /* Unit in ohm. */
#define MOTOR_KV (380)                 /* Unit in rpm/V. */
#define PID_P (1)
#define PID_I (0.0001)
#define PID_D (0)

/*Power*/
#define VOLTAGE_SUPPLY (15)     /* Unit in V. */
#define CURRENT_LIMIT (0.00005) /* Unit in A. */
#else
#error No Motor Selected
#endif

/* DRV8302 pins. */
#if defined(NUCLEO_L432KC)
#define INH_A (9)  /* PA8. */
#define INH_B (A2) /* PA3. */
#define INH_C (A1) /* PA1. */

#define EN_GATE (2) /* PA12. */
#define OC_ADJ (3)  /* PB0. */
#define M_OC (4)    /* PB7. */
#define M_PWM (5)   /* PB6. */
/*
 * SPI SCLK: D13 pin (PB3).
 * SPI MISO: D12 pin (PB4).
 * SPI MOSI: D11 pin (PB5).
 */
#define AS5047P_SPI_CS (10) /* PA11. */
#elif defined(NUCLEO_F446RE) || defined(NUCLEO_F401RE)
#define INH_A (6)
#define INH_B (5)
#define INH_C (3)

#define EN_GATE (8)
#define OC_ADJ (7)
#define M_OC (PB13)
#define M_PWM (PB14)
/*
 * SPI SCLK: D13 pin.
 * SPI MISO: D12 pin.
 * SPI MOSI: D11 pin.
 * 黃色線在上CS、CLK、MOSI、MISO、-、+
 */
#define AS5047P_SPI_CS (10)
#else
#error No Board Selected
#endif

#define AS5047P_REG_ANGLECOM (0x3FFF) /* Measured angle with dynamic angle error compensation(DAEC). */
#define AS5047P_REG_ANGLEUNC (0x3FFE) /* Measured angle without DAEC. */

HardwareSerial Serial1(USART1);
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C, EN_GATE);
MagneticSensorSPI angleSensor = MagneticSensorSPI(AS5047P_SPI_CS, 14, AS5047P_REG_ANGLECOM);

Commander command = Commander(Serial);
void onMotor(char *cmd) { command.motor(&motor, cmd); }

void onReadAngle(char *)
{
#ifndef OPENLOOP
  angleSensor.update();
  float angle = angleSensor.getAngle();

  angle -= motor.sensor_offset;
  // angle *= -1;

  char sign;
  if (angle >= 0)
  {
    sign = '+';
  }
  else
  {
    sign = '-';
  }

  angle = fabs(angle); /* Absolute value. */

  Serial.printf("%c%d%d%d.%d%d%d\r\n",
                sign,
                (int)((int)(angle) / 100 % 10),
                (int)((int)(angle) / 10 % 10),
                (int)((int)(angle) / 1 % 10),
                (int)((int)(angle * 10) / 1 % 10),
                (int)((int)(angle * 100) / 1 % 10),
                (int)((int)(angle * 1000) / 1 % 10));

#endif
}

void drv8302Setup(void);

void setup()
{
  /* Communication setup. */
  Serial.begin(BAUDRATE);
  motor.useMonitoring(Serial);
  command.add('M', onMotor, "motor");
  command.add('A', onReadAngle, "angle");

#ifndef OPENLOOP
  /* Configure angle/Position sensor. */
  angleSensor.spi_mode = SPI_MODE1; /* CPOL=0, CPHA=1. */
  angleSensor.clock_speed = 1e6;    /* 10 MHz max. */
  angleSensor.init();
  motor.linkSensor(&angleSensor);
#endif

  /* Configure driver. */
  drv8302Setup();
  driver.pwm_frequency = 20000;
  driver.voltage_power_supply = VOLTAGE_SUPPLY;
  driver.init();
  motor.linkDriver(&driver);

  /* Configure motor parameters. */
  // motor.phase_resistance = MOTOR_PHASE_RESISTANCE;
  // motor.KV_rating = MOTOR_KV * 1.5f; /* SimpleFOC suggest to set the KV value provided to the library to 50-70% higher than the one given in the datasheet.. */

  motor.current_limit = CURRENT_LIMIT;
  // motor.motion_downsample = 5;

  /* Algorithms and controllers setup. */
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::voltage;

#ifdef OPENLOOP
  /* OpenLoop should operate in 10%~20% voltage supply */
  motor.controller = MotionControlType::velocity_openloop;
  motor.voltage_limit = VOLTAGE_SUPPLY * 0.2;
#else
  motor.controller = MotionControlType::angle;
  motor.voltage_limit = VOLTAGE_SUPPLY;
#endif

  /* Velocity control loop setup. */
  motor.PID_velocity.P = PID_P;
  motor.PID_velocity.I = PID_I;
  // motor.PID_velocity.D = PID_D;
  // motor.PID_velocity.output_ramp = 500; /* Unit in volts/s. */

  motor.LPF_velocity.Tf = 0.01;
  // motor.velocity_limit = 15; /* Unit in rad/s. */

  /* Angle/Position control loop setup. */
  motor.P_angle.P = 5;
  // motor.P_angle.I = 0.5;
  // motor.P_angle.D = 0.05;
  // motor.P_angle.output_ramp = 500; /* Acceleration limit(?), unit in rad/s^2. */

  motor.init();    /* Initialize motor. */
  motor.initFOC(); /* Start FOC and aligh encoder. */

#ifndef OPENLOOP
  angleSensor.update();
  motor.target = angleSensor.getAngle(); /* Set the initial target value. */
#else
  motor.target = 0;
#endif

  Serial.println(motor.target, 4); // 後面數字代表要輸出到小數點後第幾位

  Serial.println("All Ready!");
  _delay(1000);
}

void loop()
{
  motor.loopFOC(); /* Main FOC algorithm. */
  motor.move();    /* Motion control. */
  // motor.monitor();/* motor moniter */

  command.run();
}

/* DRV8302 specific setup. */
void drv8302Setup(void)
{
  /*
   * M_PWM: Mode selection pin for PWM input configuration.
   * - LOW: 6 PWM mode.
   * - HIGH: 3 PWM mode, only INH_x pins required.
   */
  pinMode(M_PWM, OUTPUT);
  digitalWrite(M_PWM, HIGH);

  /*
   * M_OC: Mode selection pin for over-current protection options.
   * - LOW: the gate driver will operate in a cycle-by-cycle current limiting mode.
   * - HIGH: the gate driver will shutdown the channel which detected an over-current event.
   */
  pinMode(M_OC, OUTPUT);
  digitalWrite(M_OC, LOW);

  /*
   * OD_ADJ: Overcurrent trip set pin.
   * Set HIGH for the maximum over-current limit possible,
   * better option would be to use voltage divisor to set exact value.
   */
  pinMode(OC_ADJ, OUTPUT);
  digitalWrite(OC_ADJ, HIGH);
}
