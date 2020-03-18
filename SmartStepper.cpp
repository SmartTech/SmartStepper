#include "SmartStepper.h"

/**********************************************************
 *   constructor for four-pin version
 *   Sets which wires should control the motor.
 **********************************************************/
SmartStepper::SmartStepper(int number_of_steps, int motor_pin_1, int motor_pin_2,
                                      int motor_pin_3, int motor_pin_4)
{
  this->stepsPerRotate = number_of_steps; // total number of steps for this motor
  // Arduino pins for the motor control connection:
  this->motor_pin_1 = motor_pin_1;
  this->motor_pin_2 = motor_pin_2;
  this->motor_pin_3 = motor_pin_3;
  this->motor_pin_4 = motor_pin_4;
  this->begin();
}

/**********************************************************
 *   constructor for driver version
 *   Sets which wires should control the motor.
 **********************************************************/
SmartStepper::SmartStepper(int number_of_steps, int pin_step, int pin_dir, int pin_en)
{
  this->stepsPerRotate = number_of_steps; // total number of steps for this motor
  // Arduino pins for the motor control connection:
  this->pin_step = pin_step;
  this->pin_dir  = pin_dir;
  this->pin_en   = pin_en;
  this->begin();
}

void SmartStepper::begin()
{
  WiringPinMode mode = OUTPUT;
  if(power < STEPPER_MAX_PWM) mode = PWM;
  // setup the pins on the microcontroller:
  if(this->pin_step==PIN_UNCONNECTED) {
      pinMode(this->motor_pin_1, mode);
      pinMode(this->motor_pin_2, mode);
      pinMode(this->motor_pin_3, mode);
      pinMode(this->motor_pin_4, mode);
  } else {
	  pinMode(this->pin_step, mode);
	  pinMode(this->pin_dir,  mode);
  }
  
  if(this->pin_en==PIN_UNCONNECTED) {
	  pinMode(this->pin_en,   mode);
  }
  
  this->off();
  this->isRunning = false;
  this->currentSpeed = startSpeed;
  this->calcSpeed();
  
#ifdef _VARIANT_ARDUINO_STM32_
  HardwareTimer pwmTimer(3);
  pwmTimer.setPeriod(40);
#endif

}


void SmartStepper::setPulses(uint32_t pulses)
{
  this->stepsPerRotate = pulses;
  this->calcSpeed();
}

/**********************************************************
 * Sets the speed in revs per minute
 **********************************************************/
void SmartStepper::setSpeed(float whatSpeed)
{
  targetSpeed  = whatSpeed;
  this->calcSpeed();
}

void SmartStepper::setStartSpeed(float whatStartSpeed)
{
  startSpeed = whatStartSpeed;
  this->calcSpeed();
}

/**********************************************************
 * Sets the speed in revs per minute
 **********************************************************/
void SmartStepper::setAccel(float whatAccel)
{
  accel = abs(whatAccel);
  this->calcSpeed();
}

void SmartStepper::setPower(int whatPower)
{
  power = map(whatPower, 0, STEPPER_MAX_POWER, 0, STEPPER_MAX_PWM);
  if(whatPower==STEPPER_MAX_POWER) power = STEPPER_MAX_PWM;
  begin();
}

void SmartStepper::setPowerOff(bool whatState)
{
  needPowerOff = whatState;
}

void SmartStepper::setReverse(bool whatReverse)
{
  reverse = whatReverse;
}

/**********************************************************
 * Off stepper
 **********************************************************/
void SmartStepper::off()
{
	if(this->pin_step==PIN_UNCONNECTED) {
		pinMode(motor_pin_1, OUTPUT);
		pinMode(motor_pin_2, OUTPUT);
		pinMode(motor_pin_3, OUTPUT);
		pinMode(motor_pin_4, OUTPUT);
		digitalWrite(motor_pin_1, LOW);
		digitalWrite(motor_pin_2, LOW);
		digitalWrite(motor_pin_3, LOW);
		digitalWrite(motor_pin_4, LOW);
	} else {
		if(this->pin_en!=PIN_UNCONNECTED) {
			digitalWrite(this->pin_en, LOW);
		}
	}
}

/**********************************************************
 * Brake stepper
 **********************************************************/
void SmartStepper::brake()
{
	if(this->pin_step==PIN_UNCONNECTED) {
		pinMode(motor_pin_1, OUTPUT);
		pinMode(motor_pin_2, OUTPUT);
		pinMode(motor_pin_3, OUTPUT);
		pinMode(motor_pin_4, OUTPUT);
		digitalWrite(motor_pin_1, HIGH);
		digitalWrite(motor_pin_2, HIGH);
		digitalWrite(motor_pin_3, HIGH);
		digitalWrite(motor_pin_4, HIGH);
	}
}

/**********************************************************
 * Run stepper moving
 **********************************************************/
void SmartStepper::run()
{
  if(this->currentStep == this->targetStep) return;
  currentSpeed = startSpeed;
  this->isRunning = true;
  this->isStopping = false;
  if(this->pin_en==PIN_UNCONNECTED) {
	  digitalWrite(this->pin_en, HIGH);
  }
}

/**********************************************************
 * Stop stepper moving
 **********************************************************/
void SmartStepper::stop(uint8_t _breakPoint)
{
  if(this->leftSteps > this->brakePoint) {
	  if(_breakPoint) this->brakePoint = _breakPoint;
      this->leftSteps = this->brakePoint;
      this->isStopping = true;
  }
}

/**********************************************************
 * Set steps number for moving
 **********************************************************/
void SmartStepper::step(int number_of_steps, bool resetSteps)
{
  if(resetSteps) this->targetStep = this->currentStep = 0;
  this->targetStep += number_of_steps;// * (reverse ? -1 : 1);
  leftSteps = abs(number_of_steps);
  brakePoint = stepsPerRotate / 40;
}

/**********************************************************
 * Move stepper to number of steps
 **********************************************************/
void SmartStepper::move(int steps_to_move)
{
  this->step(steps_to_move, resetSteps);
  this->run();
}

void SmartStepper::calcSpeed()
{
  this->stepPeriod = 60000000L / stepsPerRotate / currentSpeed;
}

void SmartStepper::proccess()
{
  // Торможение
  if(this->leftSteps < this->brakePoint) {
    if(this->currentSpeed > this->startSpeed) {
      this->currentSpeed -= accel;
      if(this->currentSpeed < 0) this->currentSpeed = 0;
      this->calcSpeed();
    }
  }
  // Разгон
  else if(this->currentSpeed < this->targetSpeed )
  {
    this->currentSpeed += accel;
    this->calcSpeed();
  }
  // Шаг в направлении
  int direction = (this->currentStep > this->targetStep) ? -1 : 1;
  this->currentStep += direction;
  if(leftSteps > 0) leftSteps--;
  
  if(this->pin_step==PIN_UNCONNECTED) {
	  uint8_t thisStep = abs(this->currentStep) % 4;
	  if(this->currentStep < 0) thisStep = 3 - thisStep;
	  this->stepMotor(thisStep);
  } else {
	  digitalWrite(this->pin_dir,  (direction>=0)?HIGH:LOW);
	  digitalWrite(this->pin_step, !digitalRead(this->pin_step));
  }
  
  if(this->leftSteps == 0)
  {
    if(needPowerOff) this->off();
    this->isRunning = false;
    this->currentSpeed = startSpeed;
    this->calcSpeed();
  }
}

void SmartStepper::loop()
{
  microsCnt+=100;
  static uint32_t lastStepperMicros = 0;
  if(this->leftSteps == 0) return;
  if(this->isRunning == false) return;
  if(microsCnt - lastStepperMicros >= this->stepPeriod)
  {
    proccess();
    lastStepperMicros = microsCnt;
  }
}

/**********************************************************
 * Handle stepper commutation
 **********************************************************/
void SmartStepper::handle()
{
  static uint32_t lastStepperMicros = 0;
  if(this->leftSteps == 0) return;
  if(this->isRunning == false) return;
  if(micros() - lastStepperMicros >= this->stepPeriod)
  {
    proccess();
    lastStepperMicros = micros();
  }
}

/**********************************************************
 * Get state running
 **********************************************************/
bool SmartStepper::isRun()
{
  return this->isRunning;
}

bool SmartStepper::isStop()
{
  return this->isStopping;
}

void SmartStepper::pwmWriteFast(uint8_t pin, uint16_t duty_cycle) {
#ifdef _VARIANT_ARDUINO_STM32_
    timer_set_compare(PIN_MAP[pin].timer_device, PIN_MAP[pin].timer_channel, duty_cycle);
#endif
}

void SmartStepper::stepPin(uint8_t pin, uint32_t value)
{
  if(power > 1 && power < STEPPER_MAX_PWM) {
    pinMode(pin, PWM);
    //pwmWrite(pin, value);
    pwmWriteFast(pin, value);
  } else {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, value);
  }
}

/**********************************************************
 * Moves the motor forward or backwards.
 **********************************************************/
void SmartStepper::stepMotor(int thisStep)
{
  if(commonPower)
  {
	  switch (thisStep) {
		case 0:  // 1010
		  stepPin(motor_pin_1, power);
		  stepPin(motor_pin_2, LOW);
		  stepPin(motor_pin_3, power);
		  stepPin(motor_pin_4, LOW);
		break;
		case 1:  // 0110
		  stepPin(motor_pin_1, LOW);
		  stepPin(motor_pin_2, power);
		  stepPin(motor_pin_3, power);
		  stepPin(motor_pin_4, LOW);
		break;
		case 2:  //0101
		  stepPin(motor_pin_1, LOW);
		  stepPin(motor_pin_2, power);
		  stepPin(motor_pin_3, LOW);
		  stepPin(motor_pin_4, power);
		break;
		case 3:  //1001
		  stepPin(motor_pin_1, power);
		  stepPin(motor_pin_2, LOW);
		  stepPin(motor_pin_3, LOW);
		  stepPin(motor_pin_4, power);
		break;
	  }
  }
  else
  {
	  switch (thisStep) {
		case 0:  // 1010
		  stepPin(motor_pin_1, power);
		  stepPin(motor_pin_2, LOW);
		  stepPin(motor_pin_3, power);
		  stepPin(motor_pin_4, LOW);
		break;
		case 1:  // 0110
		  stepPin(motor_pin_1, LOW);
		  stepPin(motor_pin_2, power);
		  stepPin(motor_pin_3, power);
		  stepPin(motor_pin_4, LOW);
		break;
		case 2:  //0101
		  stepPin(motor_pin_1, LOW);
		  stepPin(motor_pin_2, power);
		  stepPin(motor_pin_3, LOW);
		  stepPin(motor_pin_4, power);
		break;
		case 3:  //1001
		  stepPin(motor_pin_1, power);
		  stepPin(motor_pin_2, LOW);
		  stepPin(motor_pin_3, LOW);
		  stepPin(motor_pin_4, power);
		break;
	  }
  }
}

