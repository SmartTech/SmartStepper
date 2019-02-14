#ifndef _SMART_STEPPER_H_
#define _SMART_STEPPER_H_

#include <Arduino.h>

#define STEPPER_MAX_PWM      2880
#define STEPPER_MAX_POWER     100

#ifndef PIN_UNCONNECTED
#define PIN_UNCONNECTED       255
#endif

#ifndef _VARIANT_ARDUINO_STM32_
#define PWM           OUTPUT
#define WiringPinMode uint8_t
#endif

class SmartStepper {
  
  public:

    SmartStepper(int number_of_steps, int motor_pin_1, int motor_pin_2,
                                 int motor_pin_3, int motor_pin_4);
								 
    SmartStepper(int number_of_steps, int pin_step, int pin_dir, int pin_en);

    void begin();
    void handle();
    void loop();

    void setSpeed(float whatSpeed);
    void setStartSpeed(float whatStartSpeed);
    void setAccel(float whatAccel);
    void setPower(int whatPower);
    void setReverse(bool whatReverse);
    void setPowerOff(bool whatState);
    void setPulses(uint32_t pulses);

    void move(int steps_to_move);
    void step(int number_of_steps, bool resetTargets = false);
    void stop(uint8_t _breakPoint=0);
    void run();

    void off();        // Stepper OFF
    void brake();      // stepper Brake

    bool isRun();
    bool isStop();
  
  private:

    void  stepPin(uint8_t pin, uint32_t value);
    void  stepMotor(int this_step);
    void  calcSpeed();
    void  proccess();

    uint32_t microsCnt = 0;

    bool  resetSteps = true;
    bool  reverse    = false;
    
    int   stepsPerRotate = 0;  
    int   currentStep    = 0;       // Which step the motor is on
    int   targetStep     = 0;       // 
    int   leftSteps      = 0;       // How many steps to take
    int   brakePoint     = 0;

    float currentSpeed   = 1;
    float targetSpeed    = 1;
    float startSpeed     = 1;

    float    accel       = 1;
    uint32_t power       = STEPPER_MAX_POWER;
    
    long  stepPeriod     = 0;       // Delay between steps, in us, based on speed

    bool  isRunning      = false;   // 
    bool  isStopping     = true;   // 
    bool  needPowerOff   = true;

    // motor pin numbers:
    int motor_pin_1 = PIN_UNCONNECTED;
    int motor_pin_2 = PIN_UNCONNECTED;
    int motor_pin_3 = PIN_UNCONNECTED;
    int motor_pin_4 = PIN_UNCONNECTED;
	
    int pin_step = PIN_UNCONNECTED;
    int pin_dir  = PIN_UNCONNECTED;
    int pin_en   = PIN_UNCONNECTED;

    void pwmWriteFast(uint8_t pin, uint16_t duty_cycle);
    
};


#endif // _SMART_STEPPER_H_

