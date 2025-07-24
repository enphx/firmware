#include "low_level/encodermotor.h"
#include "low_level/potentiometermotor.h"
#include "low_level/servodriver.h"
#include "low_level/shiftregister.h"
#include "low_level/stepperdriver.h"

void low_levelAssignMotors(EncoderMotor *m_leftMotor,
                           EncoderMotor *m_rightMotor,
                           PotentiometerMotor *m_shoulderMotor,
                           StepperMotor *m_asimuthStepper, Servo *m_elbowServo);
void low_levelAssignLowestLevelObjects(ShiftRegister *m_shiftRegister);
void low_level_init();
void print_adc_vals();
void set_drivebase_speed(float speed);
void low_level_update();
