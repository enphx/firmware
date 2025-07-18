#include "low_level/encodermotor.h"
#include "low_level/shiftregister.h"

void low_levelAssignMotors(EncoderMotor *m_leftMotor, EncoderMotor *m_rightMotor) ;
void low_levelAssignLowestLevelObjects(ShiftRegister *m_shiftRegister);
void low_level_init();
void print_adc_vals();
void set_speed(float speed);
void low_level_update();
