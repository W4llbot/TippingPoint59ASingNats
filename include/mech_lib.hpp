#ifndef _MECH_LIB_HPP_
#define _MECH_LIB_HPP_

void armControl(void*ignore);
void setArmPos(int pos);
void setArmClampState(bool state);
void toggleArmClampState();

void tiltControl(void*ignore);
void setTiltState(bool state);
void toggleTiltState();

void intakeControl(void*ignore);
void setIntake(double pow);

#endif
