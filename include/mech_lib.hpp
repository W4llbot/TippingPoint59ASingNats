#ifndef _MECH_LIB_HPP_
#define _MECH_LIB_HPP_

void armControl(void*ignore);
void setArmHeight(double height);
void debugArm();
void setArmPos(int pos);
void setArmClampState(bool state);
void toggleArmClampState();
void waitArmClamp(double cutoff);

void tiltControl(void*ignore);
void setTiltState(bool state);
void toggleTiltState();
void waitTiltClamp(double cutoff);

void intakeControl(void*ignore);
void setIntake(double pow);

#endif
