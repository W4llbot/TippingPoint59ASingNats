#include "main.h"

const double armHeights[] = {30, 415, 650};
const double progArmHeights [] = {30, 480, 750};
double armTarg = armHeights[0], armKP = 5;
bool tiltState = LOW, armClampState = LOW;
double intakeTarg = 0;

void armControl(void*ignore) {
  Motor arm(armPort);
  arm.set_brake_mode(MOTOR_BRAKE_BRAKE);
  ADIDigitalOut armClamp(armClampPort);
  ADIDigitalIn armLimit(armLimitPort);

  while(true) {
    double armError = armTarg - arm.get_position();
    arm.move(armError*armKP);

    if(armLimit.get_new_press()) armClampState = true;
    armClamp.set_value(armClampState);

    // printf("armtarg: %.2f\t currpos: %.2f\n", armTarg, arm.get_position());
    delay(dT);
  }
}

void setArmPos(int pos) {armTarg = armHeights[pos];}
void setArmHeight(double height) {armTarg = height;}
void setArmClampState(bool state) {armClampState = state;}
void toggleArmClampState() {armClampState = !armClampState;}
void waitArmClamp(double cutoff) {waitUntil(armClampState, cutoff);}

void tiltControl(void*ignore) {
  ADIDigitalOut tilt(tiltPort);
  ADIDigitalOut tiltClamp(tiltClampPort);
  ADIDigitalIn tiltLimit(tiltLimitPort);

  while(true) {
    if(tiltLimit.get_new_press()) tiltState = true;
    if(tiltState) {
      tiltClamp.set_value(HIGH);
      delay(100);
      tilt.set_value(HIGH);
    }else{
      tilt.set_value(LOW);
      delay(300);
      tiltClamp.set_value(LOW);
    }

    delay(dT);
  }
}

void setTiltState(bool state) {tiltState = state;}
void toggleTiltState() {tiltState = !tiltState;}
void waitTiltClamp(double cutoff) {waitUntil(tiltState, cutoff);}

void intakeControl(void*ignore) {
  Motor intake(intakePort);
  while(true) {
    intake.move(intakeTarg);
    delay(dT);
  }
}
void setIntake(double pow) {intakeTarg = pow;}
