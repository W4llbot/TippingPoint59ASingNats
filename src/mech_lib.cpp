#include "main.h"

// const double armHeights[] = {30, 415, 650};
const double progArmHeights [] = {1380, 2150, 2600};
double armTarg = progArmHeights[0], armKP = 1;
bool tiltState = LOW, armClampState = LOW;
double intakeTarg = 0;

void armControl(void*ignore) {
  Motor arm(armPort);
  arm.set_brake_mode(MOTOR_BRAKE_BRAKE);
  ADIAnalogIn armPot(armPotPort);
  ADIDigitalOut armClamp(armClampPort);
  ADIDigitalIn armLimit(armLimitPort);

  while(true) {
    double armError = armTarg - armPot.get_value();
    arm.move(armError*armKP);

    if(armLimit.get_new_press()) armClampState = true;
    armClamp.set_value(armClampState);

    // printf("armtarg: %.2f\t currpos: %.2f\n", armTarg, arm.get_position());
    delay(dT);
  }
}

void setArmPos(int pos) {armTarg = progArmHeights[pos];}
void setArmHeight(double height) {armTarg = height;}
// double getArmHeight() {return armTarg;}
void debugArm() {
  Motor arm(armPort);
  printf("armTarg: %.2f, armVal: %.2f", armTarg, arm.get_position());
}
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
      delay(150);
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
