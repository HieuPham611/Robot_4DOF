#include <AccelStepper.h>

int step_x = 4; int dir_x = 7; int ena_x = 8;

AccelStepper Step_x(1, step_x, dir_x, ena_x);

void setup() {
  Serial.begin(9600);
  Step_x.setMaxSpeed(5000);
  Step_x.setAcceleration(3000);

  Step_x.setEnablePin(ena_x);
  Step_x.setPinsInverted(false, false, true);
  Step_x.enableOutputs();
}
void loop() {
  Step_x.setCurrentPosition(0);
  int homeX = 0;
  while (Step_x.currentPosition() != -200)
  {
    
    Step_x.moveTo(homeX);
    homeX++;
    Step_x.run();
  }
  
}

