#include "FastAccelStepper.h"

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperL = NULL;
FastAccelStepper *stepperR = NULL;

const int stepPinR = 10;
const int dirPinR = 7;
const int stepPinL = 9;
const int dirPinL = 6;

int RadsPerSToHZ(float radsPerS){
  return int(abs(radsPerS) / 0.031416);
} 

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);   

  engine.init();

  stepperL = engine.stepperConnectToPin(stepPinL);
  stepperR = engine.stepperConnectToPin(stepPinR);
   
  stepperL->setDirectionPin(dirPinL);
  stepperL->setSpeedInHz(500);
  stepperL->setAcceleration(100);

  stepperR->setDirectionPin(dirPinR);
  stepperR->setSpeedInHz(500);
  stepperR->setAcceleration(100);
}

void loop() {
  while (Serial.available() > 0 && (Serial.peek() == '\n' || Serial.peek() == '\r' || Serial.peek() == ' ')) {
    Serial.read();
  }

  if (Serial.available() > 0){
      float leftVel = Serial.parseFloat();
      float rightVel = Serial.parseFloat();


      while (Serial.available() > 0 && Serial.read() != '\n') {
      }

      stepperR->setSpeedInHz(RadsPerSToHZ(rightVel));
      stepperL->setSpeedInHz(RadsPerSToHZ(leftVel));

      if (leftVel > 0){
        stepperL->runForward();
      }
      else if (leftVel < 0){
        stepperL->runBackward();
      }
      else{
        stepperL->stopMove();
      }

      if (rightVel > 0){
        stepperR->runForward();
      }
      else if (rightVel < 0){
        stepperR->runBackward();
      }
      else{
        stepperR->stopMove();
      }

      stepperL->applySpeedAcceleration();
      stepperR->applySpeedAcceleration();
  }
}