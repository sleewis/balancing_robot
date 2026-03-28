#include "Motor.h"

hw_timer_t *motorTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


TwoWire I2C_1 = TwoWire(0);
TwoWire I2C_2 = TwoWire(1);

Motor motor1(33, 32, 25, 12, 39, 36, &I2C_1, 11);
Motor motor2(26, 27, 14, 12, 34, 35, &I2C_2, 11);



volatile bool motorFlag = false;

void IRAM_ATTR onMotorTimer(){
  motorFlag = true;
}


void setup() {
  Serial.begin(115200);

  motorTimer = timerBegin(1000000);  // 1 MHz
  timerAttachInterrupt(motorTimer, &onMotorTimer);
  timerAlarm(motorTimer, 2000, true, 0);


  I2C_1.begin(19, 18, 400000);
  I2C_2.begin(23, 5, 400000);

  motor1.begin();
  motor2.begin();
}

void loop() {
  if(motorFlag){;
    motorFlag = false;
    motor1.loop(1.0f);
    motor2.loop(-0.2f);







  }
}