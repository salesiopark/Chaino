/* --------------------------------------------------------
this device가 i2c로 연결된 Devkit모듈을 제어할 때
---------------------------------------------------------- */
#include "Chaino.h"

void setup() {

    chaino.registerFunc(1, []() {
        int pin = chaino.getIntArg();
        int adc = analogRead(pin);
        chaino.setReturn(adc);
    });

    chaino.init();
}

void loop() {
    chaino.loop();
}