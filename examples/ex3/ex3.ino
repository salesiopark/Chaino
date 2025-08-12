#include "Chaino.h"

void setup() {
    Chaino::registerFunc(1, []() {
        bool b1 = Chaino::getBoolArg();  // 첫번째 int형 인수
        bool b2 = Chaino::getBoolArg();  // 첫번째 int형 인수
        float f = Chaino::getFloatArg(); // 두번째 int형 인수
        double d = Chaino::getDoubleArg(); // 세번째 double형 인수
        String s = Chaino::getStringArg(); // 네번째 String형 인수

        //Serial.print(b1);Serial.print(",");
        //Serial.print(b2);Serial.print(",");
        //Serial.print(f);Serial.print(",");
        //Serial.print(d);Serial.print(",");
        //Serial.println(s);

        Chaino::setReturn(b1); 
        Chaino::setReturn(b2); 
        Chaino::setReturn(f); 
        Chaino::setReturn(d); 
        Chaino::setReturn(f+d); 
        Chaino::setReturn("Hello, Chaino!");
    });

    //Serial.begin(460800);
    Chaino::setup(); // Chaino 초기화
}

void loop() {
    Chaino::loop(); // Chaino 루프 실행
}