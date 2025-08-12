/* --------------------------------------------------------
chaino.Hana board Library
---------------------------------------------------------- */
#pragma once

#include "Chaino.h"

class Hana : public Chaino {
public:
    
    // i2c주소가 없으면 this device가 Hana보드이다
    Hana(byte addr=0): Chaino(addr) {}
    
    void begin() {

        if (_i2cAddr == 0 ) {
            if (!isSetup) setup();
        } else {
            Chaino::setup();
        }
        
    }        
    
    /*------------------------------------------------------
    // definitions of loop() and setup() for slave device
    ------------------------------------------------------*/
    static void setup() {

        Chaino::setup();                  //반드시 호출해야 함
        
        // user function must be {void func(void)} type
        Chaino::registerFunc(1, [](){       // 람다함수 정의
            int pin = Chaino::getIntArg();  //첫번째 int형 인수
            int mode = Chaino::getIntArg();  //두번째 int형 인수
            pinMode(pin, mode);
        }); 

        Chaino::registerFunc(2, [](){
            int pin = Chaino::getIntArg();  //첫번째 int형 인수
            int val = digitalRead(pin); //두번째 int형 인수
            Chaino::setReturn(val);         // int형 반환
        });

        Chaino::registerFunc(3, [](){
            int pin = Chaino::getIntArg();  //첫번째 int형 인수
            int status = Chaino::getIntArg();  //두번째 int형 인수
            digitalWrite(pin, status);
        });
        
        Chaino::registerFunc(4, [](){       
            int pin = Chaino::getIntArg();  //첫번째 int형 인수
            int adc = analogRead(pin);
            Chaino::setReturn(adc);
        });  

        Chaino::registerFunc(5, [](){
            int pin = Chaino::getIntArg();  //첫번째 int형 인수
            int duty = Chaino::getIntArg();  //두번째 int형 인수
            analogWrite(pin, duty);
        });
        
        //Chaino::registerFunc(6, foo);
        //Chaino::registerFunc(7, goo);

        isSetup = true;

    }//satic void setup()

    
    //static inline void loop() {Chaino.loop();}

    /*------------------------------------------------------
    // definitions of public methods
    ------------------------------------------------------*/
    void set_pin_mode(int pin, int mode) {
        execFunc(1, pin, mode); //this->execFunc()를 실행한다.
    }

    int read_digital(int pin){
        execFunc(2, pin);
        return getReturnInt();
        //return execFunc(2, pin); //Params객체의 캐스팅int()연산자 작동
    }

    void write_digital(int pin, int status) {
        execFunc(3, pin, status);
    }

    int read_analog(int pin) {
        execFunc(4, pin);
        return getReturnInt();
        //return execFunc(4, pin); //Params객체의 캐스팅int()연산자 작동
    }

    void write_analog(int pin, int duty) {
        execFunc(5, pin, duty);
    }


private:
    static bool isSetup;//Divkit::setup()함수는 딱 한 번만 실행
};

bool Hana::isSetup = false;
/*
void goo() {
    ap::setReturn("1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMN(50)1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMN(100)1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMN(150)1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMN(200)012345678901234567890123<250");
}
*/
