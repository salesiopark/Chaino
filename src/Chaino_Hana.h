/* --------------------------------------------------------
chaino.Hana board Library
---------------------------------------------------------- */
#pragma once

#include "Chaino.h"

class Chaino_Hana : public Chaino {
public:
    
    // i2c주소가 없으면 this device가 Hana보드이다
    Chaino_Hana(byte addr=0): Chaino(addr) {}
    
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

        Chaino::setup("Chaino_Hana");                  //반드시 호출해야 함
        
        // user function must be {void func(void)} type
        Chaino::registerFunc(1, [](){       // 람다함수 정의
            // 함수의 인수는 반드시 명시적 변환만 가능
            int pin( Chaino::getArg() );  //첫번째 int형 인수
            int mode( Chaino::getArg() );  //두번째 int형 인수
            // 또는 
            //int pin = (int)Chaino::getArg();  
            //int mode = (int)Chaino::getArg(); 
            //라고 명시적인 형을 반환하는 함수를 사용해도 된다.
            pinMode(pin, mode);
        }); 

        Chaino::registerFunc(2, [](){
            int pin = (int)Chaino::getArg();  //첫번째 int형 인수
            int val = digitalRead(pin); //두번째 int형 인수
            Chaino::setReturn(val);         // int형 반환
        });

        Chaino::registerFunc(3, [](){
            int pin = (int)Chaino::getArg();  //첫번째 int형 인수
            int status = (int)Chaino::getArg();  //두번째 int형 인수
            digitalWrite(pin, status);
        });
        
        Chaino::registerFunc(4, [](){       
            int pin(Chaino::getArg());  //첫번째 int형 인수
            int adc = analogRead(pin);
            Chaino::setReturn(adc);
        });  

        Chaino::registerFunc(5, [](){
            int pin = (int)Chaino::getArg();  //첫번째 int형 인수
            int duty = (int)Chaino::getArg();  //두번째 int형 인수
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
        return (int)getReturn();
    }

    void write_digital(int pin, int status) {
        execFunc(3, pin, status);
    }

    int read_analog(int pin) {
        execFunc(4, pin);
        return (int)getReturn();
    }

    void write_analog(int pin, int duty) {
        execFunc(5, pin, duty);
    }


private:
    static bool isSetup;//Divkit::setup()함수는 딱 한 번만 실행
};

bool Chaino_Hana::isSetup = false;
/*
void goo() {
    ap::setReturn("1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMN(50)1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMN(100)1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMN(150)1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMN(200)012345678901234567890123<250");
}
*/
