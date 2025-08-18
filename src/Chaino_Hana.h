/**
 * @file Chaino_Hana.h
 * @brief Chaino_Hana Board Support Library
 * @details This file provides a specific implementation of the Chaino protocol
 * for a board named "Hana". It pre-registers standard Arduino hardware
 * functions (GPIO, ADC, PWM, etc.) for remote access and control.
 */
#pragma once

#define __CHAINO_HANA_VER__  "0.9.4"
#include "Chaino.h"


/**
 * @class Chaino_Hana
 * @brief A specialized class for the "Chaino Hana" board, providing an API for hardware control.
 * @details This class inherits from `Chaino` and registers a set of standard Arduino functions 
 * (like `pinMode`, `digitalRead`, `analogWrite`, etc.) for remote execution. It simplifies 
 * interaction with a Hana board, whether it's the local device or a remote slave connected via I2C.
 *
 * ### Typical Usage (Controlling a remote Hana board):
 * @code
 * #include "Chaino_Hana.h"
 * 
 * // Target a remote Hana board at I2C address 0x42
 * Chaino_Hana hana(0x42);
 *
 * void setup() {
 *     // Use the Hana board's functions remotely
 *     hana.set_pin_mode(13, OUTPUT);
 *     hana.write_digital(13, HIGH);
 * }
 *
 * void loop() {}
 * @endcode
 * 
 * ### Typical Usage (As the Hana board itself):
 * @code
 * #include "Chaino_Hana.h"
 * 
 * // This device is the Hana board
 * Chaino_Hana hana;
 *
 * void setup() {
 * }
 *
 * void loop() {
        hana.set_neopixel(255,0,0);
        delay(1000);
        hana.set_neopixel(0,0,0);
        delay(1000);
 * }
 * @endcode
 */
class Chaino_Hana : public Chaino {
public:
    
    /**
     * @brief Constructor for the Chaino_Hana class.
     * @param addr The I2C address of the target Hana board. If 0 (default), 
     * it configures the local device to be the Hana board.
     */
    Chaino_Hana(byte addr=0): Chaino(addr) {
        if (addr == 0 ) {
            if (!isSetup) setup();
        } else {
            Chaino::setup();
        }
    } 

    /*
    void begin() {

        if (_i2cAddr == 0 ) {
            if (!isSetup) setup();
        } else {
            Chaino::setup();
        }
        
    } */       
    

    /**
     * @brief Registers all built-in functions for the Chaino Hana board.
     * @details This static method is called by constructor on the local device to map function IDs 
     * to standard Arduino hardware functions. It sets up handlers for digital I/O, analog I/O, 
     * PWM, tone generation, and timing functions.
     * @note This function is intended for internal use by the constructor method and should not 
     * normally be called directly by the user.
     */
    static void setup() {

        Chaino::setup("Chaino_Hana");         //반드시 호출해야 함
        _setVer(__CHAINO_HANA_VER__);         //버전 문자열 지정
        

        // Basic In/Out functions #################################################
        // user function must be {void func(void)} type
        Chaino::registerFunc(11, [](){       // pinMode()
            // 함수의 인수는 반드시 명시적 변환만 가능
            int pin( Chaino::getArg() );  //첫번째 int형 인수
            int mode( Chaino::getArg() );  //두번째 int형 인수
            // 또는 
            //int pin = (int)Chaino::getArg();  
            //int mode = (int)Chaino::getArg(); 
            //라고 명시적인 형을 반환하는 함수를 사용해도 된다.
            pinMode(pin, mode);
        }); 


        Chaino::registerFunc(12, [](){        //digitalRead()
            int pin = (int)Chaino::getArg();  
            int val = digitalRead(pin);
            Chaino::setReturn(val);
        });


        Chaino::registerFunc(13, [](){        //digitalWrite()
            int pin = (int)Chaino::getArg();  
            int status = (int)Chaino::getArg();
            digitalWrite(pin, status);
        });
        

        Chaino::registerFunc(14, [](){       //analogRead()
            int pin(Chaino::getArg());
            int adc = analogRead(pin);
            Chaino::setReturn(adc);
        });  
        
        
        Chaino::registerFunc(15, [](){     //analogReadResolution() => set_adc_range()       
            int bits(Chaino::getArg());  
            analogReadResolution(bits);
        }); //analogReadRange()는 RP2040에서는 없다



        // PWM functions ######################################################
        Chaino::registerFunc(21, [](){          //analogWrite()
            int pin = (int)Chaino::getArg();
            int duty = (int)Chaino::getArg();
            analogWrite(pin, duty);
        });


        Chaino::registerFunc(22, [](){          //analogWriteFreq() (default:1KHz)
            int freq = (int)Chaino::getArg();
            analogWriteFreq(freq);
        });


        Chaino::registerFunc(23, [](){          //analogWriteRange()(default:255)
            int range = (int)Chaino::getArg();  
            analogWriteRange(range);
        });


        // time functions ######################################################


        Chaino::registerFunc(31, [](){       // 람다함수: millis()
            unsigned long ms = millis();
            Chaino::setReturn(ms); //unsigned long형 반환
        });

        Chaino::registerFunc(32, [](){       // 람다함수: micros()
            unsigned long us = micros();
            Chaino::setReturn(us); //unsigned long형 반환
        });


        // tone functions ######################################################


        Chaino::registerFunc(41, [](){       // 람다함수: tone()
            int pin = (int)Chaino::getArg();  //첫번째 int형 인수
            int freq = (int)Chaino::getArg();  //두번째 int형 인수
            int duration = (int)Chaino::getArg();  //세번째 int형 인수
            if (duration > 0) {
                tone(pin, freq, duration);
            } else {
                tone(pin, freq); //duration이 0이면 무한정 지속
            }
        });


        Chaino::registerFunc(42, [](){       // 람다함수: no_tone()
            int pin = (int)Chaino::getArg();  //첫번째 int형 인수
            noTone(pin);
        });




        isSetup = true;

    }//satic void setup()

    
    // Basic IO functions #################################################// definitions of public methods

    /**
     * @brief Sets the mode of a digital pin (e.g., INPUT, OUTPUT).
     * @details This function corresponds to the standard Arduino `pinMode()` function.
     * @param pin The GPIO pin number to configure.
     * @param mode The mode to set, such as `INPUT`, `OUTPUT`, or `INPUT_PULLUP`.
     * @code
     * Chaino_Hana hana(0x42);
     * hana.set_pin_mode(13, OUTPUT); // Set pin 13 as an output
     * @endcode
     */
    void set_pin_mode(int pin, int mode) {
        execFunc(11, pin, mode); //this->execFunc() 실행
    }


    /**
     * @brief Reads the state of a digital pin.
     * @details Corresponds to the standard Arduino `digitalRead()` function.
     * @param pin The GPIO pin number to read from.
     * @return The state of the pin, either `HIGH` (1) or `LOW` (0).
     * @code
     * int buttonState = hana.read_digital(2);
     * @endcode
     */
    int read_digital(int pin){
        execFunc(12, pin);
        return (int)getReturn();
    }

    
    /**
     * @brief Writes a digital value (HIGH or LOW) to a pin.
     * @details Corresponds to the standard Arduino `digitalWrite()` function.
     * @param pin The GPIO pin number to write to.
     * @param status The value to write, either `HIGH` or `LOW`.
     * @code
     * hana.write_digital(13, HIGH); // Turn on an LED
     * @endcode
     */
    void write_digital(int pin, int status) {
        execFunc(13, pin, status);
    }


     /**
     * @brief Reads an analog value from a specified analog input pin.
     * @details Corresponds to the standard Arduino `analogRead()` function.
     * @param pin The analog pin number to read from (e.g., A0, A1).
     * @return The analog value, with a resolution set by `set_adc_range()` (default 0-1023).
     * @see set_adc_range()
     * @code
     * int sensorValue = hana.read_analog(A0);
     * @endcode
     */   
    int read_analog(int pin) {
        execFunc(14, pin);
        return (int)getReturn();
    }

    /**
     * @brief Sets the resolution for `analogRead()`.
     * @details This function corresponds to the `analogReadResolution()` function on supported boards like the RP2040.
     * @param bits The desired resolution in bits (e.g., 10 for 0-1023, 12 for 0-4095).
     * @note The available resolution depends on the board's hardware capabilities.
     * @code
     * // Set ADC resolution to 12 bits (0-4095)
     * hana.set_adc_range(12);
     * int highResValue = hana.read_analog(A0);
     * @endcode
     */
    void set_adc_bits(int bits) {
        execFunc(15, bits);
    }


    // PWM functions ######################################################

    
    /**
     * @brief Generates a PWM signal on a specified pin.
     * @details Corresponds to `analogWrite()`. Sets the duty cycle of the PWM signal.
     * @param pin The PWM-capable pin to write to.
     * @param duty The duty cycle, with a value between 0 (always off) and the current PWM range.
     * @see set_pwm_freq(), set_pwm_range()
     * @code
     * // Set an LED to half brightness (assuming default range of 255)
     * hana.write_analog(9, 128);
     * @endcode
     */    
    void write_analog(int pin, int duty) {
        execFunc(21, pin, duty);
    }


    /**
     * @brief Sets the frequency for PWM signals (`analogWrite`).
     * @details Corresponds to `analogWriteFreq()` on supported boards. Affects all subsequent `analogWrite` calls.
     * @param freq The desired frequency in Hertz (Hz).
     * @code
     * // Set PWM frequency to 1000 Hz (1 kHz)
     * hana.set_pwm_freq(1000);
     * @endcode
     */
    void set_pwm_freq(int freq) {
        execFunc(22, freq);
    }

    /**
     * @brief Sets the range for PWM duty cycles (`analogWrite`).
     * @details Corresponds to `analogWriteRange()` on supported boards. The default is 255.
     * @param range The maximum value for the duty cycle (e.g., 1023 for 10-bit resolution).
     * @code
     * // Set PWM range for 10-bit resolution
     * hana.set_pwm_range(1023);
     * // Set LED to 50% brightness
     * hana.write_analog(9, 512);
     * @endcode
     */
    void set_pwm_range(int range) {
        execFunc(23, range);
    }


    // time functions ######################################################


    /**
     * @brief Gets the number of milliseconds since the board began running.
     * @details Corresponds to the standard Arduino `millis()` function.
     * @return The number of milliseconds as an `unsigned long`.
     * @code
     * unsigned long currentTime = hana.get_millis();
     * @endcode
     */
    int get_millis() { //아두이노의 millis()와 이름이 안겹쳐야함
        execFunc(31);
        return (int)getReturn();
    }


    /**
     * @brief Gets the number of microseconds since the board began running.
     * @details Corresponds to the standard Arduino `micros()` function.
     * @return The number of microseconds as an `unsigned long`.
     * @code
     * unsigned long preciseTime = hana.get_micros();
     * @endcode
     */
    int get_micros() { //아두이노 millis()와 이름이 안 겹쳐야함
        execFunc(32);
        return (int)getReturn();
    }


    // tone functions ######################################################


    /**
     * @brief Generates a square wave tone on a pin.
     * @details Corresponds to the standard Arduino `tone()` function.
     * @param pin The pin on which to generate the tone.
     * @param freq The frequency of the tone in Hertz.
     * @param duration Optional. The duration of the tone in milliseconds. If 0 or omitted, 
     * the tone plays continuously until `stop_tone()` is called.
     * @code
     * // Play a 440 Hz tone for 1 second
     * hana.start_tone(8, 440, 1000);
     * @endcode
     */    
    void start_tone(int pin, int freq, int duration = 0) {
        /* 핀에 주파수 톤을 발생시킴. duration이 0이면 무한정 지속 */
        execFunc(41, pin, freq, duration);
    }


    /**
     * @brief Stops the tone being generated on a pin.
     * @details Corresponds to the standard Arduino `noTone()` function.
     * @param pin The pin to stop the tone on.
     * @code
     * // Start a continuous tone
     * hana.start_tone(8, 500);
     * delay(2000);
     * // Stop the tone
     * hana.stop_tone(8);
     * @endcode
     */    
    void stop_tone(int pin) {
        execFunc(42, pin);
    }




private:
    static bool isSetup;//Chaino_Hana::setup()함수는 딱 한 번만 실행
};

bool Chaino_Hana::isSetup = false;
/*
void goo() {
    ap::setReturn("1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMN(50)1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMN(100)1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMN(150)1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMN(200)012345678901234567890123<250");
}
*/
