/**
 * @file Chaino_Hana.h
 * @brief Chaino_Hana Board Support Library
 * @details This file provides a specific implementation of the Chaino protocol
 * for a board named "Hana". It pre-registers standard Arduino hardware
 * functions (GPIO, ADC, PWM, etc.) for remote access and control.
 */
#pragma once

#define __CHAINO_HANA_VER__  "0.9.5"
#include "Chaino.h"


#if defined(ARDUINO_ARCH_RP2040) //|| defined(PICO_RP2350)======================

    #include <hardware/pwm.h>
    /*
    inline void _setPwmBits(byte bits) {
        long range = (long)pow(2, bits);
        //RP2040에는 analogWriteResolution() 함수가 없음
        analogWriteRange(range);
    }*/

    // RP2040에서 개별 핀별로 PWM 해상도 비트수를 설정하는 함수
    void _setPwmBits(byte pin, byte bits) {
        // 핀이 유효한 GPIO 핀인지 확인
        if (pin >= NUM_DIGITAL_PINS) {
            return; // 유효하지 않은 핀 번호
        }
        
        // 비트수 범위 제한 (1~16비트)
        if (bits < 1) bits = 1;
        if (bits > 16) bits = 16;
        
        // 해당 핀의 PWM 슬라이스 번호 가져오기
        uint slice_num = pwm_gpio_to_slice_num(pin);
        
        // 비트수에 따른 TOP 값 계산 (2^bits - 1)
        uint32_t top = (1UL << bits) - 1;
        
        // 현재 설정된 분주비 값 유지
        uint32_t current_div = pwm_hw->slice[slice_num].div;
        
        // PWM 설정 적용
        pwm_set_wrap(slice_num, top);  // TOP 값 설정으로 해상도 결정
        
        // 해당 핀을 PWM 기능으로 설정
        gpio_set_function(pin, GPIO_FUNC_PWM);
        
        // PWM 슬라이스 활성화
        pwm_set_enabled(slice_num, true);
    }


    // 현재 설정된 PWM 해상도 비트수를 확인하는 보조 함수
    byte _getPwmBits(byte pin) {
        if (pin >= NUM_DIGITAL_PINS) {
            return 0;
        }
        
        uint slice_num = pwm_gpio_to_slice_num(pin);
        uint32_t top = pwm_hw->slice[slice_num].top;
        
        // TOP 값으로부터 비트수 계산
        byte bits = 0;
        uint32_t temp = top + 1;
        while (temp > 1) {
            temp >>= 1;
            bits++;
        }
        
        return bits;
    }


    void _setPwmFreq(byte pin, long freq) {
        // 핀이 유효한 GPIO 핀인지 확인
        if (pin >= NUM_DIGITAL_PINS) {
            return; // 유효하지 않은 핀 번호
        }
        
        // 해당 핀의 PWM 슬라이스 번호 가져오기
        uint slice_num = pwm_gpio_to_slice_num(pin);
        
        // 시스템 클록 주파수 (RP2040은 기본적으로 125MHz)
        uint32_t clock_freq = clock_get_hz(clk_sys);
        
        // 주파수가 너무 낮거나 높은 경우 제한
        if (freq < 1) freq = 1;
        if (freq > clock_freq / 2) freq = clock_freq / 2;
        
        // PWM 분주비와 TOP 값 계산
        // PWM 주파수 = clock_freq / ((div + div_frac/16) * (top + 1))
        // 여기서는 정수 분주비만 사용하여 간단화
        
        uint32_t div_int = 1;
        uint32_t top = (clock_freq / freq) - 1;
        
        // TOP 값이 65535를 초과하면 분주비를 증가
        while (top > 65535 && div_int < 255) {
            div_int++;
            top = (clock_freq / (div_int * freq)) - 1;
        }
        
        // TOP 값이 여전히 너무 크면 최대값으로 제한
        if (top > 65535) {
            top = 65535;
        }
        
        // PWM 설정 적용
        pwm_set_clkdiv_int_frac(slice_num, div_int, 0); // 정수 분주비만 사용
        pwm_set_wrap(slice_num, top);                   // TOP 값 설정
        
        // 해당 핀을 PWM 기능으로 설정
        gpio_set_function(pin, GPIO_FUNC_PWM);
        
        // PWM 슬라이스 활성화
        pwm_set_enabled(slice_num, true);
    }


    // 현재 설정된 PWM 주파수를 확인하는 보조 함수
    uint32_t _getPwmFreq(byte pin) {
        if (pin >= NUM_DIGITAL_PINS) {
            return 0;
        }
        
        uint slice_num = pwm_gpio_to_slice_num(pin);
        uint32_t clock_freq = clock_get_hz(clk_sys);
        uint32_t div_int = pwm_hw->slice[slice_num].div >> PWM_CH0_DIV_INT_LSB;
        uint32_t top = pwm_hw->slice[slice_num].top;
        
        if (div_int == 0) div_int = 1;
        if (top == 0) top = 1;
        
        return clock_freq / (div_int * (top + 1));
    }


#elif defined(ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32S3) //=================

    inline void _setPwmBits(byte pin, byte bits) {
        analogWriteResolution(pin, bits);
    }

    inline void _setPwmFreq(byte pin, long freq) {
        analogWriteFrequency(pin, freq);
    }


#endif //=======================================================================





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
            byte pin = (byte)Chaino::getArg();
            int duty = (int)Chaino::getArg();
            analogWrite(pin, duty);
        });


        Chaino::registerFunc(22, [](){          //analogWriteFreq() (default:1KHz)
            byte pin = (byte)Chaino::getArg();
            long freq = (long)Chaino::getArg();
            _setPwmFreq(pin, freq);
        });


        Chaino::registerFunc(23, [](){          //analogWriteRange()(default:255)
            byte pin = (byte)Chaino::getArg();  
            byte bits = (byte)Chaino::getArg();  
            //analogWriteRange(range);
            _setPwmBits(pin, bits);
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
    void set_pwm_bits(int bits) {
        execFunc(23, bits);
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
