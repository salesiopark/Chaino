/* Chaino ver. 0.9.1
2025년07월16일 : 최초 파일 생성(개발 시작)
2025년07월30일 : <map>으로 PtrFunc을 관리하는 방식으로 수정
2025년07월31일 : Wire1으로 교체하여 Wire는 사용자가 이용하도록 수정
                i2c 슬레이브에서 생성한 패킷의 길이가 250바이트가 Fail반환
2025년08월03일 : Params 클래스 추가
2025년08월05일 : ardpy::retParams, argParams (Parmas객체들) 추가
2025년08월07일 : 이름을 chaino로 변경. Chaino, Hana 클래스 추가
*/
#pragma once


#include <map>
#include <vector>
#include <Wire.h>
#include <EEPROM.h> //RP2040zero
#include <Adafruit_NeoPixel.h>

//#define __DEBUG__I2C_MASTER__ //i2c master에서 일부러 crc오류를 낸다
//#define __DEBUG__I2C_SLAVE__  //i2c slave에서 일부러  crc오류를 낸다
//#define __DEBUG__SERIAL__     //Serial 통신에서 일부러  crc오류를 낸다.


/// \cond DOXYGEN_IGNORE
//네임스페이스명에 detail이 붙으면 (사용자가 건드릴 수 없는) 내부용이라는 암시임
namespace chaino_detail {

    const byte PIN_MASTER_SLAVE = 8;
    
    String strWho = "Chaino_Unknown";//string that returned by func#201


    Adafruit_NeoPixel neoPx(1, 16, NEO_GRB + NEO_KHZ800);
    void setNeoPxColorRGB(const byte r, const byte g, const byte b){
        neoPx.setPixelColor(0, neoPx.Color(r, g, b)); // (R,G,B)
        neoPx.show(); 
    }

    
    // 상수들: scope를 관리하기 위해 #define대신 const 사용
    const char   RS               = 0x1e;    // Record Separator
    const char   EOT              = 0x04;   // End of Transmission

    const byte   MAX_PACKET_SIZE  = 250; // 최대 패킷길이(bytes)
    const String EMPTY_STR        = "";


    /*----------------------------------------------------------
    // 패킷 정보를 담을 구조체 정의
    // Packet.payload는 끝에 {EOT}를 포함하지 않는다
    -----------------------------------------------------------*/
    class Packet {
    public:

        String   payload;  // (ASCII문자열) 데이터 페이로드(EOT는 포함하지 않음)
        uint16_t crc;      // CRC16 값
        
        Packet() : payload(""), crc(0) {}               // 기본 생성자
        Packet(char c, uint16_t c16 = 0) : payload(String(c)), crc(c16) {} //첫 문자로 생성
        Packet(const String& str) : payload(str), crc(0) {}//문자열로 생성

        // CRC-16-XMODEM (가장 일반적인 CRC-16)
        static uint16_t computeCrc(const String& strData) {

            const byte *data = (const byte*)strData.c_str();
            size_t length = strData.length();

            uint16_t crc16 = 0x0000;
            while (length--) {
                crc16 ^= (*data++) << 8;
                for (uint8_t i = 0; i < 8; ++i) {
                    if (crc16 & 0x8000)
                        crc16 = (crc16 << 1) ^ 0x1021;
                    else
                        crc16 <<= 1;
                }
            }
            return crc16;
        }

        // (전송받은)highByte와 lowByte를 이용해서 crc조합
        inline void setCrc(uint16_t hb, uint16_t lb) {
            crc = (hb<<8) | lb;
        }

        // 완성된 payload로 CRC16를 계산해서 멤버변수 crc에 저장
        inline void computeAndSetCrc() {
            crc = computeCrc(payload);
        }

        // (전송받아 저장된)crc 값과, (전송받은)payload로 여기서 계산한 crc가 같은가?
        inline bool isCrcMatched() const {
            return crc == computeCrc(payload);
        }

        // payload의 메모리를 미리 할당하여 heap영역의 파편화를 줄인다.
        inline void reserve(unsigned int size) {
            payload.reserve(size);
        }

        inline void addChar(char c) {
            payload += c;
        }
        
        inline void addStr(const String &str) {
            payload += str;
        }

        inline int length() const {
            return payload.length();
        }

        inline void init(byte size = 0) {
            crc = 0;
            payload = "";
            if (size > 0) payload.reserve(size);
        }
    
    };// class Packet

    
    const Packet PACKET_CRC_ERR('E', 0x1861); // CRC오류 패킷(상수)
    Packet packetLastSent;                    // 마지막으로 전송한 패킷


    /*----------------------------------------------------------  
        args와 rets를 저장하기 위한 클래스 (2025/08/05 추가)
    ------------------------------------------------------------*/
    class Params {
        public:

            Params():_idGet(0) {}

            //payload에서 params를 한꺼번에 모두 add하는 생성자
            //payload 구조: "param1{RS}param2{RS}...{RS}paramN"
            Params(const String& payload){
                if( !payload.isEmpty() ) addAll(payload);
            }

            inline void clear() {
                _vecStr.clear();
                _idGet = 0;
            }

            // param을 하나씩 문자열롤 변환하여 push하는 메서드들<<<<<<<<<<<<<<<<<<
            // 특수한 처리가 필요한 타입들만 명시적으로 정의
            inline void add(const bool v)   { _vecStr.push_back(v? "1":"0");}
            inline void add(const float v)  {_vecStr.push_back(_floatToStr(v));}
            inline void add(const double v) {_vecStr.push_back(_doubleToStr(v));}
            inline void add(const String& v) {_vecStr.push_back(v);}
            inline void add(const char *v) {_vecStr.push_back(String(v));} //c-string도 지원
            // 나머지 모든 타입을 위한 일반적인 template
            //T:byte, char, short, unsigned short, int, unsigned int, long, unsigned long
            template<typename T> 
            inline void add(const T& v) {_vecStr.push_back(String(v));}
            //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

            //payload에서 params를 한꺼번에 모두 add하는 메서드
            //payload 구조: "param1{RS}param2{RS}...{RS}paramN"
            void addAll(const String& payload) {
                clear();

                int start = 0, idRS;             
                while (true) {                 
                    idRS = payload.indexOf(RS, start);
                    if (idRS == -1) { // RS가 더이상 없다면                     
                        _vecStr.push_back(payload.substring(start));                     
                        break;                 
                    } else {                     
                        _vecStr.push_back(payload.substring(start, idRS));                     
                        start = idRS + 1;                 
                    }             
                }                     
            }

            //String.toInt()메서드는 long형을 반환하고 RP2040에서는 long == int 이다(4바이트)
            //더이상 get할게 없으면 단순히 기본값 반환
            inline int getInt() {
                return (_idGet<_vecStr.size()? _vecStr[_idGet++].toInt():0);
            }

            inline unsigned int getUInt() {
                return static_cast<unsigned int>(getInt()); 
            }

            inline long getLong() {
                return getInt();
            }

            inline unsigned long getULong() {
                return static_cast<unsigned long>(getInt()); 
            }

            inline byte getByte() {
                return byte(getInt());
            }

            inline char getChar() {
                return char(getInt());
            }

            inline short getShort() {
                return short(getInt());
            }

            inline unsigned short getUShort() {
                return static_cast<unsigned short>(getInt()); 
            }

            inline bool getBool() {
                if (_idGet >= _vecStr.size()) return false;//읽을 arg가 없다면 지정된 p를 반환
                else return (_vecStr[_idGet++].toInt()==1);
            }

            //String.getFloat()는 내부적으로 float(toDouble())로 정의되어 있다.
            inline double getDouble() {
                return _idGet<_vecStr.size()? _vecStr[_idGet++].toDouble():0.0;
            }

            inline float getFloat() {
                return float(getDouble());
            }

            inline String getString() {
                return (_idGet<_vecStr.size() ? _vecStr[_idGet++]:EMPTY_STR);
            }

            /*
            // 저장된 params를 가지고 함수실행 payload(String객체)를 만든다.
            // pyaload 구조: "{RS}agr1{RS}arg2 ... {RS}argN" : **첫 문자가 {RS}**
            // XX는 16진수(ascii)로 변환된 실행할 함수의 번호
            */
            String makePayload() {

                String payload; 
                size_t N = _vecStr.size(); //저장된 params의 갯수
                if (N>0) {
                    int nBytes = N; //payload의 길이(전체 바이트수)
                    for (size_t i=0; i<N; ++i) {
                        nBytes += _vecStr[i].length();             
                    }
                    payload.reserve(nBytes);             
                    for (size_t i=0; i<N; ++i) {
                        payload += RS;          //"payload += (RS+_vecStr[i]);"라고 하면
                        payload += _vecStr[i];  // 쓸데없이 임시객체가 생성되어 비효율적임
                    }
                }
                return payload;

            }

            ///*
            //2025/08/10: Params객체를 int로 캐스팅하는 연산자들 추가
            //(혼동을 피하기 위해 **암시변환 금지**)
            // int v(getArg()); 혹은 int v=(int)getArg(); 형태로 사용
            explicit operator bool() {return getBool();}
            explicit operator char() {return getChar();}
            explicit operator byte() {return getByte();}
            explicit operator short() {return getShort();}
            explicit operator unsigned short() {return getUShort();}
            explicit operator int() {return getInt();}
            explicit operator unsigned int() {return getUInt();}
            explicit operator long() {return getLong();}
            explicit operator unsigned long() {return getULong();}
            explicit operator float() {return getFloat();}
            explicit operator double() {return getDouble();}
            explicit operator String() {return getString();}
            //*/

        private:

            std::vector<String> _vecStr;
            size_t _idGet = 0;

            // float용 sprintf %g를 이용한 간단한 방법
            // sprintf()함수에서 서식문자 %g는 자동으로 일반 표기법과 지수 표기법 중 더 짧은 것을 선택
            static String _floatToStr(float v) {
                char buf[32];  // float를 문자열로 변환하기에 충분한 크기
                sprintf(buf, "%g", v); //"%g"는 고정 정밀도 6자리 유효숫자
                return String(buf);
            }

            static String _doubleToStr(double v) {
                char buf[64];  // double은 더 긴 문자열이 될 수 있으므로 큰 버퍼 사용
                sprintf(buf, "%.15g", v);//정밀도 15가 들어간다.
                return String(buf);
            }

    }; //class Params


    Params argParams; // (함수 실행에 필요한) 인수를 저장하는 Params객체
    Params retParams; // 사용자함수에서 반환값을 저장하는 Params객체 


    Packet exeFn(const String&);
    Packet makeExeFailPacket(const String&);


    namespace serial {

        const unsigned long BAUDRATE = 460800; // 921600 << **460800** >> 230400 > 115200

        void begin() {
            Serial.begin(BAUDRATE);
            //flush read/write serial buffer
            while(Serial.available()) Serial.read();
            Serial.flush();
        }


        //packet.payload는 EOT를 포함하지 않음
        void writePacket(const Packet& packet) {
            // CRC16(상위->하위바이트 순서로) => payload => EOT
            Serial.write(highByte(packet.crc));
            Serial.write(lowByte(packet.crc));
            Serial.print(packet.payload);
            Serial.write(EOT);
            packetLastSent = packet;
        }


        //이 함수가 호출된 시점에서 Serial.available()가 이미 true이다
        Packet readPacket() {
        
            Packet packet;
            // Serial.read()는 데이터가 없으면 -1, 있으면 [0,255]값을 갖는 int
            //(1) two bytes를 먼저 읽어서 CRC16을 저장
            uint16_t hb = (uint16_t)Serial.read();//상위바이트
            
            //int nbytes = Serial.available();
            //while(nbytes==0) nbytes = Serial.available(); //다음 데이터가 도착할 때 까지 기다린다.
            int nbytes;
            do { nbytes = Serial.available(); } while(nbytes == 0);

            packet.setCrc(hb, Serial.read());
            packet.reserve(nbytes-1);
            //(2) EOT가 전송될 때까지 읽어서 payload에 저장(EOT 제외)
            while (Serial.available()) {
                char c = Serial.read();
                if (c == EOT)  break;//(Serial전송의 끝을 알리는) {EOT}는 **제외**
                packet.addChar(c);
            } 

            return packet;

        } //Packet readPacket()

    
    }  //namespace serial



    namespace i2c {

        //const int MAX_I2C_PACKET_LEN = 250;
        const byte ADDR_DEFAULT = 0x40;
        volatile byte address = ADDR_DEFAULT;
        bool isMaster = false;

        namespace master{


            void begin() { //beginMaster()
                //Wire1의 기본핀이 26,27번이기 때문에 2,3번으로 교체
                Wire1.setSDA(2);
                Wire1.setSCL(3);
                Wire1.begin();
                Wire1.setClock(400000);//반드시 Wire1.begin()직후에 fast모드로 설정 (default:100kHz이므로)
                isMaster = true;
                //Serial.println("I2C Master 모드 초기화 완료");
            }


            int sendPacket(const byte slave_addr, const Packet& packet) {
                packetLastSent = packet;
                Wire1.beginTransmission(slave_addr);
                Wire1.write(highByte(packet.crc));
                Wire1.write(lowByte(packet.crc));
                Wire1.write((const byte*)packet.payload.c_str(), packet.payload.length());
                // I2C에서는 slave에서 전송될 byte수를 알 수 있으므로
                // EOT를 마지막에 보내지 않는다.
                return Wire1.endTransmission();
            }


            Packet processPacketToSlave(const byte slave_addr, const Packet& packet){
                

                byte count_try = 0;
                byte header, retPacketLength, receivedChecksum, computedChecksum;
                int sendResult;
                
                do{//checksum오류가 나도 처음부터 다시 시작한다(이게 알고리듬이 단순해짐)

                    if(++count_try > 10) {
                        //return retPacket;
                        return makeExeFailPacket("i2c communication error: Addr 0x"+String(address,HEX));
                    } 
                    sendResult = sendPacket(slave_addr, packet);
                    if(sendResult!=0) continue; //i2c 통신오류

                    Wire1.requestFrom(slave_addr, 3);    //3byte rq
                    while(Wire1.available()<3);          //데이터가 다 도착할 때까지 기다린다

                    header = Wire1.read();             // 'E'rr/'N'um /'M'ax
                    retPacketLength = Wire1.read();    // byte count of return packet
                    receivedChecksum = Wire1.read();  // checksum
                    
                    computedChecksum = ~(header + retPacketLength);    // 1의 보수checksum
                
                } while(receivedChecksum != computedChecksum or header == 'E'
#ifdef __DEBUG__I2C_MASTER__ //----------------------------------
                     or random(20)==0      
#endif //-------------------------------------------------------
                  );

                //2025/07/31: RP2040-zero보드에서 payload가 약 250byte 정도는 전송 가능
                //패킷 길이가 MAX_I2C_PACKET_LEN 보다 크면 첫 문자가 'M'이 넘어온다.
                if(header == 'M') {
                    return makeExeFailPacket("I2C slave (Addr 0x"+String(address,HEX)+") packet too long.");
                } 
                

                Packet retPacket;
                
                do {
                    //재전송일 수도 있으므로 반드시 retPacke을 여기서 초기화시켜야함
                    retPacket.init(retPacketLength-2);

                    Wire1.requestFrom(slave_addr, retPacketLength);
                    while(Wire1.available()<retPacketLength); //다 올 때까지 기다린다
                    uint16_t hb = (uint16_t)Wire1.read();
                    retPacket.setCrc(hb, (uint16_t)Wire1.read());
                    while (Wire1.available()) retPacket.addChar(Wire1.read());

                } while (!retPacket.isCrcMatched()
#ifdef __DEBUG__I2C_MASTER__ //------------------------------------
                    or random(20)==0
#endif //---------------------------------------------------------
                    );
                
                return retPacket;            
            }

        }//namespace i2c::master


        namespace slave {

            enum class RQ_PHASE {THREE_BYTES, RET_PACKET};
            volatile byte arr3b[3];
            volatile RQ_PHASE rqPhase = RQ_PHASE::THREE_BYTES;

#ifdef __DEBUG__I2C_SLAVE__ //-----------------------------------------------
            volatile uint16_t cntCrcErr1 = 0; //FOR DEBUG
            volatile uint16_t cntCrcErr2= 0; //FOR DEBUG
            volatile uint16_t cntCrcErr3= 0; //FOR DEBUG
            volatile RQ_PHASE rqPhase_1 = RQ_PHASE::RET_PACKET;
#endif //---------------------------------------------------------

            // 마스터가 데이터를 transmit했을 때 호출되는 콜백 함수
            void onReceive(int numBytes) {

                Packet packet;

                uint16_t hb = (uint16_t)Wire1.read();
                packet.setCrc(hb, Wire1.read());

                for (int i = 0; i < numBytes-2; i++) {
                    packet.addChar(Wire1.read());
                }
                
#ifdef __DEBUG__I2C_SLAVE__ //-------------------------------------
                if (random(20)==0) {
                    
                    arr3b[0] = 'E'; arr3b[1] = 0; arr3b[2] = ~arr3b[0];
                    rqPhase = RQ_PHASE::THREE_BYTES;
                    Serial.printf("crc1 Err:%d\n", cntCrcErr1++);

                    return;
                }
#endif//----------------------------------------------------------*/

                if (packet.isCrcMatched()) {
                    packetLastSent = exeFn(packet.payload);
                    int nbytes = packetLastSent.length();
                    if (nbytes >= MAX_PACKET_SIZE){
                        arr3b[0] = 'M';
                        arr3b[1] = 0;
                        arr3b[2] = ~arr3b[0];
                    } else {
                        arr3b[0] = 'N';
                        arr3b[1] = (byte)(2 + nbytes);
                        arr3b[2] = ~(arr3b[0] + arr3b[1]);
                    }
                    //Serial.printf("exeFn:0x%x[%s]:%d bytes\n",packet.crc,packetLastSent.payload.c_str(), nbytes);
                } else {
                    arr3b[0] = 'E';
                    arr3b[1] = 0;
                    arr3b[2] = ~arr3b[0];
                    //Serial.printf("rcv CRC error:%d\n", ++cntCrcErr);
                }

                rqPhase = RQ_PHASE::THREE_BYTES;

            }

            
            // 마스터가 request할 때 호출되는 콜백 함수
            void onRequest() {

#ifdef __DEBUG__I2C_SLAVE__ //-------------------------------------------
                if(rqPhase_1 == RQ_PHASE::THREE_BYTES and
                             rqPhase == RQ_PHASE::THREE_BYTES) 
                    Serial.printf("crc2 error:%d\n", ++cntCrcErr2);
                if(rqPhase_1 == RQ_PHASE::RET_PACKET and
                             rqPhase == RQ_PHASE::RET_PACKET) 
                    Serial.printf("crc3 error:%d\n", ++cntCrcErr3);
                rqPhase_1 = rqPhase;//FOR DEBUG
#endif //---------------------------------------------------------

                if (rqPhase == RQ_PHASE::THREE_BYTES) {

                    Wire1.write(arr3b[0]);
                    Wire1.write(arr3b[1]);
                    Wire1.write(arr3b[2]);
                    rqPhase = RQ_PHASE::RET_PACKET;

                } else if (rqPhase == RQ_PHASE::RET_PACKET) {

                    Wire1.write(highByte(packetLastSent.crc));
                    Wire1.write(lowByte(packetLastSent.crc));
                    Wire1.write((const byte*)packetLastSent.payload.c_str(), packetLastSent.payload.length());

                }

            }



            void begin() { //slave::begin()

                //RP2040-zero보드의 Wire1핀이 26,27번이기 때문에 2,3번으로 교체
                Wire1.setSDA(2);
                Wire1.setSCL(3);
                Wire1.begin(address); // 슬레이브 모드로 초기화
                Wire1.onReceive(onReceive); // 데이터 수신 콜백 등록
                Wire1.onRequest(onRequest); // 데이터 요청 콜백 등록
                
#ifdef __DEBUG__I2C_SLAVE__ //-------------------------------------------
                serial::begin();
                delay(2000);
                Serial.println("I2C Slave mode init (addr:0x" + String(address, HEX) + ")");
#endif //---------------------------------------------------------
            }

            
        }// namespce i2c::slave


        // 외부핀(PIN_MASTER_SLAVE)의 상태에 의해서 MASTER/SLAVE가 결정된다
        // PIN_MASTER_SLAVE핀은 내부에서 pulldown시킨 후 값을 읽는다.
        // 2025/08/08: **Master모드**일 때만 Serial을 시작한다.
        void beginMasterOrSlave() {

            pinMode(PIN_MASTER_SLAVE, INPUT_PULLDOWN); delay(1);
            byte state = digitalRead(PIN_MASTER_SLAVE);
            
            if (state == LOW) { //MASTER mode
                serial::begin();
                i2c::master::begin();
                setNeoPxColorRGB(255,0,0); //RED
            } else { //SLAVE mode 에서는 Serial을 시작하지 않는다.
                i2c::slave::begin();
                setNeoPxColorRGB(0,255,0); //GREEN
            }

        }
        
    }// namespace i2c


    typedef void (*PtrFunc)(void); // 함수 포인터 타입
    std::map<byte, PtrFunc> funcPtrMap;


    /* ---------------------------------------------------------------
    함수실행 packet 구조:
        [CRC16]'R'{RS}AD{RS}FN{RS}arg1{RS}arg2{RS}...{RS}argN
        AD는 두글자 16진수, FN은 한/두글자 16진수
    arg들 사이의 구분은 {RS}로 한다. args가 하나도 없을 수도 있다. 이경우 패킷은
        [CRC16]'R'{RS}AD{RS}FN
    ------------------------------------------------------------------*/

    Packet makeExeFailPacket(const String& msg) {
        Packet packet("F\x1e" + msg);
        packet.computeAndSetCrc();
        return packet;
    }


    Packet makeExeSuccessPacket() {
        String payload = 'S' + retParams.makePayload();
        Packet packet(payload);
        packet.computeAndSetCrc(); // CRC계산 후 패킷 완성
        return packet;
    }


    //serial통신으로 받은 packet처리
    //함수실행에 필요한 payload만 떼어서 String으로 반환
    String processSerialPacket(const Packet& packet){

        if ( !packet.isCrcMatched()

#ifdef __DEBUG__SERIAL__ //----------------------------------
        or random(20) == 0 //일부러 crc오류를 발생
#endif //-----------------------------------------------------
        
        ) { // CRC검증
            serial::writePacket(PACKET_CRC_ERR);
            return EMPTY_STR;
        }

        // CRC검증이 끝났다면 전송받은 packet은 완벽하다고 가정
        // payload: 'R'/'E'{RS}AD{RS}FN{RS}arg1{RS}arg2...{RS}argN
        const String& payload = packet.payload;
        char char0 = (char)payload[0];
        if (char0 == 'R') { 

            String strAddr = payload.substring(2,4);
            byte i2cAddr = (byte)strtol(strAddr.c_str(), NULL, 16); // 16진수를 변환
            if (i2cAddr==0) { //이 device에서 실행하는 경우

                //function파트("FN{RS}arg1{RS}...{RS}argN")만 반환
                return payload.substring(5);

            } else { //i2c슬레이브에서 실행하는 경우

                //function파트("FN{RS}arg1{RS}...{RS}argN")만 i2c로 전송
                Packet packetI2c(payload.substring(5));
                packetI2c.computeAndSetCrc();

                Packet retPacket = i2c::master::processPacketToSlave(i2cAddr, packetI2c);
                serial::writePacket(retPacket);
                return EMPTY_STR;

            }

        } else if (char0 == 'E') {// host PC에서 crc오류로 재전송을 요구한 경우
            
            serial::writePacket(packetLastSent);//재전송
            return EMPTY_STR;

        }

        return EMPTY_STR;
    }



    Packet exeFn(const String& payload) {
        //payload 구조: FN [ {RS}arg1{RS}arg2{RS}...{RS}argN ]

        //(1) 2025년7월22일 수정: FN(function_number)은 16진수 **한/두 자리**
        int idRS1 = payload.indexOf(RS);//첫 번째 {RS} index는 1 or 2이다.
        byte fn;
        if (idRS1 == -1) { // payload에 FN만 있는 경우
            fn = (byte)strtol(payload.c_str(), NULL, 16); // **16진수** 변환
        } else { //arg가 한 개 이상 있는 경우
            fn = (byte)strtol(payload.substring(0,idRS1).c_str(), NULL, 16); // **16진수** 변환
            argParams.addAll(payload.substring(idRS1+1)); //첫 번째 {RS}까지 제외한 나머지 문자열 전달
        }

        // (2) func_num에 해당하는 등록된 함수를 실행한다
        // 사용자함수 안에서 setReturn()함수를 호출하여 반환값(들)이 저장됨
        retParams.clear(); // 리턴값을 저장할 retParams 객체 초기화
        auto it = funcPtrMap.find(fn);
        if (it != funcPtrMap.end()) {
            it->second(); // 함수 실행
            return makeExeSuccessPacket();
        } else {
            return makeExeFailPacket("function #"+ String(fn) +" not found.");
        }
        
    }


    /*
    //203번 함수(고정)
    void who() {
        String str = strWho + "(I2C addr:0x"+String(i2c::address,HEX)+")";
        retParams.add(str);
    }
    */


    //201번 함수(고정):i2c어드레스를 바꾸는 함수
    void setI2cAddr() {

        byte newAddr =  argParams.getByte();
        // (AI)실무에선 7-bit 유효/안전 대역을 0x08–0x77로 두는 경우가 많다고 함
        if (newAddr <0x08 or newAddr > 0x79) {
            retParams.add("I2C address must be between 0x08 and 0x79. Fail to change I2C address.");
        } else if (newAddr == i2c::address) {
            retParams.add("I2C address is already 0x"+String(newAddr,HEX)+".");
        } else {
            EEPROM.write(0, newAddr);
            if (EEPROM.commit()){
                i2c::address = newAddr;
                retParams.add("I2C address successfully changed to 0x"+String(newAddr,HEX)+". Reset the board.");
            } else {
                retParams.add("Fail to change I2C address.");
            }
        }
    }


    //202번 함수
    void setNeopixel(){
        byte r = argParams.getByte();
        byte g = argParams.getByte();
        byte b = argParams.getByte();

        neoPx.setPixelColor(0, neoPx.Color(r, g, b)); // (R,G,B)
        neoPx.show(); 
    }


    //void setWhoStr(const String& strWhoR = strWho) {strWho = strWhoR; }


    //init()은 딱 한 번만 호출되어야 한다.
    bool isInitialized = false;

    void init(const String& strName = EMPTY_STR) {

        if (!strName.isEmpty()) {
            strWho = strName; //사용자가 지정한 strName으로 초기화
        }

        if(isInitialized) return;

        // i2c_slave_addr을 ROM에서 읽음
        EEPROM.begin(1); //RP2040zero
        i2c::address = EEPROM.read(0);
        if (i2c::address == 0xff) { //최초 실행시
            EEPROM.write(0, i2c::ADDR_DEFAULT);
            EEPROM.commit();
            i2c::address = i2c::ADDR_DEFAULT;
        }

        neoPx.begin(); //i2c::beginMasterOrSlave()보다 먼저 호출해야 함
        i2c::beginMasterOrSlave();

        // 0번 함수는 "ImChn"라는 문자열을 반환해서 Chaino장치를 인식하는데 사용
        funcPtrMap[0] = [](){retParams.add("ImChn");};
        funcPtrMap[201] = setI2cAddr;   //201번 고정
        funcPtrMap[202] = setNeopixel;  //202번 고정
        funcPtrMap[203] = [](){retParams.add(strWho);};    //203번 고정: String who()
        funcPtrMap[204] = [](){retParams.add(i2c::address);};    //204번 고정: byte get_addr()

        isInitialized = true;// init()은 **딱 한 번만** 호출되어야 한다.

    }


    inline void chainoLoop(){

        if (i2c::isMaster && Serial.available()) {
            Packet packet = serial::readPacket();
            String payloadExeFn = processSerialPacket(packet);
            if (payloadExeFn.isEmpty()) return;
            Packet retPacket = exeFn(payloadExeFn);
            serial::writePacket(retPacket);
        }

    }

}
/// \endcond


/**
 * @class Chaino
 * @brief High-level interface for registering, executing, and communicating functions 
 *        between devices via Serial, I2C or locally.
 *
 * The `Chaino` class provides a unified interface for:
 * - Registering user-defined functions with a function ID number (1–200)
 * - Executing those functions locally or on connected master/slave devices over I2C
 * - Passing arguments to functions and retrieving return values
 * - Managing I2C communication, argument parsing, and return parameter handling
 *
 * This class acts as the primary API for applications built on the Chaino protocol,
 * hiding lower-level details in the `chaino_detail` namespace.
 *
 * ### Typical usage:
 * @code
 * #include "Chaino.h"
 *
 * void setup() {
 *     Chaino::setup("Chaino_SomeDevice");
 * 
 *     // Register a Lambda function with ID 1
 *     Chaino::registerFunc(1, []() {
 *         int pin  = (int)Chaino::getArg();
 *         int mode = (int)Chaino::getArg();
 *         pinMode(pin, mode);
 *     });
 * }
 *
 * void loop() {
 *     Chaino::loop(); // Process incoming commands
 * }
 * @endcode
 *
 * @see registerFunc(), getArg(), setReturn(), execFunc()
 */
class Chaino {     
public:

    /**
     * @brief Constructor
     * @param addr I2C address (0 for master device)
     */
    Chaino(byte addr=0): _i2cAddr(addr) {}                  


    /**
     * @brief Initialize the Chaino system
     * @param name Device name string
     */
    static inline void setup(const String& name="") {chaino_detail::init(name);}
    
    /**
     * @brief Main processing loop - must be called in Arduino loop()
     */
    static inline void loop() {chaino_detail::chainoLoop();}


    /**
    * @brief Registers a user-defined function with a specific ID number.
    *
    * This function associates a function pointer (`PtrFunc`) with a unique ID number (1–200).
    * When the ID is later requested via an execution command (e.g., from I2C or serial),
    * the corresponding registered function will be executed.
    *
    * @param idNum The function ID number to register (valid range: **1–200**).
    *              IDs outside this range are ignored.
    * @param func  The function pointer (or lambda) to be registered.
    *              The function must have the signature:
    *              @code
    *              void func(void);
    *              @endcode
    *
    * @note If a function with the same `idNum` is already registered, it will be overwritten.
    * @note The ID range `1–200` is reserved for user functions. Other values are ignored.
    *
    * ### Example – Registering a function with ID 1:
    * @code
    * void myFunction() {
    *     int pin = (int)Chaino::getArg();
    *     int mode = (int)Chaino::getArg();
    *     pinMode(pin, mode);
    * }
    *
    * Chaino::registerFunc(1, myFunction);
    * @endcode
    *
    * ### Example – Registering a lambda:
    * @code
    * Chaino::registerFunc(2, []() {
    *     int pin = (int)Chaino::getArg();
    *     int adc = analogRead(pin);
    *     Chaino::setReturn(adc);
    * });
    * @endcode
    *
    * @see executeFunc(), getArg(), setReturn()
    */
    static void registerFunc(byte idNum, chaino_detail::PtrFunc func) {
        if(1<= idNum && idNum <= 200) {
            chaino_detail::funcPtrMap[idNum] = func;
        }
    }
    

    /**
    * @brief Retrieves the global parameter list object (`Params`) for reading arguments.
    *
    * This function returns a reference to the internal `chaino_detail::Params` object 
    * that stores the arguments for the currently executing function.  
    * 
    * @note The returned object supports multiple explicit type conversions  (bool, byte,
    * char, int, short, long, unsigned int, unsigned short, unsigned long, float, double, and String),
    * but these conversions are declared as `explicit` to prevent accidental type coercion.  
    * **Therefore, you must use an explicit cast** 
    *
    * ### Example – Explicit cast:
    * @code
    * int pin(Chaino::getArg());
    * float val(Chaino::getArg()); 
    * String s(Chaino::getArg());
    * @endcode
    *
    * ### Example – C style explicit cast:
    * @code
    * int pin  = (int)Chaino::getArg();
    * float val = (float)Chaino::getArg(); 
    * String s = (String)Chaino::getArg();
    * @endcode
    *
    * @return Reference to the global `Params` object containing the arguments.
    */
    static inline chaino_detail::Params& getArg() { return chaino_detail::argParams; }    


    /**
    * @brief Sets the return value(s) for the currently executing function.
    *
    * This function stores a return value into the global return parameter list (`retParams`)
    * so that it can be sent back to the caller (e.g., over I2C or serial communication).  
    *
    * @tparam T Type of the return value.(char, byte, int, short, long, float, double, String, bool, etc.)
    * @param v The value to be stored as a return parameter.
    *
    * @note It is recommnetd to call only once if there exists a return value.
    * although you can call `setReturn()` multiple times within the same function to return 
    * multiple values. The order of calls determines the order of returned parameters.
    *
    * ### Example – Returning a single integer:
    * @code
    * Chaino::setReturn(42); // Return one integer value
    * @endcode
    *
    * ### Example – Returning multiple values:
    * @code
    * Chaino::setReturn(3.14f);          // Return a float
    * Chaino::setReturn("OK");           // Return a string
    * Chaino::setReturn(true);           // Return a boolean
    * @endcode
    *
    */
    template<typename T>
    static inline void setReturn(const T& v) {chaino_detail::retParams.add(v);}


    /**
    * @brief Executes a registered function either locally or on a slave device over I2C.
    *
    * @param func_id  The function identifier (byte number between 1 and 200)
    * @param args     Zero or more arguments of any type
    *
    * @return `true` if the function executed successfully (response payload starts with 'S'),  
    *         `false` otherwise.
    *
    * @note The resulting return values can be accessed using `getReturn()` method.
    *
    * @see getReturn()
    */
    template<typename... Args>         
    bool execFunc(byte func_id, Args&&... args) {

        _args.clear();             
        ( _args.add(std::forward<Args>(args)), ... );// fold expression 사용(C++17)

        //함수번호(func_id)는 반드시 **16진수로** 인코딩해야한다             
        String payload = String(func_id, HEX) + _args.makePayload();

        chaino_detail::Packet retPacket;
        if (_i2cAddr == 0) { 
            // this (master) device에서 직접 실행
            retPacket = chaino_detail::exeFn(payload);

        } else { 
            // (i2c로 연결된) slave device에서 실행
            chaino_detail::Packet packet(payload);   // 전송할 payload로 Packet 생성
            packet.computeAndSetCrc();          // crc계산하여 저장        
            retPacket = chaino_detail::i2c::master::processPacketToSlave(_i2cAddr, packet);             

        }

        _rets.addAll(retPacket.payload.substring(2));//"Header{RS}"는 빼고 넘긴다
        return retPacket.payload.charAt(0) == 'S';          
        
    }
    


    /**
    * @brief Retrieves the 'Params' object for processing return value.
    *
    * This function returns a reference to the internal `Params` object that stores all 
    * return values from the last executed function. The `Params` object allows you
    * to cast explicitly into the specific type such as `int`, `float`, `String`, etc.
    *
    * @return Reference to the `Params` object containing return values from
    *         the last `execFunc()` call.
    *
    * ### Example – Direct access to return parameters:
    * @code
    * if (execFunc(15)) {
    *     int r = (int)getReturn();
    * }
    * @endcode
    *
    * ### Example – Processing variable return values:
    * @code
    * int read_digital(int pin) {
    *     execFunc(2, pin);
    *     return (int)getReturn();
    * }
    * @endcode
    *
    * @see execFunc(), getArg()
    */
    inline chaino_detail::Params& getReturn() {return _rets;}

    

    /**
    * @brief Changes the I2C address of the device.
    *
    * This function executes the built-in function to change the I2C slave address
    * of the target device. The new address is stored in EEPROM and requires a device
    * reset to take effect.
    *
    * @param newAddr The new I2C address to set (valid range: 0x01 to 0x79)
    *
    * @return String containing the result message:
    *         - Success: "I2C address successfully changed to 0xXX. Reset the board."
    *         - Already set: "I2C address is already 0xXX."
    *         - Invalid range: "I2C address must be between 0x01 and 0x79. Fail to change I2C address."
    *         - EEPROM error: "Fail to change I2C address."
    *
    * @note The valid I2C address range is 0x40-0x79 to avoid conflicts with reserved addresses.
    * @note A device reset is required after successful address change.
    * @note This function works on both local device (master) and remote I2C slave devices.
    *
    * ### Example:
    * @code
    * Chaino device(0x45);  // Target device at address 0x45
    * String result = device.set_addr(0x50);
    * Serial.println(result);  // Print the result message
    * 
    * // For local device
    * Chaino localDevice;
    * String result = localDevice.set_addr(0x48);
    * @endcode
    *
    */
    String set_addr(byte newAddr) {
        execFunc(201, newAddr);
        return (String)getReturn(); //결과를 알리는 문자열
    }
    
    

    /**
    * @brief Controls the onboard NeoPixel LED color.
    *
    * This function executes the built-in function to set the RGB color
    * of the device's onboard NeoPixel LED. The LED will immediately display
    * the specified color.
    *
    * @param r Red component (0-255)
    * @param g Green component (0-255)  
    * @param b Blue component (0-255)
    *
    * @note RGB values are clamped to the range 0-255.
    *
    * ### Example:
    * @code
    * Chaino device(0x45);  // Target device at address 0x45
    * device.set_neopixel(255, 0, 0);    // Set to red
    * device.set_neopixel(0, 255, 0);    // Set to green
    * device.set_neopixel(0, 0, 255);    // Set to blue
    * device.set_neopixel(255, 255, 0);  // Set to yellow
    * device.set_neopixel(0, 0, 0);      // Turn off LED
    * 
    * // For local device
    * Chaino localDevice;
    * localDevice.set_neopixel(128, 64, 192);  // Set to purple
    * @endcode
    *
    */
    void set_neopixel(int r, int g, int b) {
        execFunc(202, r, g, b);
    }
    


    /**
    * @brief Retrieves the device identification string.
    *
    * This function executes the built-in function get the device's
    * identification information, including the device name and current I2C address.
    * The device name is set during `Chaino::setup()` initialization, and if not
    * provided, defaults to "Chaino_Unknown".
    *
    * @return String containing device identification in the format:
    *         "DeviceName (I2C addr.:0xXX)"
    *         - DeviceName: The name set during `Chaino::setup(name)` or "Chaino_Unknown" if not set
    *         - 0xXX: Current I2C address in hexadecimal format (e.g., 0x40, 0x45)
    *
    * @note This function works on both local device (master) and remote I2C slave devices.
    *
    * ### Example – Basic device identification:
    * @code
    * // In setup, device was initialized with:
    * // Chaino::setup("MySensor");
    * 
    * Chaino device(0x45);  // Target device at address 0x45
    * String deviceInfo = device.who();
    * Serial.println(deviceInfo);  // Output: "MySensor (I2C addr.:0x45)"
    * 
    * // For local device
    * Chaino localDevice;
    * String localInfo = localDevice.who();
    * Serial.println(localInfo);   // Output: "MySensor(I2C addr.:0x40)"
    * @endcode
    *
    * @see Chaino::setup(), set_i2c_address()
    */
    String who() {
        execFunc(203);
        return (String)getReturn();
    }


    /**
    * @brief Gets the current I2C address of the target device.
    *
    * @return The current I2C address as a 7-bit value (e.g., 0x40).
    *
    * @note The returned value reflects the I2C address
    * when the device is used as a slave one.
    *
    * ### python Example – Read local and remote addresses
    * @code
    * Chaino local("COM9");                // master (this device)
    * byte a0 = local.get_addr();  // e.g., 0x40
    *
    * Chaino sensor("COM9", 0x45);         // remote slave at 0x45 through Serial 
    * byte a1 = sensor.get_addr(); // returns 0x45 (device's current address)
    * @endcode
    *
    * ### Micropython Example – Read local and remote addresses
    * @code
    *
    * Chaino sensor(0x45);         // remote slave at 0x45
    * byte a1 = sensor.get_addr(); // returns 0x45 (device's current address)
    * @endcode
    *
    * @see set_addr()
    */
    byte get_addr() {
        execFunc(204);
        return (byte)getReturn();
    }


protected:
    byte _i2cAddr;         

private:          
    chaino_detail::Params _args;
    chaino_detail::Params _rets;

};