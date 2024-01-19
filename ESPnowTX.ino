//                                      -------------
//                                      |    ANT    |
//                                      |-----------|
//                                3V3 --|           |-- GND
//                        RESTART, EN --|           |-- 23(PWM, VSPI_MOSI)
// (           ADC1_0        , SVP)36 --|           |-- 22(PWM, I2C_SCL)
// (           ADC1_3        , SVN)39 --|           |-- 01(PWM, TX0)
// (           ADC1_6             )34 --|           |-- 03(PWM, RX0)
// (           ADC1_7             )35 --|           |-- 21(PWM, I2C_SDA)
// (           ADC1_4, TOUCH9, PWM)32 --|           |-- GND
// (           ADC1_5, TOUCH8, PWM)33 --|           |-- 19(PWM, VSPI_MISO)
// (           ADC2_8,   DAC1, PWM)25 --|           |-- 18(PWM, VSPI_CLK)
// (           ADC2_9,   DAC2, PWM)26 --|           |-- 05(PWM, VSPI_CS)
// (           ADC2_7, TOUCH7, PWM)27 --|           |-- 17(PWM, TX2)
// (HSPI_CLK , ADC2_6, TOUCH6, PWM)14 --|           |-- 16(PWM, RX2)
// (HSPI_MISO, ADC2_5, TOUCH5, PWM)12 --|           |-- 04(PWM, ADC2_0, TOUCH0)
//                                GND --|           |-- 00(PWM, ADC2_1, TOUCH1)
// (HSPI_MOSI, ADC2_4, TOUCH4, PWM)13 --|           |-- 02(PWM, ADC2_2, TOUCH2, LED)
//                                 5V --|           |-- 15(PWM, ADC2_3, TOUCH3, HSPI_CS)
//                                      -------------
//
// * if not PWM, Input Only and no Pullup mode
// * if using WiFi, ADC2 cannot be used

/*

No.1
[Wi-Fi STA] 1C:9D:C2:53:02:C8
[Bluetooth] 1C:9D:C2:53:02:CA

No.2 
[Wi-Fi STA] 24:D7:EB:48:C8:08
[Bluetooth] 24:D7:EB:48:C8:0A

No.3
[Wi-Fi STA] 1C:9D:C2:52:B3:B8
[Bluetooth] 1C:9D:C2:52:B3:BA

No.5 
[Wi-Fi STA] 24:D7:EB:48:CD:CC
[Bluetooth] 24:D7:EB:48:CD:CE

No.6
[Wi-Fi STA] 1C:9D:C2:52:E3:2C
[Bluetooth] 1C:9D:C2:52:E3:2E

No.7
[Wi-Fi STA] 44:17:93:6C:4F:E4
[Bluetooth] 44:17:93:6C:4F:E6

No.8
[Wi-Fi STA] 44:17:93:6C:A6:08

=================

      GR LED : Vf=2V   
BATT+ >----->|-----+ (BATT : NiMH x 4 cell)
                   |
LEDout >----vvv----+
            220    |
BATchk <-----------+


================
A/D  Input CH:

      Bcurve VR
3V3 >---vvvvv---GND
          |
          V
         Ain

================
Touch sensor :

T* <---=====
       panel
*/

#include <WiFi.h>
#include <esp_now.h>

#define NoCH 3
#define NoSW 6 // trimENTER, trimCH_*(+,-), JOYstick*NoStick
#define NoTS 0 // 

#define LEDout 15 // for BBox
//#define LEDout 33
#define BATchk 34 // for BBox

const int ACH[NoCH]={35,32,33};
const int SCH[NoSW]={14, 18,5, 16,17, 12}; // Trim-Enter, TRIM[+-]for 2ch , JoyStick_SW*1

const uint8_t addr[6] = {0x44, 0x17, 0x93, 0x6C, 0xA6, 0x08}; // RX Wifi STA address : board No.8
//const uint8_t addr[6] = {0x1C, 0x9D, 0xC2, 0x53, 0x02, 0xC8}; // RX Wifi STA address : board No.1
/****************************************
// use this code to know wifi Mac address
#include "WiFi.h"
void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  delay(100);
  Serial.println();
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
}
void loop(){}
*****************************************/

uint16_t iSW, iCH, iTS, CH,CH_prev[NoCH], chkSUM, CAP_V;

//     16bit for each CH
// next 8bit for each SW and Touch sensor (max. SW+Touch is 8)
// last 8bit for CHKsum
uint8_t TXdata[2*NoCH+2]; // CH1..NoCH(MSB LSB), SW/Touch, CHKSUM

//const int TCH[NoTS] = {15, 12,14,33,32};
const int TCH[NoTS] = {};
float TouchSens[NoTS], TouchSens0[NoTS]; // float : apply LPF
const float TS_LPF   = 0.15;
const float TS_thres = 0.75;

uint8_t c_dum[1]; float f_dum[1]; int i_dum[1];

// low V chk
int   LowVolCount;
float Vbatt;
const float BAT_LPF = 0.02;

//int LEDblinkT = 5; // about 1sec : for test
int LEDblinkT = 25; // about 1sec
const float Low_mV = 4000.0; // V_Low_mV - Vf_mV(LED) : <--- V_Low = 2.0V
const float  Vf_mV = 2000.0; // modify this for your LED

int mode;

void setup(){

  Serial.begin(115200);

  pinMode(LEDout, OUTPUT);
  pinMode(BATchk, ANALOG);
  
  for(iCH=0; iCH<NoCH; iCH++) pinMode(ACH[iCH], ANALOG);
  for(iSW=0; iSW<NoSW; iSW++) pinMode(SCH[iSW], INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // before init...
  if (esp_now_init() == ESP_OK) {
	Serial.println("ESP-Now Init Success");
  }

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, addr, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if(esp_now_add_peer(&peerInfo) != ESP_OK) {
	Serial.println("Failed to add peer");
	return;
  }

  esp_now_register_recv_cb(RecvData); // downlink

  // BATchk LED
  digitalWrite(LEDout, LOW); // LED on
  LowVolCount = 1;

  delay(500); // wait until V stabilize
  Vbatt = (float)analogReadMilliVolts(BATchk) + Vf_mV;

  // get no-touch value // also until Vin stabilize
  for(iTS=0; iTS<NoTS; iTS++){
	TouchSens0[iTS] = 100.0; // almost no touch	
	for(int i=0; i<500; i++){
	  TouchSens0[iTS] = TouchSens0[iTS]*(1.0-TS_LPF) + ((float)touchRead(TCH[iTS]))*TS_LPF; // almost no touch
	  delay(1);
	}
	TouchSens[iTS] = TouchSens0[iTS];// init.
  }

  // modi. V.0.24.01.18 : added LPF to A/D
  for(iCH=0; iCH<NoCH; iCH++)
	CH_prev[iCH] = analogReadMilliVolts(ACH[iCH]); // 0..about3300 mV (usable : 300..3000mV)
}

void loop() { // default : core1
  int i;
  chkSUM = 0;

  for(iCH=0; iCH<NoCH; iCH++){
	// modi. V.0.24.01.18 : added LPF to A/D
	for(i=0; i<10; i++){
	  CH = CH_prev[iCH]/2 + analogReadMilliVolts(ACH[iCH])/2; // 0..about3300 mV (usable : 300..3000mV)
	  delayMicroseconds(2000-60*NoCH); // *10 ~= 20+mSec : 60uS for each AD
	  CH_prev[iCH] = CH;
	}
	// Serial.print("CH"); Serial.print(iCH); Serial.println(CH);
	
	// MSB
	TXdata[2*iCH]   = CH>>8;
	chkSUM += TXdata[2*iCH];
	// LSB
	TXdata[2*iCH+1] = CH;
	chkSUM += TXdata[2*iCH+1];
  }

  // lower : NoSW bit, upper: TouchSensor  
  TXdata[2*NoCH] = 0;
  for(iSW=0; iSW<NoSW; iSW++)
	if(digitalRead(SCH[iSW]) == LOW)
	  TXdata[2*NoCH] += 1 << iSW;

  for(iTS=0; iTS<NoTS; iTS++){
	TouchSens[iTS] = TouchSens[iTS]*(1.0-TS_LPF) + ((float)touchRead(TCH[iTS]))*TS_LPF;
	// Serial.print("TS"); Serial.print(iTS); Serial.println(TouchSens[iTS]/TouchSens0[iTS]);
	if(TouchSens[iTS] < TS_thres * TouchSens0[iTS])
	  TXdata[2*NoCH] += 0b10000000 >> iTS;
  }
  chkSUM += TXdata[2*NoCH];

  // last : checksum
  TXdata[2*NoCH+1] = 0x00FF & chkSUM;

  /* Serial.print("TX_CHKSUM : "); */
  /* Serial.print(TXdata[2*NoCH+1]); */
  /* Serial.print("\n"); */
  
  esp_err_t result = esp_now_send(addr, TXdata, sizeof(TXdata));

  // BATT chk
  if(LowVolCount > -LEDblinkT/2){
	digitalWrite(LEDout, LOW); // LED on
	// check low voltage while LED on
	// LPF
	Vbatt = Vbatt*(1.0-BAT_LPF) + BAT_LPF*((float)analogReadMilliVolts(BATchk) + Vf_mV);
	if((Vbatt < Low_mV) && LowVolCount > 0)
	  LowVolCount = 0; // Low V flag
  }
  
  else  digitalWrite(LEDout, HIGH);

  //Serial.print("BATchk ="); Serial.println((int)Vbatt); Serial.print("mV\n");
  
  // blink, T~=1Hz
  if(LowVolCount <= 0        ) LowVolCount--;
  if(LowVolCount < -LEDblinkT) LowVolCount = 0;	  

  //-----------
  // delay(20);
  // delay(100); // for test
}

// downlink
void RecvData(const uint8_t *mac_addr, const uint8_t *RXdata, int dataSize)
{
  /* Serial.print("RX_CHKSUM : "); */
  /* Serial.print(RXdata[2*NoCH+1]); */
  /* Serial.print("\n"); */
}


