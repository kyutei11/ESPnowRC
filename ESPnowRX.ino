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
// * if not PWM, Input Only and no Pinup mode
// * if using WiFi, ADC2 cannot be used

/*
----------------------------
PWM

Bit	
16  65536      1,220.70Hz
15  32768      2,441.41Hz
14  16384      4,882.81Hz
13   8192      9,765.63Hz
12   4096     19,531.25Hz
11   2048     39,062.50Hz
10   1024     78,125.00Hz
 9    512    156,250.00Hz
 8    256    312,500.00Hz
 7    128    625,000.00Hz
 6     64  1,250,000.00Hz


 * LEDC Ch to Group/Channel/Timer Mapping
** ledc: 0  => Group: 0, Channel: 0, Timer: 0
** ledc: 1  => Group: 0, Channel: 1, Timer: 0
** ledc: 2  => Group: 0, Channel: 2, Timer: 1
** ledc: 3  => Group: 0, Channel: 3, Timer: 1
** ledc: 4  => Group: 0, Channel: 4, Timer: 2
** ledc: 5  => Group: 0, Channel: 5, Timer: 2
** ledc: 6  => Group: 0, Channel: 6, Timer: 3
** ledc: 7  => Group: 0, Channel: 7, Timer: 3
** ledc: 8  => Group: 1, Channel: 0, Timer: 0
** ledc: 9  => Group: 1, Channel: 1, Timer: 0
** ledc: 10 => Group: 1, Channel: 2, Timer: 1
** ledc: 11 => Group: 1, Channel: 3, Timer: 1
** ledc: 12 => Group: 1, Channel: 4, Timer: 2
** ledc: 13 => Group: 1, Channel: 5, Timer: 2
** ledc: 14 => Group: 1, Channel: 6, Timer: 3
** ledc: 15 => Group: 1, Channel: 7, Timer: 3
*/

#include <WiFi.h>
#include <esp_now.h>
#include <EEPROM.h>

/*
                            LI CAP. 250F (or Li-ion BATT.)
      10k     10k               +  - 
GND --vvv--+--vvv--+-------------||--------------GND
           |       |    FET
CapV <-----+       +----| |----+---[Motor]---+---GND
                      D  =  S  |             |
                         |G    +------||-----+ 
TH   >-------------------+          0.1-1uF : if necessary
                            (coreless motor : no freewheel current : no diode)

ESP32/Servo +5V : use DC-DC converter

note : DC Motor Max.Voltage will be 2.5V
       LI Cap Voltage in [2.5..3.8]V
*/

//                       vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv copy your TX ESP32 address
const uint8_t addr[6] = {0x**, 0x**, 0x**, 0x**, 0x**, 0x**}; // TX Wifi Mac address
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

const uint16_t NoCH = 3; // CH0: TH
const uint16_t NoSW = 6; // ENTER, CH_*(+,-), Joystick
const uint16_t NoTS = 0; //

const int CH[NoCH]={22,32,33}; // pwm output : for Bboard
//const int CH[NoCH]={26,14,12}; // pwm output : TH, SV1, SV2
const int CapV = 35; // use AD ch1... ch2 is not available when using WiFi

const int TH_CH = 15; // no need to change : see above

const int MaxTRIM = 500;

uint8_t  SW_RX[NoSW], TS_RX[NoTS], EEP;
uint16_t iSW, iTS, iCH, CH_RX[NoCH], CHKSUM;

unsigned long lastRXmSec;
const int SigLost_mSec = 1000; // mSec of connection loss -> safe mode

int16_t   TRIM[NoCH];
int       trimCount;
const int MaxTrimCount = 2; // x20 mSec = trim T up/down tick

// lines below : no effect for CH0(TH)
const float SafeMode[NoCH] = {0.0, 0.0, 0.0}; // [0..1] x maxDeflection : also for safe mode
const float MaxDeflection[NoCH]={0.0, 1.0, 1.0}; // also you set CW/CCW here (by +-)

// ESP32 A/D has nonlinearity near Upper/Lower bound, so using only 10%..90% AD range
const int AD_ClipVolt = 300; // input : 0..3.3V --> output : 0.3..3.0V

int   mV_Cap;
float V_Motor, V_Cap;
const float CAP_LPF = 0.02;
int   Motor_OK = 0;

void setup() {

  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // before init.
  if (esp_now_init() == ESP_OK)
	Serial.println("ESP-Now Init Success");
  else
 	Serial.println("ESP-Now Init Fail");

  esp_now_register_recv_cb(RecvData);

  // pinMode
  pinMode(CapV, ANALOG);
  for(iCH=0; iCH<NoCH; iCH++) pinMode(CH[iCH], OUTPUT);

  // TH : coreless motor require high freq.
  //        vv using diff. timer other than SV to use different freq.
  ledcSetup(TH_CH, 50000, 10); //  (channel, Hz, maxbit)
  ledcAttachPin(CH[0], TH_CH); //  GPIO pin, PWM channel

  // other SV CH
  // PWM : about 50Hz (T = 20mSec)
  // Pulse width : 1..2mSec
  // neutral 1.5mSec <--> 5000 count : T = 19.66mSec -> 50.86Hz
  // nominal pulse width : 2000 count (3000-7000count) -> min 0.9mSec .. max 2.1mSec

  // RX data : 300-3000 mV -> neutral : 1650, deflection : 1350
  
  for(iCH=1; iCH<NoCH; iCH++){
	CH_RX[iCH] = 0;      // zero value also used in initial loop
  	//           vv diff. timer than TH...
  	ledcSetup(iCH, 50.86, 16);   //  (channel, Hz, maxbit)
  	ledcAttachPin(CH[iCH], iCH); // GPIO pin , PWM channel
  }
  
  V_Cap = 2.5; // init.
  ledcWrite(TH_CH, 0); // set TH 0

  EEPROM.begin(2*NoCH); // int16 (for each trim except CH0=TH: )

  // no trim for TH(iCH=0) : using as 1st written or not CHK instead
  // Magic No. for EEPROM init. detection : 31415
  // init Val. may be 0xFFFF..
  EEPROM.get(0, TRIM[0]);
  if(TRIM[0] == 31415){
	Serial.print("trim already set.\n");
	for(iCH=1; iCH<NoCH; iCH++){ // note : from 1
	  EEPROM.get(iCH*2, TRIM[iCH]);
	  Serial.print("Trim"); Serial.print(iCH); Serial.print('='); Serial.print(TRIM[iCH]);
	}
  }
  
  else{ // 1st run init.
	TRIM[0] = 31415;
	EEPROM.put(0, TRIM[0]);
	Serial.print("trim init. set to 0.\n"); // not be seen...
	for(iCH=1; iCH<NoCH; iCH++)
	  EEPROM.put(iCH*2, 0);
	EEPROM.commit();
  }

  trimCount = 0;
  lastRXmSec = millis();
}

void loop()
{//  vvvvvvvvvvvvvvvvvvv dont worry about overflow... its OK
 if((millis()-lastRXmSec) > SigLost_mSec ) { // signal lost
	// output preset failsafe position
	Serial.print("Signal Lost.\n");

	ledcWrite(TH_CH, 0); // TH=0

	// neutral:5000pulse +- deflection:2000 + trim
	for(iCH=1; iCH<NoCH; iCH++) // servo
	  ledcWrite(iCH, (int)(SafeMode[iCH] * MaxDeflection[iCH]* 2000.0) + 5000 + TRIM[iCH]);

	delay(100);
  }
}

// ----------------------------------------------------------------------
  
void RecvData(const uint8_t *mac_addr, const uint8_t *RXdata, int dataSize)
{
  int PWM_out, EEPflag;
  char macStr[18];
  float deflection;

  // reset count if RXdata, else signal loss
  lastRXmSec = millis();

  /* Serial.println(); */
  /* snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", */
  /* 		   mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]); */
  /* Serial.printf("Last RX Packet from: %s\n", macStr); */
  /* Serial.printf("Last Packet RX Data(%d): ", dataSize); */

  /* for (i = 0; i < dataSize; i++) { */
  /* 	Serial.print(RXdata[i]); */
  /* 	Serial.print(" "); */
  /* } */
  /* Serial.println(); */

  CHKSUM = 0;
  // Serial.print(3500); // for plot : MAX
  
  for(iCH=0; iCH<NoCH; iCH++){
	// MSB
	CHKSUM += RXdata[2*iCH]  ; CH_RX[iCH]  = RXdata[2*iCH]<<8;

	// LSB
	CHKSUM += RXdata[2*iCH+1]; CH_RX[iCH] += RXdata[2*iCH+1];

	//Serial.print("iCH_mV="); Serial.print(iCH); Serial.print(','); Serial.println(CH_RX[iCH]);
	// Serial.print(','); Serial.print(CH_RX[iCH]); // for plot

	if(CH_RX[iCH] <      AD_ClipVolt) CH_RX[iCH] =      AD_ClipVolt; // mV
	if(CH_RX[iCH] > 3300-AD_ClipVolt) CH_RX[iCH] = 3300-AD_ClipVolt;
  }
  // Serial.println(); // for plot
 
  // lower : NoSW bit, upper: TouchSensor    
  // Serial.print("iSW=");
  for(iSW=0; iSW<NoSW; iSW++){
	SW_RX[iSW] = (RXdata[2*NoCH]>>iSW) & 1 ;
	// Serial.print(SW_RX[iSW]);
  }
  // Serial.println();
  
  // upper NoTS bit
  for(iTS=0; iTS<NoTS; iTS++){
	TS_RX[iTS] = (RXdata[2*NoCH]<<iTS) & 0b10000000;
	// Serial.print("iTS="); Serial.print(iTS); Serial.print(','); Serial.println(TS_RX[iTS]);
  }
  CHKSUM += RXdata[2*NoCH];

  CHKSUM &= 0x00FF;

  //getUint16 = ((unsigned int)MSB << 8) | LSB

  // *************************
  // check trim
  trimCount += 1;

  if(trimCount > MaxTrimCount){
  	trimCount = 0;
  	Serial.print("TRIM = ");
  	for(iCH=1; iCH<NoCH; iCH++){ // from 1
  	  if(SW_RX[iCH*2-1] == 1) TRIM[iCH]+= 1; // SW==LOW --> SW_RX = 1 
  	  if(SW_RX[iCH*2  ] == 1) TRIM[iCH]-= 1;

  	  if(TRIM[iCH] >  MaxTRIM) TRIM[iCH] =  MaxTRIM;
  	  if(TRIM[iCH] < -MaxTRIM) TRIM[iCH] = -MaxTRIM;
  	  Serial.print(TRIM[iCH]); Serial.print(" ");
  	}
  	Serial.println();
  }
  
  // ENTER
  if(SW_RX[0] == 1){
  	// write EEPROM if its value is not current TRIM
  	EEPflag = 0; // check diff.
  	for(iCH=1; iCH<NoCH; iCH++){ // from 1
  	  EEPROM.get(iCH*2, TRIM[0]); // CH0 : dummy : for CHK
  	  if(TRIM[0] != TRIM[iCH]){
		EEPflag = 1; // set flag
  		EEPROM.put(iCH*2, TRIM[iCH]);
  	  }
  	}
  	if(EEPflag == 1){
  	  EEPROM.commit(); // prevent writing too many time :
  	  Serial.println("wrote TRIM to EEPROM");
  	}
	// wait while chattering
	delay(50);
  }

  // **********************************************
  // Ctrl output
  // no output update if CHKSUM is not RXdata[last]
  if(CHKSUM == RXdata[2*NoCH+1]){

	// ----------------------
	// TH output: 10bit(1024)
	// TH_RX = 300-3000 mV 
	// 300  -> V_Motor 0.0V
	// 3000 -> V_Motor 2.5V
	// 
	// V_Cap * (PWM/1024) = V_Motor

	// V_Cap -> numerical LPF
	mV_Cap  = 2 * analogReadMilliVolts(CapV); // 10k-10k Ohm : 0.5*Vcap
	V_Cap   = V_Cap*(1.0-CAP_LPF) + CAP_LPF*(0.001*(float)mV_Cap); // LPF
	V_Motor = 2.5*((float)(CH_RX[0]-300)/2700.0);

	// Motor_OK = 0:init cond. before TH up
	//            1:flight
	//            2:low V cutoff
	if(Motor_OK == 1){
	  // V_Cap < 3.0V : reduce power x0.5
	  // V_Cap < 2.5V : cutoff

	  if(V_Cap < 2.5){
		PWM_out  = 0;
		Motor_OK = 2;
	  }
	  else{
		PWM_out = (int)(1024.0*V_Motor/V_Cap);
		if(V_Cap < 3.0) PWM_out = PWM_out/2;
	  }

	  ledcWrite(TH_CH, PWM_out);
	
	  // comment out these lines below after test..
	  /* Serial.println("TH"); */

	  /* Serial.print("V_Cap  ="); Serial.println(V_Cap); */
	  /* Serial.print("V_Motor="); Serial.println(V_Motor); */
	  /* Serial.print("PWM_out="); Serial.println(PWM_out); */
	}
	else if(Motor_OK == 0){
	  ledcWrite(TH_CH, 0);
	  // CHK TH is OFF at init.state
	  if(CH_RX[0] <= AD_ClipVolt) Motor_OK = 1;
	}
	
	// ----------------------
	// servo output
	// neutral:5000pulse(=1.5msec) +- deflection:2000 + trim  <-----> RX : 1650 +- 1350

	// Serial.println("Servo:");
	for(iCH=1; iCH<NoCH; iCH++){
	  // Serial.print(iCH);
	  //               0..1          CH_RX : 300-3000 (3000+300)/2     (3000-300)/2
	  deflection = MaxDeflection[iCH]* (float)(CH_RX[iCH]-1650  )*(2000.0 / 1350.0);
	  PWM_out = (int)deflection + 5000 + TRIM[iCH];
	  ledcWrite(iCH, PWM_out);
	  
	  // Serial.print(": PWM_out="); Serial.println(PWM_out);
	}
  }
  else{ // not likely...
	Serial.println("CHKsum error...");

	Serial.print("CHKsum="); Serial.println(CHKSUM);
	Serial.print("RXdata="); Serial.println(RXdata[2*NoCH+1]);
  }

}
  
