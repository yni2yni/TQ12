// ADS Touch Sensor Test Example Program (IC P/N: TQ12 )
// Code: 
// Date: 2019.12.13  Ver.: 0.0.3
// H/W Target: ARDUINO UNO R3 / Leonardo, S/W: Arduino IDE  1.8.10
// Author: Park, Byoungbae (yni2yni@hanmail.net)
// Note: More information? Please send to e-mail.
// Uno R3, A4:SDA, A5: SCL, Leonardo 2:SDA,3:SCL

#include <Wire.h>

#define LF        0x0A //New Line
#define CR        0x0D //Carriage  return
#define SPC       0x20 //Space

// Sensitivity level (threshold, Register Value X 0.1% = (1 Step=0.1%)              
#define Sensitivity1    0x02 //ch1,Default: 0x07 X 0.1% = 0.70% (threshold)
#define Sensitivity2    0x03 //ch2
#define Sensitivity3    0x04 //ch3
#define Sensitivity4    0x05 //ch4
#define Sensitivity5    0x06 //ch5
#define Sensitivity6    0x07 //ch6
#define Sensitivity7    0x08 //ch7
#define Sensitivity8    0x09 //ch8
#define Sensitivity9    0x0A //ch9
#define Sensitivity10   0x0B //ch10
#define Sensitivity11   0x0C //ch11
#define Sensitivity12   0x0D //ch12

#define CTRL1  0x0E //System Control Register 1 (BF Mode, FTC, SingleMode, RTC)
#define CTRL2  0x0F //CDisTimeOpt, CSImpSel, SW Reset, Sleep, FastRespMode, FastRespEn)
#define Output1	0x10 //Touch Output Data Register 1Byte (OUT8~OUT1)
#define Output2	0x11 //Touch Output Data Register 1Byte (OUT8~OUT1)

#define Channel_Reset1	0x12 // Channel Reset Register(ch8~ch1),
//0: Enable Operation(Sensing + Calibration), 
//1: Reset Operation (No Sensing + Reset  Calibration)

#define Channel_Reset2	0x13 // Channel Reset Register(ch12~ch9),
//0: Enable Operation(Sensing + Calibration), 
//1: Reset Operation (No Sensing + Reset  Calibration)

#define Calibration_Hold1	0x14 // Calibration Holde Register(ch8~ch1),
//0: Enable Reference Calibration (Sensing + Calibration), 
//1: Disable Reference Calibration ( Sensing + No Calibration)

#define Calibration_Hold2	0x15 // Calibration Holde Register(ch12~ch9),
//0: Enable Reference Calibration (Sensing + Calibration), 
//1: Disable Reference Calibration ( Sensing + No Calibration)

#define Error_Percnet	0x29 //Error Percent Register
//ErrModeOpt, ErrMode_Diable, ErrPcntOpt

#define CR_Cal_Speed	0x2A //CR Channel Calibration Speed Control Register
#define CS_Cal_Speed	0x2B //CS Channel Calibration Speed Control Register

#define Sleep_Time_Control	0x2C //Sleep Time Control Register

#define CR_Cap_Control	0x30 //CR Channel Internal Capacitaice Register
#define CS1_Cap_Control	0x31 //CS1 Channel Internal Capacitaice Register
#define CS2_Cap_Control	0x32 //CS2 Channel Internal Capacitaice Register
#define CS3_Cap_Control	0x33 //CS3 Channel Internal Capacitaice Register
#define CS4_Cap_Control	0x34 //CS4 Channel Internal Capacitaice Register
#define CS5_Cap_Control	0x35 //CS4 Channel Internal Capacitaice Register
#define CS6_Cap_Control	0x36 //CS6 Channel Internal Capacitaice Register
#define CS7_Cap_Control	0x37 //CS7 Channel Internal Capacitaice Register
#define CS8_Cap_Control	0x38 //CS8 Channel Internal Capacitaice Register
#define CS9_Cap_Control	0x39 //CS9 Channel Internal Capacitaice Register
#define CS10_Cap_Control	0x3A //CS10 Channel Internal Capacitaice Register
#define CS11_Cap_Control	0x3B //CS11 Channel Internal Capacitaice Register
#define CS12_Cap_Control	0x3C //CS12 Channel Internal Capacitaice Register

#define Voltage_Threshold_Control1	0x3D //Voltage_Threshold_Control1 Register
#define Voltage_Threshold_Control2	0x3E //Voltage_Threshold_Control2 Register

#define Lock_OP_Enable	0x59 //I2C Write Lock Operation Control Register
#define Lock_Byte	0x5A 	//I2C Write Lock Operation Control Register

#define CTRL3	0x5B 	//System Control Register 3

// =============== TQ12 I2C Chip Slave Address ================================================
#define TQ12_ID_GND  0x68 //7bit address: 8bit address 0xD0>>1 //ID_SEL Pin = GND (Default)
#define TQ12_ID_VDD  0x78 //7bit address: 8bit address 0xF0>>1 //ID_SEL Pin = VDD

void  Init_TQ12(void); //Initialize TQ12

#define RESET_PIN 7 //Reset pin
#define EN_PIN    6 //I2C Enable Pin

void Register_Dump()
{
  byte read_data[1]={0};
 
  for( int i =0; i < 256; i+=16 )
  {
    for (int j = 0; j <= 15; j++)
    {
      Wire.beginTransmission(TQ12_ID_GND); // sned ic slave address
      Wire.write((i+j));             // sends register address
      Wire.endTransmission();                // stop transmitting
      Wire.requestFrom(TQ12_ID_GND, 1);      // key data read (2 byte)
      read_data[0] = Wire.read();            //Key 1~8
      Wire.endTransmission();
      print2hex(read_data, 1); //
    }
    Serial.write(LF);
    Serial.write(CR);
  }
  delay(500);
}

void print2hex(byte *data, byte length) //Print Hex code
{
       Serial.print("0x"); 
       for (int i=0; i<length; i++) { 
         if (data[i]<0x10) {Serial.print("0");} 
         Serial.print(data[i],HEX); 
         Serial.write(SPC);          
       }
}

void setup(){
  delay(100); //wait for 100[msec] for MCU BOARD
  
  Wire.begin();        // join i2c bus (address optional for master)
  Wire.setClock(2000000);// 200Khz
  Serial.begin(115200);  // start serial for output (Speed)
  // put your setup code here, to run once:
  pinMode(RESET_PIN, OUTPUT); //Reset Pin
  pinMode(EN_PIN, OUTPUT); // Enable Pin
 
//---------- Initialize TQ12 --------------------------------------------------
  delay(300); //please waiting 300[msec] after ic power on.
  Init_TQ12();// Initialize TQ12
  delay(10); // 
//---------------------- End --------------------------------------------------    
}
void loop() {

  byte read_data[2]={0};
 
  // Touch Key read
  Wire.beginTransmission(TQ12_ID_GND); // sned ic slave address
  Wire.write(byte(Output1)); // sends register address
  Wire.endTransmission(); // stop transmitting
  Wire.requestFrom(TQ12_ID_GND,2); // key data read (2 byte)
  read_data[0]=Wire.read(); //Key 1~8
  read_data[1]=Wire.read(); //Key 9~12
  Wire.endTransmission(); // 
   
  Serial.write(10); 
  Serial.print(" Touch Sensor Output Data (Hex) ---- > ");// Test Code
  
  print2hex(read_data,2);
  //Serial.print(read_data,HEX); 
  //Serial.write(SPC); 
  //Serial.write(LF);   
  //Serial.write(CR);
  
  //Register_Dump();

  delay(50);   
}

void  Init_TQ12(void)
{
  //-------------------- Register write Lock control ------------------------
  //   When using the lock function, add a function register at the beginning and the end.
  //   Wire.beginTransmission(TQ12_ID_GND);//
  //   Wire.write(byte(Lock_OP_Enable)); // address 0x59h
  //   Wire.write(byte(0x80)); // data 0x80: Unlock(Default), 0x01: locking
  //   Wire.endTransmission(); //

  //------------------ Software Reset Enable (Set)----------------------
  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(CTRL2));             //
  Wire.write(byte(0x39));              // Impedance Low, Fast Response Enable, Disable Sleep
  Wire.endTransmission();              //

  // --------------- Hidden Register Start ---------------------------------
  // user does not change the register. if you want change value, please contact to us.
  // -----------------------------------------------------------------------

  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(0x16);                    // Start Address 0x16h
  Wire.write(0x00);                    // Data  ,0x16h
  Wire.write(0x33);                    // Data  ,0x17h
  Wire.write(0x33);                    // Data  ,0x18h
  Wire.write(0x33);                    // Data  ,0x19h
  Wire.write(0x33);                    // Data  ,0x1Ah
  Wire.write(0x33);                    // Data  ,0x1Bh
  Wire.write(0x33);                    // Data  ,0x1Ch
  Wire.write(0x03);                    // Data  ,0x1Dh
  Wire.endTransmission();              //

  // --------------- Hidden Register End-------------------------------

  // ---------------- User Control Resgiter ---------------------//
  //------------ Sensitivity control  -----------------------------------
  // touch output threshold % level:
  // Do not set more Sensitivity tthan 1.0%

  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(Sensitivity1));      // 0x02h
  Wire.write(0x10);                    // HEX Value x 0.1% = 1.6%
  Wire.endTransmission();              //

  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(Sensitivity2));      // 0x03h
  Wire.write(0x10);                    // HEX Value x 0.1% = 1.6%
  Wire.endTransmission();              //

  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(Sensitivity3));      // 0x04h
  Wire.write(0x10);                    // HEX Value x 0.1% = 1.6%
  Wire.endTransmission();              //

  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(Sensitivity4));      // 0x05h
  Wire.write(0x10);                    // HEX Value x 0.1% = 1.6%
  Wire.endTransmission();              //

  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(Sensitivity5));      // 0x06h
  Wire.write(0x10);                    // HEX Value x 0.1% = 1.6%
  Wire.endTransmission();              //

  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(Sensitivity6));      // 0x07h
  Wire.write(0x10);                    // HEX Value x 0.1% = 1.6%
  Wire.endTransmission();              //

  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(Sensitivity7));      // 0x08h
  Wire.write(0x10);                    // HEX Value x 0.1% = 1.6%
  Wire.endTransmission();              //

  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(Sensitivity8));      // 0x09h
  Wire.write(0x10);                    // HEX Value x 0.1% = 1.6%
  Wire.endTransmission();              //

  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(Sensitivity9));      // 0x0Ah
  Wire.write(0x10);                    // HEX Value x 0.1% = 1.6%
  Wire.endTransmission();              //

  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(Sensitivity10));     // 0x0Bh
  Wire.write(0x10);                    // HEX Value x 0.1% = 1.6%
  Wire.endTransmission();              //

  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(Sensitivity11));     // 0x0Ch
  Wire.write(0x10);                    // HEX Value x 0.1% = 1.6%
  Wire.endTransmission();              //

  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(Sensitivity12));     // 0x0Dh
  Wire.write(0x10);                    // HEX Value x 0.1% = 1.6%
  Wire.endTransmission();              //
                                       //------------ Sensitivity control  End-----------------------------------

  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(CTRL1));             // 0x0Eh
  Wire.write(0x05);                    //Burst Fast Mode, FTC 5sec, Multi-Output, Response 5+1
  Wire.endTransmission();              //

  // ----------- Channel_Reset Register1 (Sensing Channel On/Off---------------
  // The unused CS Channel must be disabled.
  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(Channel_Reset1));    // 0x12h
  Wire.write(0x00);                    // 0: Sensing, 1: No Sensing
  Wire.endTransmission();              //

  // ----------- Channel_Reset Register2 (Sensing Channel On/Off---------------
  // The unused CS Channel must be disabled.
  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(Channel_Reset2));    // 0x13h
  Wire.write(0x00);                    // 0: Sensing, 1: No Sensing
  Wire.endTransmission();              //

  // ----------- Calibration Hold Register1 (Calibration on/off) --------------
  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(Calibration_Hold1)); // 0x14h
  Wire.write(0x00);                    // 0: Sensing + Calibration, 1: Sensing + No Calibration
  Wire.endTransmission();              //

  // ----------- Calibration Hold Register1 (Calibration Oon/off) --------------
  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(Calibration_Hold2)); // 0x15h
  Wire.write(0x00);                    // 0: Sensing + Calibration, 1: Sensing + No Calibration
  Wire.endTransmission();              //

  // ------------ Error Percent Register --------------------------------------
  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(Error_Percnet));     // 0x29h
  Wire.write(0x20);                    //
  Wire.endTransmission();              //

  // ------------ Calibration Speed Control Register -----------------------------
  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(CR_Cal_Speed));      // 0x2Ah
  Wire.write(0x43);                    //
  Wire.endTransmission();              //

  // ------------ Calibration Speed Control Register ----------------------------
  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(CS_Cal_Speed));      // 0x2Bh
  Wire.write(0x43);                    //
  Wire.endTransmission();              //

  // ------------ Sleep Time Control Register -----------------------------------
  Wire.beginTransmission(TQ12_ID_GND);  //
  Wire.write(byte(Sleep_Time_Control)); //0x2Ch
  Wire.write(0x09);                     //
  Wire.endTransmission();               //

  // ------------ System Control Register 3 -------------------------------------
  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(CTRL3));             //0x5Bh
  Wire.write(0xB0);                    //
  Wire.endTransmission();              //

  //------------------ Software Reset Disable (Clear) ---------------------
  Wire.beginTransmission(TQ12_ID_GND); //
  Wire.write(byte(CTRL2));             //
  Wire.write(byte(0x31));              //CS Impedance Low, Fast Response Enable,Disable Sleep
  Wire.endTransmission();              //

  //-------------------- Register write Lock control ------------------------
  //   When using the lock function, add a function register at the beginning and the end.
  //   Wire.beginTransmission(TQ12_ID_GND);//
  //   Wire.write(byte(Lock_OP_Enable)); // address 0x59h
  //   Wire.write(byte(0x01)); // data 0x80: Unlock(Default), 0x01: locking
  //   Wire.endTransmission(); //
 }
// End
