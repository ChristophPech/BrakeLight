//Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>
#include <EEPROM.h>
#include "TimerOne.h"

const int LIGHT_PWM_PIN=5;  //PWM pin for mosfet gate
const char PWM_DUTY_NORMAL=50; //normal back light (50=20%)
const char PWM_DUTY_BRAKING=255; //brake light duty (255=100%)
const int MIN_BRAKE_ON_DURATION=500; //(milliseconds) keep brake light on for at least

const int STATUS_LED_PIN=7; //optional blinking led to signal function without main light


const int CS=10; //SPI Chip Select: pin 10.

//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
const char POWER_CTL = 0x2D;  //Power Control Register

const char DATA_FORMAT = 0x31;
const char DF_SELFTEST = 0x80;
const char DF_FULLRES = 0x8;
const char DF_2G=0x00;
const char DF_4G=0x01;
const char DF_8G=0x02;
const char DF_16G=0x03;

const char BW_RATE=0x2C;
const char BR_3200=0x0F;
const char BR_1600=0x0E;
const char BR_0800=0x0D;
const char BR_0400=0x0C;
const char BR_0200=0x0B;
const char BR_0100=0x0A;

const char OFSX=0x1E; //calibration
const char OFSY=0x1F;
const char OFSZ=0x20;

const char DATAX0 = 0x32; //X-Axis Data 0
const char DATAX1 = 0x33; //X-Axis Data 1
const char DATAY0 = 0x34; //Y-Axis Data 0
const char DATAY1 = 0x35; //Y-Axis Data 1
const char DATAZ0 = 0x36; //Z-Axis Data 0
const char DATAZ1 = 0x37; //Z-Axis Data 1

int printcnt=0;
long on_last=0;
bool bFlip=false;
double freq=0;
long frecount=0;
const double cut=-0.15;

double redX=0,redY=0,redZ=0;
int redC=0;
unsigned int blink_cnt=0;

//This buffer will hold values read from the ADXL345 registers.
char values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z;

#define DBG 0

void setup(){ 
  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  
  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);

  pinMode(LIGHT_PWM_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);


  writeRegister(BW_RATE, BR_3200); //highest sampling rate
  //rough calibration
  writeRegister(OFSX, 0x05);
  writeRegister(OFSY, 0xfc);
  writeRegister(OFSZ, 0xa0);
  
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, DF_FULLRES);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode  

  Timer1.initialize(500);
  Timer1.attachInterrupt(pollData); // pollData run @ 2kHz

#if DBG
  //Create a serial connection to display the data on the terminal.
  Serial.begin(115200);
#endif
}


//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;
  
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// -> http://www.schwietering.com/jayduino/filtuino/index.php

double filter_x[3];
double filter_y[3];
double filter_z[3];

void filter_reset(double* v) {
  v[0]=0.0;v[1]=0.0;v[2]=0.0;
}

//Band pass bessel filter order=1 alpha1=0.01 alpha2=0.1 
//20hz, 0.2 -> 2
double filter_step(double* v,double x) {
      v[0] = v[1];
      v[1] = v[2];
      v[2] = (2.420745665556250092e-1 * x)
         + (-0.54975465219277031004 * v[0])
         + (1.51842542608997210785 * v[1]);
      return 
         (v[2] - v[0]);
}


////////////////////////////////////////////////////////////////////////////////////
void loop() {
}

void pollData(){
  //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
  //The results of the read operation will get stored to the values[] buffer.
  readRegister(DATAX0, 6, values);

  //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  //The X value is stored in values[0] and values[1].
  //x = ((int)values[1]<<8)|(int)values[0];
  //The Y value is stored in values[2] and values[3].
  //y = ((int)values[3]<<8)|(int)values[2];
  //The Z value is stored in values[4] and values[5].
  //z = ((int)values[5]<<8)|(int)values[4];

  x =(int)((((unsigned int)(values[1]&255)) << 8)|(unsigned int)(values[0])&255); 
  y =(int)((((unsigned int)(values[3]&255)) << 8)|(unsigned int)(values[2])&255); 
  z =(int)((((unsigned int)(values[5]&255)) << 8)|(unsigned int)(values[4])&255);

  double xg=x*4/pow(2,10);
  double yg=y*4/pow(2,10);
  double zg=z*4/pow(2,10);

  redX+=xg;
  redY+=yg;
  redZ+=zg;
  redC++;
  if(redC<200) return;

  redX/=redC;
  redY/=redC;
  redZ/=redC;

  double resX=filter_step(filter_x,redX);//if(isnan(resX)||isinf(resX)) filter_reset(filter_x);
  double resY=filter_step(filter_y,redY);//if(isnan(resY)||isinf(resY)) filter_reset(filter_y);
  double resZ=filter_step(filter_z,redZ);//if(isnan(resZ)||isinf(resZ)) filter_reset(filter_z);

  redX=0;redY=0;redZ=0;redC=0;
  const float resA=resY;
  const float resB=-resZ;

  float len=sqrt(resA*resA+resB*resB);
  float a=atan2(resA/len,resB/len);
  a*=len;

  int on=(len>0.175f&&a<-0.175)?HIGH:LOW;

  long t=millis();
  if(on) on_last=t;
  long d=t-on_last;
  if(d<MIN_BRAKE_ON_DURATION) on=true;

  analogWrite(LIGHT_PWM_PIN,on?PWM_DUTY_BRAKING:PWM_DUTY_NORMAL);

  blink_cnt++;
  if(on) {
    digitalWrite(STATUS_LED_PIN, (blink_cnt<2)?HIGH:LOW);
    if (blink_cnt>2) blink_cnt=0;
  } else {
    digitalWrite(STATUS_LED_PIN, (blink_cnt<4)?HIGH:LOW);
    if (blink_cnt>8) blink_cnt=0;
  }

#if DBG
  if(Serial&&Serial.availableForWrite()==63) {
    Serial.print(resA, DEC);
    Serial.print(',');
    Serial.print(resB, DEC);
    //Serial.print(',');
    //Serial.print(zg, DEC);

    Serial.print(',');
    Serial.print(on, DEC);
    Serial.print(',');
    Serial.print(a, DEC);
    Serial.print(',');
    Serial.print(len, DEC);
    Serial.print('\n');
  };
#endif
  return;

}




