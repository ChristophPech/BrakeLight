//Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>
#include <EEPROM.h>
#include "TimerOne.h"

int LED1=2;
int LED2=3;
int inPin=8;

//Assign the Chip Select signal to pin 10.
int CS=10;

//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
char POWER_CTL = 0x2D;  //Power Control Register

char DATA_FORMAT = 0x31;
char DF_SELFTEST = 0x80;
char DF_FULLRES = 0x8;
char DF_2G=0x00;
char DF_4G=0x01;
char DF_8G=0x02;
char DF_16G=0x03;

char BW_RATE=0x2C;
char BR_3200=0x0F;
char BR_1600=0x0E;
char BR_0800=0x0D;
char BR_0400=0x0C;
char BR_0200=0x0B;
char BR_0100=0x0A;

char OFSX=0x1E; //calibration
char OFSY=0x1F;
char OFSZ=0x20;

char DATAX0 = 0x32; //X-Axis Data 0
char DATAX1 = 0x33; //X-Axis Data 1
char DATAY0 = 0x34; //Y-Axis Data 0
char DATAY1 = 0x35; //Y-Axis Data 1
char DATAZ0 = 0x36; //Z-Axis Data 0
char DATAZ1 = 0x37; //Z-Axis Data 1

int printcnt=0;
long mt_last=0;
bool bFlip=false;
double freq=0;
long frecount=0;
double fResA=0;
double fResB=0;
const double cut=-0.15;

//This buffer will hold values read from the ADXL345 registers.
char values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z;

void setup(){ 
  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  
  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(inPin, INPUT);

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

  //Create a serial connection to display the data on the terminal.
  Serial.begin(9600);
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

/*long serOkCnt=0;
bool SerialCheck() {
  if(!Serial) return false;
  int av=Serial.availableForWrite();
  if(av>=32) {serOkCnt=0;return true;}
  while(serOkCnt<4) {
    delay(1);
    av=Serial.availableForWrite();
    if(av>=32) {serOkCnt=0;return true;}
  }
  return false;
}*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// -> http://www.schwietering.com/jayduino/filtuino/index.php
//Band pass bessel filter order=1 alpha1=0.0001 alpha2=0.001 
//2000Hz 0.2->2
class  FilterBeBp1
{
  public:
    FilterBeBp1() {reset();}
  private:
    double v[3];
  public:
    void reset() {v[0]=0.0;v[1]=0.0;v[2]=0.0;}
    double step(double x) //class II 
    {
      v[0] = v[1];
      v[1] = v[2];
      v[2] = (1.220571346719886898e-3 * x)
         + (-0.99774060763864502732 * v[0])
         + (1.99773997669730540849 * v[1]);
      return 
         (v[2] - v[0]);
    }
};
class  FilterBeBp2
{
  public:
    FilterBeBp2() {reset();}
  private:
    double v[3];
  public:
    void reset() {v[0]=0.0;v[1]=0.0;v[2]=0.0;}
    double step(double x) //class II 
    {
      v[0] = v[1];
      v[1] = v[2];
      v[2] = (5.295785479650015571e-4 * x)
         + (-0.99899519533613778677 * v[0])
         + (1.99899487966745104117 * v[1]);
      return 
         (v[2] - v[0]);
    }
};

FilterBeBp1 g_xFilter;
FilterBeBp2 g_xFilter2;

////////////////////////////////////////////////////////////////////////////////////
bool bWritten=false;
void loop() {
  /*
  if(Serial&&Serial.availableForWrite()==63&&frecount>0) {
      Serial.print((fResA*100), DEC);

      Serial.print(',');
      Serial.print((fResB*200), DEC);

      Serial.print(',');
      Serial.print((int)(cut*200), DEC);

      Serial.print(',');
      Serial.print((freq), DEC);
      Serial.print(',');
      Serial.print((frecount), DEC);

      
      Serial.print(',');
      Serial.print((freq/frecount), DEC);


      double dA;
      long dF;
      int x;
      EEPROM.get(8*0, dA);
      EEPROM.get(8*1, dF);
      EEPROM.get(8*2, x);

      Serial.print('$');
      Serial.print(dA, DEC);
      Serial.print('$');
      Serial.print(dF, DEC);
      Serial.print('$');
      Serial.print(x, DEC);
      
      Serial.print('\n');
    }/**/


    /*
    int val = digitalRead(inPin);
    if(val&&!bWritten) {
      bWritten=true;

      EEPROM.put(8*0, fResA);
      EEPROM.put(8*1, frecount);
      EEPROM.put(8*2, x);
    }
    digitalWrite(LED2, bWritten?HIGH:LOW);
    /**/
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

  //float xg=x*4/pow(2,10);
  //float yg=y*4/pow(2,10);
  //float zg=z*4/pow(2,10);

  float xg=((float)x);
  float zg=((float)z);
  float len=sqrt(xg*xg+zg*zg);
  xg/=len;zg/=len;
  float a=atan2(xg,zg);

  fResA=g_xFilter.step(a);
  fResB=g_xFilter2.step(a);
  if(isnan(fResA)||isinf(fResA)) g_xFilter.reset();
  if(isnan(fResB)||isinf(fResB)) g_xFilter2.reset();

  const int PMAX=200;
  printcnt++;
  if(printcnt>PMAX) {
    printcnt=0;

   long t=millis();
   long d=t-mt_last;mt_last=t;
   float fFrq=((float)PMAX)/d;
   freq+=fFrq;
   frecount++;

    digitalWrite(LED1, fResA<cut?HIGH:LOW);
    digitalWrite(LED2, fResB<cut?HIGH:LOW);
    //digitalWrite(LED2, x<0?HIGH:LOW);
    //digitalWrite(LED2, (frecount%2)==0?HIGH:LOW);
    //digitalWrite(LED2, bFlip?HIGH:LOW);bFlip=!bFlip;
    
  }

  
  //delay(10); 
}



