#include <Wire.h>
#include <LiquidCrystal.h>

#define DEVICE_ADXL345 (0x53) // Device address as specified in data sheet 

byte _buff_ADXL345[6];
float cog,a1,a2,a3;
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(37, 36, 35, 33, 32, 31, 30);
int BL = 34;
//Porting Driver Motor Vexta
int cwccw1 = 53;
int runbrake1 = 52;
int startstop1 = 51;
int cwccw2 = 50;
int runbrake2 = 10;
int startstop2 = 11;

char POWER_CTL_ADXL345 = 0x2D;	//Power Control Register
char DATA_FORMAT_ADXL345 = 0x31;
char DATAX0_ADXL345 = 0x32;	//X-Axis Data 0
char DATAX1_ADXL345 = 0x33;	//X-Axis Data 1
char DATAY0_ADXL345 = 0x34;	//Y-Axis Data 0
char DATAY1_ADXL345 = 0x35;	//Y-Axis Data 1
char DATAZ0_ADXL345 = 0x36;	//Z-Axis Data 0
char DATAZ1_ADXL345 = 0x37;	//Z-Axis Data 1

// gryo ITG3200 
#define GYRO 0x68 // configue address, connect AD0 to GND, address in Binary is 11101000 (this please refer to your schematic of sensor)
#define G_SMPLRT_DIV 0x15
#define G_DLPF_FS 0x16
#define G_INT_CFG 0x17
#define G_PWR_MGM 0x3E
 
#define G_TO_READ 8 // x,y,z every axis 2 bytes
 
// for the offest
//int g_offx = 67;
//int g_offy = 5;
//int g_offz = 41;

// for the offest
int g_offx = 0;
int g_offy = 0;
int g_offz = 0;
 
// for the offest
int a_offx = -30;
int a_offy = -8;
int a_offz = 0;
  
char str[512]; 

//initilize gyro
void initGyro()
{
  /*****************************************
   * ITG 3200
   * power managerment setting
   * clock setting = internal clock
   * no reset mode, no sleep mode
   * no standby mode
   * resolution = 125Hz
   * parameter is + / - 2000 degreee/second
   * low pass filter=5HZ
   * no interruption
   ******************************************/
  writeTo(GYRO, G_PWR_MGM, 0x00);
  writeTo(GYRO, G_SMPLRT_DIV, 0x07); // EB, 50, 80, 7F, DE, 23, 20, FF
  writeTo(GYRO, G_DLPF_FS, 0x1E); // +/- 2000 dgrs/sec, 1KHz, 1E, 19
  writeTo(GYRO, G_INT_CFG, 0x00);
}
int fuzzy(int k){
float nb=0,z=0,pb=0,ns=0,ps=0,x;
x=(float)k;
if(x<-16&&-24<=x)
{

nb=((-16)-x)/8;}
else if(x<-8&&-16<=x){
nb=(x-(-16))/8;
ns=((-8)-x)/8;
}
else if(x<-6&&-8<=x){
     nb=(x-(-16))/8;
ns=((-8)-x)/2;}
     }
     else if(x<0&&-6<=x){
z=((-6)-x
)/6;          
          }
		  else if(x<6&&0<=x){
z=(x-0)/6;          
          }
else if(x<8&&6<=x){
ps=(8-x)/2;
}
else if(x<16&&8<=x){
ps=(x-8)/8;
pb=(10-x)/8;
}
else if(x>16){
pb=(x-16)/8;
}
else{
     nb=1;
   pb=1}


if(k==0){
         cog=1;
         }
         else if(k<0){
a1=nn*(-1750);
a2=ns*(-1550);
a3=z*(-500);
a4=ps*(850);
a5=pb*(1050);                 }
              else{   
a1=nn*(-1750);
a2=ns*(-1550);
a3=z*(500);
a4=ps*(850);
a5=pb*(1050)
}
cog=(a1+a2+a3+a4+a5)/(ns+nb+z+ps+pb);
int h;
h=(int)cog;
return h;
}




int getGyroscopeData()//int getGyroscopeData(int * result)
{
  /**************************************
   * gyro ITG- 3200 I2C
   * registersï¼š
   * temp MSB = 1B, temp LSB = 1C
   * x axis MSB = 1D, x axis LSB = 1E
   * y axis MSB = 1F, y axis LSB = 20
   * z axis MSB = 21, z axis LSB = 22
   *************************************/
 
  int regAddress = 0x1B;
  int temp, x, y, z,k;
  byte buff[G_TO_READ];
 
  readFrom(GYRO, regAddress, G_TO_READ, buff); //read gyro itg3200 data
 
  //result[0] = ((buff[2] << 8) | buff[3]) + g_offx;
  k = /*result[1] =*/ ((buff[4] << 8) | buff[5]) + g_offy;
  //result[2] = ((buff[6] << 8) | buff[7]) + g_offz;
  //result[3] = (buff[0] << 8) | buff[1]; // temperature 
lcd.setCursor(5,0);
  lcd.print("        ");
  lcd.setCursor(5,0);
  lcd.print(k);
return k+8;
}
 
 
void setup()
{
  Serial.begin(9600);
  Wire.begin();
  
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeTo_ADXL345(DATA_FORMAT_ADXL345, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeTo_ADXL345(POWER_CTL_ADXL345, 0x08);
  
  initGyro();
  
  pinMode(cwccw1, OUTPUT); 
  pinMode(runbrake1, OUTPUT);
  pinMode(startstop1, OUTPUT);
  pinMode(cwccw2, OUTPUT); 
  pinMode(runbrake2, OUTPUT);
  pinMode(startstop2, OUTPUT);  
  
  lcd.begin(16, 2);  
  // Print a message to the LCD.
  lcd.print("  ROBOT badut  ");  
  pinMode(BL, OUTPUT);
  digitalWrite(BL, LOW);   // turn the LED on (HIGH is the voltage level)  
  delay(1000); 
  lcd.clear(); 
}
 
int sp1=2000;
int sp2 =1000;
int x;
int g;
void loop()
{


 
  x=readAccel(); // read the x/y/z tilt

  if(readAccel()<-6)
  {
    mundur(x,x);
  }
  else if(readAccel()=6)
  maju(x,x);
  {berhenti(1000,1000);
  }
  delay(1);
  
  }
  
  

  
    //(1/1000ms) SETING WAKTU bisa diganti monggoooo.....!!!!!!


//------------------------- Accelero -----------------------//
int readAccel() {
  uint8_t howManyBytesToRead_ADXL345 = 6;
  readFrom_ADXL345( DATAX0_ADXL345, howManyBytesToRead_ADXL345, _buff_ADXL345); //read the acceleration data from the ADXL345

  // each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  // thus we are converting both bytes in to one int
  int x_ADXL345 = (((int)_buff_ADXL345[1]) << 8) | _buff_ADXL345[0];   
 x_ADXL345 +=8;
 Serial.print("x_A: ");
  Serial.print( x_ADXL345 );
 lcd.setC`ursor(0,0);
  lcd.print("        ");
  lcd.setCursor(0,0);
  lcd.print(x_ADXL345);
 
  /*int y_ADXL345 = (((int)_buff_ADXL345[3]) << 8) | _buff_ADXL345[2];
  int z_ADXL345 = (((int)_buff_ADXL345[5]) << 8) | _buff_ADXL345[4];
  
  Serial.print("x_A: ");
  Serial.print( x_ADXL345 );
  
  Serial.print(" y_A: ");
  Serial.print( y_ADXL345 );
  
  Serial.print(" z_A: ");
  Serial.println( z_ADXL345 );    
  
  lcd.setCursor(0,0);
  lcd.print("        ");
  lcd.setCursor(0,0);
  lcd.print(x_ADXL345);          
  
  lcd.setCursor(5,0);
  lcd.print("        ");
  lcd.setCursor(5,0);
  lcd.print(y_ADXL345);
  
  lcd.setCursor(10,0);
  lcd.print("        ");
  lcd.setCursor(10,0);
  lcd.print(z_ADXL345); */ 
return x_ADXL345-2;
}

void writeTo_ADXL345(byte address_ADXL345, byte val_ADXL345) {
  Wire.beginTransmission(DEVICE_ADXL345); // start transmission to device 
  Wire.write(address_ADXL345);             // send register address
  Wire.write(val_ADXL345);                 // send value to write
  Wire.endTransmission();         // end transmission
}

// Reads num bytes starting from address register on device in to _buff array
void readFrom_ADXL345(byte address_ADXL345, int num_ADXL345, byte _buff_ADXL345[]) {
  Wire.beginTransmission(DEVICE_ADXL345); // start transmission to device 
  Wire.write(address_ADXL345);             // sends address to read from
  Wire.endTransmission();         // end transmission

  Wire.beginTransmission(DEVICE_ADXL345); // start transmission to device
  Wire.requestFrom(DEVICE_ADXL345, num_ADXL345);    // request 6 bytes from device

  int i_ADXL345 = 0;
  while(Wire.available())         // device may send less than requested (abnormal)
  { 
    _buff_ADXL345[i_ADXL345] = Wire.read();    // receive a byte
    i_ADXL345++;
  }
  Wire.endTransmission();         // end transmission
}
//----------------------- Gyroscope ------------------------------//  
//---------------- function
//write val into the address register of accelerometer
void writeTo(int DEVICE, byte address, byte val) {
  Wire.beginTransmission(DEVICE); //send to sensor
  Wire.write(address);        // send register address 
  Wire.write(val);        // send the value which needed to write
  Wire.endTransmission(); //end transmission 
}
 
 
//read data from the buffer array of address registers in the accelerometer sensor
void readFrom(int DEVICE, byte address, int num, byte buff[]) {
  Wire.beginTransmission(DEVICE); //start to send to accelerometer sensor
  Wire.write(address);        //send address which are read
  Wire.endTransmission(); //end transmission
 
  Wire.beginTransmission(DEVICE); //start to send to ACC
  Wire.requestFrom(DEVICE, num);    // require sending 6 bytes data from accelerometer sensor
 
  int i = 0;
  while(Wire.available())    //Error when the return value is smaller than required value
  { 
    buff[i] = Wire.read(); // receive data
    i++;
  }
  Wire.endTransmission(); //end transmission
}
//-------------- Function ----------------------//
//----- Robot Berhenti --------//    
void berhenti(int L, int R){       
  speed_control(L,R);
  digitalWrite(cwccw1, HIGH); 
  digitalWrite(runbrake1, HIGH);  
  digitalWrite(startstop1, LOW);    
  digitalWrite(cwccw2, LOW); 
  digitalWrite(runbrake2, HIGH);  
  digitalWrite(startstop2, LOW);
}
//----- Robot Maju --------//  
void maju(int L, int R){       
  speed_control(L,R);
  digitalWrite(cwccw1, HIGH); 
  digitalWrite(runbrake1, HIGH);  
  digitalWrite(startstop1, HIGH);    
  digitalWrite(cwccw2, LOW); 
  digitalWrite(runbrake2, HIGH);  
  digitalWrite(startstop2, HIGH);
}
//----- Robot Mundur --------//
void mundur(int L, int R){       
  speed_control(L,R);
  digitalWrite(cwccw1, LOW); 
  digitalWrite(runbrake1, HIGH);  
  digitalWrite(startstop1, HIGH);    
  digitalWrite(cwccw2, HIGH); 
  digitalWrite(runbrake2, HIGH);  
  digitalWrite(startstop2, HIGH);
}
//----- Robot Kanan --------//
void kanan(int L, int R){       
  speed_control(L,R);
  digitalWrite(cwccw1, LOW); 
  digitalWrite(runbrake1, HIGH);  
  digitalWrite(startstop1, HIGH);    
  digitalWrite(cwccw2, LOW); 
  digitalWrite(runbrake2, HIGH);  
  digitalWrite(startstop2, HIGH);
}
//----- Robot Kiri --------//
void kiri(int L, int R){       
  speed_control(L,R);
  digitalWrite(cwccw1, HIGH); 
  digitalWrite(runbrake1, HIGH);  
  digitalWrite(startstop1, HIGH);    
  digitalWrite(cwccw2, HIGH); 
  digitalWrite(runbrake2, HIGH);  
  digitalWrite(startstop2, HIGH);
}

void speed_control(int left, int right){
  Wire.beginTransmission(0b1100000);
  Wire.write(64);                        // cmd to update the DAC
  Wire.write(left >> 4);                 // the 8 most significant bits...
  Wire.write((left & 15) << 4);          // the 4 least significant bits...  
  Wire.endTransmission();
    //---------- Speed Control R--------//   
  Wire.beginTransmission(0b1100001);
  Wire.write(64);                        // cmd to update the DAC
  Wire.write(right >> 4);                // the 8 most significant bits...
  Wire.write((right & 15) << 4);         // the 4 least significant bits...  
  Wire.endTransmission();
}  

