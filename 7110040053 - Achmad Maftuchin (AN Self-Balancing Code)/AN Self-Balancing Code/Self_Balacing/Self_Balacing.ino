// MMA7455 Accelerometer
// ---------------------
// By arduino.cc user "Krodal".
// May 2012
// Open Source / Public Domain
//
// Using Arduino 1.0.1
// It will not work with an older version, since Wire.endTransmission()
// uses a parameter to hold or release the I2C bus.
//
// Documentation:
//     - The Freescale MMA7455L datasheet
//     - The AN3468 Application Note (programming).
//     - The AN3728 Application Note (calibrating offset).
//
// The MMA7455 can be used by writing and reading a single byte,
// but it is also capable to read and write multiple bytes.

// The accuracy is 10-bits.

// Register names according to the datasheet.
// Register 0x1C is sometimes called 'PW', and sometimes 'PD'.
// The two reserved registers can not be used.
#define MMA7455_XOUTL 0x00      // Read only, Output Value X LSB
#define MMA7455_XOUTH 0x01      // Read only, Output Value X MSB
#define MMA7455_YOUTL 0x02      // Read only, Output Value Y LSB
#define MMA7455_YOUTH 0x03      // Read only, Output Value Y MSB
#define MMA7455_ZOUTL 0x04      // Read only, Output Value Z LSB
#define MMA7455_ZOUTH 0x05      // Read only, Output Value Z MSB
#define MMA7455_XOUT8 0x06      // Read only, Output Value X 8 bits
#define MMA7455_YOUT8 0x07      // Read only, Output Value Y 8 bits
#define MMA7455_ZOUT8 0x08      // Read only, Output Value Z 8 bits
#define MMA7455_STATUS 0x09     // Read only, Status Register
#define MMA7455_DETSRC 0x0A     // Read only, Detection Source Register
#define MMA7455_TOUT 0x0B       // Temperature Output Value (Optional)
#define MMA7455_RESERVED1 0x0C  // Reserved
#define MMA7455_I2CAD 0x0D      // Read/Write, I2C Device Address
#define MMA7455_USRINF 0x0E     // Read only, User Information (Optional)
#define MMA7455_WHOAMI 0x0F     // Read only, "Who am I" value (Optional)
#define MMA7455_XOFFL 0x10      // Read/Write, Offset Drift X LSB
#define MMA7455_XOFFH 0x11      // Read/Write, Offset Drift X MSB
#define MMA7455_YOFFL 0x12      // Read/Write, Offset Drift Y LSB
#define MMA7455_YOFFH 0x13      // Read/Write, Offset Drift Y MSB
#define MMA7455_ZOFFL 0x14      // Read/Write, Offset Drift Z LSB
#define MMA7455_ZOFFH 0x15      // Read/Write, Offset Drift Z MSB
#define MMA7455_MCTL 0x16       // Read/Write, Mode Control Register
#define MMA7455_INTRST 0x17     // Read/Write, Interrupt Latch Reset
#define MMA7455_CTL1 0x18       // Read/Write, Control 1 Register
#define MMA7455_CTL2 0x19       // Read/Write, Control 2 Register
#define MMA7455_LDTH 0x1A       // Read/Write, Level Detection Threshold Limit Value
#define MMA7455_PDTH 0x1B       // Read/Write, Pulse Detection Threshold Limit Value
#define MMA7455_PD 0x1C         // Read/Write, Pulse Duration Value
#define MMA7455_LT 0x1D         // Read/Write, Latency Time Value (between pulses)
#define MMA7455_TW 0x1E         // Read/Write, Time Window for Second Pulse Value
#define MMA7455_RESERVED2 0x1F  // Reserved

// Defines for the bits, to be able to change
// between bit number and binary definition.
// By using the bit number, programming the MMA7455
// is like programming an AVR microcontroller.
// But instead of using "(1<<X)", or "_BV(X)",
// the Arduino "bit(X)" is used.
#define MMA7455_D0 0
#define MMA7455_D1 1
#define MMA7455_D2 2
#define MMA7455_D3 3
#define MMA7455_D4 4
#define MMA7455_D5 5
#define MMA7455_D6 6
#define MMA7455_D7 7

// Status Register
#define MMA7455_DRDY MMA7455_D0
#define MMA7455_DOVR MMA7455_D1
#define MMA7455_PERR MMA7455_D2

// Mode Control Register
#define MMA7455_MODE0 MMA7455_D0
#define MMA7455_MODE1 MMA7455_D1
#define MMA7455_GLVL0 MMA7455_D2
#define MMA7455_GLVL1 MMA7455_D3
#define MMA7455_STON MMA7455_D4
#define MMA7455_SPI3W MMA7455_D5
#define MMA7455_DRPD MMA7455_D6

// Control 1 Register
#define MMA7455_INTPIN MMA7455_D0
#define MMA7455_INTREG0 MMA7455_D1
#define MMA7455_INTREG1 MMA7455_D2
#define MMA7455_XDA MMA7455_D3
#define MMA7455_YDA MMA7455_D4
#define MMA7455_ZDA MMA7455_D5
#define MMA7455_THOPT MMA7455_D6
#define MMA7455_DFBW MMA7455_D7

// Control 2 Register
#define MMA7455_LDPL MMA7455_D0
#define MMA7455_PDPL MMA7455_D1
#define MMA7455_DRVO MMA7455_D2

// Interrupt Latch Reset Register
#define MMA7455_CLR_INT1 MMA7455_D0
#define MMA7455_CLR_INT2 MMA7455_D1

// Detection Source Register
#define MMA7455_INT1 MMA7455_D0
#define MMA7455_INT2 MMA7455_D1
#define MMA7455_PDZ MMA7455_D2
#define MMA7455_PDY MMA7455_D3
#define MMA7455_PDX MMA7455_D4
#define MMA7455_LDZ MMA7455_D5
#define MMA7455_LDY MMA7455_D6
#define MMA7455_LDX MMA7455_D7

// I2C Device Address Register
#define MMA7455_I2CDIS MMA7455_D7

// Default I2C address for the MMA7455
#define MMA7455_I2C_ADDRESS 0x1D


// When using an union for the registers and
// the axis values, the byte order of the accelerometer
// should match the byte order of the compiler and AVR chip.
// Both have the lower byte at the lower address,
// so they match.
// This union is only used by the low level functions.

//Convert rad to deg
#define Rad2Deg 57.2957795 // 1 radian = 57.2957795 degrees

typedef union xyz_union
{
  struct
  {
    uint8_t x_lsb;
    uint8_t x_msb;
    uint8_t y_lsb;
    uint8_t y_msb;
    uint8_t z_lsb;
    uint8_t z_msb;
  } 
  reg;
  struct
  {
    int x;
    int y;
    int z;
  } 
  value;
};
//******************************************************************************************************************************

//enable library gyro
#include <L3G4200D.h>
//enable I2C comunication
#include <Wire.h>

//enablee gyro
L3G4200D gyro;

//I/O pin configuration for run motor
const char LED  = 13;
const char STBY = 12;
const char PWMA = 11; //left
const char PWMB = 10; //right
const char AIN2 = 8;
const char AIN1 = 7;
const char BIN1 = 3;
const char BIN2 = 2;

//Global Variable Max Min Accelerometer
double maxValX=0,minValX=0;
double maxValY=0,minValY=0;
double maxValZ=0,minValZ=0;

//Global Variable untuk rata Gyro
float avGyroX=0,avGyroY=0,avGyroZ=0;
float rataX, rataY, rataZ;

//Buffer for read Accelero dan Gyro
double dX=0,dY=0,dZ=0;
double gX=0,gY=0,gZ=0;

//Accelero Angel value variables
float AccAngleX=0, AccAngleY=0, AccAngleZ=0;

//Gyro variable for integral
unsigned long lTime, preTime;
float dTime,dT, Roll, Pitch;
float gyroAngle[2]={0};

//PID variabel
unsigned long int lastTimeMyPID=0;
float outputPID=0, lastErrorMyPID=0, errSumMyPID=0;
const float setpointPID=94.6;
const float KP=23;
const float KI=23;
const float KD=12;

//pushbutton
char button1,button2;

//lib  kalman
unsigned long timer;
double KFaccX,KFaccY;
double KFrateX,KFrateY;
double KFpitch;

//Kalman filter variables
double Q_angle; // Process noise variance for the accelerometer
double Q_bias; // Process noise variance for the gyro bias
double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise
  
double angle; // The angle calculated by the Kalman filter - part of the 2x1 state matrix
double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state matrix
double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
    
double P[2][2]; // Error covariance matrix - This is a 2x2 matrix
double K[2]; // Kalman gain - This is a 2x1 matrix
double y; // Angle difference - 1x1 matrix
double S; // Estimate error - 1x1 matrix


void setup()
{
  // Initialize the 'Wire' class for I2C-bus communication.
  Wire.begin();
  // enable serial comunication
  Serial.begin(9600);

  // set gyro fefault
  gyro.enableDefault();
  // 2000 dps mode
  gyro.writeReg(0x23,30);

  // Initialize the MMA7455 eror
  int error;
  error = MMA7455_init();
  
  // initialize the digital pin as an output.
  pinMode(LED,OUTPUT);
  pinMode(STBY,OUTPUT);
  pinMode(BIN2,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(AIN1,OUTPUT);
  digitalWrite(LED, LOW);    // set the LED off
  digitalWrite(STBY, LOW);   // standby mode
  
  // initialize the analog pin as an input.
  pinMode(A1,INPUT);
  digitalWrite(A1, HIGH);
  pinMode(A0,INPUT);
  digitalWrite(A0, HIGH);
  
  // kalman Setup
  timer = micros();
  setAngle(0); // The angle calculated by accelerometer starts at 0 degrees
  
  /* We will set the varibles like so, these can also be tuned by the user */
  Q_angle = 0.001;
  Q_bias = 0.003;
  R_measure = 0.03;
       
  bias = 0; // Reset bias
  // Since we assume tha the bias is 0 and we know the starting angle (use setAngle),
  // the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
  P[0][0] = 0; 
  P[0][1] = 0;
  P[1][0] = 0;
  P[1][1] = 0; 
}

void loop(){
  
  // variable for stop if robot fall
  char fall=0;
  unsigned long int fallTime;
      
  // reset software button
  button2 = digitalRead(A0); 
  if(button2==LOW){    
    asm("jmp 0000");    //Reset software
  }
  
  // Stop if not yet Start
  motor(0);
  // Calibration 0 degree on first start
  bacaAccel();
  
  // Run Self-Balancing Robot
  button1 = digitalRead(A1);
  if(button1==LOW){
    while(1){     
      accelAngle();        // Read Accelero
      readGyro();          // Read Gyro
      KFaccY = AccAngleY;  // Accelero angle
      KFrateY = gY;        // Gyroscope velocity
      
      // Calculate the angle using the Kalman filter 
      KFpitch = getAngle(KFaccY,KFrateY,double(micros() - timer)/1000000);
      timer = micros();
      
//      Serial.print(KFaccY);
//      Serial.print('\t');
//      Serial.print(KFrateY);
//      Serial.print('\t');  
//      Serial.println(KFpitch);
//      delay(10);
      
//      sensorFusion();
      calculationPID();
      motor(-outputPID);
      
      //For stop if robot fall
      if(abs(outputPID)>=255){
        if(fall==0){
          fallTime=millis();
          fall=1;
        }else{
          if((millis()-fallTime)>=500){asm("jmp 0000");}
        }
      }else{fall=0;}
      
    }
  }
}

// for read Accelero angle
void accelAngle(){
  bacaAccel();
  
  //The minimum and maximum values that came from the accelerometer
  //You very well may need to change these
  float minX = -65;
  float maxX = 61;

  float minY = -67;
  float maxY = 66;

  float minZ = -59;
  float maxZ = 69;

  //convert read values to degrees -90 to 90 - Needed for atan2
  float xAng = map(dX, minX, maxX, -90, 90);
  float yAng = map(dY, minY, maxY, -90, 90);
  float zAng = map(dZ, minZ, maxZ, -90, 90);

  //Caculate 360deg values like so: atan2(-yAng, -zAng)
  //atan2 outputs the value of -π to π (radians)
  //We are then converting the radians to degrees
  AccAngleX = Rad2Deg * (atan2(-xAng, -zAng) + PI);
  AccAngleY = Rad2Deg * (atan2(-yAng, -zAng) + PI);
  
  if(AccAngleX>180)  {AccAngleX=AccAngleX-360;}
  if(AccAngleY>180)  {AccAngleY=AccAngleY-360;}
}

//read Accelero data from hardware
void bacaAccel(){
  int x,y,z;
  // The function MMA7455_xyz returns the 'g'-force as an integer in 64 per 'g'.
  // set x,y,z to zero (they are not written in case of an error).
  x = y = z = 0;
  MMA7455_xyz(&x, &y, &z); // get the accelerometer values.

  dX = (double) x;// / 64.0;          // calculate the 'g' values.
  dY = (double) y;// / 64.0;
  dZ = (double) z;// / 64.0;
}

//for Search zero value Accelerometer
void accelMaxMin(){
  bacaAccel();
  
  // mencari max
  if(dX > maxValX) {maxValX=dX;}
  if(dY > maxValY) {maxValY=dY;}
  if(dZ > maxValZ) {maxValZ=dZ;}
  // mencari min
  if(dX < minValX) {minValX=dX;}
  if(dY < minValY) {minValY=dY;}
  if(dZ < minValZ) {minValZ=dZ;}

  Serial.print(F("MAX =\t"));
  Serial.print(maxValX, 3);

  Serial.print(F("\t "));
  Serial.print(maxValY, 3);

  Serial.print(F("\t "));
  Serial.println(maxValZ, 3);

  Serial.print(F("MIN =\t"));  
  Serial.print(minValX, 3);

  Serial.print(F("\t "));
  Serial.print(minValY, 3);

  Serial.print(F("\t "));
  Serial.println(minValZ, 3);

  delay(100);
}

//for read Gyro for Prosesing data
void readGyro(){
  const float offsetGyroX=687.051;
  const float offsetGyroY=-687.460;
  const float offsetGyroZ=758.518;

  gyro.read();

  gX=(int)gyro.g.x;
  gX=(gX/32768);
  gX=(gX*2000);
  gX=gX-offsetGyroX;

  gY=(int)gyro.g.y;
  gY=(gY/32768);
  gY=(gY*2000);
  gY=gY-offsetGyroY;

  gZ=(int)gyro.g.z;
  gZ=(gZ/32768);
  gZ=(gZ*2000);
  gZ=gZ-offsetGyroZ;
  
  //Scall for angel
  gX*=0.2830;
  gY*=0.2830;
  gZ*=0.2830;
}

//for Search Zero value of Gyroscope
void rataGyro(){
  float sumGyroX=0,sumGyroY=0,sumGyroZ=0;
  int samplGyro=100,i;
  avGyroX=avGyroY=avGyroZ=0;

  for(i=1;i<=samplGyro;i++){
    readGyro();
    sumGyroX+=gX;
    sumGyroY+=gY;
    sumGyroZ+=gZ;
  }
  avGyroX=sumGyroX/samplGyro;
  avGyroY=sumGyroY/samplGyro;
  avGyroZ=sumGyroZ/samplGyro;

  Serial.print(F("rata Gyro =\t"));
  Serial.print(avGyroX, 3);

  Serial.print(F("\t "));
  Serial.print(avGyroY, 3);

  Serial.print(F("\t "));
  Serial.println(avGyroZ, 3);
  delay(100);
}

// for Test Value of Gyroscope
void integralGyro(){
  if((millis()-lTime)>=20){
    dTime=millis()-lTime;
    dTime=dTime/1000;
    lTime=millis();

    readGyro();
    gyroAngle[0]+=gX*dTime; // Integrating gyro X rateAngle to get real angle
    gyroAngle[1]+=gY*dTime; // Integrating gyro Y rateAngle to get real angle
    gyroAngle[2]+=gZ*dTime; // Integrating gyro Z rateAngle to get real angle
  }
    Serial.print("\t Gyro Angel=");
    Serial.print(F("\t "));
    Serial.print(gyroAngle[0], 3);
    Serial.print(" Y =");
    Serial.println(gyroAngle[1], 3);
    Serial.print(F("\t "));
    Serial.println(gyroAngle[2], 3);
}


//calculate Fusion sensoy by Komplementary Filter
void sensorFusion(){
  if((millis()-preTime)>=20){
    dT=millis()-preTime;
    dT=dT/1000;
    preTime=millis();
  
    readGyro();
    accelAngle();
//    Roll  = (0.98)*(Roll  + gX*dT) + (0.02)*(AccAngleX);
    Pitch = (0.98)*(Pitch + gY*dT) + (0.02)*(AccAngleY);
  } 
//  Serial.print(" ");
//  Serial.println(Pitch, 3);
}


// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
double getAngle(double newAngle, double newRate, double dt) {
  
  // KasBot V2  -  Kalman filter module www.x-firm.com/?page_id=145
  // Modified by Kristian Lauszus
  // See my blog post for more information: blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it
                        
  // Discrete Kalman filter time update equations - Time Update ("Predict")
  // Update xhat - Project the state ahead
  /* Step 1 */
  rate = newRate - bias;
  angle += dt * rate;
        
  // Update estimation error covariance - Project the error covariance ahead
  /* Step 2 */
  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;
        
  // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
  // Calculate Kalman gain - Compute the Kalman gain
  /* Step 4 */
  S = P[0][0] + R_measure;
  /* Step 5 */
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;
        
  // Calculate angle and bias - Update estimate with measurement zk (newAngle)
  /* Step 3 */
  y = newAngle - angle;
  /* Step 6 */
  angle += K[0] * y;
  bias += K[1] * y;
        
  // Calculate estimation error covariance - Update the error covariance
  /* Step 7 */
  P[0][0] -= K[0] * P[0][0];
  P[0][1] -= K[0] * P[0][1];
  P[1][0] -= K[1] * P[0][0];
  P[1][1] -= K[1] * P[0][1];
        
  return angle;
}

void setAngle(double newAngle) { angle = newAngle; } // Used to set angle, this should be set as the starting angle
//double getRate() { return rate; } // Return the unbiased rate
    
// These are used to tune the Kalman filter
//void setQangle(double newQ_angle) { Q_angle = newQ_angle; }
//void setQbias(double newQ_bias) { Q_bias = newQ_bias; }
//void setRmeasure(double newR_measure) { R_measure = newR_measure; }

// calculation PID for run self balancing robot
void calculationPID(){
  float error, dError, inputPID;
  
  inputPID=KFpitch;
  
  //execute every 20mS (50Hz)
  if((millis()-lastTimeMyPID)>=20){
        
    error=inputPID-setpointPID;  // pitch is Input
    dError=error-lastErrorMyPID; lastErrorMyPID=error;
    errSumMyPID+=error*KI;
    if(errSumMyPID>255){errSumMyPID=255;}
    if(errSumMyPID<-255){errSumMyPID=-255;}
    outputPID=KP*error+KD*dError+errSumMyPID; // PID=P+D+I
    if(outputPID>255){outputPID=255;}
    if(outputPID<-255){outputPID=-255;}
    
//    Serial.print(KFpitch)    ; Serial.print(" ");
//    Serial.print(error)      ; Serial.print(" ");
//    Serial.print(outputPID)  ; Serial.print("\n");
    
    lastTimeMyPID=millis();   
  }
}


//setup Motor for forward and revers
void motor(int buffPWM){
  int pwm;
  pwm=buffPWM;
  
  digitalWrite(STBY, HIGH);  
  
  if(pwm>=0){    // forward
    digitalWrite(LED, LOW);  // Indicaror RUN forward
//    if(pwm>255){pwm=255;}
    analogWrite(PWMA, pwm);
    analogWrite(PWMB, pwm);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }else{
    digitalWrite(LED, HIGH);// Indicaror RUN revers 
//    if(pwm<-255){pwm=-255;}
    analogWrite(PWMA, -pwm);
    analogWrite(PWMB, -pwm);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);  
  }  
}



//******************************************************************************************************************************
// MMA7455_init
//
// Initialize the MMA7455.
// Set also the offset, assuming that the accelerometer is
// in flat horizontal position.
//
// Important notes about the offset:
//    The sensor has internal registers to set an offset.
//    But the offset could also be calculated by software.
//    This function uses the internal offset registers
//    of the sensor.
//    That turned out to be bad idea, since setting the
//    offset alters the actual offset of the sensor.
//    A second offset calculation had to be implemented
//    to fine tune the offset.
//    Using software variables for the offset would be
//    much better.
//
//    The offset is influenced by the slightest vibration
//    (like a computer on the table).
//    
int MMA7455_init(void)
{
  int x, y, z, error;
  xyz_union xyz;
  uint8_t c1, c2;

  // Initialize the sensor
  //
  // Sensitivity:
  //    2g : GLVL0
  //    4g : GLVL1
  //    8g : GLVL1 | GLVL0
  // Mode:
  //    Standby         : 0
  //    Measurement     : MODE0
  //    Level Detection : MODE1
  //    Pulse Detection : MODE1 | MODE0
  // There was no need to add functions to write and read
  // a single byte. So only the two functions to write
  // and read multiple bytes are used.

  // Set mode for "2g sensitivity" and "Measurement Mode".
  c1 = bit(MMA7455_GLVL0) | bit(MMA7455_MODE0);
  error = MMA7455_write(MMA7455_MCTL, &c1, 1);
  if (error != 0)
    return (error);

  // Read it back, to test the sensor and communication.
  error = MMA7455_read(MMA7455_MCTL, &c2, 1);
  if (error != 0)
    return (error);

  if (c1 != c2)
    return (-99);

  // Clear the offset registers.
  // If the Arduino was reset or with a warm-boot,
  // there still could be offset written in the sensor.
  // Only with power-up the offset values of the sensor
  // are zero.
  xyz.value.x = xyz.value.y = xyz.value.z = 0;
  error = MMA7455_write(MMA7455_XOFFL, (uint8_t *) &xyz, 6);
  if (error != 0)
    return (error);

  // The mode has just been set, and the sensor is activated.
  // To get a valid reading, wait some time.
  delay(100); // default = 100

#define USE_INTERNAL_OFFSET_REGISTERS
#ifdef USE_INTERNAL_OFFSET_REGISTERS

  // Calcuate the offset.
  //
  // The values are 16-bits signed integers, but the sensor
  // uses offsets of 11-bits signed integers.
  // However that is not a problem,
  // as long as the value is within the range.

  // Assuming that the sensor is flat horizontal,
  // the 'z'-axis should be 1 'g'. And 1 'g' is
  // a value of 64 (if the 2g most sensitive setting
  // is used).  
  // Note that the actual written value should be doubled
  // for this sensor.

  error = MMA7455_xyz (&x, &y, &z); // get the x,y,z values
  if (error != 0)
    return (error);

  xyz.value.x = 2 * -x;        // The sensor wants double values.
  xyz.value.y = 2 * -y;
  xyz.value.z = 2 * -(z-64);   // 64 is for 1 'g' for z-axis.

  error = MMA7455_write(MMA7455_XOFFL, (uint8_t *) &xyz, 6);
  if (error != 0)
    return (error);

  // The offset has been set, and everything should be okay.
  // But by setting the offset, the offset of the sensor
  // changes.
  // A second offset calculation has to be done after
  // a short delay, to compensate for that.
  delay(200); // default = 200

  error = MMA7455_xyz (&x, &y, &z);    // get te x,y,z values again
  if (error != 0)
    return (error);

  xyz.value.x += 2 * -x;       // add to previous value
  xyz.value.y += 2 * -y;
  xyz.value.z += 2 * -(z-64);  // 64 is for 1 'g' for z-axis.

  // Write the offset for a second time.
  // This time the offset is fine tuned.
  error = MMA7455_write(MMA7455_XOFFL, (uint8_t *) &xyz, 6);
  if (error != 0)
    return (error);
#endif

  return (0);          // return : no error
}

// --------------------------------------------------------
// MMA7455_xyz
//
// Get the 'g' forces.
// The values are with integers as 64 per 'g'.
//
int MMA7455_xyz( int *pX, int *pY, int *pZ)
{
  xyz_union xyz;
  int error;
  uint8_t c;

  // Wait for status bit DRDY to indicate that
  // all 3 axis are valid.
  do
  {
    error = MMA7455_read (MMA7455_STATUS, &c, 1);
  } 
  while ( !bitRead(c, MMA7455_DRDY) && error == 0);
  if (error != 0)
    return (error);

  // Read 6 bytes, containing the X,Y,Z information
  // as 10-bit signed integers.
  error = MMA7455_read (MMA7455_XOUTL, (uint8_t *) &xyz, 6);
  if (error != 0)
    return (error);

  // The output is 10-bits and could be negative.
  // To use the output as a 16-bit signed integer,
  // the sign bit (bit 9) is extended for the 16 bits.
  if (xyz.reg.x_msb & 0x02)    // Bit 9 is sign bit.
    xyz.reg.x_msb |= 0xFC;     // Stretch bit 9 over other bits.
  if (xyz.reg.y_msb & 0x02)
    xyz.reg.y_msb |= 0xFC;
  if (xyz.reg.z_msb & 0x02)
    xyz.reg.z_msb |= 0xFC;

  // The result is the g-force in units of 64 per 'g'.
  *pX = xyz.value.x;
  *pY = xyz.value.y;
  *pZ = xyz.value.z;

  return (0);                  // return : no error
}

// --------------------------------------------------------
// MMA7455_read
//
// This is a common function to read multiple bytes
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus.
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read.
// There is no function for a single byte.
//
int MMA7455_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MMA7455_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false); // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MMA7455_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);                  // return : no error
}

// --------------------------------------------------------
// MMA7455_write
//
// This is a common function to write multiple bytes
// to an I2C device.
//
// Only this function is used to write.
// There is no function for a single byte.
//
int MMA7455_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MMA7455_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);                   // return : no error
}


