/****************************************************************************************


****************************************************************************************/
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include <Wire.h>

float angles[3]; // yaw pitch roll

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();

//Porting Driver Motor Vexta
int cwccw1 = 53;
int runbrake1 = 52;
int startstop1 = 51;
int cwccw2 = 50;
int runbrake2 = 10;
int startstop2 = 11;

void setup()
{
  Serial.begin(9600);
  Wire.begin() ;     
  pinMode(cwccw1, OUTPUT); 
  pinMode(runbrake1, OUTPUT);
  pinMode(startstop1, OUTPUT);
  pinMode(cwccw2, OUTPUT); 
  pinMode(runbrake2, OUTPUT);
  pinMode(startstop2, OUTPUT);  
  
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);   
}

int sp = 0;
  
void loop()
{
  sp++;
  delay(10);    
  maju(sp,sp); 
  
  sixDOF.getEuler(angles);  
  Serial.print(angles[0]);
  Serial.print(" | ");  
  Serial.print(angles[1]);
  Serial.print(" | ");
  Serial.println(angles[2]);    
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
