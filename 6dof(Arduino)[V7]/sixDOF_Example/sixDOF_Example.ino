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

void setup() { 
  Serial.begin(9600);
  Wire.begin();
  
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
  
  pinMode(cwccw1, OUTPUT); 
  pinMode(runbrake1, OUTPUT);
  pinMode(startstop1, OUTPUT);
  pinMode(cwccw2, OUTPUT); 
  pinMode(runbrake2, OUTPUT);
  pinMode(startstop2, OUTPUT);
  
}

void loop() { 
  
  /*sixDOF.getEuler(angles);
  
  Serial.print(angles[0]);
  Serial.print(" | ");  
  Serial.print(angles[1]);
  Serial.print(" | ");
  Serial.println(angles[2]);
  
  delay(100); 
  */
  
  kanan(); 
  delay(5000);
  kiri();
  delay(5000);
  berhenti();
  delay(5000);
}

  //Robot Maju
void maju(){  
  digitalWrite(cwccw1, HIGH); 
  digitalWrite(runbrake1, HIGH);  
  digitalWrite(startstop1, HIGH);    
  digitalWrite(cwccw2, LOW); 
  digitalWrite(runbrake2, HIGH);  
  digitalWrite(startstop2, HIGH);
  }
void kanan(){  
  digitalWrite(cwccw1, LOW); 
  digitalWrite(runbrake1, HIGH);  
  digitalWrite(startstop1, HIGH);    
  digitalWrite(cwccw2, LOW); 
  digitalWrite(runbrake2, HIGH);  
  digitalWrite(startstop2, HIGH);
  }    
void kiri(){  
  digitalWrite(cwccw1, HIGH); 
  digitalWrite(runbrake1, HIGH);  
  digitalWrite(startstop1, HIGH);    
  digitalWrite(cwccw2, HIGH); 
  digitalWrite(runbrake2, HIGH);  
  digitalWrite(startstop2, HIGH);
  }    
  //Robot Mundur
void mundur(){  
  digitalWrite(cwccw1, LOW); 
  digitalWrite(runbrake1, HIGH);  
  digitalWrite(startstop1, HIGH);    
  digitalWrite(cwccw2, HIGH); 
  digitalWrite(runbrake2, HIGH);  
  digitalWrite(startstop2, HIGH);
  }
void berhenti(){  
  digitalWrite(cwccw1, HIGH); 
  digitalWrite(runbrake1, HIGH);  
  digitalWrite(startstop1, LOW);    
  digitalWrite(cwccw2, LOW); 
  digitalWrite(runbrake2, HIGH);  
  digitalWrite(startstop2, LOW);
  }  
