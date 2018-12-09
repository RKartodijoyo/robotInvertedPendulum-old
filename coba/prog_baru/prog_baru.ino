#include <Wire.h>


//----------------------------
int sensor[8];
int e_dev,haluan;
const double*vel;
double gyro_value_average;
double gyro_value[2];

float diam=0;
float npelan=-5;
float nsedang=-10;
float ncepat=-40;
float ppelan=5;
float psedang=10;
float pcepat=40;
float diam_=0;
float npelan_=-1;
float nsedang_=-8;
float ncepat_=-15;
float ppelan_=1;
float psedang_=8;
float pcepat_=15;

float mf_nvb,mf_nb,mf_ns,mf_zerro,mf_ps,mf_pb,mf_pvb;
float mf_nvb_,mf_nb_,mf_ns_,mf_zerro_,mf_ps_,mf_pb_,mf_pvb_;
float mf_dekat,mf_sedang,mf_jauh;
float a[25],z[25];
float total_a,total_z,cog;
int lpwm,rpwm;
float in_dev1=300;
float in_dev2=150;
float in_dev3=60;
float in_dev4=0;
float in_dev5=60;
float in_dev6=150;
float in_dev7=300;
float x;
float jarak1=800;
float jarak2=900;
float jarak3=1000;

float in_hal1=200;
float in_hal2=150;
float in_hal3=50;
float in_hal4=0;
float in_hal5=50;
float in_hal6=150;
float in_hal7=200;

void mf_jarak(int inp_jarak)
  {
//membership fungction jarak
//--------------------------
//mf_dekat_
//-------------------------
if(inp_jarak<=jarak1) {mf_dekat=1;}
if ((inp_jarak.=jarak1)&&(inp_jarak,=2)) {mf_dekat=(jarak2-inp)/(jarak2-jarak1);}
if (inp_jarak.=jarak2);

//------------------------------
//mf_sedang_
//-----------------------------
if (((inp_jarak<=jarak1) (inp_jarak>=jarak3)) {mf_sedang=0;)
if ((inp_jarak>=jarak1) (inp_jarak2)) {mf_sedang=(inp_jarak=(inp_jarak-jarak)/(jarak2-jarak1);}
if ((inp_jarak>=)  (inp_jarak<=jarak3)) {mf_sedang=(jarak3-inp_jarak)/(jarak3-jarak2);}
//-------------------------
//mf_jauh_
//------------------------
if(inp_jarak<=jarak2)  {mf_jauh=0;}
if(inp_jarak>=jarak2)  (inp_jarak<=jarak3)) {mf_jauh=(inp_jarak-jarak2)/(jarak3-jarak2);}
if (inp_jarak>=jarak3)  {mf_jauh=1;}
//-------------------------
//printf("fff\n",mf_dekat,mf_sedang,mf_jauh);
}
void mf_haluan(float inp_hal){
  
//----------------------
MF_NegativeVerryBig[NVB]
//---------------------
if(inp_hal>=in_hal1) (inp_hal<=in_hal2)) {mf_nvb_=(in_nvb_=(in_hal2-inp_hal)/(in_hal2-in_hal1);}
if (inp_hal>=in_hal2)  {mf_nvb_=0;}
//------------------------
//MF_NegativeBig [NB]
//-------------------------
if ((inp_hal<=in_hal1)  (inp_hal3)) {mf_nb_=0;}
if((inp_hal>=in_hal1) (inp_hal<=in_hal2)) {mf_nb_=(inp-in_hal1)/(in_hal2-in_hal1);}
if((inp_hal>=in_hal2)  (inp_hal_hal3)) {mf_nb_=(in_hal3-inp_hal)/(in_hal3-in_hal2);}
//-----------------------
//MF_NegativeSmall [NS]
//----------------------
if ((inp_hal<=in_hal2) (inp_hal4)) {mf_ns_0;}
if((inp_hal>=in_hal2) (inp_hal3)) {mf_ns_=(inp_hal-in_hal2)/(in_hal3-in_hal2);}
if ((inp_hal>=in_hal3) (inp_hal4)) {mf_ns=(in_hal4-inp_hal)/(in_hal4-in_hal3);}
//-------------------
//mf_zerro_[Z]
//--------------------
if((inp_hal<=in_hal3) (inp_hal>=in_hal5)) [mf_zerr0_=0;}
if ((inp_hal>=in_hal3) (inp_hal<=in_hal4))  {mf_zerro_=(in_hal-in_hal3)/(in_hal4-in_hal3);}
if ((inp_hal>=in_hal4) (inp_hal<=in_hal5)) {mf_zerro_=(in_hal5-inp_hal)/(in_hal5-in_hal4);}
//-------------------
//MF_PositiveSmall [PS]
//-------------------
if ((inp_hal<=in_hal4) (inp_hal>=in_hal6)) [mf_ps_=0;}
if ((inp_hal>=in_hal4) (inp_hal<=in_hal5)) {mf_ps_=(in_hal4-inp_hal)/(in_hal5-in_hal4);}
if ((inp_hal>=in_hal5) (inp_hal<=in_hal6)) {mf_ps=(in_hal6_hal)/(in_hal6-in_hal5);}
//------------------
//MF_PositiveBig [PB]
//----------------
if((inp_hal<=in_hal5) (inp_hal7)) {mf_pb_=0);}
if ((inp_hal>=in_hal5) (inp_hal<=in_hal_hal6)) {mf_pb_=(inp_hal-in_hal5)/(in_hal6-in_hal5);}
if((inp_hal>=in_hal6) (ing_hal<=in_hal7)) {mf_pb_=(in_pbhal7)/(in_hal7-in_hal6);}
//-------------------
//MF_Positive [PVB]
//-------------------
if(inp_hal<=in_hal6) [mf_pvb_=0;}
if((ing_hal>=in_hal6)(inp_hal<=in_hal7)) {mf_pvb_=(inp_hal-in_hal6)/(in_hal7-in_hal6);}
if (inp_hal>=in_hal7) {mf_pvb_=0;}
//printf("fffffff\n",mf_nvb,mf_nb,mf_ns,mf_zerro,mf_ps,mfpb,mf_pvb);
}
void mf_devasion(float inp_dev){
  //------------------------
  //MF_NegativeVerryBig [NVB]
  //-----------------------
  if(inp_dev<=in_dev1) {mf_nvb=1;}
  if ((inp_dev>= in_dev1) && (inp_dev<= in_dev2)) {mf_nvb=(in_dev2 - inp_dev)/(in_dev2-in_dev1);}                                                                                      
  if (inp_dev>= in_dev2)                          {mf_nvb=0;} 
//----------------------==
//MF_NegativeBig [NB]
//-----------------------
if((inp_dev<=in_dev1) (inp_dev>=in_dev3)) {mf_nb=0}
if ((inp_dev>=in_dev1) (inp_dev<=in_dev2)) {mf_nb=(inp_dev-in_dev1)/(in_dev2-in_dev1);}
if ((inp_dev>=in_dev2) (inp_dev<=in_dev3)) {mf_nb=(inp_dev3-inp_dev)/(in_dev3-in_dev2);}
//------------------------
//MF_NegativeSmall [NS]
//------------------------
if((inp_dev<=in_dev2) (inp_dev>=in_dev4)) {mf_ns=0;}
if((inp_dev>=in_dev2)  (inp_dev<=in_dev3)) {mf_ns=(inp_dev-in_dev2)/(in_dev3-in_dev2);}
if((inp_dev>=in_dev3) (inp_dev<=in_dev4)) {mf_ns=(inp_dev4-in_dev)/(in_dev4-in_dev3);}
//------------------------
//mf_zerro_ [Z]
//------------------------
if ((inp_dev<=in_dev3) (inp_dev>=in_dev5)) {mf_zerro=0;}
if((inp_dev>=in_dev3) (inp_dev<=in_dev4)) {mf_zerro=(inp_dev-in_dev3)/(in_dev3)/(in_dev3);}
if ((inp_dev>=in_dev4) (inp_dev<=in_dev5)) {mf_zerro=(in_dev5-inp_dev)/(in_dev5-in_dev4);}
//--------------------
//MF_PositiveSmall [PS]
//---------------------
if ((inp_dev<=in_dev4) (inp_dev>=in_dev6)) [mf_ps=0;}
if ((inp_dev>=in_dev4) (inp_dev<=in_dev5)) {mf_ps=(in_dev4_inp_dev)/(in_dev5-in_dev4);}
if ((inp_dev<=in_dev5) (inp_dev<=in_dev6)) {mf_ps=(in_dev6_inp_dev)/(in_dev6-in_dev5);}
//-----------------------
//MF_PositiveBig [PB]
//-----------------------
if((inp_dev<=in_dev5) (inp_dev>=dev7)) [mf_pb=0;}
if((inp_dev>=in_dev5) (inp_dev<=in_dev6)) {mf_pb=(inp_dev-in_dev5)/(in_dev6-in_dev5}
if ((inp_dev<=in_dev6) (inp_dev>=in_dev7)) {mf_pb=(in_dev7-inp_dev)/(in_dev7-in_dev6);}
//---------------------------
//MF_PositifVerryBig [PVB]
//---------------------------
if((inp_dev<=in_dev6) {mf_pvb=0;}
if ((inp_dev>=in_dev6)  (inp_dev<=in_dev7)) {mf_pvb=(inp_dev-in_dev6)/(in_dev7-in_dev6);}
if (inp_dev>=in_dev7) {mf_pvb=1;} 
//printf("f f f f f f f")\n",mf_nvb,mf_nb,mf_ns,mf_zerro,mf_ps,ms_pb,mf_pvb);
}
void Fuzzyfy(){
  a[0]=mf_nvb; Z[0]=a[0]*ncepat;
  a[1]=mf_nb; Z[1}=a[1]*nsedang;
  a[2]=mf_ns; Z[2]=a[2]*npelan;
  a[3]=mf_zerro; z[3]=a[3]*diam;
  a[4]=mf_ps; z[4]=a[4]*ppelan;
  a[5]=mf_pb; Z[5]=a[5]*psedang;
  a[6]=mf_pvb; Z[6]=a[6]*pcepat;
  
  total_a=a[0]+a[1]+a[2]+a[3]+a[4]+a[5]+a[6];
  total_Z=Z[0]+Z[1]+Z[2]+Z[3]+Z[4]+Z[5]+Z[6};
  cog = total_Z/total_a;
  
//printf("lpwm=%d,rpwm=%d\n",lpwm,rpwm);
}

void Fuzzyfy(){
  
  if(mf_nvb<=mf_dekat){a[0]=mf_nvb; Z[0]=a[0]*ncepat}
  else if(mf_nvb>mf_dekat){a[0]=mf_dekat; Z[0]=a[0]*ncepat;}
  if (mf_nvb<=mf_sedang){a[1]=mf_nvb; Z[1]=a[1]*ncepat;}
  else if(mf_nvb>mf_sedang){a[1]=mf_sedang; Z[1]=a[1]*ncepat;}
  if(mf_nvb<=mf_jauh) {a[2]=mf_nvb; Z[2]=a[2]*ncepat;}
  else if (mf_nvb>mf_jauh){a[2]=mf_jauh; Z[2]=a[2]*ncepat;}
  
  if(mf_nb<=mf_dekat){a[3]=mf_nvb; Z[3]=a[3]*ncepat}
  else if(mf_nb>mf_dekat){a[3]=mf_dekat; Z[3]=a[3]*ncepat;}
  if (mf_nb<=mf_sedang){a[4]=mf_nvb; Z[4]=a[4]*ncepat;}
  else if(mf_nb>mf_sedang){a[4]=mf_sedang; Z[4]=a[4]*ncepat;}
  if(mf_nb<=mf_jauh) {a[5]=mf_jauh; Z[5]=a[5]*ncepat;}
  else if (mf_nb>mf_jauh) {a[5]=mf_jauh; Z[5]=a[5]*ncepat;}
  
  if(mf_ns<=mf_dekat){a[6]=mf_nvb; Z[6]=a[6]*ncepat}
  else if(mf_ns>mf_dekat){a[6]=mf_dekat; Z[6]=a[6]*ncepat;}
  if (mf_ns<=mf_sedang){a[7]=mf_nvb; Z[7]=a[7]*ncepat;}
  else if(mf_ns>mf_sedang){a[7]=mf_sedang; Z[7]=a[7]*ncepat;}
  if(mf_ns<=mf_jauh) {a[8]=mf_jauh; Z[8]=a[8]*ncepat;}
  else if (mf_ns>mf_jauh) {a[8]=mf_jauh; Z[8]=a[8]*ncepat;}
  
  if(mf_zerro<=mf_dekat){a[9]=mf_nvb; Z[9]=a[9]*ncepat}
  else if(mf_zerro>mf_dekat){a[9]=mf_dekat; Z[9]=a[9]*ncepat;}
  if (mf_zerro<=mf_sedang){a[10]=mf_nvb; Z[10]=a[10]*ncepat;}
  else if(mf_zerro>mf_sedang){a[10]=mf_sedang; Z[10]=a[10]*ncepat;}
  if(mf_zerro<=mf_jauh) {a[11]=mf_jauh; Z[11]=a[11]*ncepat;}
  else if (mf_zerro>mf_jauh) {a[11]=mf_jauh; Z[11]=a[11]*ncepat;}
  
  if(mf_ps<=mf_dekat){a[12]=mf_nvb; Z[12]=a[12]*ncepat}
  else if(mf_ps>mf_dekat){a[12]=mf_dekat; Z[12]=a[12]*ncepat;}
  if (mf_ps<=mf_sedang){a[13]=mf_nvb; Z[13]=a[13]*ncepat;}
  else if(mf_ps>mf_sedang){a[13]=mf_sedang; Z[13]=a[13]*ncepat;}
  if(mf_ps<=mf_jauh) {a[13]=mf_jauh; Z[13]=a[13]*ncepat;}
  else if (mf_ps>mf_jauh) {a[14]=mf_jauh; Z[14]=a[14]*ncepat;}
  
  if(mf_pb<=mf_dekat){a[15]=mf_nvb; Z[15]=a[15]*ncepat}
  else if(mf_pb>mf_dekat){a[15]=mf_dekat; Z[15]=a[15]*ncepat;}
  if (mf_pb<=mf_sedang){a[16]=mf_nvb; Z[16]=a[16]*ncepat;}
  else if(mf_pb>mf_sedang){a[16]=mf_sedang; Z[16]=a[16]*ncepat;}
  if(mf_pb<=mf_jauh) {a[17]=mf_jauh; Z[17]=a[17]*ncepat;}
  else if (mf_pb>mf_jauh) {a[17]=mf_jauh; Z[17]=a[17]*ncepat;}
   
  if(mf_pvb<=mf_dekat){a[18]=mf_nvb; Z[18]=a[18]*ncepat}
  else if(mf_pvb>mf_dekat){a[18]=mf_dekat; Z[18]=a[18]*ncepat;}
  if (mf_pvb<=mf_sedang){a[19]=mf_nvb; Z[19]=a[19]*ncepat;}
  else if(mf_pvb>mf_sedang){a[19]=mf_sedang; Z[19]=a[19]*ncepat;}
  if(mf_pvb<=mf_jauh) {a[20]=mf_jauh; Z[20]=a[20]*ncepat;}
  else if (mf_pvb>mf_jauh) {a[20]=mf_jauh; Z[20]=a[20]*ncepat;}
  
  total_a=a[0]+a[1]+a[2]+a[3]+a[4]+a[5]+a[6]+a[7]+a[8]+a[9]+a[10]+a[11]+a[12]+a[13]+a[14]+a[15]+a[16]+a[17]+a[18]+a[19]+a[20];
  total_Z=Z[0]+Z[1]+Z[2]+Z[3]+Z[4]+Z[5]+Z[6]+Z[7]+Z[8]+Z[9]+Z[10]+Z[11]+Z[12]+Z[13]+Z[14]+Z[15]+Z[16]+Z[17]+Z[18]+Z[19]+Z[20];
  
  act = total_Z/total_a;
  
  //printf("lpwm=%d,rpwm=%d\n",lpwm,rpwm);
}

int main (void){

wb_innit;/* Hemisson initialisation*/
ir0 =wb_robot_get_device("ir0"); 
ir1 =wb_robot_get_device("ir1"); 
us0 =wb_robot_get_device("us0"); 
us1 =wb_robot_get_device("us1"); 
us2 =wb_robot_get_device("us2"); 
gyro=wb_robot_get_device("gyro");
wb_distance_sensor_enable(ir0,TIME_STEP);
wb_distance_sensor_enable(ir0,TIME_STEP);
wb_distance_sensor_enable(us0,TIME_STEP);
wb_distance_sensor_enable(us1,TIME_STEP);
wb_distance_sensor_enable(us2,TIME_STEP);
wb_distance_sensor_enable_encoders(TIME_STEP);
x=0;

while (wb_robot_step(TIME_STEP)!=-1);{
  sensor(0) = wb_distance_sensor_get_value(ir0);
  sensor(0) = wb_distance_sensor_get_value(ir1);
  sensor(0) = wb_distance_sensor_get_value(us0);
  sensor(0) = wb_distance_sensor_get_value(us1);
  sensor(0) = wb_distance_sensor_get_value(us2);

        if((sensor[2]<=900 (sensor[3]<=) (sensor[4]<=900)){e_dev+sensor{1}-sensor[0];
        
