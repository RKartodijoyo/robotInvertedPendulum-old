float fuzzyku(float x){
float nb=-0,ns=0,z=0,ps=0,pb=0;
float a1,a2,a3,a4,a5,cog;
if(x<-24)&&(-32<=x){
nb=(x-(-32))/8;
}
else if(x<-16)&&(-24<=x){
nb=(-16-x)/8;
ns=(x-(-24))/4;
}
else if(x<-8)&&(-16<=x){
ns=(-8-x)/8;
z=(x-(-16))/8;
}
else if(x<0)&&(-8<=x){
z=(0-x)/8;
ps=(x-(-8))/8;
}
else if(x<8)&&(0<=x){
ps=(8-x)/8;
pb=(x-0)/8;
}
else if(x<16)&&(8<=x){
pb=(16-x)/8;
}
else{
if(x>=16){
pb=1;

}
else{
nb=1;
}
}

a1=nb*(-1550);
a2=nb*(-1300);
a3=nb*(0);
a4=nb*(1100);
a5=nb*(1320);
cog=(a1+a2+a3+a4+a5)/(nb+ns+z+ps+pb);
return cog;
}