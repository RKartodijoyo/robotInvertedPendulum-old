int fuzzy(int k){
float nb=0,z=0,pb=0;
x=(float)k;
if(x<-16&&-32<=x)

{

nb=(x-(-16))/16;}
else if(x<0&&-16<=x){
nb=(0-x)/16;
z=(x-(-16))/16;
}
else if(x<16&&0<=x){
z=(16-x)/16;
pb=(x-0)/16;
}
else if(x<32&&16<=x){

pb=(x-16)/16;

}
else{
nb=1;
}


a1=nb*(-1550);
a2=z*(0);
a3=pb*(850);

cog=(a1+a2+a3)/(nb+z+pb);
int h;
h=(int)cog;
return h;
}
