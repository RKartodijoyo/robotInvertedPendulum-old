int fuzzy(int k){
float ps=0,ns,nb=0,z=0,pb=0,x;
x=(float)k;
if(x<-16&&-24<=x)
{
nb=(x-(-16)/8;}
else if(x<-8&&-16<=x){
nb=(16-x)/8;
ns=(x-(-8))/8;
}
else if(x<-5&&-8<=x){
ns=(x-(-8))/3;
z=(0-x)/5
}
else if(x<0&&-5<=x){
     z=(x-(-0))/5;}
     }
     else if(x<5&&0<=x){
z=(5-x)/5;          
          
          }
else if(x<8&&<=5){
z=(1-x)/16;
pb=(x-0)/16;
}
else if(x<32&&16<=x){
pb=(x-16)/16;

}
else if(x>32){
pb=1;
}
else{
     nb=1;}

if(k==0){
         cog=1;
         }
         else if(k<0){
a1=nb*(-1550);
a2=z*(-500);
a3=pb*(850);              
                 }
              else{   
a1=nb*(-1550);
a2=z*(500);
a3=pb*(850);
}
cog=(a1+a2+a3)/(nb+z+pb);
int h;
h=(int)cog;
return h;
}
