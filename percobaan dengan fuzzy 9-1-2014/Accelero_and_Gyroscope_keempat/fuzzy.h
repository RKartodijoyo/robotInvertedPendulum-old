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
z=((-6)-x)/6;          
          }
		  else if(x<6&&0<=x){
z=(x-0)/6;          
          }
else if(x<8&&6<=x){
ps=(8-x)/2;
}
else if(x<16&&8<=x){
=(x-8)/8;
pb=(10-x)/8;
}
else if(x>16){
pb=(x-16)/8;
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