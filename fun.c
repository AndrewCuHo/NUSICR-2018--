#include "fun.h"



//简单延时
void my_delay(long t)
{
    while(t--);
}

void delay(uint16_t ms)
{
  uint16_t ii,jj;
  if (ms<1) ms=1;
  ms = ms*12 ;
  for(ii=0;ii<ms;ii++)
    for(jj=0;jj<1335;jj++);  //16MHz--1ms
  //   for(jj=0;jj<4006;jj++);  //48MHz--1ms
  //for(jj=0;jj<5341;jj++);    //64MHz--1ms
}


//32位绝对值函数
int mylabs(int temp_data)
{
  return(temp_data>0?temp_data:-temp_data);
}

//16位绝对值函数
int16 myabs(int16 temp_data)
{
  return(temp_data>0?temp_data:-temp_data);
}

//32位限幅函数   将X限幅在-y到+y之间
int limit(int x, int y)
{
    if(x>y)             return(y);
    else if(x<-y)       return(-y);
    return(x);
}

//32位限幅函数   将X限幅在 a到 b之间
int limit_ab(int x, int a, int b)
{
    if(x>b)             return(b);
    else if(x<a)        return(a);
    return(x);
}

float Slope_Calculate(uint8 begin,uint8 end,float *p)    //最小二乘法拟合斜率
{
  float xsum=0,ysum=0,xysum=0,x2sum=0;
   uint8 i=0;
   float result=0;
   static float resultlast;
   p=p+begin;
   for(i=begin;i<end;i++)
   {
	   xsum+=i;
	   ysum+=*p;
	   xysum+=i*(*p);
	   x2sum+=i*i;
	   p=p+1;
   }
  if((end-begin)*x2sum-xsum*xsum) //判断除数是否为零 
  {
    result=((end-begin)*xysum-xsum*ysum)/((end-begin)*x2sum-xsum*xsum);
    resultlast=result;
  }
  else
  {
   result=resultlast;
  }
  return result;
}
