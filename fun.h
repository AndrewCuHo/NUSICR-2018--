#ifndef _fun_h
#define _fun_h

#include "headfile.h"


void  my_delay(long t);
void  delay(uint16_t ms);
int   mylabs(int  temp_data);
int16 myabs(int16 temp_data);
int   limit(int x, int y);
int   limit_ab(int x, int a, int b);
float Slope_Calculate(uint8 begin,uint8 end,float *p);   //��С���˷����б��

/*
 * �����ֵ����Сֵ
 */
#define MAX( x, y ) ( ((x) > (y)) ? (x) : (y) )
#define MIN( x, y ) ( ((x) < (y)) ? (x) : (y) )


/*
 * ��������Ԫ�صĸ���
 */
#define ARR_SIZE( a ) ( sizeof( (a) ) / sizeof( ((a)[0]) ) )

/*
 * �궨��ʵ�ַ��ؾ���ֵ��x�ﲻ�����Լ��Լ�����䣬�����������
 */
#define ABS(x) (((x) > 0) ? (x) : (-(x)))

/*
 * ��ȡ�ṹ��ĳ��Աƫ��
 */
#define OFFSET(type, member)    (uint32)(&(((type *)0)->member))

/*
 * ȷ��x�ķ�ΧΪ min~max
 */
#define LIMIT(x,max,min)        (((x)<(min) ? (min) : ( (x)>(max) ? (max):(x) )))




/*
 * ȡһ�����ݵĸ���λ 
 */
#define BYTE0(Temp)       (*(char *)(&Temp))     
#define BYTE1(Temp)       (*((char *)(&Temp) + 1))
#define BYTE2(Temp)       (*((char *)(&Temp) + 2))
#define BYTE3(Temp)       (*((char *)(&Temp) + 3))














#endif 