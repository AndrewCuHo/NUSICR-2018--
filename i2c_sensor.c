#include "headfile.h"


extern int16  Gyro_X_Offset,Gyro_Y_Offse;

int read_buff[6];


/*
 * I2C_Init
 * ��ʼ��I2C��������
 */
void I2C_Init(void)
{ 
 gpio_init(I2C_SCL_PIN,GPO,1);                     //��ʼ��SCL�ܽ�
 gpio_init(I2C_SDA_PIN,GPO,1);                     //��ʼ��SDA�ܽ�
 MPU3050_Init();                             // MPU6050
 MMA8451_Init();
}
/*
 * I2C_WriteReg
 * дI2C�豸�Ĵ���
 */
void I2C_WriteReg(uint8 dev_addr,uint8 reg_addr , uint8 data)
{		
  I2C_Start();
  I2C_SendByte(dev_addr<<1);         
  I2C_SendByte(reg_addr );   
  I2C_SendByte(data);   
  I2C_Stop(); 
}
//���Ĵ���
uint8 I2C_ReadByte(uint8 dev_addr,uint8 reg_addr)
{
  uint8 data;
  I2C_Start();
  I2C_SendByte( dev_addr<<1); 
  I2C_SendByte( reg_addr ); 
  I2C_Start();	
  I2C_SendByte((dev_addr<<1)+1); 
  data= I2C_ReceiveByte();
  I2C_NoAck();
  I2C_Stop();
  return data;
}

//���Ĵ���
int16 I2C_ReadWord(uint8 dev_addr,uint8 reg_addr)
{
  char h,l;
  I2C_Start();
  I2C_SendByte( dev_addr<<1); 
  I2C_SendByte( reg_addr); 
  I2C_Start();	
  I2C_SendByte((dev_addr<<1)+1); 
  h= I2C_ReceiveByte();
  I2C_Ack();
  l= I2C_ReceiveByte();
  I2C_NoAck();
  I2C_Stop();
  return (h<<8)+l;
}

void I2C_ReadGryo(uint8 dev_addr,uint8 reg_addr,int16 *x,int16 *y)
{
  char h,l;
  I2C_Start();
  I2C_SendByte( dev_addr<<1); 
  I2C_SendByte( reg_addr); 
  I2C_Start();	
  I2C_SendByte((dev_addr<<1)+1); 
  h= I2C_ReceiveByte();
  I2C_Ack();
  l= I2C_ReceiveByte();
  I2C_Ack();
  *x=(h<<8)+l;
  h= I2C_ReceiveByte();
  I2C_Ack();
  l= I2C_ReceiveByte();
  I2C_Ack();
  h= I2C_ReceiveByte();
  I2C_Ack();
  l= I2C_ReceiveByte();
  I2C_NoAck();
  *y=(h<<8)+l;
  I2C_Stop();
}
/*
 * I2C_Start
 * I2C��ʼ�źţ��ڲ�����
 */
void I2C_Start(void)
{
  I2C_SDA_OUT();
  I2C_DELAY();
  I2C_DELAY();
  I2C_SDA_O=1; 
  I2C_SCL=1;
  I2C_DELAY();
  I2C_DELAY();
  I2C_SDA_O=0; 
  I2C_DELAY();
  I2C_DELAY();
  I2C_SCL=0;
  I2C_DELAY();
  I2C_DELAY();
}

/*
 * I2C_Stop
 * I2Cֹͣ�źţ��ڲ�����
 */
 void I2C_Stop(void)
{ 
    I2C_SDA_O=0;
    I2C_SCL=0; 
  I2C_DELAY();
  I2C_DELAY();
    I2C_SCL=1;
  I2C_DELAY();
  I2C_DELAY();
    I2C_SDA_O=1;
  I2C_DELAY();
  I2C_DELAY();
    I2C_SCL=0;
}

/*
 * I2C_Stop
 * I2CӦ���źţ��ڲ�����
 */
void I2C_Ack(void)
{	
  I2C_SCL=0;
  I2C_DELAY();
  
  I2C_SDA_O=0;
  I2C_DELAY();
  
  I2C_SCL=1;
  I2C_DELAY();
  
  I2C_SCL=0;
  I2C_DELAY();
}

/*
 * I2C_NoAck
 * I2C��Ӧ���źţ��ڲ�����
 */
 void I2C_NoAck(void)
{	
  I2C_SCL=0;
  I2C_DELAY();
  I2C_SDA_O=1;
  I2C_DELAY();
  I2C_SCL=1;
  I2C_DELAY();
  I2C_SCL=0;
  I2C_DELAY();
}


/*
 * I2C_SendByte
 * I2C�������ݣ��ڲ�����
 */
void I2C_SendByte(uint8 data) 
{
  uint8 i=8;
  while(i--)
  {
    I2C_SCL=0;
    if(data&0x80)
      I2C_SDA_O=1; 
    else 
      I2C_SDA_O=0;   
    data<<=1;
    I2C_DELAY();
    I2C_SCL=1;
    I2C_DELAY();
  }
  I2C_SCL=0;
  I2C_DELAY();
  I2C_SDA_I=1;
  I2C_DELAY();
  I2C_SCL=1; 
  I2C_DELAY();
  I2C_SCL=0;
}

/*
 * I2C_SendByte
 * I2C�������ݣ��ڲ�����
 */
 uint8 I2C_ReceiveByte(void)  
{ 
  uint8 i=8;
  uint8 ReceiveByte=0;
  
  I2C_SDA_O=1;	
  I2C_SDA_IN();	
  
  while(i--)
  {
    ReceiveByte<<=1;      
    I2C_SCL=0;
    I2C_DELAY();
    I2C_SCL=1;
    I2C_DELAY();	
    if(I2C_SDA_I)
    {
      ReceiveByte|=0x01;
    }
    
  }
  I2C_SDA_OUT();
  I2C_SCL=0;
  
  return ReceiveByte;
}

/*
 * I2C_SendByte
 * I2C��ʱ�������ڲ�����
 */
void I2C_DELAY()
{
  int i=10; //48mhz��Ƶ��ԼΪ 400k 
  while(i) 
    i--; 
}
/******************���X����ٶ�******************/
//int16 Get_X_Acc()
//{
//  int16 temp;
//  temp=I2C_ReadWord(I2C_MMA8451_ADR,0X05);
//  //temp=I2C_ReadWord(I2C_MPU3050_ADR,0X3B);
//  return temp;
//}
///******************���Y����ٶ�******************/
//int16 Get_Y_Acc()
//{
//  int16 temp;
//  temp=I2C_ReadWord(I2C_MMA8451_ADR,0X05);
//  //temp=I2C_ReadWord(I2C_MPU3050_ADR,0X3D);
//  return temp;
//}
///******************���Z����ٶ�******************/
int16 Get_Z_Acc()
{
  int16 temp;
  temp=I2C_ReadWord(I2C_MMA8451_ADR,0X05);
  //temp=I2C_ReadWord(I2C_MPU3050_ADR,0X3F);
  return temp;
}

/*****************���x����ٶ�*****************/
int16 Get_X_Gyro()
{
  int16 temp;
  temp=I2C_ReadWord(I2C_MPU3050_ADR,0X1D);
  //temp=I2C_ReadWord(I2C_MPU3050_ADR,0X43);
  return temp;
}
/*****************���y����ٶ�*****************/
int16 Get_Y_Gyro()
{
  int16 temp; 
  temp=I2C_ReadWord(I2C_MPU3050_ADR,0X1F);
  //temp=I2C_ReadWord(I2C_MPU3050_ADR,0X45);
  return temp;
}
/*****************���z����ٶ�*****************/
//int16 Get_Z_Gyro()
//{
//  int16 temp;
//  temp=I2C_ReadWord(I2C_MPU3050_ADR,0X1f);
//  //temp=I2C_ReadWord(I2C_MPU3050_ADR,0X47);
//  return temp;
//}

void MPU3050_Init()
{ int32 xgtemp,ygtemp;
  uint8 ii;
  I2C_WriteReg(I2C_MPU3050_ADR,0x3E , 0X80); //��λMPU3050
  I2C_DELAY();
  I2C_WriteReg(I2C_MPU3050_ADR,0x15,  0);    //��������Ƶ
  I2C_DELAY();
  I2C_WriteReg(I2C_MPU3050_ADR,0x16 , 0x11);  //��������8kHz&&������Χ��1000��
  I2C_DELAY();
  I2C_WriteReg(I2C_MPU3050_ADR,0x17 , 0);    //�������ж�
  I2C_DELAY();
  I2C_WriteReg(I2C_MPU3050_ADR,0x3E , 0X00); //����MPU3050
  
// I2C_WriteReg(I2C_MPU3050_ADR,0x6b , 0x00); // 
// I2C_DELAY();
// I2C_WriteReg(I2C_MPU3050_ADR,0x6c,  0x00);    // 
// I2C_DELAY();
// I2C_WriteReg(I2C_MPU3050_ADR,0x1a , 0x06);   //  5HZ ,19ms DELAY
// I2C_DELAY(); 
// I2C_WriteReg(I2C_MPU3050_ADR,0x1b , 0x08);    // Gyroscope Configuration ��500��
// I2C_DELAY();
// I2C_WriteReg(I2C_MPU3050_ADR,0x1c , 0x08); //   ACC Configuration  ��2g  
// I2C_DELAY();
  
// xgtemp=0;
// ygtemp=0;
// for(ii=0;ii<100;ii++)
// {  xgtemp += Get_X_Gyro();//��x����ٶȣ�ת��
//    I2C_DELAY();
//    ygtemp += Get_Y_Gyro();//��z����ٶȣ�ת��
//    I2C_DELAY();
//  }
//  Gyro_X_Offset =  -311; //xgtemp/100;        //��̬X����ٶ�ֵ��ֱ����������Ϊ0����ʵ�ʿ��ܲ�Ϊ0
//  Gyro_Y_Offset =  58;   //ygtemp/100;        //��̬X����ٶ�ֵ��ֱ����������Ϊ0����ʵ�ʿ��ܲ�Ϊ0

}

void MMA8451_Init()
{ 
 I2C_WriteReg(I2C_MMA8451_ADR,0x0E, 1);  //0Ϊ+-2g��1Ϊ+-4g��2Ϊ+-8g
 I2C_DELAY();
 I2C_WriteReg(I2C_MMA8451_ADR,0x0F, 0X10);  // ʹ�ܵ�ͨ�˲�
 I2C_DELAY();
 I2C_WriteReg(I2C_MMA8451_ADR,0x2A , 1); //0x2aΪ���ƼĴ���1 ����mma8451
 I2C_DELAY();
}
	
