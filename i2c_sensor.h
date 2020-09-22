#ifndef _I2C_H_
#define _I2C_H_

#include "headfile.h"
   

//����I2C�豸��ַ
#define I2C_MMA8451_ADR  0X1C
#define I2C_MPU3050_ADR  0x68

#define I2C_SCL_PIN    PTA3
#define I2C_SDA_PIN    PTA2

#define I2C_SCL        PTA3_OUT   
#define I2C_SDA_I      PTA2_IN
#define I2C_SDA_O      PTA2_OUT 
//����SDA�������
#define I2C_SDA_OUT()   gpio_init(I2C_SDA_PIN,1,0);  //����Ϊ���
#define I2C_SDA_IN()    gpio_init(I2C_SDA_PIN,0,0);  //����Ϊ����



extern int read_buff[6];   
   
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
void I2C_SendByte(uint8);
uint8 I2C_ReceiveByte(void);
void I2C_DELAY();

void I2C_Init(void);

void I2C_WriteReg(uint8 dev_addr,uint8 reg_addr , uint8 data);

uint8 I2C_ReadByte(uint8 dev_addr,uint8 reg_addr);//��һ���ֽڵ�����

int16 I2C_ReadWord(uint8 dev_addr,uint8 reg_addr);//�������ֽڵ�����

void I2C_ReadGryo(uint8 dev_addr,uint8 reg_addr,int16 *x,int16 *y);

//int16 Get_X_Acc();
//int16 Get_Y_Acc();
int16 Get_Z_Acc();
int16 Get_X_Gyro();
int16 Get_Y_Gyro();
//int16 Get_Z_Gyro();

  //MPU3050��ʼ��
void  MPU3050_Init();
  //MMA8451��ʼ��
void  MMA8451_Init();


#endif 
