/*
Filename:       MEMS.c
Revised:        $Date: 2018-11-11  $
Revision:       $Revision: ck $
*/


#include "modules\mems.h"
#include "stm32f4xx_bsp.h"
#include "drivers\stm32f4_system.h"


uint8_t transBuf; 

/*********************Mems functions ******************************************/
void IIC_Init(void);
void usDelay(volatile uint16_t delay);
static bool Mems_ReadReg(uint8_t Reg, uint8_t* Data);
static bool Mems_WriteReg(uint8_t Reg, uint8_t Data);
/********************************************
* Mems functions *端口 SCL:PB10  SDA:PB12
********************************************/
bool MemsOpen(void)
{	
  IIC_Init();

  usDelay(10);
  
    //read chip id
//  while(1){  
    Mems_ReadReg(0x01, &transBuf);
    if(transBuf != 0x13)
      return FALSE;
//	}
  //Mode: normal
  transBuf = 0x35;
  Mems_WriteReg(0x11, transBuf);
  
  //ODR=125HZ
  transBuf = 0x07;
  Mems_WriteReg(0x10, transBuf);	
  
  //Set active_ths default:g_Rang +/-2g 
  //threshold of active interrupt=Active_th*K(mg)
  //K = 3.91(2g range)
  //K = 7.81(4g range)
  //K = 15.625(8g range)
  //K = 31.25(16g range)
  transBuf = 50;
  Mems_WriteReg(0x28, transBuf);
  
  //Enable active interrupt
  transBuf = 0x07;
  Mems_WriteReg(0x16, transBuf);			
  
  //mapping active interrupt to INT1
  transBuf = 0x04;
  Mems_WriteReg(0x19, transBuf);	
  
  return TRUE;
}

void MemsLowPwMode(void)
{
  //Mode: suspend
  transBuf = 0x35 | (1<<3);
  Mems_WriteReg(0x11, transBuf);
}

/********************************************
* IIC Functions * F4中GPIO_TypeDef，没有分BSRR和BRR，统一用了BSRR
********************************************/
void IIC_Init(void)
{
  GPIO_InitTypeDef GPIO_Init_Structure;
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  GPIO_Init_Structure.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_Init_Structure.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_Init_Structure.Pull = GPIO_NOPULL;   
  GPIO_Init_Structure.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB,&GPIO_Init_Structure);
  GPIOB->BSRR = GPIO_PIN_6|GPIO_PIN_7;
}

void usDelay(volatile uint16_t delay)
{
  volatile uint16_t i;
  while(delay--)
  {
    i = 100;
    for(;i>0;i--);
  }
}

static void SDA_In(void)
{ 
  //	GPIOB->MODER&=~(3<<11);
  //	GPIOB->MODER|=0<<11;
  GPIO_InitTypeDef GPIO_Init_Structure;

  GPIO_Init_Structure.Pin = GPIO_PIN_7;
  GPIO_Init_Structure.Mode = GPIO_MODE_INPUT;
  GPIO_Init_Structure.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB,&GPIO_Init_Structure);	
	usDelay(2);
}

static void SDA_Out(void)
{	
  //	GPIOB->MODER&=~(3<<11);
  //	GPIOB->MODER|=1<<11;
  GPIO_InitTypeDef GPIO_Init_Structure;

  GPIO_Init_Structure.Pin = GPIO_PIN_7;
  GPIO_Init_Structure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_Init_Structure.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB,&GPIO_Init_Structure);
	usDelay(2);
}

//sda OUTPUT 1bit
static void SDA_OutPutBit(GPIO_PinState bit)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,bit);
}
//scl OUTPUT 1bit
static void SCL_OutPutBit(GPIO_PinState bit)
{ 
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,bit);
}

static uint8_t Read_SDA(void)
{
  //    return (uint8_t)(GPIOB->IDR & GPIO_PIN_12);   
  return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
}

void IIC_Start(void)
{
  SDA_OutPutBit(GPIO_PIN_SET);
  usDelay(1);
  SCL_OutPutBit(GPIO_PIN_SET);
  usDelay(2);
  
  SDA_OutPutBit(GPIO_PIN_RESET);
  usDelay(2);
  
  /* 钳住I2C总线，准备发送或接收数据 */
  SCL_OutPutBit(GPIO_PIN_RESET);
  usDelay(2);
}

void IIC_Stop(void)
{  
  SDA_OutPutBit(GPIO_PIN_RESET);   
  usDelay(1);
  
  SCL_OutPutBit(GPIO_PIN_SET); 
  usDelay(2);
  
  SDA_OutPutBit(GPIO_PIN_SET);	 
  usDelay(2);							   	
}

uint8_t IIC_Wait_Ack(void)
{
  uint8_t ucErrTime = 0;
  
  usDelay(2);
  SDA_OutPutBit(GPIO_PIN_SET);
  usDelay(2);
  SDA_In();
  
  SCL_OutPutBit(GPIO_PIN_SET);    
  
  while(Read_SDA())
  {
    ucErrTime++;
    usDelay(2);
    if(ucErrTime > (20))
    {
      IIC_Stop();
      
      return 1;
    }
  }
  
  SCL_OutPutBit(GPIO_PIN_RESET);
  usDelay(2);
  return 0;  
}

void IIC_Ack(void)
{
  SCL_OutPutBit(GPIO_PIN_RESET);
  
  SDA_Out();
  
  SDA_OutPutBit(GPIO_PIN_RESET);	 
  usDelay(2);
  
  SCL_OutPutBit(GPIO_PIN_SET);
  usDelay(2);
  
  SCL_OutPutBit(GPIO_PIN_RESET);
}

void IIC_NAck(void)
{
  SCL_OutPutBit(GPIO_PIN_RESET);
  
  SDA_Out();
  SDA_OutPutBit(GPIO_PIN_SET);    
  usDelay(2);
  
  SCL_OutPutBit(GPIO_PIN_SET);   
  usDelay(2);
  
  SCL_OutPutBit(GPIO_PIN_RESET);
}

void IIC_Send_Byte(uint8_t txd)
{                        
  uint8_t t;
  uint8_t bit;
  
  SCL_OutPutBit(GPIO_PIN_RESET);
  
  for(t = 0; t < 8; t++)
  {              
    bit =(txd & 0x80) >> 7;
    
    if(bit == GPIO_PIN_RESET)
      SDA_OutPutBit(GPIO_PIN_RESET);
    else
      SDA_OutPutBit(GPIO_PIN_SET);
    
    txd <<= 1; 	  
//    usDelay(2);
    
    SCL_OutPutBit(GPIO_PIN_SET);
    usDelay(2); //2>>1
    
    SCL_OutPutBit(GPIO_PIN_RESET);
    usDelay(2); //2>>1
  }
} 

uint8_t IIC_Read_Byte(uint8_t ack)
{
  uint8_t i,receive = 0;
  
  
  SDA_OutPutBit(GPIO_PIN_SET);
  SDA_In();
  
  SCL_OutPutBit(GPIO_PIN_RESET);
  
  for(i=0;i<8;i++ )
  {
    SCL_OutPutBit(GPIO_PIN_SET);
    usDelay(2);
    
    receive <<= 1;
    
    if(Read_SDA())
      receive ++;  
    
    SCL_OutPutBit(GPIO_PIN_RESET);   
    usDelay(2); 
  }	
  
  if ( !ack )
    IIC_NAck();
  else
    IIC_Ack(); 
  
  return receive;
}

static bool Mems_ReadReg(uint8_t Reg, uint8_t* Data) 
{    
  SDA_Out();
  IIC_Start();  
  /* 写指令 */    
  IIC_Send_Byte(MEMS_SADW);		 
  if( IIC_Wait_Ack() )  
    return FALSE;
  
  SDA_Out();
  IIC_Send_Byte(Reg);   		
  if( IIC_Wait_Ack() ) 
    return FALSE;
  
  SDA_Out();
  IIC_Start();  	 
  /* 读指令 */    
  IIC_Send_Byte(MEMS_SADR);	     	 
  if( IIC_Wait_Ack() )
    return FALSE;
  
  SDA_Out();
  *Data = IIC_Read_Byte(0);		    	   
  IIC_Stop();			        
  
  return TRUE;			 
}

static bool Mems_WriteReg(uint8_t Reg, uint8_t Data) 
{ 
  SDA_Out();
  IIC_Start();  
  /* 写指令 */       
  IIC_Send_Byte(MEMS_SADW);     	
  if( IIC_Wait_Ack() )
    return FALSE; 
  
  SDA_Out();
  IIC_Send_Byte(Reg);           
  if( IIC_Wait_Ack() )
    return FALSE;  
  
  SDA_Out();
  IIC_Send_Byte(Data);     						   
  if( IIC_Wait_Ack() )
    return FALSE;
  
  IIC_Stop();			            
  
  return TRUE;
}
