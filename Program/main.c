/**
  *      __            ____
  *     / /__ _  __   / __/                      __  
  *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
  *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
  *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
  *                    /_/   github.com/KitSprout    
  * 
  * @file    main.c
  * @author  KitSprout
  * @date    27-Nov-2016
  * @brief   
  * 
  */

/* Includes --------------------------------------------------------------------------------*/
#include "drivers\stm32f4_system.h"
#include "modules\serial.h"
#include "modules\dw1000.h"
#include "stm32f4xx_bsp.h"
#include "modules\mems.h"

/** @addtogroup STM32_Program
  * @{
  */

/* Private typedef -------------------------------------------------------------------------*/
/* Private define --------------------------------------------------------------------------*/

/*20190304：添加不同接收端的地址*/
//#define ANCHOR_ADDR               1
#define MAX_ANCHOR                4
#define MODULE1                 0
#define MODULE2                 1
#define MODULE3                 2
#define MODULE4                 3

/* Default antenna delay values for 64 MHz PRF. */
#define TX_ANT_DLY                    16436
#define RX_ANT_DLY                    16436

/* Length of the common part of the message. */
#define ALL_MSG_COMMON_LEN            10

/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX                2
#define FINAL_MSG_TS_LEN              4
#define FINAL_MSG_POLL_TX_TS_IDX      10
#define FINAL_MSG_RESP_RX_TS_IDX      14
#define FINAL_MSG_FINAL_TX_TS_IDX     18

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 ms and 1 ms = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME               65536

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT                299702547

//------------------------------------------------------------------//
/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
//#define POLL_TX_TO_RESP_RX_DLY_UUS 150
#define POLL_TX_TO_RESP_RX_DLY_UUS 150
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
//#define RESP_RX_TO_FINAL_TX_DLY_UUS 595
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 2700	//2>>3

/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 32 //8>>16

/* Frames used in the ranging process. See NOTE 2 below. */

uint8_t tx_poll_msg[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};//原代码
uint8_t rx_resp_msg[]  = {0x41, 0x88, 0, 0x10, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};//原代码

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN  20
uint8_t rx_buffer[RX_BUF_LEN];

uint32_t frameLen, final_tx_time;
uint64_t poll_tx_ts, resp_rx_ts, final_tx_ts;
float64_t tof,distance=0;
uint32_t poll_tx_ts1, resp_rx_ts1, poll_rx_ts, resp_tx_ts;
int32_t rtd_init, rtd_resp;
	
/*---enough time to wkup DWT---*/
#define DUMMY_BUFFER_LEN 600
static uint8_t dummy_buffer[DUMMY_BUFFER_LEN];
//-------------------------------------------------------------------//
/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/
__IO uint8_t RTC_WKUP_FLAG = 0;
	
__IO uint8_t FLAG2 = 0;//MEMS INTs

__IO uint32_t FLAG1 = 0;//Count1计数
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

static uint32_t status;

static DWT_ConfigTypeDef dwtConfig = {
  .LoadCode          = DW_LOAD_UCODE,
  .Channel           = DW_CHANNEL_2,
  .PulseRepFreq      = DW_PRF_64M,
  .TxPreambLen       = DW_TX_PLEN_128,//
  .PreambleAcqChunk  = DW_PAC_8,//
  .TxCode            = 9,
  .RxCode            = 9,
  .NonStandardSFD    = DISABLE,//
  .DataRate          = DW_DATARATE_6M8,//注意要修改对应的前导码延迟时间参数PRE_TIMEOUT
  .PhrMode           = DW_PHR_MODE_STD,
  .SFDTimeout        = (128 + 1 + 8 - 8) // TxPreambLen + 1 + SFD length - PAC
};


/* Private function prototypes -------------------------------------------------------------*/
void DEMO_SSTWR_INITIATOR( void );
void DEMO_SSTWR_RESPONDER( void );
void ReConfig_WKUP(void);
void io_ctrl(void);
void rtc_work_sleep(void);
void INITIATOR_Function(void);
void Performance_Function (float64_t distance);
/* Private functions -----------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------------------- *
 *                                                                                          *
 *                          Single-Sided Two-Way Ranging Initiator                          *
 *                                                                                          *
 * ---------------------------------------------------------------------------------------- */

uint64_t get_tx_timestamp_u64( void )
{
  uint8_t ts_tab[5];
  uint64_t ts = 0;
  DWT_ReadTxTimestamp(ts_tab);
  for (int i = 4; i >= 0; i--) {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}

uint64_t get_rx_timestamp_u64( void )
{
  uint8_t ts_tab[5];
  uint64_t ts = 0;
  DWT_ReadRxTimestamp(ts_tab);
  for (int i = 4; i >= 0; i--) {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}

void final_msg_set_ts( uint8_t *ts_field, uint64_t ts )
{
  for (int i = 0; i < FINAL_MSG_TS_LEN; i++) {
    ts_field[i] = (uint8_t) ts;
    ts >>= 8;
  }
}
static void final_msg_get_ts( const uint8_t *ts_field, uint32_t *ts )
{
  *ts = 0;
  for (int i = 0; i < FINAL_MSG_TS_LEN; i++) {
    *ts += ts_field[i] << (i * 8);
  }
}
static void resp_msg_set_ts( uint8_t *ts_field, const uint64_t ts )
{
  for (uint32_t i = 0; i < 4; i++) {
    ts_field[i] = (ts >> (i * 8)) & 0xFF;
  }
}
static void resp_msg_get_ts( uint8_t *ts_field, uint32_t *ts )
{
  *ts = 0;
  for (uint32_t i = 0; i < 4; i++) {
    *ts += ts_field[i] << (i * 8);
  }
}

void EXTI0_IRQCallback(void)
{
	FLAG2++;//CALLBACK wrong. Set in the IRQ_handler
}	

int main( void )
{
  HAL_Init();

//	MemsOpen();
  BSP_GPIO_Config();
//	BSP_EXTIx_Config(EXTI0_IRQCallback);
  BSP_UART_Config(NULL, NULL);
  BSP_UWB_Config();
	
	LED_R_Reset();
  LED_G_Set();
  LED_VCC_Reset();
	BEEP_Reset();
	KEY_Reset();//置位IO状态

  while (1) { 

			ReConfig_WKUP();
		
			RTC_WKUP_FLAG = 0;
			LED_R_Set();
			LED_VCC_Reset();
//			delay_us(1000000);
			LED_VCC_Set();
			
			DEMO_SSTWR_INITIATOR();
		
			io_ctrl();
			DWT_ConfigureSleep(DWT_PRESRV_SLP | DWT_CONFIG, DWT_WAKE_SLPCNT | DWT_WAKE_CS | DWT_SLP_EN);
			DWT_SetSysStatus(SYS_STATUS_RXFCG);
			DWT_EnterSleep();
			BSP_GPIO_DisConfig();//IO 漏电流处理
			HAL_PWREx_EnableFlashPowerDown();
//			BSP_GPIO_DisConfig();//IO 漏电流处理
//			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
			HAL_PWR_EnterSTANDBYMode();
//			delay_us(5000000);
	}
}

int8_t checkPacket( uint8_t *pMsg1, uint8_t *pMsg2 )
{
  for (uint8_t i = 0; i < ALL_MSG_COMMON_LEN; i++) {
    if (pMsg1[i] != pMsg2[i]) {
      return -1;
    }
  }
  return 0;
}

void ReConfig_WKUP(void){
	DWT_SpiCsWakeup(dummy_buffer,DUMMY_BUFFER_LEN);
	DWT_SetSpeedLow();
	DWT_Initialise(&dwtConfig);
	DWT_SetSpeedHigh();
		/* Configure DW1000.*/
	DWT_Configure(&dwtConfig);

	/* Apply default antenna delay value.*/
	DWT_SetRxAntennaDelay(RX_ANT_DLY);
	DWT_SetTxAntennaDelay(TX_ANT_DLY);

	DWT_SetRxAfterTxDelay(POLL_TX_TO_RESP_RX_DLY_UUS);
	DWT_SetRxTimeout(RESP_RX_TIMEOUT_UUS);
}

void io_ctrl(void){
//	LED_VCC_Set();
	LED_G_Set();
	LED_R_Reset();
//			delay_us(1000000);
	LED_VCC_Reset();
	BEEP_Reset();
	KEY_Reset();//置位IO状态
}

void DEMO_SSTWR_INITIATOR( void )
{
	for (uint8_t anchor=0;anchor< 2;anchor++){ 
		tx_poll_msg[5]= anchor;//发给车1，车2
		tx_poll_msg[6]= 0;//人1>>0;人2>>1
		INITIATOR_Function();
			
		for(int i = 0; i<2 ; i++){
			if(distance == 0){
				INITIATOR_Function();
			}
			delay_us(10);
		}
	}
}

void INITIATOR_Function(void){
//		for (uint8_t anchor=0;anchor< 2;anchor++){   //添加一个for循环 20190305
//			tx_poll_msg[5]= anchor;//发给车1，车2
//			tx_poll_msg[6]= 1;//人1>>0;人2>>1
			
    /* Write frame data to DW1000 and prepare transmission. See NOTE 8 below. */
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    DWT_WriteTxData(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
    DWT_WriteTxFCtrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

    /* Start transmission, indicating that a response is expected so that reception is enabled
     * automatically after the frame is sent and the delay set by dwt_setrxaftertxdelay()
     * has elapsed. */
    DWT_StartTx(DWT_TX_IMMEDIATE | DWT_RESPONSE);

    /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 9 below. */
    do {
      status = DWT_GetSysStatus();
    } while (!(status & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)));

    /* Increment frame sequence number after transmission of the poll message (modulo 256). */
    frame_seq_nb++;

    if (status & SYS_STATUS_RXFCG) {
      /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
      DWT_SetSysStatus(SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

      /* A frame has been received, read it into the local buffer. */
      frameLen = DWT_ReadData32(DW1000_RX_FINFO, 0x00) & 0x0000007FUL;
      if (frameLen <= RX_BUF_LEN) {
        DWT_ReadRxData(rx_buffer, frameLen, 0x00);
      }

      /* Check that the frame is the expected response from the companion "DS TWR responder" example.
       * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */

      rx_buffer[ALL_MSG_SN_IDX] = 0;
      if (memcmp(rx_buffer, rx_resp_msg, 5) == 0) {
				//获取两个用于initiator计算的本地时间戳
				poll_tx_ts1 = DWT_ReadTxTimestampL32();
        resp_rx_ts1 = DWT_ReadRxTimestampL32();
				//*-----------接收外来时间戳数据阶段-----------*/
				final_msg_get_ts(&rx_buffer[5], &poll_rx_ts);
        final_msg_get_ts(&rx_buffer[9], &resp_tx_ts);
				
        /* Retrieve poll transmission and response reception timestamp. */
        poll_tx_ts = get_tx_timestamp_u64();
        resp_rx_ts = get_rx_timestamp_u64();

        /* Compute final message transmission time. See NOTE 10 below. */
        final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
        DWT_SetDelayedTxRxTime(final_tx_time);

        /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
        final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

        /* Write all timestamps in the final message. See NOTE 11 below. */
        final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
        final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
        final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

        /* Write and send final message. See NOTE 8 below. */
        tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        DWT_WriteTxData(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
        DWT_WriteTxFCtrl(sizeof(tx_final_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
        status = DWT_StartTx(DWT_TX_DELAYED);
				
				//*---------最后一次发送完之后处理数据阶段---------*/
				rtd_init = resp_rx_ts1 - poll_tx_ts1;
				rtd_resp = resp_tx_ts - poll_rx_ts;
				tof = ((rtd_init - rtd_resp) / 2.0) * DWT_TIME_UNITS;
				distance = tof * SPEED_OF_LIGHT;
//				distance = 0.91 * distance + 0.92;
				distance = 0.01253*distance*distance + 0.7913*distance + 1.056;
				Performance_Function(distance);		

        /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 12 below. */
        if (status == HAL_OK) {
          /* Poll DW1000 until TX frame sent event set. See NOTE 9 below. */
          while (!(DWT_GetSysStatus() & SYS_STATUS_TXFRS)) { };

          /* Clear TXFRS event. */
          DWT_SetSysStatus(SYS_STATUS_TXFRS);

          /* Increment frame sequence number after transmission of the final message (modulo 256). */
          frame_seq_nb++;

//          LED_G_Toggle();
        }
      }
    }
    else {
      /* Clear RX error/timeout events in the DW1000 status register. */
      DWT_SetSysStatus(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

      /* Reset RX to properly reinitialise LDE operation. */
      DWT_RxReset();
			
//			//无测距就赋值0，以重发
//			distance = 0;
    }

//	}
}

void Performance_Function (float64_t distance){
	if(distance <= 2){
		BEEP_Set();
		LED_G_Reset();
		delay_us(100000);
		BEEP_Reset();
		LED_G_Set();
		delay_us(130000);
		BEEP_Set();
		LED_G_Reset();
		delay_us(100000);
		BEEP_Reset();
		LED_G_Set();
		delay_us(130000);
		BEEP_Set();
		LED_G_Reset();
		delay_us(100000);
		BEEP_Reset();
		LED_G_Set();
	}else if(distance > 2 && distance <=10 ){
		BEEP_Set();
		LED_G_Reset();
		delay_us(300000);
		BEEP_Reset();
		LED_G_Set();
		delay_us(300000);
		BEEP_Set();
		LED_G_Reset();
		delay_us(300000);
		BEEP_Reset();
		LED_G_Set();
	}else{
		BEEP_Reset();
		LED_G_Set();
	}
}	

/*************************************** END OF FILE ****************************************/
