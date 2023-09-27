/****************************************Copyright (c)************************************************
**                                      [����ķ�Ƽ�]
**                                        IIKMSIK 
**                            �ٷ����̣�https://acmemcu.taobao.com
**                            �ٷ���̳��http://www.e930bbs.com
**                                   
**--------------File Info-----------------------------------------------------------------------------
** File name         : main.c
** Last modified Date: 2021-8-7         
** Last Version      :		   
** Descriptions      : ʹ�õ�SDK�汾-SDK_17.0.2
**						
**----------------------------------------------------------------------------------------------------
** Created by        : [����ķ]Macro Peng
** Created date      : 2021-8-7  
** Version           : 1.0
** Descriptions      : 2.4G˽��Э��(ESB)�����Ͷ�-���ߵ��PTX��PTX-NRF_ESB_EVENT_RX_RECEIVED�¼��´�ӡACK�غ�
**---------------------------------------------------------------------------------------------------*/
#include <stdbool.h>   
#include <stdint.h>
#include <math.h>

#include "nrf_delay.h"
#include "boards.h"
//Log��Ҫ���õ�ͷ�ļ�
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_uart.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_timer.h"
#include "app_timer.h" 
#include "app_button.h"
#include "nrf_esb.h"
#include "nrf_esb_error_codes.h"
#include "nrf_drv_spi.h"
#include "w25q128.h"
#include "ICM_20948.h"
#include "radio_config.h"


static float  packet[3];                    /**< Packet to transmit. */

/**@brief Function for sending packet.
 */
void send_packet()
{
    // send the packet:
    NRF_RADIO->EVENTS_READY = 0U;	//�¼�׼����
    NRF_RADIO->TASKS_TXEN   = 1;	//����ʹ��

    while (NRF_RADIO->EVENTS_READY == 0U)	//�ȴ�READY�¼�
    {
        // wait
    }

	
    NRF_RADIO->EVENTS_END  = 0U;	//�¼���������
    NRF_RADIO->TASKS_START = 1U;	//����ʼ��ֵ

    while (NRF_RADIO->EVENTS_END == 0U)		//�ȴ����ͽ���
    {
        // wait
    }

	#if 0	//ָʾ�ƺʹ�ӡ�����Բ�Ҫ
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_SENT_OK);
    NRF_LOG_INFO("The packet was sent");
    APP_ERROR_CHECK(err_code);
	#endif

    NRF_RADIO->EVENTS_DISABLED = 0U;	//�¼���ֹ����
    // Disable radio
    NRF_RADIO->TASKS_DISABLE = 1U;		//��ֹ

    while (NRF_RADIO->EVENTS_DISABLED == 0U)	//�ȴ�ֹͣ
    {
        // wait
    }
}

/**@brief Function for initialization oscillators.
 */
void clock_initialization()
{
    /* Start 16 MHz crystal oscillator */
	//����16M����
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    /* Wait for the external oscillator to start up */
	//�ȴ���������
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }

    /* Start low frequency crystal oscillator for app_timer(used by bsp)*/
	//������Ƶ���񣬹�����32768���Ǹ�
    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }
}

#define UART_TX_BUF_SIZE 256       //���ڷ��ͻ����С���ֽ�����
#define UART_RX_BUF_SIZE 256       //���ڽ��ջ����С���ֽ�����




//�����¼��ص��������ú������ж��¼����Ͳ����д���
void uart_error_handle(app_uart_evt_t * p_event)
{
    static uint8_t index = 0;
	  static uint8_t length = 0;
	  static uint8_t data_array[NRF_ESB_MAX_PAYLOAD_LENGTH];
	  //���ݽ����¼�        
	  if (p_event->evt_type ==  APP_UART_DATA_READY)
    {
        UNUSED_VARIABLE(app_uart_get(&data_array[index]));
        index++;
			  //���յ������ַ����߽��յ����ݳ��ȴ�������غɳ���
			  if ((data_array[index - 1] == '#') || (index > NRF_ESB_MAX_PAYLOAD_LENGTH))
				{
				    if(index > NRF_ESB_MAX_PAYLOAD_LENGTH)length = NRF_ESB_MAX_PAYLOAD_LENGTH;
					  else length = index-1;
					  //ESB��������
//				    esb_data_send(data_array,length);
					
					  //���ݳ������㣬�Ա������һ������
					  index = 0;
					  //��תָʾ��D1״̬��ָʾ���ݷ���
					  nrf_gpio_pin_toggle(LED_1);
				}			
    }
		//ͨѶ�����¼�
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    //FIFO�����¼�
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}
//��������
void uart_config(void)
{
	uint32_t err_code;
	
	//���崮��ͨѶ�������ýṹ�岢��ʼ��
  const app_uart_comm_params_t comm_params =
  {
    RX_PIN_NUMBER,//����uart��������
    TX_PIN_NUMBER,//����uart��������
    RTS_PIN_NUMBER,//����uart RTS���ţ����عرպ���Ȼ������RTS��CTS���ţ����������������ԣ������������������ţ����������Կ���ΪIOʹ��
    CTS_PIN_NUMBER,//����uart CTS����
    APP_UART_FLOW_CONTROL_DISABLED,//�ر�uartӲ������
    false,//��ֹ��ż����
    NRF_UART_BAUDRATE_115200//uart����������Ϊ115200bps
  };
  //��ʼ�����ڣ�ע�ᴮ���¼��ص�����
  APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

  APP_ERROR_CHECK(err_code);
	
}


uint16_t times;
uint8_t time_5ms_flag;
uint8_t time_10ms_flag;
uint8_t time_100ms_flag;

const nrf_drv_timer_t SIMPLE_TIMER1 = NRFX_TIMER_INSTANCE(0);

void imu_timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
	if(event_type==NRF_TIMER_EVENT_COMPARE0)
	{
		times++;
			if(times%5==0)
			{ 
				time_5ms_flag++;		
			}
		  if(times%10==0)
			{ 
				time_10ms_flag++;		
			}
			if(times%100==0)
			{   
				time_100ms_flag++;
			}
	}
}

void imu_timer_init(void)
{
	  uint32_t err_code = NRF_SUCCESS;
	  uint32_t time_ms = 1; 

    uint32_t time_ticks;
	
	  nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
	
    err_code = nrfx_timer_init(&SIMPLE_TIMER1, &timer_cfg, imu_timer_event_handler);
    APP_ERROR_CHECK(err_code);
	  
	  time_ticks = nrfx_timer_ms_to_ticks(&SIMPLE_TIMER1, time_ms);
    nrfx_timer_extended_compare( &SIMPLE_TIMER1, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

}



int main()
{
	nrf_gpio_cfg_output(19);
	nrf_gpio_pin_set(19);

	SPI_Flash_Init(); 
//	uart_config();
 	clock_initialization();
	radio_configure();
	NRF_RADIO->PACKETPTR = (uint32_t)&packet[0]; 
	ICM_20948_SPI_begin();

	printf("Device connected!\r\n");
	bool success = true; // Use success to show if the DMP configuration was successful
	success &= (ICM_20948_initializeDMP() == ICM_20948_Stat_Ok);
	success &= (ICM_20948_enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, true) == ICM_20948_Stat_Ok);

  success &= (ICM_20948_setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum


  // Enable the FIFO
  success &= (ICM_20948_enableFIFO(true) == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (ICM_20948_enableDMP(true) == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (ICM_20948_resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (ICM_20948_resetFIFO() == ICM_20948_Stat_Ok);
//	icm20948_init();
//	imu_timer_init();
//	nrf_drv_timer_enable(&SIMPLE_TIMER1);
  // Check success
  if (success)
  {
    printf("DMP enabled!\r\n");
  }
  else
  {
    printf("Enable DMP failed!\r\n");
    printf("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h...\r\n\r\n");
    while (1)
      ; // Do nothing more
  }
	while(true)
	{
			icm_20948_DMP_data_t data;

			ICM_20948_readDMPdataFromFIFO(&data);
//			printf("header %x\r\n",data.header);

			if ((status == ICM_20948_Stat_Ok) || (status == ICM_20948_Stat_FIFOMoreDataAvail) ) // Was valid data available?
			{


				if ((data.header & DMP_header_bitmap_Quat9) > 0 ) // We have asked for GRV data so we should receive Quat6
				{
		//      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
		//      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
		//      // The quaternion data is scaled by 2^30.
					double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
					double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
					double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30


					double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

					double q2sqr = q2 * q2;

					// roll (x-axis rotation)
					double t0 = +2.0 * (q0 * q1 + q2 * q3);
					double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
					double roll = atan2(t0, t1) * 180.0 / 3.1415926;

					// pitch (y-axis rotation)
					double t2 = +2.0 * (q0 * q2 - q3 * q1);
					t2 = t2 > 1.0 ? 1.0 : t2;
					t2 = t2 < -1.0 ? -1.0 : t2;
					double pitch = asin(t2) * 180.0 / 3.1415926;

					// yaw (z-axis rotation)
					double t3 = +2.0 * (q0 * q3 + q1 * q2);
					double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
					double yaw = atan2(t3, t4) * 180.0 / 3.1415926;
					packet[0] = (float)(roll+0.3);
					packet[1] = (float)(pitch-1.1);
					packet[2] = (float)yaw;
//					send_packet();
		//			printf("q1:%lf  ",q1);
		//			printf("q2:%lf  ",q2);
		//			printf("q3:%lf  ",q3);
		//			printf("q0:%lf\r\n",q0);
//					printf("roll:%lf  ",roll);
//					printf("pitch:%lf  ",pitch);
//					printf("yaw:%lf\r\n",yaw);
					nrf_gpio_pin_toggle(19);
				}
			}

			if ((status != ICM_20948_Stat_FIFOMoreDataAvail) ) // If more data is available then we should read it right away - and not delay
			{
				nrf_delay_ms(10);
			}
		}
}


/********************************************END FILE**************************************/
