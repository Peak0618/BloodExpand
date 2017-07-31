/************************************************************************************************************************
overview: �Ժ󱸰���й�װ���ʱ����
          1�����巢�ض�������ָ��ἤ��󱸰�������ģʽ
          2���ڲ��Գ�������
*************************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_timer.h"
#include "r_cg_serial.h"

#include "h_type_define.h"
#include "m_com.h"
#include "m_ad.h"
#include "m_test.h"
#include "m_peripheral_control.h"

#include <stdlib.h>


//��������
void test_mode_com();
void test_com_rx_int(uint8_t rx_data);
void test_com_tx_int();
void test_com_rx_data_deal(void);   
void test_com_tx_init(void);
void test_in_out_pin(void);
void test_error_code_deal(void);

//����
flag_type flg_test;


uint8_t guc_test_mode_code;     //�������

//�궨�� 
#define   IN_PIN_DOOR_CN31     P5.0       //��������
#define   OUT_PIN_LOCK_CN5     P1.7       //�����

/*******************************************************************************************************************************
�������ܣ�����ģʽ�½��е�ͨѶ����

����λ�ã���ѭ��-----------------------------------------ok
********************************************************************************************************************************/
void test_mode_com(void)
{
    if (bflg_com_allow_rx == 1)  //����������
    {
        bflg_com_allow_rx = 0;
        
        com_rx_init();   
        COM_RX_MODE;
        R_UART0_Start();
    }
    if (bflg_com_rx_end == 1)    //������ս���
    {
        bflg_com_rx_end = 0;
        
        R_UART0_Stop();
    }
    if (bflg_com_rx_ok == 1)     //������ճɹ�
    {
        bflg_com_rx_ok = 0;
        
        R_UART0_Stop();
        test_com_rx_data_deal();   
    }
    if (bflg_com_allow_tx == 1)  //���������
    {
        bflg_com_allow_tx = 0;
        
        R_UART0_Start();
        COM_TX_MODE;
        test_com_tx_init();   
    }
    if (bflg_com_tx_ok == 1)     //������ͳɹ�
    {
        bflg_com_tx_ok = 0;      
        
        R_UART0_Stop();
    }
}

/*******************************************************************************************************************************
�������ܣ����ݽ���

����λ�ã�ϵͳ���ڽ����ж�-----------------------------------------δ��
********************************************************************************************************************************/
void test_com_rx_int(uint8_t rx_data)
{
    uint8_t luc_com_rx_byte;  //�����ֽ���ʱ������

    gss_com_rx_end_delaytimer = 0; //���������ʱ��ʱ��
    
    luc_com_rx_byte = rx_data;

    if((bflg_com_rx_busy == 0) && (luc_com_rx_byte == 0x02))    //LED�ĵ�ַ
    {
        bflg_com_rx_busy = 1;
        
        guc_com_rx_buffer[0] = luc_com_rx_byte;
        guc_com_rx_point = 1;
    }
    else if(bflg_com_rx_busy == 1)
    {
        guc_com_rx_buffer[guc_com_rx_point++] = luc_com_rx_byte;
        
        if ((guc_com_rx_buffer[1] == 0xaa) && (guc_com_rx_point >= 8))     
        {
            guc_com_rx_point = 0; 
            bflg_com_rx_busy = 0;  
            bflg_com_rx_ok = 1;    
            
            bflg_com_rx_delaytime = 1;   
            gss_com_rx_delaytimer = 50;  
        }
        else if(guc_com_rx_buffer[1] != 0xaa)
        {
            guc_com_rx_point = 0;
            bflg_com_rx_busy = 0;
            
            R_UART0_Stop();
            bflg_com_rx_delaytime = 1;   
            gss_com_rx_delaytimer = 10;  
        }
        else if(guc_com_rx_point >= 8)  
        {
            guc_com_rx_point = 0;
            bflg_com_rx_busy = 0;
            
            R_UART0_Stop();
            bflg_com_rx_delaytime = 1;   
            gss_com_rx_delaytimer = 10;  
        }        
    }
}
/*******************************************************************************************************************************
�������ܣ���������

����λ�ã�ϵͳ���ڷ����ж�-----------------------------------------δ��
********************************************************************************************************************************/
void test_com_tx_int(void)
{
    if (bflg_com_tx_busy == 1)
    {
        if (guc_com_tx_point < 9)
        {
            TXD0 = guc_com_tx_buffer[guc_com_tx_point++];    //��������
        }
        else
        {
            guc_com_tx_point = 0;   //�巢���ֽ�����
            bflg_com_tx_busy = 0;   //�巢��æ��־
            bflg_com_tx_ok = 1;
            
            bflg_com_rx_delaytime = 1;   //��comͨѶ������ʱ��־
            gss_com_rx_delaytimer = 5;   //comͨѶ������ʱ��ʱ����ֵ
        }
    }
}

/*******************************************************************************************************************************
�������ܣ�����ģʽ�¶Խ��յ����ݴ���

����λ�ã���ѭ�� test_mode_com()-----------------------------------------ok
********************************************************************************************************************************/
void test_com_rx_data_deal(void)   
{
    uint8_t i, msb_byte = 0,lsb_byte = 0;
    uint16_t lus_tmp;
    word_type com_crc;
    //------------------------------------------------------
    if ((guc_com_rx_buffer[1] == 0xaa)) //����ǲ�ѯ����
    {
        com_crc.uword = CRC16(guc_com_rx_buffer, 6);
        if ((com_crc.ubyte.low == guc_com_rx_buffer[6])
         && (com_crc.ubyte.high == guc_com_rx_buffer[7]))
        {
            bflg_com_tx_delaytime = 1;       //�÷�����ʱ��־
            gss_com_tx_delaytimer = 10;      //������ʱ��ʱ����ֵ
            
            bflg_com_rx_delaytime = 0;       //�巢����ʱ��־
            gss_com_rx_delaytimer = 0;       //������ʱ��ʱ������
            
            //bflg_com_fault = 0;              //��ͨѶ���ϱ�־
            //gss_com_fault_delaytimer = 0;    //��ͨѶ���ϼ�ʱ��
        }
    } 
    else
    {
        bflg_com_rx_delaytime = 1;        
        gss_com_rx_delaytimer = 10;       
    }
}

/*******************************************************************************************************************************
�������ܣ�����ģʽ�³�ʼ�����͵�����

����λ�ã���ѭ�� test_mode_com()-----------------------------------------ok
********************************************************************************************************************************/
void test_com_tx_init(void)
{
    word_type com_crc;      
    uint8_t  msb_byte = 0, lsb_byte = 0;

    guc_com_tx_buffer[0] = guc_com_rx_buffer[0];
    guc_com_tx_buffer[1] = guc_com_rx_buffer[1];
    guc_com_tx_buffer[2] = 4;  //�ֽڸ���4
    //--------------------------------------------------
    guc_com_tx_buffer[3] = (uint8_t)gss_TA;   //�ȴ����λ
    guc_com_tx_buffer[4] = (uint8_t)(gss_TA>>8);     
    guc_com_tx_buffer[5] = guc_test_mode_code;         
    guc_com_tx_buffer[6] = 0;          

    //--------------------------------------------------
    com_crc.uword = CRC16(guc_com_tx_buffer, 7);
    guc_com_tx_buffer[7] = com_crc.ubyte.low;
    guc_com_tx_buffer[8] = com_crc.ubyte.high;
    
    guc_com_tx_point = 0;
    bflg_com_tx_busy = 1;
    TXD0 = guc_com_tx_buffer[guc_com_tx_point++];
}



/*******************************************************************************************************************************
�������ܣ����Ժ󱸰��ϵ����������

����λ�ã���ѭ��-----------------------------------------ok
********************************************************************************************************************************/
void test_in_out_pin(void)
{
    if(IN_PIN_DOOR_CN31 == 1)
    {
        OUT_PIN_LOCK_CN5 = 1;
    }
    else
    {
        OUT_PIN_LOCK_CN5 = 0;
    }
    
    //--------------------------------------------------------------

    if(bflg_test_pwm == 0)
    {
        bflg_test_pwm = 1;
        STERILIZE_POWER = 1;       //ɱ��ģ�鹩�翪
        TDR03 = 48828;
        R_TAU0_Channel2_Start();   //pwm����ɱ��ģ������
    }
}



/*******************************************************************************************************************************
�������ܣ����ϴ���Ĵ�����Ҫ���͸����壬Ȼ������������ʾ����ʾ
          1��AD�ڣ���װ�Ͻ����AD�ڹ̶��ӵ�5v���������жϼ�⵽����5v���Ҽ��ɣ�
          2��E2���
          
����λ�ã���ѭ��-----------------------------------------ok
********************************************************************************************************************************/
void test_error_code_deal(void)
{  
    if(abs(25 - guc_reserved_ad) >= 5)  //2.5v ��ƫ����0.5vΪ����  ��Щֵ���ݹ�װ����ͼ��ֵ�������
    {
        guc_test_mode_code = 1;
    }
    else if(abs(430 - gss_THW) >= 50)  //43��    ƫ����5��Ϊ����
    {
        guc_test_mode_code = 2;
    }
    else if(abs(57 - guc_humidity) >= 5)//ʪ��57��ƫ����5Ϊ����
    {
        guc_test_mode_code = 3;
    }
    else if(abs(39 - guc_sterilize_monitor) >= 10) //3.9v ,ƫ����1vΪ����
    {
        guc_test_mode_code = 4;
    }
    /*else if()                       //E2��+
    {
        guc_test_mode_code = 2;
    }*/
    else
    {
        guc_test_mode_code = 0;       //�޹���
    }
}











/**************************************END OF THE FILE******************************************************************/
