
//------------------------------------------------------------------------------
//�����ļ�����
#include "r_cg_macrodriver.h"
#include "r_cg_serial.h"

#include "h_type_define.h"
#include "m_test.h"
#include "m_ad.h"
#include "m_peripheral_control.h"
#include "m_com.h"

//------------------------------------------------------------------------------
//��������
void com_init(void);      //ͨѶ��ʼ�������ڳ�ʼ�������е���

void com_rx_init(void);  //ͨѶ���ճ�ʼ����������ѭ�������е���

void com_rx_data_deal(void);  //ͨѶ�������ݴ����������ѭ�������е���

void com_tx_init(void);         //ͨѶ���ͳ�ʼ����������ѭ�������е���

void com_rx_delaytime(void);  //ͨѶ������ʱ������1ms��ʱ�����е���

void com_tx_delaytime(void);  //ͨѶ������ʱ������1ms��ʱ�����е���

void com_rx_end_delaytime(void);   //ͨѶ���������ʱ������1ms��ʱ�����е���

void com_fault_delaytime(void);    //ͨѶ������ʱ������1s��ʱ�����е���

void com_rx_int(uint8_t rx_data);       //ͨѶ�����жϳ���

void com_tx_int(void);                  //ͨѶ�����жϳ���

unsigned int CRC16(unsigned char *puchmsg, unsigned int usDataLen);    //����CRCУ��ĳ���
//------------------------------------------------------------------------------
//��־����
flag_type flg_com;
//------------------------------------------------------------------------------
//��������
uint8_t guc_com_tx_buffer[9];  //ͨѶ���ͻ�����
uint8_t guc_com_rx_buffer[53];  //ͨѶ���ջ�����  23

int16_t gss_com_tx_delaytimer;     //ͨѶ������ʱ��ʱ��
int16_t gss_com_rx_delaytimer;     //ͨѶ������ʱ��ʱ��

int16_t gss_com_rx_end_delaytimer; //ͨѶ�������־
int16_t gss_com_fault_delaytimer;  //ͨѶ������ʱ��ʱ��

uint8_t guc_com_tx_point;     //ͨѶ��������
uint8_t guc_com_rx_point;     //ͨѶ��������

//------------------------------------------------------------------------------
uint8_t guc_led_buffer[LED_BUF_LEN];    //����ܼĴ���
uint8_t guc_key_buffer[KEY_BUF_LEN];    //�����Ĵ���

uint8_t guc_key_val[KEY_BUF_LEN];       //����ֵ  ����ֻ���� guc_key_val[0]

/*************************************************************************************************************************************************************
�������ܣ���ʼ�ϵ�ʱ�ĳ�ʼ��ͨѶ�ڣ�Ĭ������/�ӻ���ѡ��

����λ�ã����̳�ʼ��---------------------------------ok
**************************************************************************************************************************************************************/
void com_init(void) 
{
    bflg_com_rx_delaytime = 1;   //Ĭ��Ϊ���գ��ӻ�
    gss_com_rx_delaytimer = 5;
    
    bflg_com_tx_delaytime = 0;
    gss_com_tx_delaytimer = 0;   
}

/*************************************************************************************************************************************************************
�������ܣ�����ʱ���ջ����ʼ��

����λ�ã���ѭ��----------------------------------δ��
**************************************************************************************************************************************************************/
void com_rx_init(void)    
{
    uint8_t i;
    
    for (i = 0; i < 53; i++)      //23
    {
        guc_com_rx_buffer[i] = 0;  //����ջ�����
    }
    
    bflg_com_rx_busy = 0;
    guc_com_rx_point = 0;
}
/*************************************************************************************************************************************************************
�������ܣ��Խ��յĺ������д���

����λ�ã���ѭ��----------------------------------δ��
**************************************************************************************************************************************************************/
void com_rx_data_deal(void)   //ͨѶ�������ݴ����������ѭ�������е���
{
    uint8_t i, msb_byte = 0,lsb_byte = 0;
    uint16_t lus_tmp;
    word_type com_crc;
    //------------------------------------------------------
    if ((guc_com_rx_buffer[1] == 0x03)) //����ǲ�ѯ����
    {
        com_crc.uword = CRC16(guc_com_rx_buffer, 6);
        if ((com_crc.ubyte.low == guc_com_rx_buffer[6])
         && (com_crc.ubyte.high == guc_com_rx_buffer[7]))
        {
            bflg_com_tx_delaytime = 1;       //�÷�����ʱ��־
            gss_com_tx_delaytimer = 10;      //������ʱ��ʱ����ֵ
            
            //bflg_com_rx_delaytime = 0;       //�巢����ʱ��־
            //gss_com_rx_delaytimer = 0;       //������ʱ��ʱ������
            
            bflg_com_fault = 0;              //��ͨѶ���ϱ�־
            gss_com_fault_delaytimer = 0;    //��ͨѶ���ϼ�ʱ��
            //----------------------------------------------
            //���ݴ���
            bflg_read_write_data = 0;   //��ǰ�Ƕ�����  

            //LOCK_STATE = 1;  //debug ָʾ
        }
        else
        {
            bflg_com_rx_delaytime = 1;        
            gss_com_rx_delaytimer = 10;       
        }
    }
    else if (guc_com_rx_buffer[1] == 0x10)   //����Ƕ���趨����
    {
        lus_tmp = guc_com_rx_buffer[6] + 7;
        
        com_crc.uword = CRC16(guc_com_rx_buffer, lus_tmp);
        if ((com_crc.ubyte.low == guc_com_rx_buffer[lus_tmp])
         && (com_crc.ubyte.high == guc_com_rx_buffer[lus_tmp + 1]))
        {
            bflg_com_tx_delaytime = 1;  //�÷�����ʱ��־
            gss_com_tx_delaytimer = 10; //������ʱ��ʱ����ֵ
            
            //bflg_com_rx_delaytime = 0;       //�巢����ʱ��־
            //gss_com_rx_delaytimer = 0;       //������ʱ��ʱ������
            
            bflg_com_fault = 0;              //��ͨѶ���ϱ�־
            gss_com_fault_delaytimer = 0;    //��ͨѶ���ϼ�ʱ��
            //----------------------------------------------
            //���ݴ���
            bflg_read_write_data = 1;   //��ǰ��д����
            //----------------------------------------------
            gss_THW_calibration |= (guc_com_rx_buffer[7]<<8);      //����У׼H
            gss_THW_calibration |= guc_com_rx_buffer[8];           //����У׼L
            
            gss_humidity_calibration |= (guc_com_rx_buffer[9]<<8); //ʪ��У׼H
            gss_humidity_calibration |= guc_com_rx_buffer[10];     //ʪ��У׼L
            
            gus_max_sterilization_time |= (guc_com_rx_buffer[11]<<8);  //ɱ��ʱ��H
            gus_max_sterilization_time |= guc_com_rx_buffer[12];       //ɱ��ʱ��L
            
            guc_sterilization_type = guc_com_rx_buffer[13];            //ɱ��ģʽ

            guc_lock_state = guc_com_rx_buffer[14];                    //����״̬
            guc_door_state = guc_com_rx_buffer[15];                    //�ŵ�״̬
            //LOCK_STATE = 0;
        }
        else
        {
            bflg_com_rx_delaytime = 1;        
            gss_com_rx_delaytimer = 10;       
        }
    }
    else if((guc_com_rx_buffer[1] == 0xaa))      //�������ָ��
    {
        com_crc.uword = CRC16(guc_com_rx_buffer, 6);
        if ((com_crc.ubyte.low == guc_com_rx_buffer[6])
         && (com_crc.ubyte.high == guc_com_rx_buffer[7]))
        {
            bflg_test_mode = 1;
            
            bflg_com_tx_delaytime = 1;       //�÷�����ʱ��־
            gss_com_tx_delaytimer = 10;      //������ʱ��ʱ����ֵ
            
            bflg_com_fault = 0;              //��ͨѶ���ϱ�־
            gss_com_fault_delaytimer = 0;    //��ͨѶ���ϼ�ʱ��
            //----------------------------------------------
            //���ݴ���
            bflg_read_write_data = 0;   //��ǰ�Ƕ�����  
        }
        else
        {
            bflg_com_rx_delaytime = 1;        
            gss_com_rx_delaytimer = 10;       
        }
    }    
    else    //������
    {
        bflg_com_rx_delaytime = 1;   
        gss_com_rx_delaytimer = 10;   
    }
}

/*************************************************************************************************************************************************************
�������ܣ����ͳ�ʼ��������Ҫ���͵����ݸ������ͻ���

����λ�ã���ѭ��----------------------------------δ��
**************************************************************************************************************************************************************/
void com_tx_init(void)   //ͨѶ���ͳ�ʼ����������ѭ�������е���
{
    word_type com_crc;      
    uint8_t  msb_byte = 0, lsb_byte = 0;
    //------------------------------------------------------cc
    if (bflg_read_write_data == 0) //����Ƕ�����  //peak ��ȡ������ֵ
    {
        guc_com_tx_buffer[0] = guc_com_rx_buffer[0];
        guc_com_tx_buffer[1] = guc_com_rx_buffer[1];
        guc_com_tx_buffer[2] = 6;  //�ֽڸ���6
        //--------------------------------------------------
        
        guc_com_tx_buffer[3] |= (uint8_t)(gss_THW>>8);    //����H �ѷŴ�10
        guc_com_tx_buffer[4] |= (uint8_t)gss_THW;    
        guc_com_tx_buffer[5] = 0x00;                      //ʪ��H       
        guc_com_tx_buffer[6] = guc_humidity;          
        guc_com_tx_buffer[7] = 0;
        guc_com_tx_buffer[7] |= bflg_sterilization_state; //ɱ��״̬bit0
        guc_com_tx_buffer[7] |= bflg_THW_fault<<1;        //���¹���bit1
        guc_com_tx_buffer[7] |= bflg_humidity_fault<<2;   //ʪ�ȹ���bit2
        guc_com_tx_buffer[8] = 0x00;                      //Ԥ��
        //--------------------------------------------------
        com_crc.uword = CRC16(guc_com_tx_buffer, 9);
        guc_com_tx_buffer[9] = com_crc.ubyte.low;
        guc_com_tx_buffer[10] = com_crc.ubyte.high;
    }
    else  //�����д���� //��������ܡ�LED��ʾ
    {
        guc_com_tx_buffer[0] = guc_com_rx_buffer[0];  //peak���豸��ַ
        guc_com_tx_buffer[1] = guc_com_rx_buffer[1];  //������
        guc_com_tx_buffer[2] = guc_com_rx_buffer[2];   
        guc_com_tx_buffer[3] = guc_com_rx_buffer[3];
        guc_com_tx_buffer[4] = guc_com_rx_buffer[4];
        guc_com_tx_buffer[5] = guc_com_rx_buffer[5];
        //--------------------------------------------------
        com_crc.uword = CRC16(guc_com_tx_buffer, 6);
        guc_com_tx_buffer[6] = com_crc.ubyte.low;
        guc_com_tx_buffer[7] = com_crc.ubyte.high;
    }
    
    guc_com_tx_point = 0;
    bflg_com_tx_busy = 1;
    TXD0 = guc_com_tx_buffer[guc_com_tx_point++];
}
/*************************************************************************************************************************************************************
�������ܣ���ʱ�೤ʱ���ٽ���

����λ�ã�1ms��ʱ��----------------------------------ok
**************************************************************************************************************************************************************/
void com_rx_delaytime(void)     //ͨѶ������ʱ������1ms��ʱ�����е���
{
    if (bflg_com_rx_delaytime == 1)
    {
        gss_com_rx_delaytimer--;
        if (gss_com_rx_delaytimer <= 0)
        {
            gss_com_rx_delaytimer = 0;
            bflg_com_rx_delaytime = 0;
            
            bflg_com_allow_rx = 1;
        }
    }
}
/*************************************************************************************************************************************************************
�������ܣ���ʱ�೤ʱ���ٷ���

����λ�ã�1ms��ʱ��----------------------------------ok
**************************************************************************************************************************************************************/
void com_tx_delaytime(void)     //ͨѶ������ʱ������1ms��ʱ�����е���
{
    if (bflg_com_tx_delaytime == 1)
    {
        gss_com_tx_delaytimer--;
        if (gss_com_tx_delaytimer <= 0)
        {
            gss_com_tx_delaytimer = 0;
            bflg_com_tx_delaytime = 0;
            
            bflg_com_allow_tx = 1;
        }
    }
}
/*******************************************************************************************************************************
�������ܣ��ڽ���æ������û���յ�������,��ʱ��

����λ�ã�1ms��ʱ����-------------------ok
********************************************************************************************************************************/
void com_rx_end_delaytime(void) //ͨѶ���������ʱ������1ms��ʱ�����е���
{
    if (bflg_com_rx_busy == 1)  //�����ǰ����æ
    {
        gss_com_rx_end_delaytimer++;
        if (gss_com_rx_end_delaytimer >= 10) // >= 2   //����
        {
            gss_com_rx_end_delaytimer = 0;
            bflg_com_rx_busy = 0;
            
            bflg_com_rx_end = 1;
            
            //bflg_com_rx_delaytime = 1;
            //gss_com_rx_delaytimer = 10;
        }
    }
    //else ����ɾ��
    //{
      //  gss_com_rx_end_delaytimer = 0; //peak add
    //}
}
/*******************************************************************************************************************************
�������ܣ�������ʱ����

����λ�ã�1ms��ʱ����-------------------δ��
********************************************************************************************************************************/
void com_fault_delaytime(void)     //ͨѶ������ʱ������1s��ʱ�����е���
{
    if (bflg_com_fault == 0)
    {
        gss_com_fault_delaytimer++;
        if (gss_com_fault_delaytimer >= 30)  //30
        {
            gss_com_fault_delaytimer = 0;
            bflg_com_fault = 1;
        }
    }
}

/*******************************************************************************************************************************
�������ܣ��������ݣ��˺�����ϵͳ�����ж��У�����ж�ÿ�ж�һ���ǽ���һ���ֽ�

����λ�ã���ϵͳ���ڽ����ж���------------------ok
********************************************************************************************************************************/
void com_rx_int(uint8_t rx_data)   //ͨѶ�����жϳ���
{
    uint8_t luc_com_rx_byte;  //�����ֽ���ʱ������
    
    gss_com_rx_end_delaytimer = 0; //���������ʱ��ʱ��
    //--------------------------------------------------
    luc_com_rx_byte = rx_data;
    
    if ((bflg_com_rx_busy == 0) && (luc_com_rx_byte == 0x02))    //����޽���æ���Ҵӵ�ַ��ȷ
    {
        bflg_com_rx_busy = 1;
        
        guc_com_rx_buffer[0] = luc_com_rx_byte;
        guc_com_rx_point = 1;
    }
    else if (bflg_com_rx_busy == 1)     //�������æ
    {
        guc_com_rx_buffer[guc_com_rx_point++] = luc_com_rx_byte; //�������++
        
        if ((guc_com_rx_buffer[1] == 0x03) && (guc_com_rx_point >= 8))     //����ǲ�ѯ����ҽ����������
        {
            guc_com_rx_point = 0;  //������ֽ�����
            bflg_com_rx_busy = 0;  //�����æ��־
            bflg_com_rx_ok = 1;    //�ý��ճɹ���־
            
            //bflg_com_rx_delaytime = 1;   //�ý�����ʱ��־
            //gss_com_rx_delaytimer = 50;  //������ʱ��ʱ����ֵ
        }
        else if ((guc_com_rx_buffer[1] == 0x10) && (guc_com_rx_point >= 23))     //����Ƕ���趨����ҽ��յ��ֽ�������֡
        {
            guc_com_rx_point = 0;   //������ֽ�����
            bflg_com_rx_busy = 0;   //�����æ��־
            bflg_com_rx_ok = 1;     //�ý��ճɹ���־
            
            //bflg_com_rx_delaytime = 1;   //�ý�����ʱ��־
            //gss_com_rx_delaytimer = 50;  //������ʱ��ʱ����ֵ
        }
        else if ((guc_com_rx_buffer[1] == 0xaa) && (guc_com_rx_point >= 8))     //����ǲ���ģʽ
        {
            //bflg_test_mode = 1;       //����ģʽ  �������ģʽ����Ϊ����ģʽ

            guc_com_rx_point = 0;     //������ֽ�����
            bflg_com_rx_busy = 0;     //�����æ��־
            bflg_com_rx_ok = 1;     //�ý��ճɹ���־
            
            //R_UART0_Stop();
            //bflg_com_tx_delaytime = 1;    
            //gss_com_tx_delaytimer = 100;   
        }
        else if((guc_com_rx_buffer[1] != 0x03)&&(guc_com_rx_buffer[1] != 0x10)&&(guc_com_rx_buffer[1] != 0xaa))
        {
            guc_com_rx_point = 0;
            bflg_com_rx_busy = 0;
            
            R_UART0_Stop();
            bflg_com_rx_delaytime = 1;   
            gss_com_rx_delaytimer = 10;  
        }
        else if(guc_com_rx_point >= 23)  
        {
            guc_com_rx_point = 0;
            bflg_com_rx_busy = 0;
            
            R_UART0_Stop();
            bflg_com_rx_delaytime = 1;   
            gss_com_rx_delaytimer = 10;  
        }
    }
}


/*************************************************************************************************************************************************************
�������ܣ��ֽ����ж���һ��������

����λ�ã�ϵͳ�жϳ�����---------------------------------ok
**************************************************************************************************************************************************************/
void com_tx_int(void)    //ͨѶ�����жϳ���
{
    if (bflg_com_tx_busy == 1)
    {                                   //��Ӧ��ָ��
        if (bflg_read_write_data == 0)  
         {   
            if (guc_com_tx_point < 11)
            {
                TXD0 = guc_com_tx_buffer[guc_com_tx_point++];    //��������
            }
            else
            {
                guc_com_tx_point = 0;   //�巢���ֽ�����
                bflg_com_tx_busy = 0;   //�巢��æ��־
                bflg_com_tx_ok = 1;
                
                //bflg_com_rx_delaytime = 1;   //��comͨѶ������ʱ��־
                //gss_com_rx_delaytimer = 5;   //comͨѶ������ʱ��ʱ����ֵ
            }
        }
        else    //�����д���� peak��������ʾ��д�������ʾ������
        {
            if (guc_com_tx_point < 8)
            {
                TXD0 = guc_com_tx_buffer[guc_com_tx_point++];   //��������
            }
            else
            {
                guc_com_tx_point = 0;   //�巢���ֽ�����
                bflg_com_tx_busy = 0;   //�巢��æ��־
                bflg_com_tx_ok = 1;
                
                //bflg_com_rx_delaytime = 1;   //��ͨѶ������ʱ��־
                //gss_com_rx_delaytimer = 5;   //ͨѶ������ʱ��ʱ����ֵ
            }
        }
    }
}
/*************************************************************************************************************************************************************
�������ܣ�CRCУ��

����λ�ã�����/��������ʱ�����ݽ��е�У��----------------------------------δ��
**************************************************************************************************************************************************************/
unsigned int CRC16(unsigned char *puchmsg, unsigned int usDataLen)    
{
    unsigned int uchCRC;
    unsigned int uIndex_x, uIndex_y;
    
    uchCRC = 0xFFFF;
    
    if (usDataLen > 0)
    {
        for (uIndex_x = 0; uIndex_x <= (usDataLen - 1); uIndex_x++)
        {
            uchCRC = uchCRC ^ (unsigned int) (*puchmsg);
            puchmsg++;
            
            for (uIndex_y = 0; uIndex_y <= 7; uIndex_y++)
            {
                if ((uchCRC & 0x0001) != 0)
                {
                    uchCRC = (uchCRC >> 1) ^ 0xA001;
                }
                else
                {
                    uchCRC = uchCRC >> 1;
                }
            }
        }
    }
    else
    {
        uchCRC = 0;
    }
    
    return uchCRC;
}

/*****************************************END OF THE FILE*********************************************************************************************************************/
