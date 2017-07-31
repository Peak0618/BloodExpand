/************************************************************************************************************************
overview: 对后备板进行工装检测时进入
          1、主板发特定的数据指令会激活后备板进入测试模式
          2、在测试程序运行
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


//函数声明
void test_mode_com();
void test_com_rx_int(uint8_t rx_data);
void test_com_tx_int();
void test_com_rx_data_deal(void);   
void test_com_tx_init(void);
void test_in_out_pin(void);
void test_error_code_deal(void);

//变量
flag_type flg_test;


uint8_t guc_test_mode_code;     //错误代码

//宏定义 
#define   IN_PIN_DOOR_CN31     P5.0       //门输入检测
#define   OUT_PIN_LOCK_CN5     P1.7       //锁输出

/*******************************************************************************************************************************
函数功能：测试模式下进行的通讯处理

函数位置：主循环-----------------------------------------ok
********************************************************************************************************************************/
void test_mode_com(void)
{
    if (bflg_com_allow_rx == 1)  //如果允许接收
    {
        bflg_com_allow_rx = 0;
        
        com_rx_init();   
        COM_RX_MODE;
        R_UART0_Start();
    }
    if (bflg_com_rx_end == 1)    //如果接收结束
    {
        bflg_com_rx_end = 0;
        
        R_UART0_Stop();
    }
    if (bflg_com_rx_ok == 1)     //如果接收成功
    {
        bflg_com_rx_ok = 0;
        
        R_UART0_Stop();
        test_com_rx_data_deal();   
    }
    if (bflg_com_allow_tx == 1)  //如果允许发送
    {
        bflg_com_allow_tx = 0;
        
        R_UART0_Start();
        COM_TX_MODE;
        test_com_tx_init();   
    }
    if (bflg_com_tx_ok == 1)     //如果发送成功
    {
        bflg_com_tx_ok = 0;      
        
        R_UART0_Stop();
    }
}

/*******************************************************************************************************************************
函数功能：数据接收

函数位置：系统串口接收中断-----------------------------------------未调
********************************************************************************************************************************/
void test_com_rx_int(uint8_t rx_data)
{
    uint8_t luc_com_rx_byte;  //接收字节临时缓冲区

    gss_com_rx_end_delaytimer = 0; //清接收完延时计时器
    
    luc_com_rx_byte = rx_data;

    if((bflg_com_rx_busy == 0) && (luc_com_rx_byte == 0x02))    //LED的地址
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
函数功能：发送数据

函数位置：系统串口发送中断-----------------------------------------未调
********************************************************************************************************************************/
void test_com_tx_int(void)
{
    if (bflg_com_tx_busy == 1)
    {
        if (guc_com_tx_point < 9)
        {
            TXD0 = guc_com_tx_buffer[guc_com_tx_point++];    //发送数据
        }
        else
        {
            guc_com_tx_point = 0;   //清发送字节索引
            bflg_com_tx_busy = 0;   //清发送忙标志
            bflg_com_tx_ok = 1;
            
            bflg_com_rx_delaytime = 1;   //置com通讯接收延时标志
            gss_com_rx_delaytimer = 5;   //com通讯接收延时计时器赋值
        }
    }
}

/*******************************************************************************************************************************
函数功能：测试模式下对接收的数据处理

函数位置：主循环 test_mode_com()-----------------------------------------ok
********************************************************************************************************************************/
void test_com_rx_data_deal(void)   
{
    uint8_t i, msb_byte = 0,lsb_byte = 0;
    uint16_t lus_tmp;
    word_type com_crc;
    //------------------------------------------------------
    if ((guc_com_rx_buffer[1] == 0xaa)) //如果是查询命令
    {
        com_crc.uword = CRC16(guc_com_rx_buffer, 6);
        if ((com_crc.ubyte.low == guc_com_rx_buffer[6])
         && (com_crc.ubyte.high == guc_com_rx_buffer[7]))
        {
            bflg_com_tx_delaytime = 1;       //置发送延时标志
            gss_com_tx_delaytimer = 10;      //发送延时计时器赋值
            
            bflg_com_rx_delaytime = 0;       //清发送延时标志
            gss_com_rx_delaytimer = 0;       //发送延时计时器清零
            
            //bflg_com_fault = 0;              //清通讯故障标志
            //gss_com_fault_delaytimer = 0;    //清通讯故障计时器
        }
    } 
    else
    {
        bflg_com_rx_delaytime = 1;        
        gss_com_rx_delaytimer = 10;       
    }
}

/*******************************************************************************************************************************
函数功能：测试模式下初始化发送的数据

函数位置：主循环 test_mode_com()-----------------------------------------ok
********************************************************************************************************************************/
void test_com_tx_init(void)
{
    word_type com_crc;      
    uint8_t  msb_byte = 0, lsb_byte = 0;

    guc_com_tx_buffer[0] = guc_com_rx_buffer[0];
    guc_com_tx_buffer[1] = guc_com_rx_buffer[1];
    guc_com_tx_buffer[2] = 4;  //字节个数4
    //--------------------------------------------------
    guc_com_tx_buffer[3] = (uint8_t)gss_TA;   //先传输低位
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
函数功能：测试后备板上的输入输出口

函数位置：主循环-----------------------------------------ok
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
        STERILIZE_POWER = 1;       //杀菌模块供电开
        TDR03 = 48828;
        R_TAU0_Channel2_Start();   //pwm控制杀菌模块的输出
    }
}



/*******************************************************************************************************************************
函数功能：故障代码的处理，需要传送给主板，然后主板再让显示板显示
          1、AD口：工装上将这个AD口固定接到5v处，所以判断检测到的在5v左右即可；
          2、E2检测
          
函数位置：主循环-----------------------------------------ok
********************************************************************************************************************************/
void test_error_code_deal(void)
{  
    if(abs(25 - guc_reserved_ad) >= 5)  //2.5v ，偏差在0.5v为故障  这些值根据工装连接图的值计算出的
    {
        guc_test_mode_code = 1;
    }
    else if(abs(430 - gss_THW) >= 50)  //43℃    偏差在5℃为故障
    {
        guc_test_mode_code = 2;
    }
    else if(abs(57 - guc_humidity) >= 5)//湿度57，偏差在5为故障
    {
        guc_test_mode_code = 3;
    }
    else if(abs(39 - guc_sterilize_monitor) >= 10) //3.9v ,偏差在1v为故障
    {
        guc_test_mode_code = 4;
    }
    /*else if()                       //E2待+
    {
        guc_test_mode_code = 2;
    }*/
    else
    {
        guc_test_mode_code = 0;       //无故障
    }
}











/**************************************END OF THE FILE******************************************************************/
