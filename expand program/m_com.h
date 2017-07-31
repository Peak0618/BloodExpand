#ifndef _M_COM_H_
#define _M_COM_H_
//外部函数声明//

extern void com_init(void);   //通讯初始化程序，在初始化程序中调用

extern void com_rx_init(void);     //通讯接收初始化程序，在主循环程序中调用

extern void com_rx_data_deal(void);     //通讯接收数据处理程序，在主循环程序中调用

extern void com_tx_init(void);     //通讯发送数据处理程序，在主循环程序中调用

extern void com_rx_delaytime(void);     //通讯接收延时程序，在1ms定时程序中调用

extern void com_tx_delaytime(void);     //通讯发送延时程序，在1ms定时程序中调用

extern void com_rx_end_delaytime(void); //通讯接收结束延时程序，在1ms定时程序中调用

extern void com_fault_delaytime(void);  //通讯故障延时程序，在1s定时程序中调用

extern void com_rx_int(uint8_t rx_data);     //通讯接收中断程序

extern void com_tx_int(void); //通讯发送中断程序

extern unsigned int CRC16(unsigned char *puchmsg, unsigned int usDataLen);    

//------------------------------------------------------------------------------
//外部变量声明//
extern flag_type flg_com;
          #define   bflg_com_tx_delaytime    flg_com.bits.bit0   //通讯发送延时标志
          #define   bflg_com_rx_delaytime    flg_com.bits.bit1   //通讯接收延时标志
          #define   bflg_com_allow_tx        flg_com.bits.bit2   //通讯允许发送标志
          #define   bflg_com_allow_rx        flg_com.bits.bit3   //通讯允许接收标志
          #define   bflg_com_tx_busy         flg_com.bits.bit4   //通讯发送忙标志
          #define   bflg_com_rx_busy         flg_com.bits.bit5   //通讯接收忙标志
          #define   bflg_com_tx_ok           flg_com.bits.bit6   //通讯发送成功标志
          #define   bflg_com_rx_ok           flg_com.bits.bit7   //通讯接收成功标志
          #define   bflg_com_rx_end          flg_com.bits.bit8   //通讯接收结束标志
          
          #define   bflg_com_fault           flg_com.bits.bit9   //通讯故障标志
          
          #define   bflg_read_write_data     flg_com.bits.bit10  //读写数据标志

          
//------------------------------------------------------------------------------
#define   LED_ADDR  0x00
#define   KEY_ADDR  0x42

#define   LED_BUF_LEN    13
#define   KEY_BUF_LEN    4

#define   COM_RX_MODE    (P1.0 = 0)    //接收模式
#define   COM_TX_MODE    (P1.0 = 1)    //发送模式


extern uint8_t guc_led_buffer[LED_BUF_LEN];  //数码管寄存器
extern uint8_t guc_key_buffer[KEY_BUF_LEN];  //按键寄存器

extern uint8_t guc_key_val[KEY_BUF_LEN];     //按键值
extern uint8_t guc_com_tx_buffer[9];   //通讯发送缓存器
extern uint8_t guc_com_rx_buffer[53];  //通讯接收缓存器  //23

extern int16_t gss_com_tx_delaytimer;     //通讯发送延时计时器
extern int16_t gss_com_rx_delaytimer;     //通讯接收延时计时器
extern uint8_t guc_com_tx_point;          //通讯发送索引
extern uint8_t guc_com_rx_point;
extern int16_t gss_com_rx_end_delaytimer; //通讯接收完标志

#endif
