/************************************************************************************************************************************
overview:  1、对e2的操作
           
*************************************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_wdt.h"

#include "h_type_define.h"
#include "m_rom_para2.h"
#include "m_e2.h"

//函数声明
void init_ram_para2(void);
void init_ram_para(void);
void eeprom2_read_deal(void)  ;
void eeprom_read_deal(void);   
void delay_10us(uint16_t _10us);
void delay_1ms(uint16_t _1ms);      
void eeprom_write(uint16_t lus_addr, int16_t lss_eep_data);    
int16_t eeprom_read(uint16_t lus_addr);  

//变量定义
flag_type flg_e2;

//------------------------------------------------------------------------------
void init_ram_para2(void)
{
    uint8_t i;
    for (i = 0; i < MAX_PARA_NUMBER2; i++)
    {
        //ram_para2[i] = EEPROM_init2[i][0];
        eeprom_write(i + 256, EEPROM_init2[i][0]);
    }
}
//------------------------------------------------------------------------------
void init_ram_para(void)
{
    uint8_t i;
    for (i = 0; i < MAX_PARA_NUMBER; i++)
    {
        //ram_para[i] = EEPROM_init[i][0];
        eeprom_write(i, EEPROM_init[i][0]);
    }
}
//------------------------------------------------------------------------------
void eeprom2_read_deal(void)  //eeprom2读取处理程序，在初始化程序中调用
{
    int16_t i;
    uint8_t luc_read_para_cnt;
    int16_t lss_verify_word;
    
    do
    {
        bflg_askfor_read_eeprom = 0;    //清请求读EEP标志
        //----------------------------------------
        for (i = 0; i < MAX_PARA_NUMBER2; i++)
        {
            ram_para2[i] = eeprom_read(i + 256); //从EEP读参数到RAM中
            R_WDT_Restart();
            
            if ((ram_para2[i] < EEPROM_init2[i][min_row])
               || (ram_para2[i] > EEPROM_init2[i][max_row]))
            {
                bflg_read_eeprom_error = 1;  //置读EEPROM错标志
                break;
            }
        }
        //--------------------------------------------------
        if (bflg_read_eeprom_error == 1)     //如果有读EEPROM错标志
        {
            bflg_read_eeprom_error = 0;      //清读EEPROM错标志
            
            luc_read_para_cnt++;   //读EEP次数加1
            if (luc_read_para_cnt >= 3)      //如果读3次错
            {
                //EEPROM故障处理
                bflg_eeprom_fault = 1;  //置EEP故障标志
                break;
            }
            else
            {
                bflg_askfor_read_eeprom = 1; //置请求读EEPROM标志
            }
        }
    }
    while (bflg_askfor_read_eeprom == 1);
}
//------------------------------------------------------------------------------
void eeprom_read_deal(void)   //eeprom读取处理程序，在初始化程序中调用
{
    int16_t i;
    uint8_t luc_read_para_cnt;
    int16_t lss_verify_word;
    
    do
    {
        bflg_askfor_read_eeprom = 0;    //清请求读EEP标志
        //----------------------------------------
        for (i = 0; i < MAX_PARA_NUMBER; i++)
        {
            ram_para[i] = eeprom_read(i);     //从EEP读参数到RAM中
            R_WDT_Restart();
            
            if ((ram_para[i] < EEPROM_init[i][min_row])
               || (ram_para[i] > EEPROM_init[i][max_row]))
            {
                bflg_read_eeprom_error = 1;  //置读EEPROM错标志
                break;
            }
        }
        //--------------------------------------------------
        if (bflg_read_eeprom_error == 1)     //如果有读EEPROM错标志
        {
            bflg_read_eeprom_error = 0;      //清读EEPROM错标志
            
            luc_read_para_cnt++;   //读EEP次数加1
            if (luc_read_para_cnt >= 3)      //如果读3次错
            {
                //EEPROM故障处理
                bflg_eeprom_fault = 1;  //置EEP故障标志
                break;
            }
            else
            {
                bflg_askfor_read_eeprom = 1; //置请求读EEPROM标志
            }
        }
    }
    while (bflg_askfor_read_eeprom == 1);
}
//------------------------------------------------------------------------------
void delay_10us(uint16_t _10us)    //10us延时程序，在eeprom读写程序中调用
{
    uint16_t i,j;
    for (i = 0; i < (_10us * 2); i++)
    {
        j = i * 3;
        R_WDT_Restart();
    }
}
//------------------------------------------------------------------------------
void delay_1ms(uint16_t _1ms)      //1ms延时程序，在eeprom读写程序中调用
{
    uint16_t i,j;
    for (i = 0; i < (_1ms * 159); i++)
    {
        j = i * 3;
        R_WDT_Restart();
    }
}
//------------------------------------------------------------------------------
void eeprom_write(uint16_t lus_addr, int16_t lss_eep_data)    //写eeprom程序
{
    uint8_t luc_page_addr;
    uint8_t luc_unit_addr;
    uint8_t luc_driver_write;
    
    uint8_t luc_eep_data_H;
    uint8_t luc_eep_data_L;
    
    luc_page_addr = (uint8_t) (lus_addr * 2 / 256);
    luc_unit_addr = (uint8_t) (lus_addr * 2 % 256);
    luc_driver_write = (uint8_t) (( luc_page_addr << 1) & 0x0E | 0xA0);
    
    luc_eep_data_H = (uint8_t) (((uint16_t) lss_eep_data) / 256);
    luc_eep_data_L = (uint8_t) (((uint16_t) lss_eep_data) % 256);
    //------------------------------------------------------
    R_WDT_Restart();
    while (IICBSY0 == 1) {}        //总线忙，则等待
    
    STT0 = 1;                      //产生start信号
    WREL0 = 0;                     //发送模式
    ACKE0 = 0U;                    //无ACK信号
    IICA0 = luc_driver_write;      //写驱动地址
    delay_10us(20);                //等待
    
    IICA0 = luc_unit_addr;         //单元地址
    delay_10us(20);                //等待
    
    IICA0 = luc_eep_data_H;            
    delay_10us(20);                //等待
    SPT0 = 1;                      //产生stop信号
    //------------------------------------------------------
    delay_1ms(3);
    //------------------------------------------------------
    R_WDT_Restart();
    while (IICBSY0 == 1){}         //总线忙，则等待
    
    STT0 = 1;                      //产生start信号
    WREL0 = 0;                     //发送模式
    ACKE0 = 0U;                    //无ACK信号
    IICA0 = luc_driver_write;      //写驱动地址
    delay_10us(20);                //等待
    
    IICA0 = (uint8_t)(luc_unit_addr + 1);     //单元地址
    delay_10us(20);                //等待
    
    IICA0 = luc_eep_data_L;
    delay_10us(20);                //等待
    SPT0 = 1;                      //产生stop信号
    //------------------------------------------------------
    delay_1ms(3);
}
//------------------------------------------------------------------------------
int16_t eeprom_read(uint16_t lus_addr)  //读eeprom程序
{
    uint8_t luc_page_addr;
    uint8_t luc_unit_addr;
    uint8_t luc_driver_write;
    uint8_t luc_driver_read;
    uint8_t luc_eep_data_H;
    uint8_t luc_eep_data_L;
    
    luc_page_addr = (uint8_t) (lus_addr * 2 / 256);
    luc_unit_addr = (uint8_t) (lus_addr * 2 % 256);
    luc_driver_write = (uint8_t) (( luc_page_addr << 1) & 0x0E | 0xA0);
    luc_driver_read = (uint8_t) (( luc_page_addr << 1) & 0x0E | 0xA1);
    //------------------------------------------------------
    R_WDT_Restart();
    while (IICBSY0 == 1) {}        //总线忙，则等待
    
    STT0 = 1;                      //产生start信号
    WREL0 = 0;                     //发送模式
    ACKE0 = 0U;                    //无ACK信号
    IICA0 = luc_driver_write;      //写驱动地址
    delay_10us(20);                //等待
    
    IICA0 = luc_unit_addr;         //单元地址
    delay_10us(20);                //等待
    
    STT0 = 1;                      //产生start信号
    IICA0 = luc_driver_read;       //读驱动地址    
    delay_10us(20);                //等待
    WREL0 = 1;                     //接收模式
    
    delay_10us(20);                //等待
    luc_eep_data_H = IICA0;        //得到数据
    ACKE0 = 1;                     //产生ACK信号
    SPT0 = 1;                      //产生stop信号
    //------------------------------------------------------
    delay_1ms(3);
    //------------------------------------------------------
    R_WDT_Restart();
    while (IICBSY0 == 1){}         //总线忙，则等待
    
    STT0 = 1;                      //产生start信号
    WREL0 = 0;                     //发送模式
    ACKE0 = 0U;                    //无ACK信号
    IICA0 = luc_driver_write;      //写驱动地址
    delay_10us(20);                //等待
    
    IICA0 = (uint8_t)(luc_unit_addr + 1);     //单元地址
    delay_10us(20);                //等待
    
    STT0 = 1;                      //产生start信号
    IICA0 = luc_driver_read;       //读驱动地址
    delay_10us(20);                //等待
    WREL0 = 1;                     //接收模式
    
    delay_10us(20);                //等待
    luc_eep_data_L = IICA0;        //读数据
    ACKE0 = 1;                     //产生ACK信号
    SPT0 = 1;                      //产生stop信号
    //------------------------------------------------------
    delay_1ms(3);
    //------------------------------------------------------
    return ((int16_t) (luc_eep_data_H) * 256 + luc_eep_data_L);
}







/*****************************************END OF THE FILE*********************************************************************************************************************/
