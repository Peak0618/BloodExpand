#ifndef _M_TEST_H_
#define _M_TEST_H_

//Íâ²¿º¯Êý
extern void test_mode_com();
extern void test_com_rx_int(uint8_t rx_data);
extern void test_com_tx_int();
extern void test_in_out_pin(void);
extern void test_error_code_deal(void);



extern flag_type flg_test;
       #define   bflg_test_mode     flg_test.bits.bit0
       #define   bflg_test_pwm      flg_test.bits.bit1



















#endif
/**************************************END OF THE FILE******************************************************************/
