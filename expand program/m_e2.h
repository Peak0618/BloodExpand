#ifndef _M_E2_H_
#define _M_E2_H_

//外部函数
extern void init_ram_para2(void);
extern void init_ram_para(void);
extern void eeprom2_read_deal(void)  ;
extern void eeprom_read_deal(void);   
extern void eeprom_write(uint16_t lus_addr, int16_t lss_eep_data);    
extern int16_t eeprom_read(uint16_t lus_addr);  


extern flag_type flg_e2;
          #define   bflg_askfor_read_eeprom  flg_e2.bits.bit0    //请求读eeprom标志
          #define   bflg_read_eeprom_error   flg_e2.bits.bit1    //读eeprom错标志
          #define   bflg_eeprom_fault        flg_e2.bits.bit2    //eeprom故障标志
          
          












#endif
/***************************************END OF THE FILE******************************************************************************************/
