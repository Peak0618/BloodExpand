#ifndef _M_MAIN_H_
#define _M_MAIN_H_

//外部函数
extern void system_init(void);   
extern void system_start(void);
extern void main_loop(void);    
extern void timer_op(void); 
extern void timer_int(void);    


//外部变量声明//
extern flag_type flg_time;
          
          #define   bflg_1ms_reach      flg_time.bits.bit0  //1ms计时到标志
          #define   bflg_5ms_reach      flg_time.bits.bit1  //5ms计时到标准
          #define   bflg_10ms_reach     flg_time.bits.bit2  //10ms计时到标准



#endif 
/**************************************END OF THE FILE******************************************************************/
