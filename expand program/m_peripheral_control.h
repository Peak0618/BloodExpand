#ifndef _M_PERIPHERAL_CONTROL_H_
#define _M_PERIPHERAL_CONTROL_H_


//外部函数
extern void sterilize_deal(void);
extern void sterilize_monitor(void);
extern void lock_deal(void);

//flag
extern flag_type flg_peripheral;
        #define bflg_sterilization_state        flg_peripheral.bits.bit0   //此时是否在杀菌

//variate
extern uint16_t gus_max_sterilization_time;     //最大杀菌时间
extern uint8_t  guc_sterilization_type;         //杀菌模式收的/自动
extern uint8_t  guc_lock_state;                 //锁的状态
extern uint8_t  guc_door_state;                 //门的状态


//macro
#define     STERILIZE_POWER  P5.1              //杀菌模块的供电口 1打开 0关闭
#define     LOCK_STATE       P1.7              //仅测试时用，查看通讯是否正常















#endif
/***************************************END OF THE FILE**************************************************/
