#ifndef _M_PERIPHERAL_CONTROL_H_
#define _M_PERIPHERAL_CONTROL_H_


//�ⲿ����
extern void sterilize_deal(void);
extern void sterilize_monitor(void);
extern void lock_deal(void);

//flag
extern flag_type flg_peripheral;
        #define bflg_sterilization_state        flg_peripheral.bits.bit0   //��ʱ�Ƿ���ɱ��

//variate
extern uint16_t gus_max_sterilization_time;     //���ɱ��ʱ��
extern uint8_t  guc_sterilization_type;         //ɱ��ģʽ�յ�/�Զ�
extern uint8_t  guc_lock_state;                 //����״̬
extern uint8_t  guc_door_state;                 //�ŵ�״̬


//macro
#define     STERILIZE_POWER  P5.1              //ɱ��ģ��Ĺ���� 1�� 0�ر�
#define     LOCK_STATE       P1.7              //������ʱ�ã��鿴ͨѶ�Ƿ�����















#endif
/***************************************END OF THE FILE**************************************************/
