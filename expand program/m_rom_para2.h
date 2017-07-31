#ifndef _ROM_PARA2_H_
#define _ROM_PARA2_H_
//------------------------------------------------------------------------------
//参数个数定义
#define   MAX_PARA_NUMBER2     150      //参数总数
//------------------------------------------------------------------------------
//参数编号定义
#define   num_key_locked_delaytime                0
#define   num_key_long_delaytime                  1

#define   num_comp2_TS_min                        2
#define   num_comp2_TS_max                        3
#define   num_comp2_ALH_min                       4
#define   num_comp2_ALH_max                       5
#define   num_comp2_ALL_min                       6
#define   num_comp2_ALL_max                       7
//----------------------------------------------------------
#define   num_comp2_run_Tdelt                     8   // 2 即 0.2
#define   num_comp2_stop_Tdelt                    9   //-4 即 -0.4
#define   num_comp2_mode_T                        10  //10 即 1

#define   num_comp2_run_T_delaytime               11
#define   num_comp2_stop_T_delaytime              12
//----------------------------------------------------------
#define   num_comp2_run_delaytime                 13
#define   num_comp2_stop_delaytime                14

#define   num_comp2_TA_fault_run_delaytime        15
#define   num_comp2_TA_fault_stop_delaytime       16

#define   num_comp2_first_run_space_delaytime     17
#define   num_comp2_run_space_delaytime           18

#define   num_comp2_continue_first_run_delaytime  19
#define   num_comp2_continue_run_delaytime        20
#define   num_comp2_force_stop_delaytime          21

#define   num_comp21_run_T                        22
//----------------------------------------------------------
#define   num_outfan2_on_T                        23
#define   num_outfan2_off_T                       24
#define   num_outfan2_off_delaytime               25
//----------------------------------------------------------
#define   num_heat20_space_delaytime              26
#define   num_heat20_T                            27
#define   num_heat20_wait_delaytime               28
#define   num_heat20_comp2_stop_delaytime         29
#define   num_heat20_run_delaytime                30
#define   num_heat20_force_run_delaytime          31
#define   num_heat20_stop_delaytime               32

#define   num_heat21_T                            33
#define   num_heat21_run_delaytime                34
#define   num_heat21_stop_delaytime               35
//----------------------------------------------------------
#define   num_prot2_TA_high_Tdelt                 36
#define   num_prot2_TA_high_delaytime             37
#define   num_prot2_TA_low_Tdelt                  38
#define   num_prot2_TA_low_delaytime              39

#define   num_prot2_THW_Tdelt                     40
#define   num_prot2_THW_high_delaytime            41

#define   num_power_off_12V                       42
#define   num_power_on_12V                        43
#define   num_power_fautl_VAC1                    44
#define   num_power_fautl_VAC2                    45
#define   num_port2_bat_discnnt_V1                46
#define   num_port2_bat_discnnt_V2                47
#define   num_prot2_bat_discnnt_I                 48

#define   num_bat_low_V1                          49
#define   num_bat_low_V2                          50
#define   num_bat_low_V3                          51
#define   num_bat_low_V4                          52
#define   num_prot2_bat_low_delaytime             53

#define   num_prot2_TC_first_delaytime            54
#define   num_prot2_cond_dirty_TC1                55
#define   num_prot2_cond_dirty_TC2                56
#define   num_prot2_cond_dirty_delaytime          57

#define   num_prot2_volt_V1                       58
#define   num_prot2_volt_V2                       59
#define   num_prot2_volt_V3                       60
#define   num_prot2_volt_V4                       61
#define   num_prot2_volt_V5                       62
#define   num_prot2_volt_V6                       63
#define   num_prot2_volt_V7                       64
#define   num_prot2_volt_V8                       65

#define   num_comp2_volt_low_220V1                66
#define   num_comp2_volt_low_220V2                67
#define   num_comp2_volt_low_115V1                68
#define   num_comp2_volt_low_115V2                69
#define   num_comp2_volt_low_delaytime            70

#define   num_comp2_volt_delaytime                71
#define   num_comp21_volt_delaytime               72

#define   num_volt_down_220V1                     73
#define   num_volt_down_220V2                     74
#define   num_volt_up_220V1                       75
#define   num_volt_up_220V2                       76
#define   num_volt_down_115V1                     77
#define   num_volt_down_115V2                     78
#define   num_volt_down_115V3                     79
#define   num_volt_down_115V4                     80
#define   num_volt_up_115V1                       81
#define   num_volt_up_115V2                       82
#define   num_volt_up_down_delaytime              83

#define   num_bat1_charge_V1                      84
#define   num_bat1_charge_V2                      85
#define   num_bat1_charge_V3                      86
#define   num_bat1_charge_I1                      87
#define   num_bat1_charge_I2                      88
#define   num_bat1_charge_I3                      89
#define   num_bat1_charge_I4                      90
#define   num_bat1_charge_V4                      91
#define   num_bat1_charge_V5                      92
#define   num_bat1_charge_continue_delaytime      93
#define   num_bat1_charge_sapce_delaytime         94

#define   num_bat2_charge_V1                      95
#define   num_bat2_charge_V2                      96
#define   num_bat2_charge_V3                      97
#define   num_bat2_charge_I1                      98
#define   num_bat2_charge_I2                      99
#define   num_bat2_charge_I3                      100
#define   num_bat2_charge_I4                      101
#define   num_bat2_charge_V4                      102
#define   num_bat2_charge_V5                      103
#define   num_bat2_charge_continue_delaytime      104
#define   num_bat2_charge_sapce_delaytime         105

#define   num_bat1_discharge_off_V                106
#define   num_bat2_discharge_off_V                107   //到此为止，后为新加

#define   num_bat2_discharge_off_T                108
#define   num_bat2_discharge_on_T                 109

#define   num_heat22_T                            110   //门体加热丝 peak add
#define   num_heat22_run_delaytime                111
#define   num_heat22_stop_delaytime               112

#define   num_outfan2_T                           113   //风机控制 peak add 温度
#define   num_outfan2_delaytime                   114                    //时间
#define   num_outfan2_rad                         115                    //转速

//peak add 包工新的功能书添加参数
#define  num_door_alarm_delaytime_min             116
#define  num_door_alarm_delaytime_max             117

#define  num_usb_read_delaytime_min               118
#define  num_usb_read_delaytime_max               119

#define  num_IC_password_min                      120
#define  num_IC_password_max                      121

#define  num_pt100_calibration_min                122
#define  num_pt100_calibration_max                123

#define  num_voltage_calibration_min              124
#define  num_voltage_calibration_max              125

#define  num_THW_calibration_min                  126
#define  num_THW_calibration_max                  127

#define  num_buzz_alarm_delaytime_min             128
#define  num_buzz_alarm_delaytime_max             129

#define  num_THW_hight_alarm_min                  130
#define  num_THW_hight_alarm_max                  131

#define  num_comp_startup_delaytime_min           132
#define  num_comp_startup_delaytime_max           133

#define  num_unlock_password_min                  134
#define  num_unlock_password_max                  135

#define  num_TE_fault_shield_min                  136
#define  num_TE_fault_shield_max                  137

#define  num_password_val_min                     138
#define  num_password_val_max                     139

#define  num_usb_P6_min                           140
#define  num_usb_P6_max                           141


//------------------------------------------------------------------------------
extern int16_t ram_para2[MAX_PARA_NUMBER2 + 1];  //参数值数组

extern const int16_t EEPROM_init2[MAX_PARA_NUMBER2 + 1][5];     //参数初始化值数组
//------------------------------------------------------------------------------
//参数个数定义
#define   MAX_PARA_NUMBER      40       //参数总数
//------------------------------------------------------------------------------
//参数编号定义
#define   num_password_val                        0
#define   num_comp2_TS                            1
#define   num_comp2_ALH                           2
#define   num_comp2_ALL                           3
#define   num_comp2_mode                          4

#define   num_comp2_power_on_delaytime            5
#define   num_alarm_buzz_off_delaytime            6
#define   num_alarm_buzz_level                    7
#define   num_comp2_PT100_ref                     8
#define   num_comp2_PT100_ref_backup              9
#define   num_comp2_backup                        10
#define   num_comp2_backup_TS                     11
//----------------------------------------------------------
#define   num_prot2_TA_first_delaytime            12
#define   num_prot2_door_delaytime                13

#define   num_comp2_VAC_ref                       14
#define   num_comp2_THW_ref                       15

#define   num_prot2_THW_T                         16
#define   num_comp2_prot_T                        17

#define   num_comp2_PT100_shield                  18
#define   num_comp2_THW_shield                    19
#define   num_comp2_TC_shield                     20
#define   num_comp2_TE_shield                     21
#define   num_comp2_PT100_backup_shield           22

//------------------------------------------------------------
//按照包工新加的功能书增加的参数
#define   num_door_alarm_delaytime                23  //开门报警延时
#define   num_usb_read_delaytime                  24  //usb取数周期
#define   num_IC_password                         25  //IC卡注册密码
#define   num_pt100_calibration                   26  //pt100的温度校准
#define   num_voltage_calibration                 27  //电压校准
#define   num_THW_calibration                     28  //环温校准
#define   num_buzz_alarm_delaytime                29  //蜂鸣报警延时
#define   num_THW_hight_alarm                     30  //环温过高报警
#define   num_comp_startup_delaytime              31  //高温压机开机延时
#define   num_TE_fault_shield                     32  //热交换器故障屏蔽
#define   num_usb_P6                              33  //usb导出数据的方式
#define   num_secret_key                          34  //密钥使能
#define   num_usb_language                        35  //usb板导出数据的语言类型
#define   num_tank_mouth_heat                     36  //柜口加热开关
#define   num_debug_P20                           37  //调试参数的P20
#define   num_debug_P21                           38  //调试参数的P21


//------------------------------------------------------------------------------
extern int16_t ram_para[MAX_PARA_NUMBER + 1];   //参数值数组

extern const int16_t EEPROM_init[MAX_PARA_NUMBER + 1][5];       //参数初始化值数组
//------------------------------------------------------------------------------
#endif
