#ifndef _AD_C_
#define _AD_C_
//------------------------------------------------------------------------------
//�����ļ�����
#include "h_type_define.h"
#include "m_ad.h"
#include "m_ad_tab.h"

//------------------------------------------------------------------------------
//��������
void ad_val_deal(void);  //adֵ���������ADת�������е���

void ad_temp_calc(void); //ad�¶ȼ����������ѭ�������е���

int16_t get_temp3(uint16_t lus_ad);     //����adֵ���õ��¶�ֵ3

int16_t get_temp4(uint16_t lus_ad);     //����adֵ���õ��¶�ֵ4

int16_t get_temp5(uint16_t lus_ad);     //����adֵ���õ��¶�ֵ5

uint8_t get_temp6(uint16_t lus_ad);      //���ݵ�ѹֵ���õ�ʪ��ֵ6

//------------------------------------------------------------------------------
//��������
flag_type flg_adc;
//----------------------------------------------------------
#define   AD_SUM_INDEX   6    //��ƽ����λֵ
#define   AD_SUM_CNT     64   //��ƽ������

#define   AD_VAL_MIN     5    //AD����ֵ
#define   AD_VAL_MAX     1000 //AD����ֵ

//----------------------------------------------------------
uint16_t  gus_ad_val;             //�� ADCRת������Ĵ����л�ȡ��ֵ
uint16_t  gus_ad_buffer[12];      //ֻ����˲ʱ��ѹʱ�ù�
uint32_t  gul_ad_sum_buffer[12];  //12��Ԫ�أ�û��Ԫ��(ͨ��)����64���ۼ�
uint16_t  gus_ad_aver_buffer[12];
uint8_t   guc_ad_index;
uint8_t   guc_ad_cnt;
//----------------------------------------------------------
uint8_t   guc_ad_fault_cnt[12];         //ad���ϴ���������
uint8_t   guc_ad_fault_release_cnt[12]; //ad�����ͷŴ���������

//----------------------------------------------------------
int16_t   gss_TA;   //��������           �Ŵ�10������������
int16_t   gss_THW;  //���´�����          �Ŵ�10
int16_t   gss_TC;   //�������¶ȴ�����
int16_t   gss_TE;   //�Ƚ������¶ȴ�����

int16_t   gss_VAC;  //��Դ��ѹ
uint8_t   guc_12V;  //12V��ѹ

int16_t   gss_Tbat1;     //﮵���¶�1
int16_t   gss_Tbat2;     //﮵���¶�2

uint8_t   guc_bat_Vin;   //��س���ѹ
uint8_t   guc_bat_Vout;  //��طŵ��ѹ___�Ŵ�10��
uint8_t   guc_bat_Vout_instant;  //��طŵ��ѹ˲ʱֵ

uint8_t   guc_humidity;          //ʪ��ֵ(����%)
uint8_t   guc_sterilize_monitor; //ɱ����ⷴ����ѹ--�Ŵ�10��
uint8_t   guc_reserved_ad;       //Ԥ����cn13�˿ڵ�ad�ɼ� --�Ŵ�10��

uint32_t  gul_bat_Vin;   //peak���������ʱʹ��
uint32_t  gul_bat_Vout;  //peak���������ʱʹ��
int16_t   gss_bat_I;     //��س�����

int16_t   gss_adg_Temp[10];
int16_t   gss_adg_Temp2;

uint8_t   guc_ad_calc_cnt;

int16_t   gss_THW_calibration;         //����У׼ֵ
int16_t   gss_humidity_calibration;    //ʪ��У׼ֵ  ----����ע��֮��Ҫ������ת��

/************************************************************************************************************************
��������:  1����ȡÿһ·AD�ɼ���ֵ��ͨ����βɼ�ȡƽ��;
         2���ж��Ƿ��й���

����λ��: 1ms��ʱ��ad_convert_deal()-----------------------------ok 
*************************************************************************************************************************/
void ad_val_deal(void)   //adֵ���������ADת�������е���  peak 1ms�� 
{
    //------------------------------------------------------
    //adֵ�ۼ�
    gus_ad_buffer[guc_ad_index] = gus_ad_val;
    gul_ad_sum_buffer[guc_ad_index] += gus_ad_val;
    //------------------------------------------------------
    
    //adֵ��ƽ��
    if (guc_ad_cnt == 0)
    {
        gus_ad_aver_buffer[guc_ad_index] = (uint16_t) (gul_ad_sum_buffer[guc_ad_index] >> AD_SUM_INDEX);
        gul_ad_sum_buffer[guc_ad_index] = 0;
    }
    
    //------------------------------------------------------
    //ad�����ж�����Ͻ��
    if ((gus_ad_val < AD_VAL_MIN) || (gus_ad_val > AD_VAL_MAX))
    {
        guc_ad_fault_release_cnt[guc_ad_index] = 0;
        //--------------------------------------------------
        guc_ad_fault_cnt[guc_ad_index]++;
        if (guc_ad_fault_cnt[guc_ad_index] >= 100)
        {
            guc_ad_fault_cnt[guc_ad_index] = 0;
            
            switch (guc_ad_index)
            {
                //+�������
                case 0:        //Ԥ��

                    break;
                case 1:        //�����ص�ѹ

                    break;
                case 2:        //����
                    bflg_THW_fault = 1;
                    break;
                case 3:        //ʪ��
                    bflg_humidity_fault = 1;
                    break;
                case 4:        //���ɱ��ģ��
                    
                    break;
                default:
                    break;
            }
        }
    }
    else
    {
        guc_ad_fault_cnt[guc_ad_index] = 0;
        //--------------------------------------------------
        guc_ad_fault_release_cnt[guc_ad_index]++;
        if (guc_ad_fault_release_cnt[guc_ad_index] >= 100)
        {
            guc_ad_fault_release_cnt[guc_ad_index] = 0;
            
            switch (guc_ad_index)
            {
                //+�������
                case 0:

                    break;
                case 1:

                    break;
                case 2:
                    bflg_THW_fault = 0;
                    break;
                case 3:
                    bflg_humidity_fault = 0;
                    break;
                case 4:        //���ɱ��ģ��
                    
                    break;                    
                default:
                    break;
            }
        }
    }
    //------------------------------------------------------    
    //�����ۼ�
    guc_ad_index++;
    if (guc_ad_index >= 5)              //5��ͨ�����ɼ���һ�� ad2��ad3��ad16��ad17��ad19
    {
        guc_ad_index = 0;
        //------------------------------
        if (guc_ad_cnt == 0)
        {
            bflg_allow_ad_calc = 1;     //������ad�����־, ����1ʱ ad_temp_calc() ����ִ�У�����0��
        }
        //------------------------------
        guc_ad_cnt++;
        if (guc_ad_cnt >= AD_SUM_CNT)   //12·AD���ɼ���64�κ��������㣻
        {
            guc_ad_cnt = 0;
        }
    }
}

/************************************************************************************************************************
��������: ͨ���ɼ���ADֵ��ת��Ϊʵ�ʵ��¶ȡ���ѹ��

����λ��: ��ѭ����     --------------------------------ok
*************************************************************************************************************************/
void ad_temp_calc(void)  //ad�¶ȼ����������ѭ�������е���
{
    int32_t lsl_tmp=0;
    
    gss_THW = get_temp5(gus_ad_aver_buffer[2]);  //����

    
    //��طŵ��ѹ���㣨��λ��0.1V��
    lsl_tmp = gus_ad_aver_buffer[1];
    //lsl_tmp *= 224;
    lsl_tmp *= 224;                   //�Ŵ�10��
    gul_bat_Vout = lsl_tmp;           //gul_bat_Voutֻ�ڼ������ʱ��,û��>>10�������ǷŴ���1024��
    lsl_tmp >>= 10;
    guc_bat_Vout = (uint8_t) ((guc_bat_Vout * 3 + lsl_tmp) >> 2);

    //ʪ�ȼ��� (%)
    lsl_tmp = gus_ad_aver_buffer[3];
    //lsl_tmp = (5*lsl_tmp)/1023*1000; //�Ŵ�1000����ĵ�ѹ
    lsl_tmp = (5000*lsl_tmp)>>10;
    guc_humidity = get_temp6((uint16_t)lsl_tmp);

    //ɱ��ģ�鷴��
    guc_sterilize_monitor =  (uint8_t)((10*5*gus_ad_aver_buffer[4])>>10); 
    
    guc_reserved_ad = (uint8_t)((10*5*gus_ad_aver_buffer[0]) >> 10);     //Ԥ��ad�ڲɼ��ĵ�ѹ
    
    guc_ad_calc_cnt++;              //�жϲɼ��Ƿ����ʱ���տ�ʼΪ���ȶ���ɼ��������ж�
    if (guc_ad_calc_cnt >= 5)
    {
    	  bflg_ad_calc_ok = 1;
    }
}


/*******************************************************************************************************
��������: 

����λ��: ad_temp_calc() �У�

��ȡ����: ��ȡPT100���¶�ֵ 
********************************************************************************************************/
int16_t get_temp3(uint16_t lus_ad)      //����adֵ���õ��¶�ֵ3
{
    int16_t lss_delt;
    
    int16_t lss_temp_index;
    lss_temp_index = (TEMP3_TAB_LENGTH >> 1);
    
    if (lus_ad > ad_to_temp3[lss_temp_index])
    {
        do
        {
            lss_temp_index--;
            if (lss_temp_index < 0)
            {
                lss_temp_index = 0;
                break;
            }
            //------------------------------------
            if (lus_ad < ad_to_temp3[lss_temp_index])
            {
                //lss_temp_index++;
                break;
            }
        }
        while (lus_ad > ad_to_temp3[lss_temp_index]);
    }
    else if (lus_ad < ad_to_temp3[lss_temp_index])
    {
        do
        {
            lss_temp_index++;
            if (lss_temp_index >= TEMP3_TAB_LENGTH)
            {
                lss_temp_index = TEMP3_TAB_LENGTH;
                break;
            }
            //------------------------------------
            if (lus_ad > ad_to_temp3[lss_temp_index])
            {
                lss_temp_index--;
                break;
            }
        }
        while (lus_ad < ad_to_temp3[lss_temp_index]);
    }
    //����¶ȸ�1�棬�ڴμ�һ��ָ�룬�¶Ƚ�1��
    /*lss_temp_index--;
    if (lss_temp_index < 0)
    {
        lss_temp_index = 0;
    }*/
    
    lss_delt = lus_ad;
    lss_delt -= ad_to_temp3[lss_temp_index];
    lss_delt *= 10;    //��Ϊpt100������Ŵ���10�������Դ˴���С�����ֵ�ֵҲҪ�Ŵ�10��
    lss_delt >>= 2;    //�μ�"ѪҺ"
    
    //return temp3_for_ad[lss_temp_index];
    return (temp3_for_ad[lss_temp_index] * 10 - lss_delt);
}
/*******************************************************************************************************
��������: 

����λ��: ad_temp_calc() �У�

��ȡ����: ��ȡ��ص��¶�ֵ 
********************************************************************************************************/
int16_t get_temp4(uint16_t lus_ad)      //����adֵ���õ��¶�ֵ4
{
    int16_t lss_temp_index;
    lss_temp_index = (TEMP4_TAB_LENGTH >> 1);
    
    if (lus_ad > ad_to_temp4[lss_temp_index])
    {
        do
        {
            lss_temp_index--;
            if (lss_temp_index < 0)
            {
                lss_temp_index = 0;
                break;
            }
            //------------------------------------
            if (lus_ad < ad_to_temp4[lss_temp_index])
            {
                //lss_temp_index++;
                break;
            }
        }
        while (lus_ad > ad_to_temp4[lss_temp_index]);
    }
    else if (lus_ad < ad_to_temp4[lss_temp_index])
    {
        do
        {
            lss_temp_index++;
            if (lss_temp_index >= TEMP4_TAB_LENGTH)
            {
                lss_temp_index = TEMP4_TAB_LENGTH;
                break;
            }
            //------------------------------------
            if (lus_ad > ad_to_temp4[lss_temp_index])
            {
                lss_temp_index--;
                break;
            }
        }
        while (lus_ad < ad_to_temp4[lss_temp_index]);
    }
    //����¶ȸ�1�棬�ڴμ�һ��ָ�룬�¶Ƚ�1��
    /*lss_temp_index--;
    if (lss_temp_index < 0)
    {
        lss_temp_index = 0;
    }*/
    
    return temp4_for_ad[lss_temp_index];
}
//------------------------------------------------------------------------------
int16_t get_temp5(uint16_t lus_ad)      //����adֵ���õ��¶�ֵ5
{
    int16_t lss_delt;
     
    int16_t lss_temp_index;
    lss_temp_index = (TEMP5_TAB_LENGTH >> 1);  //peak 2�ַ���ȡ�м�ֵ���������ݵĲ����ٶ�
    
    if (lus_ad > ad_to_temp5[lss_temp_index])  //ADֵԽ���¶�ԽС
    {
        do
        {
            lss_temp_index--;
            if (lss_temp_index < 0)
            {
                lss_temp_index = 0;
                break;
            }
            //------------------------------------
            if (lus_ad < ad_to_temp5[lss_temp_index])
            {
                //lss_temp_index++;  ԭע
                break;
            }
        }
        while (lus_ad > ad_to_temp5[lss_temp_index]);
    }
    else if (lus_ad < ad_to_temp5[lss_temp_index])
    {
        do
        {
            lss_temp_index++;
            if (lss_temp_index >= TEMP5_TAB_LENGTH)
            {
                lss_temp_index = TEMP5_TAB_LENGTH;
                break;
            }
            //------------------------------------
            if (lus_ad > ad_to_temp5[lss_temp_index])
            {
                lss_temp_index--;  //ԭδע                 //peak Ϊ�˿��Ժ�������һ���ֹ�ͬʹ������Ĺ�ʽ
                break;
            }
        }
        while (lus_ad < ad_to_temp5[lss_temp_index]);
    }
    //����¶ȸ�1�棬�ڴμ�һ��ָ�룬�¶Ƚ�1��
    /*lss_temp_index--;
    if (lss_temp_index < 0)
    {
        lss_temp_index = 0;
    }*/
    
    //lss_delt = lus_ad;
    //lss_delt -= ad_to_temp5[lss_temp_index];

    lss_delt = ad_to_temp5[lss_temp_index] - lus_ad;
    lss_delt *= 10;                                  
    //lss_delt >>= 2;
    lss_delt /= (ad_to_temp5[lss_temp_index] - ad_to_temp5[lss_temp_index + 1]);   //index ԽС��ADֵԽ���¶�ԽС
    
    return (temp5_for_ad[lss_temp_index] * 10 + lss_delt);                         //������ԱȲɼ���ADֵ����Ǹ�Ԫ�أ���ΪADֵС���¶���������+
}

//------------------------------------------------------------------------------
uint8_t get_temp6(uint16_t lus_ad)      //���ݵ�ѹֵ��ȡʪ��ֵ
{
    int16_t lss_delt;
     
    int16_t lss_temp_index;
    lss_temp_index = (TEMP6_TAB_LENGTH >> 1);  
    
    if (lus_ad > ad_to_temp6[lss_temp_index])  //ADֵԽ���¶�ԽС
    {
        do
        {
            lss_temp_index--;
            if (lss_temp_index < 0)
            {
                lss_temp_index = 0;
                break;
            }
            //------------------------------------
            if (lus_ad < ad_to_temp6[lss_temp_index])
            {
                //lss_temp_index++;  ԭע
                break;
            }
        }
        while (lus_ad > ad_to_temp6[lss_temp_index]);
    }
    else if (lus_ad < ad_to_temp6[lss_temp_index])
    {
        do
        {
            lss_temp_index++;
            if (lss_temp_index >= TEMP6_TAB_LENGTH)
            {
                lss_temp_index = TEMP6_TAB_LENGTH;
                break;
            }
            //------------------------------------
            if (lus_ad > ad_to_temp6[lss_temp_index])
            {
                lss_temp_index--;  //ԭδע                 //peak Ϊ�˿��Ժ�������һ���ֹ�ͬʹ������Ĺ�ʽ
                break;
            }
        }
        while (lus_ad < ad_to_temp6[lss_temp_index]);
    }

    lss_delt = ad_to_temp6[lss_temp_index] - lus_ad;
    lss_delt = lss_delt*5;                          //��Ϊ��Ӧ�ֲ�����ʪ�ȵ���С���Ϊ5
    lss_delt /= (ad_to_temp6[lss_temp_index] - ad_to_temp6[lss_temp_index + 1]);   //index ԽС��ADֵԽ���¶�ԽС
    
    return (temp6_for_ad[lss_temp_index] - lss_delt);                         //������ԱȲɼ���ADֵ����Ǹ�Ԫ�أ���ΪADֵԽС��ʪ��ԽС������-
}                                                                             //���¶Ȳ�ͬ


#endif
