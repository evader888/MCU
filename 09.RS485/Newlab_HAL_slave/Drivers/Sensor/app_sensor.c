# include "app_sensor.h"
#include "flash.h"

SensorType sen_type_t = TemHum_Sensor;

static u8 change_sensor_state (SensorType* type);



//  1. ������������
//	BodyInfrared_Sensor=1,   //������⴫����
//	Sound_Sensor=2,					 //����������
//	Infrared_Sensor=3,			   //���⴫����
//  2. ģ����
//	Photosensitive_Sensor=4, //����������
//	Flame_Sensor=5,					 //���洫����
//	FlammableGas_Sensor=6,	 //��ȼ���崫����
//	AirQuality_Sensor=7,		 //��������������
//  3. ������
//	TemHum_Sensor=8,         //��ʪ�ȴ�����
void get_sensor(SensorType    type)
{
    uint8_t read_io=0;
    uint16_t vol=0;
    uint16_t sensor_hum = 0; //ʪ��
    uint16_t sensor_tem = 0; //�¶�
    switch(type) {
    //1. ������������
    case BodyInfrared_Sensor:
    read_io =Switching_Value();
        mb_funcdisc_states.val.disc_BIT0=(unsigned )read_io;
        DBG_B_INFO("��ǰ������⴫���� ����ֵΪ %d     ",read_io);

        break;
    case Sound_Sensor:
			read_io =Switching_Value();
        DBG_B_INFO("��ǰ���������� ����ֵΪ %d",read_io);
        mb_funcdisc_states.val.disc_BIT1=(unsigned )read_io;
        break;

    case Infrared_Sensor:
        read_io =Switching_Value();
        mb_funcdisc_states.val.disc_BIT2=(unsigned )read_io;
        DBG_B_INFO("��ǰ���⴫���� ����ֵΪ %d   ��ɢ��ֵ 0x%x",read_io,mb_funcdisc_states.VAL);

        break;

    //2. ģ����
    case Photosensitive_Sensor:
        vol =Get_Voltage();
        DBG_B_INFO("��ǰ����������������adcΪ %d",vol);
        inbuf[0]=vol;
        break;
    case AirQuality_Sensor:
        vol =Get_Voltage();
        inbuf[1]=vol;
        DBG_B_INFO("��ǰ��������������adcΪ %d",vol);
        break;
    case Flame_Sensor:
        vol =Get_Voltage();
        DBG_B_INFO("��ǰ���洫����������adcΪ %d",vol);
        inbuf[2]=vol;
        break;
    case FlammableGas_Sensor:
        vol =Get_Voltage();
        DBG_B_INFO("��ǰ��ȼ���崫����adcΪ %d",vol);
        inbuf[3]=vol;
        break;

    //3.��ʪ��
    case TemHum_Sensor:
        call_sht11(&sensor_tem, &sensor_hum);
        DBG_B_INFO("��ǰ�¶�ֵΪ %d , ��ǰʪ��ֵΪ %d ",sensor_tem,sensor_hum);
        holdbuf[0]=(u16)(sensor_tem<<8)+sensor_hum;
        break;

    default:
        break;

    }
}


void sensor_init(void)
{
    u8  read_state=0;
//    Adc_Init();
    hal_temHumInit();
    InfraredSensor_Init();

    //������ȡ�ϴδ���������״̬
    bsp_flash_read(FLASH_SAVE_SENSOR_STATE, &read_state,1);
    sen_type_t =(SensorType)read_state;
    DBG_B_INFO("��ǰ������״̬Ϊ %d ",sen_type_t);
    holdbuf[2] =(u16)sen_type_t;

}


/*
  * @brief   getsensor_task
  * @param   none
  * @note    ÿ5S�ɼ�һ�δ���������
  * @Date:   2019.8.7
  * @updatge:2019.8.7
  * @author: zhao
  * @return: void
*/
void getsensor_task(void)
{
    static uint32_t getsensor_value_time;
    static uint32_t getsensor_set_time;
    u8 res;
    if((uint32_t)(HAL_GetTick()-getsensor_value_time>=500)) {
        getsensor_value_time=HAL_GetTick();
//      DBG_B_INFO("test getsensor_value_time");
        get_sensor(sen_type_t);
    }


    if((uint32_t)(HAL_GetTick()-getsensor_set_time>=100)) {
        getsensor_set_time=HAL_GetTick();
//      DBG_B_INFO("test change_sensor_state");
        res=change_sensor_state(&sen_type_t);
    }
}


/*
  * @brief   change_sensor_state
  * @param   SensorType* type
  * @note    ÿ5S�ɼ�һ�δ���������
  * @Date:   2019.8.7
  * @updatge:2019.8.7
  * @author: zhao
  * @return: 0  �ޱ��
             1  ��д״̬�����ɹ�
             2  ��д״̬����ʧ��
*/
static u8 change_sensor_state (SensorType* type)
{
    SensorType old_state;
    SensorType read_state;
    old_state = *type;
    if(old_state != (SensorType)(holdbuf[2])) {
        DBG_B_INFO("������ģʽ�л�");
        *type=(SensorType)(holdbuf[2]);
        bsp_flash_erase(FLASH_SAVE_SENSOR_STATE, 1);
        bsp_flash_write(FLASH_SAVE_SENSOR_STATE,(u8*)type,sizeof(SensorType));
        bsp_flash_read(FLASH_SAVE_SENSOR_STATE,(u8*)&read_state,sizeof(SensorType));

        if(read_state==*type) {
            DBG_B_INFO("�����ɹ�");
            return 1;
        } else {
            DBG_B_INFO("����ʧ��");
            return 2;
        }
    } else {
        return 0;
    }

}



