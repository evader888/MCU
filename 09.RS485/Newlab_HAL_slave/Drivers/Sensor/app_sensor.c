# include "app_sensor.h"
#include "flash.h"

SensorType sen_type_t = TemHum_Sensor;

static u8 change_sensor_state (SensorType* type);



//  1. 开关量传感器
//	BodyInfrared_Sensor=1,   //人体红外传感器
//	Sound_Sensor=2,					 //声音传感器
//	Infrared_Sensor=3,			   //红外传感器
//  2. 模拟量
//	Photosensitive_Sensor=4, //光敏传感器
//	Flame_Sensor=5,					 //火焰传感器
//	FlammableGas_Sensor=6,	 //可燃气体传感器
//	AirQuality_Sensor=7,		 //空气质量传感器
//  3. 数字量
//	TemHum_Sensor=8,         //温湿度传感器
void get_sensor(SensorType    type)
{
    uint8_t read_io=0;
    uint16_t vol=0;
    uint16_t sensor_hum = 0; //湿度
    uint16_t sensor_tem = 0; //温度
    switch(type) {
    //1. 开关量传感器
    case BodyInfrared_Sensor:
    read_io =Switching_Value();
        mb_funcdisc_states.val.disc_BIT0=(unsigned )read_io;
        DBG_B_INFO("当前人体红外传感器 开关值为 %d     ",read_io);

        break;
    case Sound_Sensor:
			read_io =Switching_Value();
        DBG_B_INFO("当前声音传感器 开关值为 %d",read_io);
        mb_funcdisc_states.val.disc_BIT1=(unsigned )read_io;
        break;

    case Infrared_Sensor:
        read_io =Switching_Value();
        mb_funcdisc_states.val.disc_BIT2=(unsigned )read_io;
        DBG_B_INFO("当前红外传感器 开关值为 %d   离散量值 0x%x",read_io,mb_funcdisc_states.VAL);

        break;

    //2. 模拟量
    case Photosensitive_Sensor:
        vol =Get_Voltage();
        DBG_B_INFO("当前光敏传感器传感器adc为 %d",vol);
        inbuf[0]=vol;
        break;
    case AirQuality_Sensor:
        vol =Get_Voltage();
        inbuf[1]=vol;
        DBG_B_INFO("当前空气质量传感器adc为 %d",vol);
        break;
    case Flame_Sensor:
        vol =Get_Voltage();
        DBG_B_INFO("当前火焰传感器传感器adc为 %d",vol);
        inbuf[2]=vol;
        break;
    case FlammableGas_Sensor:
        vol =Get_Voltage();
        DBG_B_INFO("当前可燃气体传感器adc为 %d",vol);
        inbuf[3]=vol;
        break;

    //3.温湿度
    case TemHum_Sensor:
        call_sht11(&sensor_tem, &sensor_hum);
        DBG_B_INFO("当前温度值为 %d , 当前湿度值为 %d ",sensor_tem,sensor_hum);
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

    //开机获取上次传感器保存状态
    bsp_flash_read(FLASH_SAVE_SENSOR_STATE, &read_state,1);
    sen_type_t =(SensorType)read_state;
    DBG_B_INFO("当前传感器状态为 %d ",sen_type_t);
    holdbuf[2] =(u16)sen_type_t;

}


/*
  * @brief   getsensor_task
  * @param   none
  * @note    每5S采集一次传感器数据
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
  * @note    每5S采集一次传感器数据
  * @Date:   2019.8.7
  * @updatge:2019.8.7
  * @author: zhao
  * @return: 0  无变幻
             1  改写状态操作成功
             2  改写状态操作失败
*/
static u8 change_sensor_state (SensorType* type)
{
    SensorType old_state;
    SensorType read_state;
    old_state = *type;
    if(old_state != (SensorType)(holdbuf[2])) {
        DBG_B_INFO("传感器模式切换");
        *type=(SensorType)(holdbuf[2]);
        bsp_flash_erase(FLASH_SAVE_SENSOR_STATE, 1);
        bsp_flash_write(FLASH_SAVE_SENSOR_STATE,(u8*)type,sizeof(SensorType));
        bsp_flash_read(FLASH_SAVE_SENSOR_STATE,(u8*)&read_state,sizeof(SensorType));

        if(read_state==*type) {
            DBG_B_INFO("操作成功");
            return 1;
        } else {
            DBG_B_INFO("操作失败");
            return 2;
        }
    } else {
        return 0;
    }

}




