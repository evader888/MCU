/******************************************************************************
******************************************************************************
* @file    app/app_slave.c
* @author  zhao
* @version V1.0.0
* @date    2019.07.10
* @update  2019.09.14
* @brief   ������
******************************************************************************
******************************************************************************/

/**************include*************************/

#include  "app_slave.h"
#include "tim.h"
#include  "flash.h"
void modbus_slave_init(void);


u8  SLAVE_ADDRESS =0;



void app_modbus_init(void)
{

    MX_TIM3_Init();

    RS4851_Init(115200);  //��ʼ��RS485
    RS4852_Init(115200);  //��ʼ��RS4852
    RS4853_Init(115200);  //��ʼ��RS4853

    modbus_slave_init();
}

void modbus_slave_init(void)
{
    u8	read_state=0;
    u8  write_value=0;

    bsp_flash_read(FLASH_SAVE_FIRST, &read_state,1);
    if(read_state!=0xAA) {
        DBG_B_INFO("��һ���ϵ����� Ĭ�ϴ�����ģʽΪ1 Ĭ�ϴӻ���ַ0x01");
        write_value=0XAA;
        bsp_flash_erase(FLASH_SAVE_FIRST, 1);
        bsp_flash_write(FLASH_SAVE_FIRST,&write_value,1);
        write_value=0X01;
        bsp_flash_erase(FLASH_SAVE_ADDRESS, 1);
        bsp_flash_write(FLASH_SAVE_ADDRESS,&write_value,1);
        write_value=0X01;
        bsp_flash_erase(FLASH_SAVE_SENSOR_STATE, 1);
        bsp_flash_write(FLASH_SAVE_SENSOR_STATE,&write_value,1);
    }

    //������ȡ�ϴδ���������״̬
    bsp_flash_read(FLASH_SAVE_ADDRESS, &read_state,1);
    SLAVE_ADDRESS =read_state;
    DBG_B_INFO("��ǰ�ӻ���ַΪ״̬Ϊ %d ",SLAVE_ADDRESS);
    holdbuf[1] =(u16)SLAVE_ADDRESS;


}


void app_modbus_slave(void)
{
    u8 res;
    m_frame_typedef rxframe;
    static uint32_t slaveaddress_set_time;
//   DBG_B_INFO("modbus_slave_test");
    if(m_ctrl_dev.frameok) {
        res=mb_unpack_frame(&rxframe);
        if(res==MR_OK) {	//�������ɹ�
//            DBG_B_INFO("�������ɹ�");
//            mb_packsend_frame(&rxframe);
//            modbus_show_result(&rxframe);
//            modbus_action(&rxframe);
//            if(rxframe.datalen) {
//                myfree(SRAMIN,rxframe.data);    //�ͷ��ڴ�
//            }
        } else if(res==MR_FRAME_FORMAT_ERR) {
            DBG_R_E("֡��ʽ����");
        } else if(res==MR_FRAME_CHECK_ERR) {
            DBG_R_E("֡У�����");
        } else if(res==MR_FRAME_SLAVE_ADDRESS) {
            DBG_R_E("�ӻ���ַ����");
        } else if(res==MR_FRANE_ILLEGAL_FUNCTION) {
            DBG_R_E("���������");
        }
    }

    if((uint32_t)(HAL_GetTick()-slaveaddress_set_time>=500)) {
        slaveaddress_set_time=HAL_GetTick();
        DBG_B_INFO("slaveaddress_set_time");
        change_address_state(&SLAVE_ADDRESS);
    }
}



/*
  * @brief   change_address_state
  * @param   SensorType* type
  * @note    �ı䴫����״̬
  * @Date:   2019.8.7
  * @updatge:2019.8.7
  * @author: zhao
  * @return: 0  �ޱ��
             1  ��д״̬�����ɹ�
             2  ��д״̬����ʧ��
*/
u8 change_address_state (u8* add_value)
{
    u8 old_state;
    u8 read_state;
    old_state = *add_value;
    if(old_state != (u8)(holdbuf[1])) {
        DBG_B_INFO("��ַ�л�");
        *add_value=(u8)(holdbuf[1]);
        bsp_flash_erase(FLASH_SAVE_ADDRESS, 1);
        bsp_flash_write(FLASH_SAVE_ADDRESS,(u8*)add_value,1);
        bsp_flash_read(FLASH_SAVE_ADDRESS,(u8*)&read_state,1);

        if(read_state==*add_value) {
            DBG_B_INFO("��ַ�л������ɹ�");
            return 1;
        } else {
            DBG_B_INFO("��ַ�л�����ʧ��");
            return 2;
        }
    } else {
        return 0;
    }

}




