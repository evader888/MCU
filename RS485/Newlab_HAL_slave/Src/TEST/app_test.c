/******************************************************************************
******************************************************************************
* @file    app/app_test.c
* @author  zhao
* @version V1.0.0
* @date    2019.07.10
* @update  2019.09.14
* @brief   ??????
******************************************************************************
******************************************************************************/

/**************include*************************/

#include  "app_test.h"

void RS_485test(u8 cmd)
{
    /**485 ģ?????Դ???2**/
    u8 i=0;
    uint8_t rs485buf[5];
    /**485 ģ?????Դ???2 end**/

    /**485 ģ?????Դ???3**/
//   for(i=0; i<5; i++) {
//     rs485buf[i]=i;//???䷢?ͻ?????
//   }

//    RS4851_Send_Buffer(rs485buf,5);//????5???ֽ?
//    RS4852_Send_Buffer(rs485buf,5);//????5???ֽ?

    if(cmd==1) {
        RS485_SendStr("test111111 \r\n");  // 485 1 ???Ͳ???
//        RS4851_Send_Buffer(rs485buf, 5);
        DBG_B_INFO("????485ͨ??1????");
        HAL_Delay(50);
    } else if (cmd ==2) {
        RS4852_SendStr("test222222\r\n");    // 485 2 ???Ͳ???
//        RS4852_Send_Buffer(rs485buf, 5);
        DBG_B_INFO("????485ͨ??2????");
        HAL_Delay(50);
    } else if (cmd ==3) {
        RS4853_SendStr("test33333 \r\n"); // 485 3 ???Ͳ???
//        RS4853_Send_Buffer(rs485buf, 5);
        DBG_B_INFO("????485ͨ??3????");
        HAL_Delay(50);
    } else if (cmd ==4) {
        kfifo_pull_out(&usart2_fifo,rs485buf,5);    // 485 1 ???ղ???
        DBG_B_INFO("buf[0]=0x%x buf[1]=0x%x buf[2]=0x%x buf[3]=0x%x buf[4]=0x%x ",   \
                   rs485buf[0],rs485buf[1],rs485buf[2],rs485buf[3],rs485buf[4]);
    } else if(cmd ==5) {
        kfifo_pull_out(&usart3_fifo,rs485buf,5);    // 485 2 ???ղ???
        DBG_B_INFO("buf[0]=0x%x buf[1]=0x%x buf[2]=0x%x buf[3]=0x%x buf[4]=0x%x ",   \
                   rs485buf[0],rs485buf[1],rs485buf[2],rs485buf[3],rs485buf[4]);
    } else if(cmd ==6) {
        kfifo_pull_out(&uart5_fifo,rs485buf,5);     // 485 3 ???ղ???
        DBG_B_INFO("buf[0]=0x%x buf[1]=0x%x buf[2]=0x%x buf[3]=0x%x buf[4]=0x%x ",   \
                   rs485buf[0],rs485buf[1],rs485buf[2],rs485buf[3],rs485buf[4]);
    }
    /**485 ģ?????Դ???3 end**/

}

void modbus_show_result(m_frame_typedef * fx)
{
    DBG_B_INFO("address: %02X",fx->address);
    DBG_B_INFO("function: %02X",fx->function);
    DBG_B_INFO("count: %02X",fx->count);
    DBG_B_INFO("datalen: %02X",fx->datalen);
}
/*
  * @brief   modbus_action
  * @param   none
  * @note    void modbus_action(m_frame_typedef * fx)
  * @Date:   2019.7.30
  * @updatge:2019.7.30
  * @author: zhao
  * @return: int
*/
void modbus_action(m_frame_typedef   *fx)
{
    if(fx->address==0X01) {
        if(fx->datalen) {
            if(fx->data[0]==0X00) {
                DBG_B_INFO("test1 ");
            }
            if(fx->data[1]==0X01) {
                DBG_B_INFO("test2 ");
            }
        }
    }
}



void modbus_slave_test(void)
{
    u8 res;
    m_frame_typedef rxframe;
//   DBG_B_INFO("modbus_slave_test");
    if(m_ctrl_dev.frameok) {
        res=mb_unpack_frame(&rxframe);
        if(res==MR_OK) {	//???????ɹ?
            DBG_B_INFO("???????ɹ?");
//            mb_packsend_frame(&rxframe);
//            modbus_show_result(&rxframe);
//            modbus_action(&rxframe);
//            if(rxframe.datalen) {
//                myfree(SRAMIN,rxframe.data);    //?ͷ??ڴ?
//            }
        } else if(res==MR_FRAME_FORMAT_ERR) {
            DBG_R_E("֡??ʽ????");
        } else if(res==MR_FRAME_CHECK_ERR) {
            DBG_R_E("֡У??????");
        } else if(res==MR_FRAME_SLAVE_ADDRESS) {
            DBG_R_E("?ӻ???ַ????");
        } else if(res==MR_FRANE_ILLEGAL_FUNCTION) {
            DBG_R_E("??????????");
        }
    }
}




void shell_485_connect(void)
{
    printf("\r\n");
    printf("shell_485_connect is ok\r\n");
}

void shell_485_set_address(u16 address)
{
    holdbuf[1]=address;

    printf("\r\n");
    printf("shell_485_set_address is ok\r\n");

}

void shell_485_set_mode(u16 mode)
{
    holdbuf[2]=mode;

    printf("\r\n");
    printf("shell_485_set_mode is ok\r\n");
}


void shell_485_set(u16 address,u16 mode)
{

//    holdbuf[2]=mode;

    switch (mode) {
    case 1: //??ʪ??
        holdbuf[2]=TemHum_Sensor;
        break;
    case 2:
        holdbuf[2]=BodyInfrared_Sensor;
        break;
    case 3:
        holdbuf[2]=Flame_Sensor;
        break;
    //2. ģ????
    case 4:
        holdbuf[2]=FlammableGas_Sensor;
        break;
    case 5:
        holdbuf[2]=AirQuality_Sensor;
        break;
    case 6:
        holdbuf[2]=Photosensitive_Sensor;
        break;
    case 7:
        holdbuf[2]=Sound_Sensor;
        break;

    case 8:
        holdbuf[2]=Infrared_Sensor;
        break;

    default:
        holdbuf[2]=0x0A;
        break;
	}
	    printf("\r\n");
	  	printf("shell_485_set_mode is ok holdbuf[2] %d	\r\n",mode);

        holdbuf[1]=address;

        printf("\r\n");
        printf("shell_485_set is ok\r\n");
    }


 void shell_485_check(void) {
	u8 mode=0;
    switch(sen_type_t) {
    case BodyInfrared_Sensor:mode=2;
         break;
    case Sound_Sensor:mode=7;
		 break;
    case Infrared_Sensor:mode=8;
         break;

    //2. ģ????
    case Photosensitive_Sensor:mode=6;
       break;
    case AirQuality_Sensor:mode=5;
        break;
    case Flame_Sensor:mode=3;
        break;
    case FlammableGas_Sensor:mode=4;
        break;

    //3.??ʪ??
    case TemHum_Sensor:mode=1;
        break;

    default:mode=10;
        break;

    }
        printf("\r\n");
        printf("address %d  mode %d\r\n",(u8)(holdbuf[1]),mode);
			

    }


