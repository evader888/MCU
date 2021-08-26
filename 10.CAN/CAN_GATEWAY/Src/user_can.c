#include "user_can.h"
#include "user.h"

uint32_t TxMailbox;

#define  Exd_ID_1   0x08ac0006
#define  Exd_ID_2   0x182C0004
#define  Exd_ID_3   0x08AC0003 

CAN_TxHeaderTypeDef     TxMeg;

#define CAN_ID_STDaa  	  0xAA
#define CAN_ID_STDbb  	  0xBB
#define CAN_ID_STDcc      0xCC
#define CAN_ID_STDdd  	  0xDD
extern CAN_HandleTypeDef  hcan1;


/******************************************************************************
 * FunctionName : void CAN_User_Config(CAN_HandleTypeDef* hcan )
 * Description  : CAN��������
 * Parameters   : hcan
 * Returns      : none
*******************************************************************************/
void CAN_User_Config(CAN_HandleTypeDef* hcan )  
{
	  
	      CAN_FilterTypeDef  sFilterConfig;    
    HAL_StatusTypeDef  HAL_Status;    
        TxMeg.IDE=CAN_ID_EXT;//CAN_ID_STD; CAN_ID_EXT  
        TxMeg.RTR=CAN_RTR_DATA;  
          
//CAN filter0��ʼ��  ID��Ϊ��08AC0006  FMI=0 @ FIFO0   
      sFilterConfig.FilterBank=0;                                                           //������ 0�������ַ�Χֻ��0��13  
    sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;           //��ʶ������λģʽ  
    sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;            //ѡ�������λ��32bit  
    sFilterConfig.FilterIdHigh=FilterIDH(Exd_ID_1) ;            //�����趨��������ʶ����λ  
    sFilterConfig.FilterIdLow=FilterIDL(Exd_ID_1);              //�����趨��������ʶ����λ  
      sFilterConfig.FilterMaskIdHigh=0xFFFF;                                //�����趨���������α�ʶ�����߹�������ʶ����λ  
    sFilterConfig.FilterMaskIdLow=0xFFFF;                                   //�����趨���������α�ʶ�����߹�������ʶ����λ  
    sFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0;                //������FIFO0ָ�������x�����ڽ���ʱʹ��FIFO0������  
    sFilterConfig.FilterActivation=ENABLE;                              //��������� 0  
          
      sFilterConfig.SlaveStartFilterBank  = 0;    
//      HAL_Status=HAL_CAN_ConfigFilter(hcan, &sFilterConfig);    //��������ʼ��    
//    if(HAL_Status!=HAL_OK)    
//    {    
//        printf("��������ʼ��error!\r\n");    
//    }   
////CAN filter0��ʼ��  ID��Ϊ��182C0004   FMI=1 @ FIFO0   
      sFilterConfig.FilterBank=1;                                                           //������ 1�������ַ�Χֻ��0��13  
    sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;             //��ʶ������λģʽ  
    sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;            //ѡ�������λ��32bit  
    sFilterConfig.FilterIdHigh=FilterIDH(Exd_ID_2);                 //�����趨��������ʶ����λ  
    sFilterConfig.FilterIdLow=FilterIDL(Exd_ID_2);              //�����趨��������ʶ����λ  
    sFilterConfig.FilterMaskIdHigh=0xFFFF;                              //�����趨���������α�ʶ�����߹�������ʶ����λ  
    sFilterConfig.FilterMaskIdLow=0xFFFF;                                           //�����趨���������α�ʶ�����߹�������ʶ����λ  
    sFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0;                //������FIFO0ָ�������x�����ڽ���ʱʹ��FIFO0������  
    sFilterConfig.FilterActivation=ENABLE;                              //��������� 0  
        sFilterConfig.SlaveStartFilterBank  = 1;    
//      HAL_Status=HAL_CAN_ConfigFilter(hcan, &sFilterConfig);  //��������ʼ��    
//    if(HAL_Status!=HAL_OK)    
//    {    
//         printf("��������ʼ��error!\r\n");    
//    }   
//CAN filter0��ʼ�� ID��Ϊ��08AC0003  FMI=2 @ FIFO0     
        sFilterConfig.FilterBank=2;                                                             //������ 2�������ַ�Χֻ��0��13  
    sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;                 //��ʶ������λģʽ  
    sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;                //ѡ�������λ��32bit  
    sFilterConfig.FilterIdHigh=FilterIDH(Exd_ID_3);                 //�����趨��������ʶ����λ  
    sFilterConfig.FilterIdLow=FilterIDL(Exd_ID_3);                  //�����趨��������ʶ����λ  
    sFilterConfig.FilterMaskIdHigh=0xFFFF;                                  //�����趨���������α�ʶ�����߹�������ʶ����λ  
    sFilterConfig.FilterMaskIdLow=0xFFFF;                                               //�����趨���������α�ʶ�����߹�������ʶ����λ  
    sFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0;                      //������FIFO0ָ�������x�����ڽ���ʱʹ��FIFO0������  
    sFilterConfig.FilterActivation=ENABLE;                                  //��������� 2  
        sFilterConfig.SlaveStartFilterBank  = 2;    
    HAL_Status=HAL_CAN_ConfigFilter(hcan, &sFilterConfig);    //��������ʼ��  
        if(HAL_Status!=HAL_OK)    
    {    
         printf("��������ʼ��error!\r\n");    
    }   
          
    HAL_Status=HAL_CAN_Start(hcan);  //����CAN       
    if(HAL_Status!=HAL_OK)    
    {    
        printf("����CANʧ��\r\n");    
    }   
          
        HAL_Status=HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);       
    if(HAL_Status!=HAL_OK)    
    {    
        printf("���������ж�����ʧ��\r\n");           
    }   
		
}





void CAN_User_Init(CAN_HandleTypeDef* hcan)  
{
	 CAN_FilterTypeDef  sFilterConfig;  
   HAL_StatusTypeDef  HAL_Status;  
      
  TxMeg.IDE=CAN_ID_STD;  
  TxMeg.RTR=CAN_RTR_DATA;  
  sFilterConfig.FilterBank = 0;  
  sFilterConfig.FilterMode =  CAN_FILTERMODE_IDLIST;  
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;  
  sFilterConfig.FilterIdHigh = CAN_ID_STDaa<<5;  
  sFilterConfig.FilterIdLow  =  CAN_ID_STDbb<<5;    
  sFilterConfig.FilterMaskIdHigh = CAN_ID_STDcc<<5;  
  sFilterConfig.FilterMaskIdLow = CAN_ID_STDdd<<5;  
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;      
  sFilterConfig.FilterActivation = ENABLE;    
  sFilterConfig.SlaveStartFilterBank  =0;   
      
  HAL_Status=HAL_CAN_ConfigFilter(hcan, &sFilterConfig);  
     if(HAL_Status!=HAL_OK)    
   {    
        printf("  HAL_CAN_ConfigFilterʧ��\r\n");    
   }   
      HAL_Status=HAL_CAN_Start(hcan);      
    if(HAL_Status!=HAL_OK)    
    {    
        printf("����canʧ�ܣ�\r\n");    
    }   
        HAL_Status=HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);       
    if(HAL_Status!=HAL_OK)    
    {    
        printf("�������������ж�ʧ��\r\n");           
    } 
}







/******************************************************************************
 * FunctionName : void can_start(void)
 * Description  : ����CAN����
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void can_start(void)
{
	HAL_CAN_Start(&hcan1);   
}

/******************************************************************************
 * FunctionName : void can_stop(void)
 * Description  : ֹͣCAN����
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void can_stop(void)
{
	HAL_CAN_Stop(&hcan);  
}

/******************************************************************************
 * FunctionName : uint8_t Can_Send_Msg(uint8_t* msg,uint8_t len)
 * Description  : can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
 * Parameters   : len:���ݳ���(���Ϊ8)
 *                msg:����ָ��,���Ϊ8���ֽ�.
 * Returns      : 0,�ɹ�;
 *            		����,ʧ��;
*******************************************************************************/
uint8_t Can_Send_Msg(uint8_t* msg,uint8_t len)
{	
		uint16_t i=0;  
    uint8_t data[8];  
      
    CAN_TxHeaderTypeDef  TxMeg;   
  
    TxMeg.StdId=0x12;           // ��׼��ʶ��   
    TxMeg.ExtId=0x12;           // ������չ��ʾ��   
    TxMeg.IDE=CAN_ID_STD;   // ��׼֡  
    TxMeg.RTR=CAN_RTR_DATA;       // ����֡  
    TxMeg.DLC=len;              // Ҫ���͵����ݳ���  
    for(i=0;i<len;i++)  
    {  
        data[i]=msg[i];  
    }  
      
    if (HAL_CAN_AddTxMessage(&hcan, &TxMeg, data, &TxMailbox) != HAL_OK)  
    {  
        printf("Can send data error\r\n");  
    }  
    else  
    {  
        printf("Can send data success\r\n");  
    }  
    return 0;
}

/******************************************************************************
 * FunctionName : uint8_t Can_Send_Msg_StdId(uint16_t My_StdId,uint8_t len,uint8_t Type_Sensor)
 * Description  : can����һ������(�̶���ʽ:IDΪMy_StdId,��׼֡,����֡)	
 * Parameters   : My_StdId:��׼��ʶ�� 
 *                len:���ݳ���(���Ϊ8)
 *                msg:����ָ��,���Ϊ8���ֽ�.
 * Returns      : 0,�ɹ�;
 *            		����,ʧ��;
*******************************************************************************/
uint8_t Can_Send_Msg_StdId(uint16_t My_StdId,uint8_t len,uint8_t Type_Sensor)
{	
	        CAN_TxHeaderTypeDef  TxMeg;     
    ValueType ValueType_t;    
    uint8_t vol_H,vol_L;    
    uint16_t i=0;    
    uint8_t data[8];    
    
    TxMeg.StdId=My_StdId;                
    TxMeg.ExtId=0x00;                    
    TxMeg.IDE=CAN_ID_STD;                         
    TxMeg.RTR=CAN_RTR_DATA;            
    TxMeg.DLC=len;                       
    for(i=0;i<len;i++)    
    {    
        data[i]=0;    
    }   
        data[0] = Sensor_Type_t;      
    printf("Can_Send_Msg_StdId >>My_StdId��׼֡ID= %x   \r\n",My_StdId);    
    printf("Can_Send_Msg_StdId >>Sensor_Type_t %d \r\n",data[0]);    
    ValueType_t=ValueTypes(Type_Sensor);    
    printf("Can_Send_Msg_StdId >>ValueType_t %d \r\n",ValueType_t);    
     data[3] = (uint8_t) My_StdId&0x00ff;         
     data[4] = My_StdId>>8;  
    switch(ValueType_t)    
    {    
        case Value_ADC:    
                
                    vol_H = (vol&0xff00)>>8;    
                    vol_L = vol&0x00ff;    
                    data[1]=vol_H;    
                    data[2]=vol_L;          
                    printf("Can_Send_Msg_StdId >> Value_ADC TxMessage.Data[1]= %d \r\n",data[1]);    
                    printf("Can_Send_Msg_StdId >> Value_ADC TxMessage.Data[2]= %d \r\n",data[2]);    
            break;    
        case Value_Switch:    
                    data[1]=switching;    
                    data[2]=0;    
            break;    
        case Value_I2C:    
                    data[1]=sensor_tem;    
                    data[2]=sensor_hum;    
                    printf("Can_Send_Msg_StdId >> Value_I2C TxMessage.Data[1]= %d \r\n",data[1]);    
                    printf("Can_Send_Msg_StdId >> Value_I2C TxMessage.Data[2]= %d \r\n",data[2]);     
            break;    
        default:    
            break;    
    }    
    if (HAL_CAN_AddTxMessage(&hcan, &TxMeg, data, &TxMailbox) != HAL_OK)    
    {    
        printf("Can send data error\r\n");    
    }    
    else    
    {    
        printf("Can send data success\r\n");    
    }    
    return 0;
}



//CAN���ջص�����
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)      
{
	    CAN_RxHeaderTypeDef RxMessage;    
    uint8_t  Data[8] = {0};    
    HAL_StatusTypeDef   HAL_RetVal;     
        
    uint8_t FMI;  
    uint32_t Id;  
    int i=0;  
    RxMessage.StdId=0x00;  
    RxMessage.ExtId=0x00;  
    RxMessage.IDE=0;  
    RxMessage.DLC=0;  
    RxMessage.FilterMatchIndex=0;  
    HAL_RetVal=HAL_CAN_GetRxMessage(hcan,  CAN_RX_FIFO0, &RxMessage,  Data);    
    if ( HAL_OK==HAL_RetVal)    
    {                                           
        for(i=0;i<RxMessage.DLC;i++)       
        Can_data[i]= Data[i];           
    }   
    Id=RxMessage.StdId;   
    FMI=RxMessage.FilterMatchIndex;          
    Can_data[7]=RxMessage.DLC;                   
    switch(Id)  
    {  
       case 0xAA:  
                 flag_send_data=1;    
                 printf(" [FMI= %d ] [ STD_ID= %x ]  \r\n",FMI ,Id );  
                 printf("StdId== 0x%X  Data[0]=0x%x Data[1]=0x%x Data[2]=0x%x \r\n ", Id,Can_data[0], Can_data[1],Can_data[2]);  
                    break;  
       case 0xBB:  
                 flag_send_data=1;            
                printf("StdId== 0x%X Data[0]=0x%x Data[1]=0x%x Data[2]=0x%x \r\n ", Id,Can_data[0], Can_data[1],Can_data[2]);  
                    break;  
       case 0xDD:  
                 flag_send_data=1;                
                printf("StdId== 0x%X  Data[0]=0x%x Data[1]=0x%x Data[2]=0x%x \r\n ", Id,Can_data[0], Can_data[1],Can_data[2]);  
            default:  
                break;    
        }  
		
}
   
/*******************************************************************************
* ������ 		: FilterID
* ��������    	: �������ν���ID
* �������     	: gaodiweiΪ0ʱѡ���16λ��1ʱѡ���16λ��FIDΪҪ���ε�ID��
* ������     	: ��
* ����ֵ       	: Filter_IDL/Filter_IDH ,IDֵ
*******************************************************************************/
unsigned short FilterID(uint8_t gaodiwei,uint32_t FID)  
{  	
   uint32_t Filter_ID;
   uint16_t Filter_IDH;
   uint16_t Filter_IDL;
   if(gaodiwei==0)				 	//ȡ��CAN-ID��λ�ֽ�
   {
	    Filter_IDL=FID&0x0000FFFF;		 
		Filter_IDL=Filter_IDL<<3;
		Filter_IDL=Filter_IDL|0x0004;
		return	Filter_IDL; 
   }
   else if(gaodiwei==1)				//ȡ��CAN-ID��λ�ֽ�
   {
	 	Filter_ID=FID&0xffff0000;
		Filter_IDH=Filter_ID>>16;
	    Filter_IDH=Filter_IDH<<3;
	   	return	Filter_IDH;	 	 
   }
   else
   {
   		return 0x0000;
   } 		 	
}

