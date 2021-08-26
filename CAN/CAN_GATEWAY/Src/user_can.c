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
 * Description  : CAN总线配置
 * Parameters   : hcan
 * Returns      : none
*******************************************************************************/
void CAN_User_Config(CAN_HandleTypeDef* hcan )  
{
	  
	      CAN_FilterTypeDef  sFilterConfig;    
    HAL_StatusTypeDef  HAL_Status;    
        TxMeg.IDE=CAN_ID_EXT;//CAN_ID_STD; CAN_ID_EXT  
        TxMeg.RTR=CAN_RTR_DATA;  
          
//CAN filter0初始化  ID号为：08AC0006  FMI=0 @ FIFO0   
      sFilterConfig.FilterBank=0;                                                           //过滤器 0，该数字范围只能0到13  
    sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;           //标识符屏蔽位模式  
    sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;            //选择过滤器位宽32bit  
    sFilterConfig.FilterIdHigh=FilterIDH(Exd_ID_1) ;            //用来设定过滤器标识符高位  
    sFilterConfig.FilterIdLow=FilterIDL(Exd_ID_1);              //用来设定过滤器标识符低位  
      sFilterConfig.FilterMaskIdHigh=0xFFFF;                                //用来设定过滤器屏蔽标识符或者过滤器标识符高位  
    sFilterConfig.FilterMaskIdLow=0xFFFF;                                   //用来设定过滤器屏蔽标识符或者过滤器标识符低位  
    sFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0;                //过滤器FIFO0指向过滤器x，用于接收时使用FIFO0的邮箱  
    sFilterConfig.FilterActivation=ENABLE;                              //激活过滤器 0  
          
      sFilterConfig.SlaveStartFilterBank  = 0;    
//      HAL_Status=HAL_CAN_ConfigFilter(hcan, &sFilterConfig);    //过滤器初始化    
//    if(HAL_Status!=HAL_OK)    
//    {    
//        printf("过滤器初始化error!\r\n");    
//    }   
////CAN filter0初始化  ID号为：182C0004   FMI=1 @ FIFO0   
      sFilterConfig.FilterBank=1;                                                           //过滤器 1，该数字范围只能0到13  
    sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;             //标识符屏蔽位模式  
    sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;            //选择过滤器位宽32bit  
    sFilterConfig.FilterIdHigh=FilterIDH(Exd_ID_2);                 //用来设定过滤器标识符高位  
    sFilterConfig.FilterIdLow=FilterIDL(Exd_ID_2);              //用来设定过滤器标识符低位  
    sFilterConfig.FilterMaskIdHigh=0xFFFF;                              //用来设定过滤器屏蔽标识符或者过滤器标识符高位  
    sFilterConfig.FilterMaskIdLow=0xFFFF;                                           //用来设定过滤器屏蔽标识符或者过滤器标识符低位  
    sFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0;                //过滤器FIFO0指向过滤器x，用于接收时使用FIFO0的邮箱  
    sFilterConfig.FilterActivation=ENABLE;                              //激活过滤器 0  
        sFilterConfig.SlaveStartFilterBank  = 1;    
//      HAL_Status=HAL_CAN_ConfigFilter(hcan, &sFilterConfig);  //过滤器初始化    
//    if(HAL_Status!=HAL_OK)    
//    {    
//         printf("过滤器初始化error!\r\n");    
//    }   
//CAN filter0初始化 ID号为：08AC0003  FMI=2 @ FIFO0     
        sFilterConfig.FilterBank=2;                                                             //过滤器 2，该数字范围只能0到13  
    sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;                 //标识符屏蔽位模式  
    sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;                //选择过滤器位宽32bit  
    sFilterConfig.FilterIdHigh=FilterIDH(Exd_ID_3);                 //用来设定过滤器标识符高位  
    sFilterConfig.FilterIdLow=FilterIDL(Exd_ID_3);                  //用来设定过滤器标识符低位  
    sFilterConfig.FilterMaskIdHigh=0xFFFF;                                  //用来设定过滤器屏蔽标识符或者过滤器标识符高位  
    sFilterConfig.FilterMaskIdLow=0xFFFF;                                               //用来设定过滤器屏蔽标识符或者过滤器标识符低位  
    sFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0;                      //过滤器FIFO0指向过滤器x，用于接收时使用FIFO0的邮箱  
    sFilterConfig.FilterActivation=ENABLE;                                  //激活过滤器 2  
        sFilterConfig.SlaveStartFilterBank  = 2;    
    HAL_Status=HAL_CAN_ConfigFilter(hcan, &sFilterConfig);    //过滤器初始化  
        if(HAL_Status!=HAL_OK)    
    {    
         printf("过滤器初始化error!\r\n");    
    }   
          
    HAL_Status=HAL_CAN_Start(hcan);  //开启CAN       
    if(HAL_Status!=HAL_OK)    
    {    
        printf("开启CAN失败\r\n");    
    }   
          
        HAL_Status=HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);       
    if(HAL_Status!=HAL_OK)    
    {    
        printf("开启挂起中段允许失败\r\n");           
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
        printf("  HAL_CAN_ConfigFilter失败\r\n");    
   }   
      HAL_Status=HAL_CAN_Start(hcan);      
    if(HAL_Status!=HAL_OK)    
    {    
        printf("开启can失败！\r\n");    
    }   
        HAL_Status=HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);       
    if(HAL_Status!=HAL_OK)    
    {    
        printf("开启挂起允许中断失败\r\n");           
    } 
}







/******************************************************************************
 * FunctionName : void can_start(void)
 * Description  : 启动CAN总线
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void can_start(void)
{
	HAL_CAN_Start(&hcan1);   
}

/******************************************************************************
 * FunctionName : void can_stop(void)
 * Description  : 停止CAN总线
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void can_stop(void)
{
	HAL_CAN_Stop(&hcan);  
}

/******************************************************************************
 * FunctionName : uint8_t Can_Send_Msg(uint8_t* msg,uint8_t len)
 * Description  : can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
 * Parameters   : len:数据长度(最大为8)
 *                msg:数据指针,最大为8个字节.
 * Returns      : 0,成功;
 *            		其他,失败;
*******************************************************************************/
uint8_t Can_Send_Msg(uint8_t* msg,uint8_t len)
{	
		uint16_t i=0;  
    uint8_t data[8];  
      
    CAN_TxHeaderTypeDef  TxMeg;   
  
    TxMeg.StdId=0x12;           // 标准标识符   
    TxMeg.ExtId=0x12;           // 设置扩展标示符   
    TxMeg.IDE=CAN_ID_STD;   // 标准帧  
    TxMeg.RTR=CAN_RTR_DATA;       // 数据帧  
    TxMeg.DLC=len;              // 要发送的数据长度  
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
 * Description  : can发送一组数据(固定格式:ID为My_StdId,标准帧,数据帧)	
 * Parameters   : My_StdId:标准标识符 
 *                len:数据长度(最大为8)
 *                msg:数据指针,最大为8个字节.
 * Returns      : 0,成功;
 *            		其他,失败;
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
    printf("Can_Send_Msg_StdId >>My_StdId标准帧ID= %x   \r\n",My_StdId);    
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



//CAN接收回调函数
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
* 函数名 		: FilterID
* 函数描述    	: 过滤屏蔽接收ID
* 输入参数     	: gaodiwei为0时选择低16位，1时选择高16位，FID为要屏蔽的ID号
* 输出结果     	: 无
* 返回值       	: Filter_IDL/Filter_IDH ,ID值
*******************************************************************************/
unsigned short FilterID(uint8_t gaodiwei,uint32_t FID)  
{  	
   uint32_t Filter_ID;
   uint16_t Filter_IDH;
   uint16_t Filter_IDL;
   if(gaodiwei==0)				 	//取出CAN-ID低位字节
   {
	    Filter_IDL=FID&0x0000FFFF;		 
		Filter_IDL=Filter_IDL<<3;
		Filter_IDL=Filter_IDL|0x0004;
		return	Filter_IDL; 
   }
   else if(gaodiwei==1)				//取出CAN-ID高位字节
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

