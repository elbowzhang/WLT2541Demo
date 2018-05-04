/******************************************************************************

 @file  simpleBLEPeripheral.c

 @brief This file contains the Simple BLE Peripheral sample application for use
        with the CC2540 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2010-2016, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: ble_sdk_1.4.2.2
 Release Date: 2016-06-09 06:57:10
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"
#include "customprofile.h"

#include "peripheral.h"

#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"
#include "hal_uart.h"
#include <string.h>
#include "SerialInterface.h"
#include "osal_snv.h"
#include "att.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;
bool connected_flag = FALSE;
uint8 setBaudNum;
uint8 setPowerNum;
uint32 setPincode;
uint16 setAdvInterval;
uint8 scanRspData[40];
uint8 advertData[40];
uint8 attDeviceName[16];

module_mode_t module_mode;
module_state_t module_state;
static uint16 buffer_tail = 0;  

uint8 Sleep=0,After_wake_up=0;
extern uint8 serialBuffer[100];
extern long serialBufferOffset;
extern bool restart_adv;

// GAP GATT Attributes

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEPeripheral_ProcessGATTMsg( gattMsgEvent_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void simpleProfileChangeCB( uint8 paramID );
static void customProfileChangeCB( uint8 paramID );
static void UART_Init(uint8 baudrate);
static uint8 sendData(uint16 diff);
static void ProcessPasscodeCB(uint8 *deviceAddr,uint16 connectionHandle,uint8 uiInputs,uint8 uiOutputs ); 
static void ProcessPairStateCB( uint16 connHandle, uint8 state, uint8 status );

char *bdAddr2Str ( uint8 *pAddr );



/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  	peripheralStateNotificationCB,  // Profile State Change Callbacks
  	NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  	ProcessPasscodeCB,                     // 密码回调  
  	ProcessPairStateCB                     // 绑定状态回调 
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  	simpleProfileChangeCB    // Charactersitic value change callback
};

// Custom Profile Callbacks
static customProfileCBs_t simpleBLEPeripheral_CustomProfileCBs =
{
 	customProfileChangeCB    // Charactersitic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

// Get BLE MAC address
static void getMACAddr(uint8* macAddr)
{
	macAddr[5] = XREG(0x780E);
	macAddr[4] = XREG(0x780F);    
  	macAddr[3] = XREG(0x7810);    
  	macAddr[2] = XREG(0x7811);                   
  	macAddr[1] = XREG(0x7812);    
  	macAddr[0] = XREG(0x7813);
}

// 0x to char
static void xToc(uint8 *MacAddrx , uint8 *MacAddrChar)
{
	for(int i = 0; i < 6; i++)
	{
		/* 判断目标字节的高4位是否小于10 */  
    	if((MacAddrx[i] >> 4) < 0x0a)  
        	MacAddrChar[i*2] = ((MacAddrx[i] >> 4) + '0'); //小于10  ,则相应发送0-9的ASCII  
    	else  
        	MacAddrChar[i*2] = ((MacAddrx[i] >> 4) - 0x0a + 'A'); //大于等于10 则相应发送 A-F  
  
    	/* 判断目标字节的低4位 是否小于10*/  
    	if((MacAddrx[i] & 0x0f) < 0x0a)  
        	MacAddrChar[i*2 + 1] = ((MacAddrx[i] & 0x0f) + '0');//小于10  ,则相应发送0-9的ASCII  
    	else  
        	MacAddrChar[i*2 + 1] = ((MacAddrx[i] & 0x0f) - 0x0a + 'A');//大于等于10 则相应发送 A-F
	}
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{
  
  simpleBLEPeripheral_TaskID = task_id;
  
  HCI_EXT_HaltDuringRfCmd(HCI_EXT_HALT_DURING_RF_DISABLE);
  
  uint32 baud_read;
  if(osal_snv_read(BLE_NVID_BAUD,4,&baud_read)==0)//get baud from flash
  {  
    switch(baud_read)
    {
	case 9600:
		setBaudNum = 0x00;
		break;
	case 19200:
		setBaudNum = 0x01;
		break;
	case 38400:
		setBaudNum = 0x02;
		break;
	case 57600:
		setBaudNum = 0x03;
		break;
	case 115200:
		setBaudNum = 0x04;
		break;
    }
  }
  else
    setBaudNum=0x00;
  
  UART_Init(setBaudNum);
  
  memset(scanRspData,0,sizeof(scanRspData));
  memset(advertData,0,sizeof(advertData));
  
  uint8 macAddr[6];
  uint8 macAddrChar[12];
  
  VOID getMACAddr(macAddr);
  VOID xToc(macAddr, macAddrChar);
  
  uint8 name_read[16];
  memset(attDeviceName,0,sizeof(attDeviceName));
  if(osal_snv_read(BLE_NVID_NAME,16,name_read)==0)//get name from flash
    memcpy(attDeviceName,name_read,strlen(name_read));
  else
    memcpy(attDeviceName,"WLT2541_BLE_",strlen("WLT2541_BLE_"));
  memcpy(&attDeviceName[strlen("WLT2541_BLE_")],&macAddrChar[8],4);	//add device mac low-4 bits

   scanRspData[0] = strlen(attDeviceName)+1;
   scanRspData[1] =0x09 ;
 
	int i;
	for( i=0;i<strlen(attDeviceName);i++)
	{
		scanRspData[2+i] = attDeviceName[i];
	}
	scanRspData[2+i] = 0x05;
	scanRspData[2+i+1] =GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE;
	scanRspData[2+i+2] =LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL );
	scanRspData[2+i+3] =HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL );
	scanRspData[2+i+4] =LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL );
	scanRspData[2+i+5] =HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL );
    
    advertData[0]=0x02;
    advertData[1]=GAP_ADTYPE_FLAGS;
    advertData[2]=DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED;
    advertData[3]=0x03;
    
    advertData[4]=GAP_ADTYPE_16BIT_MORE;
    advertData[5]=LO_UINT16( SIMPLEPROFILE_SERV_UUID );
    advertData[6]=HI_UINT16( SIMPLEPROFILE_SERV_UUID );
    advertData[7]=strlen(attDeviceName)+1;
    advertData[8]=GAP_ADTYPE_LOCAL_NAME_COMPLETE;
    
    for(i=0;i<strlen(attDeviceName);i++)
      advertData[9+i] = attDeviceName[i];
   
  	// Setup the GAP Peripheral Role Profile
  	{
    uint8 initial_advertising_enable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;
    
    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, strlen( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, strlen( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
    }

	// Set the GAP Characteristics
	GGS_SetParameter( GGS_DEVICE_NAME_ATT, 16, attDeviceName );

	// Set advertising interval
	{
	uint16 advint_read;
	if(osal_snv_read(BLE_NVID_ADVI,2,&advint_read)==0)
		setAdvInterval = advint_read;
	else
		setAdvInterval=160;

	GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, setAdvInterval );
	GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, setAdvInterval );
	GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, setAdvInterval );
	GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, setAdvInterval );
	
	// 更改广播时间为20s
	//GAP_SetParamValue( TGAP_LIM_ADV_TIMEOUT, tgap_lim_adv_timeout );
	}
    
  // Setup the GAP Bond Manager
  {
    uint32 passkey_read;
    uint8 pair_enable_read;
   
    uint8 pairMode=GAPBOND_PAIRING_MODE_INITIATE;
      
    if(osal_snv_read(BLE_NVID_PINCODE,4,&passkey_read)==0)
      setPincode=passkey_read;
    else
      setPincode=8888;
    
	/*
    if(osal_snv_read(BLE_NVID_PAIR_ENABLE,1,&pair_enable_read)==0)
    {
      if(pair_enable_read)
        pairMode=GAPBOND_PAIRING_MODE_INITIATE;
      else
        pairMode=GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    }
    else
      pairMode=GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
	*/
    
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &setPincode );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
  CustomProfile_AddService( GATT_ALL_SERVICES );  // Custom GATT Profile

  // Setup the SimpleProfile Characteristic Values
  {
    uint8 scharValue1 = 1;
    uint8 charValue2 = 2;
    uint8 scharValue3 = 3;
    uint8 scharValue4 = 4;
	uint8 scharValue5[SIMPLEPROFILE_CHAR5_LEN] = "abcde";
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &scharValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &scharValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &scharValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, scharValue5 );
  }
  
  // Setup the CustomProfile Characteristic Values
  {
  	uint8 cchar1Value = 1;
	uint8 cchar2Value = 2;
	uint8 cchar3Value[CUSTOMPROFILE_CHAR3_LEN] = "WLT2541";
	
	CustomProfile_SetParameter( CUSTOMPROFILE_CHAR1, sizeof ( uint8 ), &cchar1Value );
	CustomProfile_SetParameter( CUSTOMPROFILE_CHAR2, sizeof ( uint8 ), &cchar2Value );
	CustomProfile_SetParameter( CUSTOMPROFILE_CHAR3, CUSTOMPROFILE_CHAR3_LEN, cchar3Value );
  }
  
  {
    uint8 power_read;
    if(osal_snv_read(BLE_NVID_POWER,1,&power_read)==0)
      setPowerNum=power_read;
    else
      setPowerNum=2;
  }

//  P0DIR&=0x0C;
//  P0INP&=0x0C;
//  
//  P1DIR=0;
//  P2DIR&=0xE0;
//  P1INP&=0x03;
//  P2INP=0;
  
  //拉高P1，P2 防止漏电
  P1SEL=0;
  P2SEL=0;
  P1DIR=0;
  P2DIR=0;
  P1INP=0;
  P2INP=0;
  

#if (defined HAL_LCD) && (HAL_LCD == TRUE)

#if defined FEATURE_OAD
  #if defined (HAL_IMAGE_A)
    HalLcdWriteStringValue( "BLE Peri-A", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #else
    HalLcdWriteStringValue( "BLE Peri-B", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #endif // HAL_IMAGE_A
#else
  HalLcdWriteString( "BLE Peripheral", HAL_LCD_LINE_1 );
#endif // FEATURE_OAD

#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

  //Initialize Module Mode
  module_mode = MODULE_MODE_ATCMD;
  
  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );
  
  // Register callback with CustomProfile
  VOID CustomProfile_RegisterAppCBs( &simpleBLEPeripheral_CustomProfileCBs );

  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );

}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

    //发送初始化数据
    HalUARTWrite(HAL_UART_PORT_0,"Start WLT2541\r\n",strlen("Start WLT2541\r\n"));
    HalUARTWrite(HAL_UART_PORT_0,"+IND=",strlen("+IND="));
    HalUARTWrite(HAL_UART_PORT_0,SOFTWARE_NAME,strlen(SOFTWARE_NAME));
    HalUARTWrite(HAL_UART_PORT_0,",",strlen(","));
    HalUARTWrite(HAL_UART_PORT_0,SOFTWARE_VERSION,strlen(SOFTWARE_VERSION));
    HalUARTWrite(HAL_UART_PORT_0,"\r\n",strlen("\r\n"));
    HalUARTWrite(HAL_UART_PORT_0,"+IND=ENTER AT CMD MODE\r\n",strlen("+IND=ENTER AT CMD MODE\r\n"));
    
    return ( events ^ SBP_START_DEVICE_EVT );
  }

  if ( events & SBP_PERIODIC_EVT )
  {
    // Restart timer
    if ( connected_flag )
    {
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    }

    // Perform periodic application task
    performPeriodicTask();

    return (events ^ SBP_PERIODIC_EVT);
  }
  
  if ( events & SBP_SLEEP_EVT )
  {  
	//实现串口收发功能初始化配置
     P0SEL&=0xF3;            //P0_2(RX)、P0_3(TX)模式选择GPIO
     P0DIR|=0x08;            //设置P0_3(TX)方向为输出
     P0DIR&=0xFB;            //设置P0_2(RX)方向为输入
     P0|=0x08;               //设置P0_3(TX)输出高电平
                                                        
     P0IFG = 0;              //P0_2(RX)中断标志清零
     P0IF = 0;               //P0口中断标志清零
     PICTL|=0x01;            //设置P0口中断方式为下降沿触发
     P0IEN|=0x04;            //打开P0_2(RX)中断
     P0IE=1;                 //打开P0口中断
     EA = 1;                 //打开总中断
     
     Sleep=1;
     
     return (events ^ SBP_SLEEP_EVT);
  }

  if ( events & SBP_RESTARTADV_EVT )
  {
    uint8 i=true;
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &i);
    return (events ^ SBP_RESTARTADV_EVT);
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {     
    case GATT_MSG_EVENT:
      // Process GATT message
      simpleBLEPeripheral_ProcessGATTMsg( (gattMsgEvent_t *)pMsg );
      break;
      
    default:
      // do nothing
      break;
  }
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessGATTMsg( gattMsgEvent_t *pMsg )
{  
  GATT_bm_free( &pMsg->msg, pMsg->method );
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          // Display device address
          HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
          HalLcdWriteString( "Initialized",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ADVERTISING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
       
        connected_flag = FALSE;
	module_state = MODULE_STATE_ADVERTISE;
        if(!restart_adv)
          HalUARTWrite(HAL_UART_PORT_0,"+IND=START ADVERTISING\r\n",strlen("+IND=START ADVERTISING\r\n"));
        else
          restart_adv=FALSE;
        
        switch(setPowerNum)
	{
          case 0:
            HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_MINUS_23_DBM);
          break;
          case 1:
            HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_MINUS_6_DBM);
          break;
          case 2:	
            HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);
          break;
          default:
            HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);
          break;
        }
        
      }
      break;
     
	//设备连接成功，进入透传模式
	case GAPROLE_CONNECTED:
 	{        
		#if (defined HAL_LCD) && (HAL_LCD == TRUE)
			HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
		#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
		
			
		/*苹果蓝牙参数要求
			1，Interval Max * (Slave Latency + 1) ≤ 2 seconds
			2，Interval Min ≥ 20 ms
			3，Interval Min + 20 ms ≤ Interval Max Slave Latency ≤ 4
			4，connSupervisionTimeout ≤ 6 seconds
			5，Interval Max * (Slave Latency + 1) * 3 < connSupervisionTimeout
		*/
		//更新参数
		uint16 minConnInterval = 40;	//*1.25ms 50ms
		uint16 maxConnInterval = 80;	//*1.25ms 100ms
		uint16 latency = 0;
		uint16 connTimeOut = 500;		//*10ms  苹果手机超时设置必须小于6s
		
		HalUARTWrite(HAL_UART_PORT_0, "update conn para\r\n", strlen("update conn para\r\n"));
		GAPRole_SendUpdateParam(minConnInterval, maxConnInterval, latency, connTimeOut, GAPROLE_TERMINATE_LINK);
		  
		
		
    	module_mode = MODULE_MODE_TRANS;
      	module_state = MODULE_STATE_CONNECTED_TRANS;
      	connected_flag = TRUE;
      	if(Sleep)
     	{  
         	P0IEN&=0xFB;            //关闭P0_2(RX)中断
         	P0IE=0;                 //关闭P0口中断
            P0SEL|=0x0C;            //P0_2(RX)、P0_3(TX)模式选择UART
            Sleep=0;
            HalUARTWrite(HAL_UART_PORT_0,"+IND=WAKE UP\r\n",strlen("+IND=WAKE UP\r\n"));
     	}
      	HalUARTWrite(HAL_UART_PORT_0,"+IND=CONNECTED\r\n",strlen("+IND=CONNECTED\r\n"));
      	HalUARTWrite(HAL_UART_PORT_0,"+IND=ENTER THROUGHPUT MODE\r\n",strlen("+IND=ENTER THROUGHPUT MODE\r\n"));
       	osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
   	}
   	break;
	
	//连接成功且在广播
    case GAPROLE_CONNECTED_ADV:
   	{
     	#if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
  	}
  	break;      
	
	//等待状态进入AT模式
	//断开连接状态
    case GAPROLE_WAITING:
  	{
     	#if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        
        module_state=MODULE_STATE_IDLE;
        
        if(connected_flag)
		{
          connected_flag = FALSE;
          module_mode = MODULE_MODE_ATCMD;
          
          HalUARTWrite(HAL_UART_PORT_0,"+IND=DISCONNECTED\r\n",strlen("+IND=DISCONNECTED\r\n"));
          HalUARTWrite(HAL_UART_PORT_0,"+IND=ENTER AT CMD MODE\r\n",strlen("+IND=ENTER AT CMD MODE\r\n"));
          osal_stop_timerEx(simpleBLEPeripheral_TaskID,SBP_PERIODIC_EVT);
		  
		  //然后关闭广播
		  HalUARTWrite(HAL_UART_PORT_0,"close advertising\n",strlen("close advertising\n"));
		  uint8 initial_advertising_enable = FALSE;
		  GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
        }
        
  	}
  	break;
	
	
    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ERROR:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    default:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

  }

  gapProfileState = newState;

  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations

}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the forth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicTask( void )
{
	
 if(module_mode==MODULE_MODE_TRANS)
 {  
   if ((buffer_tail != dataBufferOffset) && (connected_flag == TRUE))
   {
        //calculate how many bytes can be sent
     uint16 diff = circular_diff(dataBufferOffset, buffer_tail);
        //send data and update tail
     uint8 bytes_sent = sendData(diff);
     buffer_tail = circular_add(buffer_tail,bytes_sent);
    }
 }
}

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR1:
		//获取变化的新值
      	SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue );

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 1:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &newValue );

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 3:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;

    default:
      // should not reach here!
      break;
  }
}

/*********************************************************************
 * @fn      customProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void customProfileChangeCB( uint8 paramID )
{
  uint8 newValue;

  switch( paramID )
  {
    case CUSTOMPROFILE_CHAR1:
      CustomProfile_GetParameter( CUSTOMPROFILE_CHAR1, &newValue );
	  
	  HalUARTWrite(HAL_UART_PORT_0,&newValue,sizeof(newValue));

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 1:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;

    default:
      // should not reach here!
      break;
  }
}


/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}


/*********************************************************************
*********************************************************************/
static void UART_Init(uint8 baudrate)
{
  halUARTCfg_t UARTConfig;
  UARTConfig.configured=true;
  UARTConfig.baudRate=baudrate;
  UARTConfig.flowControl=HAL_UART_FLOW_OFF;
  UARTConfig.flowControlThreshold=MT_UART_THRESHOLD;
  UARTConfig.idleTimeout=MT_UART_IDLE_TIMEOUT;
  UARTConfig.rx.maxBufSize=MT_UART_RX_BUFF_MAX;
  UARTConfig.tx.maxBufSize=MT_UART_TX_BUFF_MAX;
  UARTConfig.intEnable=TRUE;
  UARTConfig.callBackFunc=AT_Comand_Parser;
  HalUARTOpen(HAL_UART_PORT_0,&UARTConfig);
}

uint32 curTime = 0;
uint32 prvTime = 0;

static uint8 sendData(uint16 diff)
{
  //can send max 4 packets per connection interval
  uint8 packets_sent = 0;
  //ensure queue of notification is successful
  bool send_error = FALSE;
  //return value to update tail and send ack to msp
  uint8 bytes_sent = 0;
  
  attHandleValueNoti_t noti;      
  //dummy handle
  noti.handle = 0x2E;  
  
  //counter
  uint8 i;
  uint16 len;
  
  curTime = osal_GetSystemClock();
  
  while ((packets_sent < 4) &&  (diff >= 20) && (send_error == FALSE))
  {  
    //send 20 bytes
    noti.len = 20;
  
    noti.pValue = (uint8 *)GATT_bm_alloc( 0, ATT_HANDLE_VALUE_NOTI,
                                              20, &len );
    for (i = 0; i < 20; i++)
    {
      noti.pValue[i]= dataBuffer[circular_add(buffer_tail , bytes_sent+i)];
    }
    
    //connection handle currently hardcoded
    if (!(GATT_Notification(0, &noti, FALSE))) //if sucessful
    {
      bytes_sent += 20;
      diff -= 20;
      packets_sent++;
    }
    else
    {
      send_error = TRUE;
    }
  }
  if(diff > 0 && diff < 20)
  {
	if(!prvTime)
	{
      prvTime = curTime;
	}
	if((curTime - prvTime) > 50)
	{
	  //send remaining bytes  
	  if ((packets_sent < 4) && (diff > 0) && (send_error == FALSE))
	  {
	    noti.len = diff;

            noti.pValue = (uint8 *)GATT_bm_alloc( 0, ATT_HANDLE_VALUE_NOTI,
                                              20, &len );
            for (i = 0; i < diff; i++)
	    {
	      noti.pValue[i] = dataBuffer[circular_add(buffer_tail, bytes_sent + i)];
	    }
              
	    if (!(GATT_Notification(0, &noti, FALSE))) //if sucessful
	    {
	      bytes_sent += i;
	      diff -= i;//amount of data sent
	    }
	    else
	    {
	      send_error = TRUE;
	    }
	  }
	  prvTime = 0;
	}
	
  }

  return bytes_sent;
}

#pragma vector = P0INT_VECTOR
__interrupt void p0_ISR(void)
{
  if(P0IFG>0) 
	P0IFG = 0;      		//清除P0_2(RX)中断标志 
    
  P0IF = 0;             	//清除P0口中断标志
 
  P0IEN&=0xFB;        		//关闭P0_2(RX)中断
  P0IE=0;             		//关闭P0口中断
  P0SEL|=0x0C;        		//P0_2(RX)、P0_3(TX)模式选择UART
  
  After_wake_up=1;
  
  Sleep=0;
  
  HalUARTWrite(HAL_UART_PORT_0,"+OK\r\n+IND=WAKE UP\r\n",strlen("+OK\r\n+IND=WAKE UP\r\n"));
}

static void ProcessPasscodeCB(uint8 *deviceAddr,uint16 connectionHandle,uint8 uiInputs,uint8 uiOutputs )  
{  
  uint32  passcode;  
  uint8   str[7];  
  
//  //设置密码  
//  #if 0  
//   LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));  
//    passcode %= 1000000;  
//  #else  
//    passcode = 8888;          
//  #endif  
  
// //在LCD上显示  
//  if ( uiOutputs != 0 )  
//  {  
//    HalLcdWriteString( "Passcode:",  HAL_LCD_LINE_1 );  
//    HalLcdWriteString( (char *) _ltoa(passcode, str, 10),  HAL_LCD_LINE_2 );  
//  }  
    
  //发送密码响应给主机  
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, setPincode );  
}  

static void ProcessPairStateCB( uint16 connHandle, uint8 state, uint8 status )  
{  
  //主机发起连接，会进入开始配对状态  
  if ( state == GAPBOND_PAIRING_STATE_STARTED )  
  {  
    HalUARTWrite(HAL_UART_PORT_0,"Pairing started\r\n",strlen("Pairing started\r\n"));  
  }  
    
  //当主机提交密码后，会进入配对完成状态    
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )  
  {  
    bool pair_success=FALSE;
    
    //配对成功  
    if ( status == SUCCESS )      
    {  
      HalUARTWrite(HAL_UART_PORT_0,"Pairing success\r\n",strlen("Pairing success\r\n"));
      pair_success=TRUE;  
   }  

   //手机点取消  
    else if(status == SMP_PAIRING_FAILED_UNSPECIFIED)  
    {       
      HalUARTWrite(HAL_UART_PORT_0,"Pairing Canceled\r\n",strlen("Pairing Canceled\r\n"));
      pair_success=FALSE;  
    }  
     
    //配对失败  
    else  
    {  
      HalUARTWrite(HAL_UART_PORT_0,"Pairing fail\r\n",strlen("Pairing fail\r\n"));
      pair_success=FALSE;  
    }  
      
    //配对失败则断开连接  
    if(pair_success==FALSE)  
    {  
      GAPRole_TerminateConnection();  
    }  
  }  
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )  
  {  
    if ( status == SUCCESS )  
    {  
      HalUARTWrite(HAL_UART_PORT_0,"Bonding success\r\n", strlen("Bonding success\r\n"));
    }  
  }  
}

