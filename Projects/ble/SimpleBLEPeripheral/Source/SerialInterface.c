#include "SerialInterface.h"
#include "bcomdef.h"
#include "hal_uart.h"
#include "Att.h"
#include "osal_snv.h"
#include "gap.h"
#include "peripheral.h"
#include "gapgattserver.h"
#include "hci.h"
#include "simpleBLEPeripheral.h"
#include "osal_snv.h"
#include "simpleGATTprofile.h"
#include <string.h>
#include <stdio.h>
#include <gapbondmgr.h>

uint8 serialBuffer[100];
long serialBufferOffset = 0;
uint16 setNum = 0;
uint16 exitTransNum=0;
uint8 dataBuffer[RX_BUFF_SIZE];
uint16 dataBufferOffset = 0;
bool restart_adv=FALSE;

extern uint8 After_wake_up;
extern bool connected_flag;
extern char *bdAddr2Str( uint8 *pAddr );
extern uint8 scanRspData[40];

void AT_Comand_Parser( uint8 port, uint8 event )
{  
  //unused input parameters
  (void)port;
  (void)event;
   uint8 numBytes;
   uint8 cmd_temp_buf[50];
   uint8 w_ret = 1;
	
   // get the number of available bytes to process
   numBytes = Hal_UART_RxBufLen(HAL_UART_PORT_0);

   if(numBytes > 0)
   {
        
        uint8 *temp_buf;

        temp_buf = osal_mem_alloc( numBytes);
         //store dma buffer into temp buffer
        HalUARTRead(HAL_UART_PORT_0,temp_buf, numBytes);

        switch(module_mode)
        {
			//AT命令模式
			case MODULE_MODE_ATCMD:
			  	
			  	//读入命令
				for(int i=0;i<numBytes;i++)
				{
					serialBuffer[serialBufferOffset]=temp_buf[i];
					serialBufferOffset++;
				} 
				if((serialBuffer[serialBufferOffset-1] == '\n')&&(serialBuffer[serialBufferOffset-2] == '\r'))
				{
					serialBuffer[serialBufferOffset-1]=0;
					serialBuffer[serialBufferOffset-2]=0;
					serialBufferOffset=serialBufferOffset-2;
					
					//识别到“AT+”
					if(memcmp(&serialBuffer[0],"AT+",strlen("AT+")) == 0)
					{   
						//进入设置波特率
						if(memcmp(&serialBuffer[3],"SETBAUD",strlen("SETBAUD")) == 0)
						{
							if(serialBufferOffset-11 > 0)
							{
								for(int i=0;i<serialBufferOffset-11;i++)
								{
									cmd_temp_buf[i] = serialBuffer[11+i];
								}
									
								uint32 baud_write;
								if(memcmp(cmd_temp_buf,"9600",strlen("9600")) == 0)
								{
									baud_write=9600;
									w_ret = osal_snv_write( BLE_NVID_BAUD, sizeof(uint32), &baud_write );
									if(w_ret == 0)
										HalUARTWrite(HAL_UART_PORT_0,"+OK\r\n",strlen("+OK\r\n"));
									else
										HalUARTWrite(HAL_UART_PORT_0,"+ERR\r\n",strlen("+ERR\r\n"));
									  
								}
								else if(memcmp(cmd_temp_buf,"115200",strlen("115200")) == 0)
								{
									  
									baud_write=115200;
									w_ret = osal_snv_write( BLE_NVID_BAUD, sizeof(uint32), &baud_write );
									if(w_ret == 0)
										HalUARTWrite(HAL_UART_PORT_0,"+OK\r\n",strlen("+OK\r\n"));
									else
										HalUARTWrite(HAL_UART_PORT_0,"+ERR\r\n",strlen("+ERR\r\n"));
								}
								else if(memcmp(cmd_temp_buf,"19200",strlen("19200")) == 0)
								{
									baud_write=19200;
									w_ret = osal_snv_write( BLE_NVID_BAUD, sizeof(uint32), &baud_write );
									if(w_ret == 0)
										HalUARTWrite(HAL_UART_PORT_0,"+OK\r\n",strlen("+OK\r\n"));
									else
										HalUARTWrite(HAL_UART_PORT_0,"+ERR\r\n",strlen("+ERR\r\n"));
									  
								}
								else if(memcmp(cmd_temp_buf,"38400",strlen("38400")) == 0)
								{
									baud_write=38400;
									w_ret = osal_snv_write( BLE_NVID_BAUD, sizeof(uint32), &baud_write );
									if(w_ret == 0)
										HalUARTWrite(HAL_UART_PORT_0,"+OK\r\n",strlen("+OK\r\n"));
									else
										HalUARTWrite(HAL_UART_PORT_0,"+ERR\r\n",strlen("+ERR\r\n"));
								}
								else if(memcmp(cmd_temp_buf,"57600",strlen("57600")) == 0)
								{
									baud_write=57600;
									w_ret = osal_snv_write( BLE_NVID_BAUD, sizeof(uint32), &baud_write );
									if(w_ret == 0)
										HalUARTWrite(HAL_UART_PORT_0,"+OK\r\n",strlen("+OK\r\n"));
									else
										HalUARTWrite(HAL_UART_PORT_0,"+ERR\r\n",strlen("+ERR\r\n"));
									  
								}
								else
								{
									HalUARTWrite(HAL_UART_PORT_0,"+ERR=-4\r\n",9);
								}
									
							}
							else
							{
								HalUARTWrite(HAL_UART_PORT_0,"+ERR=-1\r\n",9);	
							}
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
							break;
								  
						}
						
						//进入获取波特率
						else if(memcmp(&serialBuffer[3],"GETBAUD",strlen("GETBAUD")) == 0)
						{         
							if(setBaudNum == 0x00)
							{
								HalUARTWrite(HAL_UART_PORT_0,"+OK=9600\r\n",strlen("+OK=9600\r\n"));
							}
							else if(setBaudNum == 0x04)
							{
								HalUARTWrite(HAL_UART_PORT_0,"+OK=115200\r\n",strlen("+OK=115200\r\n"));
							}
							else if(setBaudNum == 0x01)
							{
								HalUARTWrite(HAL_UART_PORT_0,"+OK=19200\r\n",strlen("+OK=19200\r\n"));
							}
							else if(setBaudNum == 0x02)
							{
								HalUARTWrite(HAL_UART_PORT_0,"+OK=38400\r\n",strlen("+OK=38400\r\n"));
							}
							else if(setBaudNum == 0x03)
							{
								HalUARTWrite(HAL_UART_PORT_0,"+OK=57600\r\n",strlen("+OK=57600\r\n"));
							}
							else
							{
								HalUARTWrite(HAL_UART_PORT_0,"+ERR=-3\r\n",strlen("+ERR=-3\r\n"));
							}
								  
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
							break;
						}
						
						//进入设置名称
						else if(memcmp(&serialBuffer[3],"SETNAME",strlen("SETNAME")) == 0)
						{
							memset(cmd_temp_buf,0,50);
							if(serialBufferOffset-11 > 0)
							{
								for(int i=0;i<serialBufferOffset-11;i++)
								{
									cmd_temp_buf[i] = serialBuffer[11+i];
								}
									
								int i;
								if((cmd_temp_buf !=NULL)&&((strlen(cmd_temp_buf)) <15))
								{
									  
									memset(attDeviceName,0,sizeof(attDeviceName));
									memcpy(attDeviceName,cmd_temp_buf,strlen(cmd_temp_buf));
									uint8 length=0;
									length = strlen(cmd_temp_buf)+1;
									memset(scanRspData,0,sizeof(scanRspData));
									memset(advertData,0,sizeof(advertData));
									scanRspData[0] = length;
									scanRspData[1] =0x09 ;
									for(i=0;i<strlen(attDeviceName);i++)
									{
										scanRspData[2+i] = attDeviceName[i];
									}
									scanRspData[2+i] = 0x05;
									scanRspData[2+i+1] =GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE;
									scanRspData[2+i+2] =LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL );
									scanRspData[2+i+3] =HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL );
									scanRspData[2+i+4] =LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL );
									scanRspData[2+i+5] = HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL );
									  
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
									  
									  
									GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, strlen ( scanRspData ), scanRspData );//joe
									GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
									GGS_SetParameter( GGS_DEVICE_NAME_ATT, strlen(attDeviceName), attDeviceName );
									int ret=1;
									  
									ret = osal_snv_write(BLE_NVID_NAME,16,attDeviceName);
									if(ret == 0)
									{
										
										HalUARTWrite(HAL_UART_PORT_0,"+OK\r\n",strlen("+OK\r\n"));
									}
									else
									{
										HalUARTWrite(HAL_UART_PORT_0,"+ERR\r\n",strlen("+ERR\r\n"));
									}				
									  
								}
								else
								{
									HalUARTWrite(HAL_UART_PORT_0,"+ERR=-4\r\n",strlen("+ERR=-4\r\n"));
								}
								//module_cmd = MODULE_CMD_SET_NAME;
									
							}
							else
							{
								HalUARTWrite(HAL_UART_PORT_0,"+ERR=-1\r\n",9);
							}
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
							break;
						}
						
						//进入获取名称
						else if(memcmp(&serialBuffer[3],"GETNAME",strlen("GETNAME")) == 0)
						{
							HalUARTWrite(HAL_UART_PORT_0,"+OK=",strlen("+OK="));
							HalUARTWrite(HAL_UART_PORT_0,attDeviceName,strlen(attDeviceName));
							HalUARTWrite(HAL_UART_PORT_0,"\r\n",strlen("\r\n"));
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
							break;
						}
						
						//进入获取版本号
						else if(memcmp(&serialBuffer[3],"MINFO",strlen("MINFO")) == 0)
						{
							HalUARTWrite(HAL_UART_PORT_0,"+OK=",strlen("+OK="));
							HalUARTWrite(HAL_UART_PORT_0,SOFTWARE_NAME,strlen(SOFTWARE_NAME));
							HalUARTWrite(HAL_UART_PORT_0,",",strlen(","));
							HalUARTWrite(HAL_UART_PORT_0,SOFTWARE_VERSION,strlen(SOFTWARE_VERSION));
							HalUARTWrite(HAL_UART_PORT_0,"\r\n",strlen("\r\n"));
								  
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
							break;
						}
						
						//进入测试
						else if(memcmp(&serialBuffer[3],"TEST",strlen("TEST")) == 0)
						{
							HalUARTWrite(HAL_UART_PORT_0,"+OK=This is a Test Message",strlen("+OK=This is a Test Message"));
							HalUARTWrite(HAL_UART_PORT_0,"\r\n",strlen("\r\n"));
							
							memset(serialBuffer, 0, 100);
							serialBufferOffset = 0;
							break;
						}
						
						//断开连接
						else if(memcmp(&serialBuffer[3],"DISCONNECT",strlen("DISCONNECT")) == 0)
						{
								  
							if((module_state == MODULE_STATE_CONNECTED)||(module_state == MODULE_STATE_CONNECTED_TRANS))
							{
								GAPRole_TerminateConnection();
								HalUARTWrite(HAL_UART_PORT_0,"+OK\r\n",5);
							}
							else
							{
								HalUARTWrite(HAL_UART_PORT_0,"+ERR=-5\r\n",9);
							}
								  
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
							break;
						}
						
						//设置广播间隔时间
						else if(memcmp(&serialBuffer[3],"SETADVINTERVAL",strlen("SETADVINTERVAL")) == 0)
						{
							memset(cmd_temp_buf,0,50);
							if(serialBufferOffset-18 > 0)
							{
								for(int i=0;i<serialBufferOffset-18;i++)
								{
									cmd_temp_buf[i] = serialBuffer[18+i];
								}
									
								setNum =StringNumToUnsignedInteger(cmd_temp_buf);
									
								int sadvi_ret = 1;
									
								if(setNum > 0)
								{
									setAdvInterval = setNum;//advInterval*625us
									GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, setAdvInterval);
									GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, setAdvInterval );
									GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, setAdvInterval );
									GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, setAdvInterval );
									GAP_EndDiscoverable(gapRole_TaskID);
									osal_set_event( simpleBLEPeripheral_TaskID, SBP_RESTARTADV_EVT );
									restart_adv=TRUE;
									  
									sadvi_ret = osal_snv_write(BLE_NVID_ADVI,2,&setAdvInterval);
									if(sadvi_ret == 0)
									{
										HalUARTWrite(HAL_UART_PORT_0,"+OK\r\n",strlen("+OK\r\n"));
									}
									else
									{
										HalUARTWrite(HAL_UART_PORT_0,"+ERR\r\n",strlen("+ERR\r\n"));
									} 
								}
								else
								{
									HalUARTWrite(HAL_UART_PORT_0,"+ERR=-4\r\n",strlen("+ERR=-4\r\n"));
								} 
							}
							else
							{
								HalUARTWrite(HAL_UART_PORT_0,"+ERR=-1\r\n",9);	
							}
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
							break;
						}
						
						//获取当前广播间隔时间
						else if(memcmp(&serialBuffer[3],"GETADVINTERVAL",strlen("GETADVINTERVAL")) == 0)
						{
								  
							uint8 getAdvInterval[20];
							memset(getAdvInterval,0,20);
							sprintf(getAdvInterval,"+OK=%d\r\n",setAdvInterval);
							HalUARTWrite(HAL_UART_PORT_0,getAdvInterval,strlen(getAdvInterval));
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
							break;
						}
						
						//停止广播
						else if(memcmp(&serialBuffer[3],"STOPADVERTISING",strlen("STOPADVERTISING")) == 0)
						{
							GAP_EndDiscoverable(gapRole_TaskID);
							HalUARTWrite(HAL_UART_PORT_0,"+OK\r\n",strlen("+OK\r\n"));
							HalUARTWrite(HAL_UART_PORT_0,"+IND=STOP ADVERTISING\r\n",strlen("+IND=STOP ADVERTISING\r\n"));
								  
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
							break;
						}
						
						//开始广播
						else if(memcmp(&serialBuffer[3],"STARTADVERTISING",strlen("STARTADVERTISING")) == 0)
						{
							uint8 i=true;
							GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &i);
							HalUARTWrite(HAL_UART_PORT_0,"+OK\r\n",strlen("+OK\r\n"));
							module_state = MODULE_STATE_ADVERTISE;
								  
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
							break;
						}
						
						//进入透传模式
						else if(memcmp(&serialBuffer[3],"ENTM",strlen("ENTM")) == 0)
						{     
							if(module_state == MODULE_STATE_CONNECTED)
							{
								module_state = MODULE_STATE_CONNECTED_TRANS;
								module_mode = MODULE_MODE_TRANS;
								HalUARTWrite(HAL_UART_PORT_0,"+OK\r\n",strlen("+OK\r\n"));
								HalUARTWrite(HAL_UART_PORT_0,"+IND=ENTER THROUGHPUT MODE\r\n",strlen("+IND=ENTER THROUGHPUT MODE\r\n"));
							}
							else
							{
								HalUARTWrite(HAL_UART_PORT_0,"+ERR=-3\r\n",strlen("+ERR=-3\r\n"));
							}
								  
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
							break;
						}
						
						//休眠
						else if(memcmp(&serialBuffer[3],"SLEEP",strlen("SLEEP")) == 0)
						{
							HalUARTWrite(HAL_UART_PORT_0,"+OK\r\n+IND=SLEEP\r\n",strlen("+OK\r\n+IND=SLEEP\r\n"));
							osal_start_timerEx(simpleBLEPeripheral_TaskID,SBP_SLEEP_EVT,100);
								  
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
							break;
						}
						
						//进入设置发射功率
						else if(memcmp(&serialBuffer[3],"SETPOWER",strlen("SETPOWER")) == 0)
						{
							memset(cmd_temp_buf,0,50);
							if(serialBufferOffset-12 > 0)
							{
								for(int i=0;i<serialBufferOffset-12;i++)
								{
									cmd_temp_buf[i] = serialBuffer[12+i];
								}
								setNum = 0;
								setNum =StringNumToUnsignedInteger(cmd_temp_buf);
									
								int power_ret=1;
								if(setNum == 0)
								{
									setPowerNum = setNum;
									HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_MINUS_23_DBM);
									  
									power_ret = osal_snv_write(BLE_NVID_POWER,1,&setPowerNum);
									if(power_ret == 0)
									{
										HalUARTWrite(HAL_UART_PORT_0,"+OK\r\n",strlen("+OK\r\n"));  
									}
									else
									{
										HalUARTWrite(HAL_UART_PORT_0,"+ERR\r\n",strlen("+ERR\r\n"));
									} 
								}
								else if(setNum == 1)
								{
									setPowerNum = setNum;
									HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_MINUS_6_DBM);
									  
									power_ret = osal_snv_write(BLE_NVID_POWER,1,&setPowerNum);
									if(power_ret == 0)
									{
										HalUARTWrite(HAL_UART_PORT_0,"+OK\r\n",strlen("+OK\r\n"));
									}
									else
									{
										HalUARTWrite(HAL_UART_PORT_0,"+ERR\r\n",strlen("+ERR\r\n"));
									}  
								}
								else if(setNum == 2)
								{
									setPowerNum = setNum;
									HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);
									power_ret = osal_snv_write(BLE_NVID_POWER,1,&setPowerNum);
									if(power_ret == 0)
									{
										HalUARTWrite(HAL_UART_PORT_0,"+OK\r\n",strlen("+OK\r\n"));
									}
									else
									{
										HalUARTWrite(HAL_UART_PORT_0,"+ERR\r\n",strlen("+ERR\r\n"));
									}
								}
								else
								{
									HalUARTWrite(HAL_UART_PORT_0,"+ERR=-4\r\n",strlen("+ERR=-4\r\n"));
								}
							}
							else
							{
								HalUARTWrite(HAL_UART_PORT_0,"+ERR=-1\r\n",9);	
							}
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
							break;
						}
						
						//复位
						else if(memcmp(&serialBuffer[3],"RESET",strlen("RESET")) == 0)
						{
							SRCRC|=0x20;            //启动看门狗复位
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
							break;
						}
						
						//获取模块地址
						else if(memcmp(&serialBuffer[3],"GETADDRESS",strlen("GETADDRESS")) == 0)
						{
							uint8 ownAddress[B_ADDR_LEN];
							GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
							HalUARTWrite(HAL_UART_PORT_0,"+OK=",strlen("+OK="));
							HalUARTWrite(HAL_UART_PORT_0,bdAddr2Str(ownAddress),14);
							HalUARTWrite(HAL_UART_PORT_0,"\r\n",2);
								  
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
							break;
						}
						
						//写入1K数据到flash
						else if(memcmp(&serialBuffer[3],"WRITE1KFLASH",strlen("WRITE1KFLASH")) == 0)
						{
							osal_snv_write(BLE_NVID_1KFLASH,64,&serialBuffer[19]);
							HalUARTWrite(HAL_UART_PORT_0,"+OK\r\n",strlen("+OK\r\n"));
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
							break;
						}
						
						//从flash读取1K数据
						else if(memcmp(&serialBuffer[3],"READ1KFLASH",strlen("READ1KFLASH")) == 0)
						{
							osal_snv_read(BLE_NVID_1KFLASH,64,&serialBuffer);
							HalUARTWrite(HAL_UART_PORT_0,"+OK=",strlen("+OK="));
							HalUARTWrite(HAL_UART_PORT_0,serialBuffer,64);
							HalUARTWrite(HAL_UART_PORT_0,"\r\n",2);
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
							break;
						}
						
						//恢复初始设置
						else if(memcmp(&serialBuffer[3],"SETDEFAULT",strlen("SETDEFAULT")) == 0)
						{
							uint32 default_baud=9600;
							uint8 default_name[16] = "WLT2541";
							uint16 defaut_advInterval=160;
							uint8 default_power=2;
							uint8 default_pair_enable=0;
							uint32 default_pincode=8888;
								  
							osal_snv_write(BLE_NVID_BAUD,4,&default_baud);
							osal_snv_write(BLE_NVID_NAME,16,default_name);
							osal_snv_write(BLE_NVID_ADVI,2,&defaut_advInterval);
							osal_snv_write(BLE_NVID_POWER,1,&default_power);
							osal_snv_write(BLE_NVID_PAIR_ENABLE,1,&default_pair_enable);
							osal_snv_write(BLE_NVID_PINCODE,4,&default_pincode);
								  
							uint8 temp[64];
							memset(temp,0,64);
							osal_snv_write(BLE_NVID_1KFLASH,64,temp);
								  
							SRCRC|=0x20;            //启动看门狗复位
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
							break;
						}
						
						//设置配对
						else if(memcmp(&serialBuffer[3],"SETPAIR",strlen("SETPAIR")) == 0)
						{
							uint8 pair_enable_write,pairMode;
							if(serialBuffer[11]=='1')
							{
								pairMode=GAPBOND_PAIRING_MODE_INITIATE;
								GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );

								pair_enable_write=1;
								osal_snv_write(BLE_NVID_PAIR_ENABLE,1,&pair_enable_write);
							}
							else
							{
								pairMode=GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
								GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );

								pair_enable_write=0;
								osal_snv_write(BLE_NVID_PAIR_ENABLE,1,&pair_enable_write);
							}

							HalUARTWrite(HAL_UART_PORT_0,"+OK\r\n",strlen("+OK\r\n"));
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
							break;
						}
						
						//设置配对码
						else if(memcmp(&serialBuffer[3],"SETPINCODE",strlen("SETPINCODE")) == 0)
						{
							memset(cmd_temp_buf,0,50);
							for(int i=0;i<serialBufferOffset-14;i++)
							{
								cmd_temp_buf[i] = serialBuffer[14+i];
							}

							setPincode=StringNumToUnsignedInteger(cmd_temp_buf);
							GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &setPincode );
								  
							osal_snv_write(BLE_NVID_PINCODE,4,&setPincode);

							HalUARTWrite(HAL_UART_PORT_0,"+OK\r\n",strlen("+OK\r\n"));
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
							break;
						}
						
						//其他指令报错
						else
						{
							HalUARTWrite(HAL_UART_PORT_0,"+ERR=-2\r\n",9);
							memset(serialBuffer,0,100);
							serialBufferOffset = 0;
						}	
					}
					else
					{
						HalUARTWrite(HAL_UART_PORT_0,"+ERR=-1\r\n",9);
						memset(serialBuffer,0,100);
						serialBufferOffset = 0;
					}
				}
				else if(After_wake_up)
				{
					if(serialBufferOffset==10)
					{
						memset(serialBuffer,0,100);
						serialBufferOffset = 0;
						After_wake_up=0;
					}
				}
				break;
				
			//透传模式
			case MODULE_MODE_TRANS:        
				if(numBytes == 1)
				{	
					if(temp_buf[0] == 'a')
					{
						exitTransNum++;
						if(exitTransNum == 3)
						{
							module_mode = MODULE_MODE_ATCMD;
							module_state = MODULE_STATE_CONNECTED;
							exitTransNum = 0;
							HalUARTWrite(HAL_UART_PORT_0,"+IND=ENTER AT CMD MODE\r\n",strlen("+IND=ENTER AT CMD MODE\r\n"));
						}
					}
					else
					{
						exitTransNum = 0;
					}
					for (uint8 i = 0; i < numBytes; i++)
         			{
           				//copy one byte to data buffer
           				dataBuffer[dataBufferOffset] = temp_buf[i];                    
           				//update offset
           				dataBufferOffset = circular_add(dataBufferOffset,1);
         			}
				}
				else
				{
					
					exitTransNum = 0;
					for (uint8 i = 0; i < numBytes; i++)
         			{
           				//copy one byte to data buffer
           				dataBuffer[dataBufferOffset] = temp_buf[i];                    
           				//update offset
           				dataBufferOffset = circular_add(dataBufferOffset,1);
         			}
				}
				break;
			default:
				break;

		}
	    osal_mem_free(temp_buf);
   }
}

uint16 circular_diff(uint16 offset, uint16 tail)
{
  if (offset > tail)
  {
    return (offset - tail);
  }
  else
  {
    return (RX_BUFF_SIZE - tail) + offset;
  }    
}

uint16 circular_add(uint16 x, uint16 y)
{
  uint16 sum = x + y;
  if (sum != RX_BUFF_SIZE)
  {
    sum = sum % RX_BUFF_SIZE;
  }
  else
  {
    sum = 0;
  }
  return sum;
}

unsigned long StringNumToUnsignedInteger(char *StringInteger)
{

	int Index;

	Index = 0;
	int ret_val = 0;
	while(1)
	{
	   /* First check to make sure that this is a valid decimal    */
	   /* digit.												   */
	   if((StringInteger[Index] >= '0') && (StringInteger[Index] <= '9'))
	   {
		  /* This is a valid digit, add it to the value being	   */
		  /* built. 											   */
		  ret_val += (StringInteger[Index] & 0xF);

		  /* Determine if the next digit is valid.				   */
		  if(((Index + 1) < strlen(StringInteger)) && (StringInteger[Index+1] >= '0') && (StringInteger[Index+1] <= '9'))
		  {
			 /* The next digit is valid so multiply the current    */
			 /* return value by 10. 							   */
			 ret_val *= 10;
		  }
		  else
		  {
			 /* The next value is invalid so break out of the loop.*/
			 break;
		  }
	   }
	   else
	   {
	   		return ret_val;
	   }

	   Index++;
	}
	return ret_val;
}