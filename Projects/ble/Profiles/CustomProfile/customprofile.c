/******************************************************************************

 @file  customprofile.c

 @brief This file contains the Custom Profile

 @date  2018.04.26

 Target Device: CC2540, CC2541

 ******************************************************************************/

#include <string.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"
#include "hal_uart.h"

#include "customprofile.h"

#define CUSTOMSERVAPP_NUM_ATTR_SUPPORTED 	11

// Custom Profile Service UUID: 0xFFE0
CONST uint8 customProfileServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(CUSTOMPROFILE_SERV_UUID), HI_UINT16(CUSTOMPROFILE_SERV_UUID)
};

// Characteristic 1 UUID: 0xFFE1
CONST uint8 customProfilechar1UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(CUSTOMPROFILE_CHAR1_UUID), HI_UINT16(CUSTOMPROFILE_CHAR1_UUID)
};

// Characteristic 2 UUID: 0xFFE2
CONST uint8 customProfilechar2UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(CUSTOMPROFILE_CHAR2_UUID), HI_UINT16(CUSTOMPROFILE_CHAR2_UUID)
};

// Characteristic 3 UUID: 0xFFE3
CONST uint8 customProfilechar3UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(CUSTOMPROFILE_CHAR3_UUID), HI_UINT16(CUSTOMPROFILE_CHAR3_UUID)
};


static customProfileCBs_t *customProfile_AppCBs = NULL;


// Profile Attributes - variables

// Custom Profile Service attribute
static CONST gattAttrType_t customProfileService = { ATT_BT_UUID_SIZE, customProfileServUUID };


// Custom Profile Characteristic 1 Properties
static uint8 customProfileChar1Props = GATT_PROP_WRITE_NO_RSP;	//0x04 prop

// Characteristic 1 Value
static uint8 customProfileChar1[CUSTOMPROFILE_CHAR1_LEN] = {0};

// Custom Profile Characteristic 1 User Description
static uint8 customProfileChar1UserDesp[17] = "Char1 WR NO RSP";

// Custom Profile Characteristic 2 Properties
static uint8 customProfileChar2Props = GATT_PROP_NOTIFY;

// Characteristic 2 Value
static uint8 customProfileChar2 = 0;

// Custom Profile Characteristic 2 Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t *customProfileChar2Config;

// Custom Profile Characteristic 2 User Description
static uint8 customProfileChar2UserDesp[17] = "Char2 Noti";

// Custom Profile Characteristic 3 Properties
static uint8 customProfileChar3Props = GATT_PROP_READ;

// Characteristic 3 Value
static uint8 customProfileChar3[CUSTOMPROFILE_CHAR3_LEN] = {0,0,0,0,0,0,0};

// Custom Profile Characteristic 3 User Description
static uint8 customProfileChar3UserDesp[17] = "Char3 RD STR";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t customProfileAttrTbl[CUSTOMSERVAPP_NUM_ATTR_SUPPORTED] = 
{
  	// Custom Profile Service
  	{ 
    	{ ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    	GATT_PERMIT_READ,                         /* permissions */
    	0,                                        /* handle */
    	(uint8 *)&customProfileService            /* pValue */
  	},

    // Characteristic 1 Declaration
    { 
      	{ ATT_BT_UUID_SIZE, characterUUID },
      	GATT_PERMIT_READ, 
      	0,
      	&customProfileChar1Props 
    },

    // Characteristic Value 1
    { 
        { ATT_BT_UUID_SIZE, customProfilechar1UUID },
        GATT_PERMIT_WRITE, 
        0, 
        customProfileChar1 
    },

   	// Characteristic 1 User Description
  	{ 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        customProfileChar1UserDesp
  	},      

    // Characteristic 2 Declaration
    { 
      	{ ATT_BT_UUID_SIZE, characterUUID },
      	GATT_PERMIT_READ, 
      	0,
      	&customProfileChar2Props 
    },

   	// Characteristic Value 2
   	{ 
        { ATT_BT_UUID_SIZE, customProfilechar2UUID },
        0, 
        0, 
        &customProfileChar2
   	},

  	// Characteristic 2 configuration
  	{ 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&customProfileChar2Config 
  	},
      
   	// Characteristic 2 User Description
  	{ 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        customProfileChar2UserDesp 
 	},
      
    // Characteristic 3 Declaration
    { 
      	{ ATT_BT_UUID_SIZE, characterUUID },
      	GATT_PERMIT_READ, 
      	0,
      	&customProfileChar3Props 
    },

  	// Characteristic Value 3
 	{ 
        { ATT_BT_UUID_SIZE, customProfilechar3UUID },
        GATT_PERMIT_READ, 
        0, 
        customProfileChar3
 	},

  	// Characteristic 3 User Description
   	{ 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        customProfileChar3UserDesp 
 	},
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t customProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                                           uint8 *pValue, uint8 *pLen, uint16 offset,
                                           uint8 maxLen, uint8 method );
static bStatus_t customProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint8 len, uint16 offset,
                                            uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Custom Profile Service Callbacks
CONST gattServiceCBs_t customProfileCBs =
{
  	customProfile_ReadAttrCB,  // Read callback function pointer
  	customProfile_WriteAttrCB, // Write callback function pointer
  	NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      customProfile_AddService
 *
 * @brief   Initializes the custom Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t CustomProfile_AddService( uint32 services )
{
  uint8 status;
  
  //char2 notify
  // Allocate Client Characteristic Configuration table
  customProfileChar2Config = (gattCharCfg_t *)osal_mem_alloc( sizeof(gattCharCfg_t) *
                                                              linkDBNumConns );
  if ( customProfileChar2Config == NULL )
  {     
    return ( bleMemAllocError );
  }
  
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, customProfileChar2Config );
  
  if ( services & CUSTOMPROFILE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( customProfileAttrTbl, 
                                          GATT_NUM_ATTRS( customProfileAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &customProfileCBs );
  }
  else
  {
    status = SUCCESS;
  }
  
  return ( status );
}

/*********************************************************************
 * @fn      customProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t CustomProfile_RegisterAppCBs( customProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    customProfile_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
 * @fn      customProfile_SetParameter
 *
 * @brief   Set a custom Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t CustomProfile_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case CUSTOMPROFILE_CHAR1:
      if ( len <= CUSTOMPROFILE_CHAR1_LEN ) 
      {
        VOID memcpy( customProfileChar1, value, len );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case CUSTOMPROFILE_CHAR2:
      if ( len == sizeof ( uint8 ) ) 
      {
        customProfileChar2 = *((uint8*)value);
        
        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( customProfileChar2Config, &customProfileChar2, FALSE,
                                    customProfileAttrTbl, GATT_NUM_ATTRS( customProfileAttrTbl ),
                                    INVALID_TASK_ID, customProfile_ReadAttrCB );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case CUSTOMPROFILE_CHAR3:
      if ( len == CUSTOMPROFILE_CHAR3_LEN ) 
      {
        VOID memcpy( customProfileChar3, value, CUSTOMPROFILE_CHAR3_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      customProfile_GetParameter
 *
 * @brief   Get a custom Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t CustomProfile_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case CUSTOMPROFILE_CHAR1:
      VOID memcpy( value, customProfileChar1, CUSTOMPROFILE_CHAR1_LEN );
      break; 

    case CUSTOMPROFILE_CHAR2:
      *((uint8*)value) = customProfileChar2;
      break;

    case CUSTOMPROFILE_CHAR3:
      VOID memcpy( value, customProfileChar3, CUSTOMPROFILE_CHAR3_LEN );
      break;      
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          customProfile_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t customProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                                           uint8 *pValue, uint8 *pLen, uint16 offset,
                                           uint8 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      // characteristics 3 have read permissions
      // characteritisc 1 does not have read permissions; therefore it is not
      //   included here
      // characteristic 2 does not have read permissions, but because it
      //   can be sent as a notification, it is included here
      case CUSTOMPROFILE_CHAR2_UUID:
        *pLen = 1;
        pValue[0] = *pAttr->pValue;
        break;
		
	  case CUSTOMPROFILE_CHAR3_UUID:
	    *pLen = CUSTOMPROFILE_CHAR3_LEN;
        VOID memcpy( pValue, pAttr->pValue, CUSTOMPROFILE_CHAR3_LEN );
        break;
        
      default:
        // Should never get here! (characteristics 3 and 4 do not have read permissions)
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      customProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t customProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint8 len, uint16 offset,
                                            uint8 method )
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  
  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case CUSTOMPROFILE_CHAR1_UUID:

        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len > CUSTOMPROFILE_CHAR1_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
		//Write the value
        if ( status == SUCCESS )
        {
          	uint8 *pCurValue = (uint8 *)pAttr->pValue;        
          	VOID memset(pCurValue, 0, CUSTOMPROFILE_CHAR1_LEN);
			VOID memcpy(pCurValue, pValue, len);
			//
			//send to uart
		  	HalUARTWrite(HAL_UART_PORT_0,pValue,len);
/*
          if( pAttr->pValue == &customProfileChar1 )
          {
            notifyApp = CUSTOMPROFILE_CHAR1;        
          }
          else
          {
            notifyApp = CUSTOMPROFILE_CHAR2;           
          }
*/
        }
             
        break;

      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;
        
      default:
        // Should never get here! (characteristics 2 and 4 do not have write permissions)
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && customProfile_AppCBs && customProfile_AppCBs->pfnCustomProfileChange )
  {
    customProfile_AppCBs->pfnCustomProfileChange( notifyApp );  
  }
  
  return ( status );
}

/*********************************************************************
*********************************************************************/



