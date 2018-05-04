/******************************************************************************

 @file  customprofile.h

 @brief This file contains the Custom Profile definitions

 @date  2018.04.26

 Target Device: CC2540, CC2541

 ******************************************************************************/

#ifndef CUSTOMPROFILE_H
#define CUSTOMPROFILE_H

#ifdef __cplusplus
extern "C"
{
#endif
	
// Profile Parameters
#define CUSTOMPROFILE_CHAR1		0
#define CUSTOMPROFILE_CHAR2 	1
#define CUSTOMPROFILE_CHAR3		2
	
// Custom Profile Service UUID
#define CUSTOMPROFILE_SERV_UUID		0xFFE0
	
// Key Pressed UUID
#define CUSTOMPROFILE_CHAR1_UUID	0xFFE1
#define CUSTOMPROFILE_CHAR2_UUID	0xFFE2
#define CUSTOMPROFILE_CHAR3_UUID	0xFFE3

// Simple Keys Profile Services bit fields
#define CUSTOMPROFILE_SERVICE		0x000000002
	
// Length of Characteristic 3 in bytes
#define CUSTOMPROFILE_CHAR3_LEN           7
	
/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*customProfileChange_t)( uint8 paramID );

typedef struct
{
  	customProfileChange_t        pfnCustomProfileChange;  // Called when characteristic value changes
} customProfileCBs_t;

/*********************************************************************
 * API FUNCTIONS 
 */


/*
 * CustomProfile_AddService- Initializes the Custom Profile service 
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t CustomProfile_AddService( uint32 services );

/*
 * CustomProfile_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t CustomProfile_RegisterAppCBs( customProfileCBs_t *appCallbacks );

/*
 * CustomProfile_SetParameter - Set a Custom Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t CustomProfile_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * CustomProfile_GetParameter - Get a Custom Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t CustomProfile_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

	
#ifdef __cplusplus
}
#endif

#endif