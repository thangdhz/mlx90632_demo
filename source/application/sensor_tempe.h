/*******************************************************************************
 *
 * Copyright (c) 2021
 * thangdh92@gmail.com
 * All Rights Reserved
 *
 *
 * Description:
 *
 * Author: 
 *
 * Last Changed By:  $ Author:  $
 * Revision:         $ Revision:  $
 * Last Changed:     $ Date:  $
 *
 ******************************************************************************/
#ifndef __SENSOR_TEMPERATURE_H
#define __SENSOR_TEMPERATURE_H
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "scommon.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

void sensor_tempe_init(void);

int32_t sensor_tempe_read_celciusx100();

#endif // __SENSOR_TEMPERATURE_H
