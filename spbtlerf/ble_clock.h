/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
* File Name          : ble_clock.h
* Author             : AMS - HEA&RF BU
* Version            : V1.0.1
* Date               : 19-July-2012
* Description        : Header file for clock library, that gives a simple time
*                       reference to the BLE Stack.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifndef __BLE_CLOCK_H__
#define __BLE_CLOCK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef uint32_t tClockTime;
#define CLOCK_SECOND 1000
#define Clock_Init()
tClockTime Clock_Time(void);
void Clock_Wait(tClockTime t);

#ifdef __cplusplus
}
#endif

#endif /* __BLE_CLOCK_H__ */
