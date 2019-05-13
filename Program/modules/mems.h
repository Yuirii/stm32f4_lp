/**************************************************************************************************
  Filename:       MEMS.h
  Revised:        $Date: 2018-11-11  $
  Revision:       $Revision: ck  $

**************************************************************************************************/

#ifndef MEMS_H
#define MEMS_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "drivers\stm32f4_system.h"
#include "stdint.h"
  
/*********************************************************************
 * TYPEDEFS
 */
typedef void (*MemsActiveCB_t)(uint8_t pinID);
typedef enum {FALSE = 0,TRUE = 1} bool;//增添布尔型数据

/*********************************************************************
 * CONSTANTS
 */
 
#define  MEMS_SADW    0x4e 
#define  MEMS_SADR    0x4F

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

bool MemsOpen(void);
void MemsClose(void);
void MemsLowPwMode(void);
void MemsLowPwMgr(void);
bool Mems_ActivePin_Enable(MemsActiveCB_t memsActiveCB);
void Mems_ActivePin_Disable(void);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SX1278_H */
