/******************************************************************************
* File Name          : LcdTask.h
* Date First Issued  : 04/05/2020
* Description        : LCD display 
*******************************************************************************/

#ifndef __LCDTASK
#define __LCDTASK

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "semphr.h"
#include "lcd_hd44780_i2c.h"

#define LCDCIRPTRSIZE 8	// Size of LCD circular buffer of pointers

typedef struct LCDI2C_UNIT punit; // Avoid circular definition error
/* I2C Line buffer */
struct LCDTASK_LINEBUF
{
	struct LCDI2C_UNIT* punit;  // Point to linked list entry for this I2C:address unit
	SemaphoreHandle_t semaphore;// Semaphore handle
	uint8_t* pbuf;              // Pointer to byte buffer to be sent
	uint8_t size;               // Number of bytes to be sent
	uint8_t lines;              // Row start
    uint8_t columns;            // Column start
};

typedef struct LCDPARAMS lcdparams;

/* Linked list of entries for each I2C:address (i.e. LCD units) */
struct LCDI2C_UNIT
{
	struct LCDI2C_UNIT* pnext; // Next bus unit on this I2C bus
	I2C_HandleTypeDef* phi2c;  // HAL I2C Handle for this bus
	struct LCDTASK_LINEBUF** ppbegin; // Pointer to circular buffer of line buffer pointers
	struct LCDTASK_LINEBUF** ppend;   // Pointer to circular buffer of line buffer pointers
	struct LCDTASK_LINEBUF** ppadd;   // Pointer to circular buffer of line buffer pointers
	struct LCDTASK_LINEBUF** pptake;  // Pointer to circular buffer of line buffer pointers
	struct LCDTASK_LINEBUF* pcb[LCDCIRPTRSIZE];

	struct LCDPARAMS lcdparams;
	uint8_t address;  // Not-shifted address
	uint8_t state;    // State machine
	uint8_t delayctr; // Working counter for delays
	uint8_t ctr;      // Delay counter
};

/* *************************************************************************/
 void vStartLcdTask(void);
/*	@brief	: Task startup
 * *************************************************************************/
osThreadId xLcdTaskCreate(uint32_t taskpriority, uint16_t numbcb);
/* @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @param	: numbcb = number of buffer control blocks to allocate for queue
 * @return	: LcdTaskHandle
 * *************************************************************************/
struct LCDI2C_UNIT* xLcdTaskcreateunit(I2C_HandleTypeDef* phi2c, 
    uint8_t address, 
    uint8_t numrow, 
    uint8_t numcol);
/*	@brief	: Instantiate a LCD unit on I2C peripheral and address
 * @param	: phi2c = pointer to I2C handle
 * @param	: address = I2C bus address
 * @param	: numrow = number of LCD rows
 * @param	: numcol = number of LCD columns
 * @return	: NULL = fail, otherwise pointer to unit on linked list
 * *************************************************************************/
struct LCDI2C_LINEBUF* xLcdTaskintgetbuf(struct LCDI2C_UNIT* p);
/* @brief	: Get a buffer for a LCD on I2C peripheral, and address
 * @param	: phi2c = pointer to I2C handle
 * @param	: address = I2C bus address
 * @return	: NULL = fail, otherwise pointer to buffer struct
 * *************************************************************************/



extern osMessageQId LcdTaskQHandle;
extern osThreadId   LcdTaskHandle;

/* Macros for simplifying the wait and loading of the queue */
#define mLcdTaskWait( noteval, bit){while((noteval & bit) == 0){xTaskNotifyWait(bit, 0, &noteval, 5000);}}
#define mLcdTaskQueueBuf(bcb){uint32_t qret;do{qret=xQueueSendToBack(LcdTaskQHandle,bcb,portMAX_DELAY);}while(qret == errQUEUE_FULL);}

#define mLcdTaskQueueBuf2(bcb,bit){uint32_t qret;do{noteval&=~bit;qret=xQueueSendToBack(LcdTaskQHandle,bcb,portMAX_DELAY);}while(qret == errQUEUE_FULL);}

#endif

