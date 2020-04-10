/******************************************************************************
* File Name          : LcdTask.c
* Date First Issued  : 04/05/2020
* Description        : LCD display 
*******************************************************************************/

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include "malloc.h"
#include "stm32f4xx_hal.h"
#include "LcdTask.h"
#include "lcd_hd44780_i2c.h"
#include "morse.h"

enum LCD_STATE
{
	LCD_IDLE,
	LCD_ROW,
	LCD_COL,
	LCD_CHAR
};

osThreadId LcdTaskHandle = NULL;

static I2C_HandleTypeDef* phi2c;	// Local copy of uart handle for LCD

/* Queue */
#define QUEUESIZE 32	// Total size of bcb's other tasks can queue up
osMessageQId LcdTaskQHandle;

/* Linked list head. */
static struct LCDI2C_UNIT* phdunit = NULL;   // Units instantiated; last = NULL
/* *************************************************************************
 * struct LCDI2C_UNIT* xLcdTaskcreateunit(I2C_HandleTypeDef* phi2c, 
 *    uint8_t address, 
 *    uint8_t numrow, 
 *    uint8_t numcol);
 *	@brief	: Instantiate a LCD unit on I2C peripheral and address
 * @param	: phi2c = pointer to I2C handle
 * @param	: address = I2C bus address
 * @param	: numrow = number of LCD rows
 * @param	: numcol = number of LCD columns
 * @return	: NULL = fail, otherwise pointer to unit on linked list
 * *************************************************************************/
struct LCDI2C_UNIT* xLcdTaskcreateunit(I2C_HandleTypeDef* phi2c, 
    uint8_t address, 
    uint8_t numrow, 
    uint8_t numcol)
{
	struct LCDI2C_UNIT* punit;
	struct LCDI2C_UNIT* ptmp;
	struct LCDPARAMS* pu1;


taskENTER_CRITICAL();
	/* Check if this I2C bus & address (i.e. unit) is already present. */
	punit = phdunit; // Get pointer to first item on list
	if (punit == NULL)
	{ // Linked list is empty, so add first unit
		punit = (struct LCDI2C_UNIT*)calloc(1, sizeof(struct LCDI2C_UNIT));
		if (punit == NULL) morse_trap(230);
		phdunit = punit;  // Head points to first entry
	}
	else
	{ // Here, one or more on list.
		/* Search list for this I2C-address */
		while (punit != NULL)
		{
			if ((punit->phi2c == phi2c) && (punit->address == address))
			{ // Here this I2C-address is already on list.
				morse_trap(231); // ### ERROR: Duplicate ###
			}
			punit = punit->pnext;
		}
		/* Here, one or more is on list, but not this one. */
		ptmp = punit;
		punit = (struct LCDI2C_UNIT*)calloc(1, sizeof(struct LCDI2C_UNIT));
		if (punit == NULL) morse_trap(236);
		ptmp->pnext    = punit;  // Previous last entry points this new entry
	}

	/* Populate the control block for this unit. */
	punit->phi2c   = phi2c;   // HAL I2C Handle for this bus
	punit->address = address; // I2C bus address (not shifted)

	/* Set start, end, and working pointers for circular buffer of pointers. */
	punit->ppbegin = &punit->pcb[0];
	punit->ppadd   = &punit->pcb[0];
	punit->pptake  = &punit->pcb[0];
	punit->ppend   = &punit->pcb[LCDCIRPTRSIZE];

	/* The remainder of LCDPARAMS will be initialized in the lcdInit() below. */
	punit->lcdparams.lines    = numrow;  // number of LCD rows
	punit->lcdparams.columns  = numcol;  // number of LCD columns

taskEXIT_CRITICAL();

	/* Complete the initialization the LCD unit */
	pu1 = lcdInit(punit);
	if (pu1 == NULL) morse_trap(237);

	return punit;
}
/* *************************************************************************
 * struct LCDI2C_LINEBUF* xLcdTaskintgetbuf(struct LCDI2C_UNIT* p);
 * @brief	: Get a buffer for a LCD on I2C peripheral, and address
 * @param	: phi2c = pointer to I2C handle
 * @param	: address = I2C bus address
 * @return	: NULL = fail, otherwise pointer to buffer struct
 * *************************************************************************/
struct LCDI2C_LINEBUF* xLcdTaskintgetbuf(struct LCDI2C_UNIT* p)
{
	struct LCDI2C_LINEBUF* plb;
	struct LCDI2C_UNIT* punit;
	
taskENTER_CRITICAL();
	if ( p == NULL) morse_trap(232); 

	/* Look up unit. */
	punit = phdunit;
	while (punit != NULL)
	{
		/* Check for I2C bus and address match. */
		if ((punit->phi2c == p->phi2c) && (punit->address == p->address))
		{ // Here found. Get struct for this LCD line buffer
			plb = (struct LCDTASK_LINEBUF*)calloc(1,sizeof(struct LCDTASK_LINEBUF));
			if (plb == NULL) morse_trap(233);

			// Pointer to lcd unit on linked list
			plb->punit = punit;

			// Get char buffer for up to max chars this unit displays. */
			plb->pbuf = (uint8_t*)calloc((punit->lcdparams.lines * punit->lcdparams.columns),sizeof(uint8_t));

			// Create buffer semaphore
			plb->semaphore = xSemaphoreCreateBinary(); // Semaphore for this buffer
			if (plb->semaphore == NULL) morse_trap(234);
			xSemaphoreGive(plb->semaphore); // Initialize

			taskEXIT_CRITICAL();
			return plb;
		}
		punit = punit->pnext;
	}
	/* Not found. Can this really happen? */
	morse_trap(235);

taskEXIT_CRITICAL();
	return plb;
}
/* *************************************************************************
 * void StartLcdTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
void StartLcdTask(void* argument1)
{
	BaseType_t Qret;	// queue receive return

	struct LCDTASK_LINEBUF* pssb; // Copied item from queue
	struct LCDI2C_UNIT* punit;				
	struct LCDI2C_UNIT* ptmp;				

  /* Infinite loop */
  for(;;)
  {
		osDelay(6); // Polling Q & timing states

		do // new queued items in unit circular buffers.
		{
			Qret = xQueueReceive( LcdTaskQHandle,&pssb,0);
			if (Qret == pdPASS)
			{ // Here we have a queued item
				ptmp = pssb->punit;  // Point to "unit" struct
				*ptmp->ppadd = pssb; // Add pointer to circular buffer
				ptmp->ppadd += 1;    // Advance working pointer
				if (ptmp->ppadd == ptmp->ppend) ptmp->ppadd = ptmp->ppbegin;
			}
		} while (Qret == pdPASS);

		/* Check for active units and timing of states. */
		punit = phdunit;
		while (punit != NULL)
		{
			if (punit->state == LCD_IDLE)
			{
				if (punit->take != punit->add)
				{ // Here start a new buffer sequence
				}
			}
			else
			{ // State check
				

			}	
			punit = punit->pnext;
		}
	}
}
/* #######################################################################
   I2C interrupt callback: file|size has been sent
   ####################################################################### */
#ifdef HALCALLBACKUSE
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
    HAL_I2C_Master_Transmit_IT(p2->hi2c, p2->address, (uint8_t*)p2->lcdCommandBuffer, 6) != HAL_OK) 
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	return;
}
#endif
/* *************************************************************************
 * osThreadId xLcdTaskCreate(uint32_t taskpriority, uint16_t numbcb);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @param	: numbcb = number of buffer control blocks to allocate
 * @return	: LcdTaskHandle
 * *************************************************************************/
osThreadId xLcdTaskCreate(uint32_t taskpriority, uint16_t numbcb)
{
	BaseType_t ret = xTaskCreate(StartLcdTask, "LcdTask",\
     192, NULL, taskpriority,\
     &LcdTaskHandle);
	if (ret != pdPASS) return NULL;

	LcdTaskQHandle = xQueueCreate(numbcb, sizeof(struct LCDTASK_LINEBUF*) );
	if (LcdTaskQHandle == NULL) return NULL;

	return LcdTaskHandle;
}
/* **************************************************************************************
 * int lcdprintf_init(void);
 * @brief	: 
 * @return	: 
 * ************************************************************************************** */
struct LCDTASKBCB* lcdprintf_init(void)
{
	yprintf_init();	// JIC not init'd
	return;
}
/* **************************************************************************************
 * int yprintf(struct LCDTASK_LINEBUF** ppbcb, const char *fmt, ...);
 * @brief	: 'printf' for uarts
 * @param	: pbcb = pointer to pointer to stuct with info
 * @param	: format = usual printf format
 * @param	: ... = usual printf arguments
 * @return	: Number of chars "printed"
 * ************************************************************************************** */
int lcdprintf(struct LCDTASK_LINEBUF** ppbcb, int row, int col, const char *fmt, ...)
{
	struct LCDTASKBCB* pbcb = *ppbcb;
	va_list argp;

	/* Block if this buffer is not available. SerialSendTask will 'give' the semaphore 
      when the buffer has been sent. */
	xSemaphoreTake(pbcb->semaphore, 6001);

	/* Block if vsnprintf is being uses by someone else. */
	xSemaphoreTake( vsnprintfSemaphoreHandle, portMAX_DELAY );

	/* Construct line of data.  Stop filling buffer if it is full. */
	va_start(argp, fmt);
	pbcb->size = vsnprintf((char*)(pbcb->pbuf+2),pbcb->bufsize, fmt, argp);
	va_end(argp);

	/* Line to send has two leading control/command bytes. */
	pbcb->size += 2; // Adjust size

	/* Limit byte count in BCB to be put on queue, from vsnprintf to max buffer sizes. */
	if (pbcb->size > pbcb->bufsize) 
			pbcb->size = pbcb->bufsize;

	/* Set row & column codes */
	uint8_t* p = pbcb->pbuf;
	*p++ = (254); // move cursor command

	// determine position
	if (row == 0) {
		*p = (128 + col);
	} else if (row == 1) {
		*p = (192 + col);
	} else if (row == 2) {
		*p = (148 + col);
	} else if (row == 3) {
		*p = (212 + col);
	}

	/* Release semaphore controlling vsnprintf. */
	xSemaphoreGive( vsnprintfSemaphoreHandle );

	/* JIC */
	if (pbcb->size == 0) return 0;

	/* Place Buffer Control Block on queue to SerialTaskSend */
	vSerialTaskSendQueueBuf(ppbcb); // Place on queue

	return pbcb->size;
}



