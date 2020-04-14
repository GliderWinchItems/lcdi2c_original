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
//#include "yprintf.h"

//osSemaphoreId vsnprintfSemaphoreHandle;
#define NOYPRINTFISPRESENT
#ifdef NOYPRINTFISPRESENT
SemaphoreHandle_t vsnprintfSemaphoreHandle;
#endif

enum LCD_STATE
{
	LCD_IDLE,
	LCD_SETRC,
	LCD_ROW,
	LCD_COL,
	LCD_CHR
};

#define DELAY_ROW 1; // Delay for row 
#define DELAY_COL 1; // Delay for col
#define DELAY_CHR 1; // Delay following each char
#define DELAY_SETRC 20; // Delay following set cursor row column

osThreadId LcdTaskHandle = NULL;

//static I2C_HandleTypeDef* phi2c;	// Local copy of uart handle for LCD

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
	punit->lcdparams.numrows  = numrow;  // number of LCD rows
	punit->lcdparams.numcols  = numcol;  // number of LCD columns

	punit->state = LCD_IDLE; // Start off in idle (after final intialization)

taskEXIT_CRITICAL();

	/* Complete the initialization the LCD unit */
	int tmp = lcdInit(punit);
	if (tmp != 0) morse_trap(237);

	return punit;
}
/* *************************************************************************
 * struct LCDTASK_LINEBUF* xLcdTaskintgetbuf(struct LCDI2C_UNIT* p);
 * @brief	: Get a buffer for a LCD on I2C peripheral, and address
 * @param	: phi2c = pointer to I2C handle
 * @param	: address = I2C bus address
 * @return	: NULL = fail, otherwise pointer to buffer struct
 * *************************************************************************/
struct LCDTASK_LINEBUF* xLcdTaskintgetbuf(struct LCDI2C_UNIT* p)
{
	struct LCDTASK_LINEBUF* plb = NULL;
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

			/* Save max size of buffer allocated. */
			plb->bufmax = (punit->lcdparams.lines * punit->lcdparams.columns);

			// Get char buffer for up to max chars this unit displays. */
			plb->pbuf = (uint8_t*)calloc(plb->bufmax,sizeof(uint8_t));
			if (plb->pbuf == NULL) morse_trap(236);

			// Create buffer semaphore
			plb->semaphore = xSemaphoreCreateBinary(); // Semaphore for this buffer
			if (plb->semaphore == NULL) morse_trap(234);
			xSemaphoreGive(plb->semaphore); // Initialize

			taskEXIT_CRITICAL();
			return plb;
		}
		punit = punit->pnext;
	}
	/* Here: Likely this unit never got instantiated. */
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

while(1==1) osDelay(10);

	BaseType_t Qret;	// queue receive return

	struct LCDTASK_LINEBUF* pssb; // Copied item from queue
	struct LCDTASK_LINEBUF* plbtmp;
	struct LCDI2C_UNIT* punit;				
	struct LCDI2C_UNIT* ptmp;
	TickType_t ttx;

  /* Infinite loop */
  for(;;)
  {
		osDelay(4); // Polling Q & timing states

		do // new queued items in unit circular buffers.
		{
			Qret = xQueueReceive( LcdTaskQHandle,&pssb,0);
			if (Qret == pdPASS)
			{ // Here we have a queued item
				ptmp = pssb->punit;  // Point to "unit" struct
				*ptmp->ppadd = pssb; // Add pointer to circular buffer
				ptmp->ppadd += 1;    // Advance "add" pointer
				if (ptmp->ppadd == ptmp->ppend) ptmp->ppadd = ptmp->ppbegin;

				if (ptmp->state == LCD_IDLE)
				{ // Here, this LCD unit is idle, so begin.
					// Set cursor position
					lcdSetCursorPosition(&ptmp->lcdparams, pssb->columns, pssb->lines);

					// Get time to end delay after sending row/column
    				ptmp->untiltickct = xTaskGetTickCount() + DELAY_SETRC; 
    				ptmp->state = LCD_CHR; // Next state
				}
			}
		} while (Qret == pdPASS);

		/* Check for active units and timing of states. */
		punit = phdunit;
		while (punit != NULL)
		{
			ttx = xTaskGetTickCount();
			plbtmp = *punit->pptake;
			switch (punit->state)
			{
			case LCD_IDLE: // Waiting for app to load queue
				if (punit->pptake != punit->ppadd)
				{ // Here start a new buffer sequence
					lcdSetCursorPosition(&punit->lcdparams, pssb->columns, pssb->lines);

					// Get time to end delay after sending row/column
	   				punit->untiltickct = ttx + DELAY_SETRC; 
  					punit->state = LCD_CHR; // Next state
				}
				break;

			case LCD_SETRC: // Waiting for set cursor
				if ((int)(ttx - punit->untiltickct) >= 0)
				{
					plbtmp->pbuf += 1;
   	    			lcdPrintStr(&punit->lcdparams,plbtmp->pbuf, 1);
   					punit->untiltickct = ttx + DELAY_COL; 
   					punit->state = LCD_CHR;
				}
				break;

			case LCD_CHR:
				// Wait for time expiration after sending a display char
				if ((int)(ttx - punit->untiltickct) >= 0)
				{
					plbtmp->pbuf += 1;
					plbtmp->size -= 1;
					if (plbtmp->size > 0)
					{ // Continue sending
   	    				lcdPrintStr(&punit->lcdparams,plbtmp->pbuf, 1);
    					punit->untiltickct = ttx + DELAY_CHR; 
   	    			}
   	    			else
   	    			{ // Here, buffer has been sent (or last char is sending)

						// Release buffer just sent so it can be reused. 
						xSemaphoreGive( plbtmp->semaphore );

   	    				// Advance ptr to circular buffer of ptrs
   	    				punit->pptake += 1;
   	    				if (punit->pptake == punit->ppend) 
   	    					punit->pptake = punit->ppbegin;

   	    				// Any more buffers waiting?
   	    				if (punit->pptake == punit->ppadd)
   	    				{ // No. We are done!
   	    					punit->state = LCD_IDLE;	
   	    				}
   	    				else
   	    				{ // Start next buffer
							lcdSetCursorPosition(&punit->lcdparams, pssb->columns, pssb->lines);

   							punit->untiltickct = ttx + DELAY_SETRC; 
   							punit->state = LCD_CHR;
   	    				}    	    				
   	    			}
				}
				break;
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
	
    if (HAL_I2C_Master_Transmit_IT(p2->hi2c, p2->address, (uint8_t*)p2->lcdCommandBuffer, 6) != HAL_OK) 
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
                          256, NULL, taskpriority, LcdTaskHandle);
	if (ret != pdPASS) return NULL;

	LcdTaskQHandle = xQueueCreate(numbcb, sizeof(struct LCDTASK_LINEBUF*) );
	if (LcdTaskQHandle == NULL) return NULL;

#ifdef NOYPRINTFISPRESENT
	vsnprintfSemaphoreHandle = xSemaphoreCreateMutex();
#endif
	return LcdTaskHandle;
}
/* **************************************************************************************
 * int lcdprintf_init(void);
 * @brief	: 
 * @return	: 
 * ************************************************************************************** */
//{
//	yprintf_init();	// JIC not init'd
//	return;
//}
/* **************************************************************************************
 * int lcdi2cprintf(struct LCDTASK_LINEBUF** pplb, int row, int col, const char *fmt, ...);
 * @brief	: 'printf' for uarts
 * @param	: pblb = pointer to pointer to line buff struct
 * @param   : row = row number (0-n) to position cursor
 * @param   : col = column number (0-n) to position cursor
 * @param	: format = usual printf format
 * @param	: ... = usual printf arguments
 * @return	: Number of chars "printed"
 * ************************************************************************************** */
int lcdi2cprintf(struct LCDTASK_LINEBUF** pplb, int row, int col, const char *fmt, ...)
{
	struct LCDTASK_LINEBUF* plb = *pplb;
	va_list argp;

	/* Block if this buffer is not available. SerialSendTask will 'give' the semaphore 
      when the buffer has been sent. */
	xSemaphoreTake(plb->semaphore, 6001);

	plb->lines   = row;
	plb->columns = col;

	/* Block if vsnprintf is being uses by someone else. */
	xSemaphoreTake( vsnprintfSemaphoreHandle, portMAX_DELAY );

	/* Construct line of data.  Stop filling buffer if it is full. */
	va_start(argp, fmt);
	plb->size = vsnprintf((char*)plb->pbuf,plb->bufmax, fmt, argp);
	va_end(argp);

	/* Limit byte count to be put on queue, from vsnprintf to max buffer sizes. */
	if (plb->size > plb->bufmax) 
			plb->size = plb->bufmax;

	/* Release semaphore controlling vsnprintf. */
	xSemaphoreGive( vsnprintfSemaphoreHandle );

	/* JIC */
	if (plb->size == 0) return 0;

	/* Place Buffer Control Block pointer on queue to LcdTask */
	xQueueSendToBack(LcdTaskQHandle, pplb, 0);

	return plb->size;
}
/* **************************************************************************************
 * int lcdi2cputs(struct LCDTASK_LINEBUF** pplb, int row, int col, char* pchr);
 * @brief	: Send zero terminated string to SerialTaskSend
 * @param	: pbcb = pointer to pointer to stuct with uart pointers and buffer parameters
 * @param   : row = row number (0-n) to position cursor
 * @param   : col = column number (0-n) to position cursor
 * @return	: Number of chars sent
 * ************************************************************************************** */
int lcdi2cputs(struct LCDTASK_LINEBUF** pplb, int row, int col, char* pchr)
{
	struct LCDTASK_LINEBUF* plb = *pplb;
	int sz = strlen(pchr); // Check length of input string
	if (sz == 0) return 0;

	/* Block (for a while) if this buffer is not yet available. 
      when the buffer has been sent. */
	xSemaphoreTake(plb->semaphore, 1024);

	// Save cursor position
	plb->lines   = row; 	
	plb->columns = col;

	strncpy((char*)plb->pbuf,pchr,plb->bufmax);	// Copy and limit size.

	/* Set size sent. */
	if (sz >= plb->bufmax)	// Did strcpy truncate?
		plb->size = plb->bufmax;	// Yes
	else
		plb->size = sz;	// No

	/* Place pointer to Buffer Control Block pointer on queue to LcdTask */
	xQueueSendToBack(LcdTaskQHandle, pplb, 0);
	return plb->size; 
}