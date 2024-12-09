/*
 * systick.c
 *
 * @brief  This file contains functions for handling system tick operations, including
 *         incrementing the system tick counter and retrieving the current tick count
 *         in both milliseconds and microseconds.
 *
 * @author  Marek Sykorka
 * @date    Nov 18, 2024
 */

#include "systick.h"

volatile uint32_t LL_TickCounter = 0;

/**
  * @brief  Increments the millisecond system tick counter.
  * @param  None
  * @retval None
  * @note   This function is typically called in a SysTick interrupt handler to update the
  *         millisecond tick counter periodically.
  */
void SysTickCountUp(void)
{
	LL_TickCounter++;
}

/**
  * @brief  Retrieves the current system tick count in both milliseconds and microseconds.
  * @param  None
  * @retval SystemTime structure containing:
  *         - msTicks: the current tick count in milliseconds.
  *         - usTicks: the current tick count in microseconds.
  */
SystemTime LL_GetTick(void)
{
	SystemTime time;
    time.msTicks = LL_TickCounter;
    time.usTicks = LL_usGetTick();
	return time;
}
