/*
 * systick.h
 *
 * @brief  This header file contains the declarations for functions and variables related
 *         to system tick operations, including the millisecond tick counter and time retrieval.
 *
 * @author  Marek Sykorka
 * @date    Nov 18, 2024
 */

#ifndef INC_SYSTICK_H_
#define INC_SYSTICK_H_

#include <stdio.h>
#include <stdint.h>

#define SCS_BASE            (0xE000E000UL)                          /*!< System Control Space Base Address */
#define SysTick_BASE        (SCS_BASE +  0x0010UL)                  /*!< SysTick Base Address */
#define SysTick_VAL_ADDR    (SysTick_BASE + 0x008UL)                /*!< SysTick Current Value Register Address */
#define SysTick_VAL_PTR		(*((uint32_t *)SysTick_VAL_ADDR))		/*!< Pointer to SysTick Current Value Register */

#define SysCoreClock		(64000000U)								/*!< System Core Clock Freq */
#define US_SHIFT			(7U)									/*!< 1000 / (SysCoreClock / 1000) = 1/64 */

typedef struct {
    uint32_t msTicks;
    uint32_t usTicks;
} SystemTime;


void SysTickCountUp(void);
SystemTime LL_GetTick(void);

/**
  * @brief  Retrieves the current system tick count in microseconds.
  * @param  None
  * @retval uint32_t: The current tick count in microseconds, obtained by shifting the
  *         current value of the SysTick counter register to the right.
  * @note   This function assumes that the SysTick counter value is updated at a frequency
  *         that allows for precise microsecond timing, based on the system clock configuration.
  */
static inline uint32_t LL_usGetTick(void)
{
	return SysTick_VAL_PTR >> US_SHIFT;
}

#endif /* INC_SYSTICK_H_ */
