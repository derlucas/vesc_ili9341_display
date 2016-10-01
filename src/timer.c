#include "timer.h"
#include "cortexm/ExceptionHandlers.h"


// Forward declarations.
void timer_tick(void);

void timer_init() {
    TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;
    TIM_TimeBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBase_InitStructure.TIM_Period = 999;
    TIM_TimeBase_InitStructure.TIM_Prescaler = 71;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBase_InitStructure);
}


volatile timer_ticks_t timer_delayCount;
volatile timer_ticks_t systicks;

void timer_start(void) {
    // Use SysTick as reference for the delay loops.
    SysTick_Config(SystemCoreClock / TIMER_FREQUENCY_HZ);
}

void timer_sleep(timer_ticks_t ticks) {
    timer_delayCount = ticks;

    // Busy wait until the SysTick decrements the counter to zero.
    while (timer_delayCount != 0u)
        ;
}

void timer_tick(void) {
    // Decrement to zero the counter used by the delay routine.
    if (timer_delayCount != 0u) {
        --timer_delayCount;
    }
}

// ----- SysTick_Handler() ----------------------------------------------------

void SysTick_Handler(void) {
    timer_tick();
    systicks++;
}

