#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

// FreeRTOS Headers
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// C Standard Headers
#include <string.h>
#include <stdio.h>

/**
 *  Minimal assert implementation, so that we can include assert.h
 *
 *  DOES NOT RETURN. Resets the MCU.
 *
 *  @param [in] expr: The expression that failed in assertion
 *  @param [in] file: The filename where the assert occurred
 *  @param [in] line: The line number of the assertion within the file
 */
void __aeabi_assert(const char *expr, const char *file, int line)
{
    while(1);
}

static void config_pin_output_pp(GPIO_TypeDef* port,uint16_t pin )
{
    LL_GPIO_ResetOutputPin(port, pin);

    GPIO_InitTypeDef init = {.Pin = pin,
                            .Mode = GPIO_MODE_OUTPUT_PP,
                            .Pull = GPIO_NOPULL,
                            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
                            .Alternate = 0};
    HAL_GPIO_Init(port,&init);
}

static void config_pin_it_rising(GPIO_TypeDef* port,uint16_t pin )
{
    GPIO_InitTypeDef init = {.Pin = pin,
                            .Mode = GPIO_MODE_IT_RISING,
                            .Pull = GPIO_PULLUP,
                            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
                            .Alternate = 0};
    HAL_GPIO_Init(port,&init);
}

static bool sysclk_driver_enter_sleep(void)
{

    LL_LPM_EnableSleep(); /**< Clear SLEEPDEEP bit of Cortex System Control Register */

    /**
    * This option is used to ensure that store operations are completed
    */
    __force_stores();

    __WFI();
    return true;
}



/******************************************************************************/
// CMSIS AND ST HAL WEAK INTERRUPT FUNCTION FORWARDING
/******************************************************************************/

#define DIFF( x, y ) (((x) >= (y)) ? ( (x) - (y) ) : ( ( 0xFFFF - (y) ) + (x) + 1 ))

//------------------- FREE RTOS configuration --------------------------

extern void xPortSysTickHandler(void);
uint32_t input_clock_rate;
uint16_t auto_reload_value = 0xFFFF;
uint16_t ulTimerCountsForOneTick;

// Variables in LPTIM2 IRQ Handler
volatile uint16_t pending_match_val;
volatile uint16_t last_match_val;
volatile uint16_t now_irq;

// LPTIM2 Timestamping variables
uint16_t vPSTAS_start;
uint16_t temp_match_val;
uint16_t now_match;
uint16_t now_async;
uint16_t now_async_aligned;

uint16_t elapsed_timer_counts;
uint16_t expected_idle_counts;
uint32_t ulCompleteTickPeriods;

// Debug 
//uint16_t elapsed_ticks_buf[100];
//uint16_t async_elapsed_ticks_buf[100];
//uint8_t buf_counter_1 = 0;
//uint8_t buf_counter_2 = 0;
uint16_t now_debug;

uint32_t isr_snapshot_before;
uint32_t isr_snapshot_after;
uint16_t expected_idle_time;

static uint16_t get_lptim_value(void)
{
    uint16_t current_lptim_val;
    uint16_t past_lptim_val;
    bool lptim_val_verified = false;
    
    /* Per ST's comment: When the LPTIM instance is running with an asynchronous clock, reading
    the LPTIMx_CNT register may return unreliable values. So in this case
    it is necessary to perform two consecutive read accesses and verify
    that the two returned values are identical. */
    past_lptim_val = LL_LPTIM_GetCounter(LPTIM2);
    while( !lptim_val_verified )
    {
        current_lptim_val = LL_LPTIM_GetCounter(LPTIM2);
        if( current_lptim_val == past_lptim_val )
        {
            lptim_val_verified = true;
        }
        past_lptim_val = current_lptim_val;
    }
    
    return current_lptim_val;
}

static void set_compare_match(uint16_t match_val)
{
    /* Set compare match */
    LL_LPTIM_SetCompare( LPTIM2, match_val ); 
    
    /* Wait for CMPOK to be set */
    while (!LL_LPTIM_IsActiveFlag_CMPOK(LPTIM2));
    
    /* Immediately clear CMPOK flag */
    LL_LPTIM_ClearFlag_CMPOK(LPTIM2);  
    
    /* Clear the CMP Match interrupt flag */
    LL_LPTIM_ClearFLAG_CMPM(LPTIM2);
}

void LPTIM2_IRQHandler(void)
{   
    // ENC_B_MCU
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1);
    
    /* Clear the perihperal interrupt flag */
    LL_LPTIM_ClearFLAG_CMPM(LPTIM2);
    
    now_irq = get_lptim_value();
    assert(now_irq >= pending_match_val);
    
    last_match_val = pending_match_val;
    pending_match_val += ulTimerCountsForOneTick;
    //LL_LPTIM_SetCompare( LPTIM2, pending_match_val ); 
    set_compare_match(pending_match_val);
    assert(pending_match_val == LL_LPTIM_GetCompare(LPTIM2));
    
    /* TODO + FIXME: This won't work in a tickless-based system */
    /* Increment the system time. */
    //increment_system_time();
    
    /* The FreeRTOS tick handler gets called here */
    xPortSysTickHandler();
    
    // ENC_B_MCU
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);
    
    // Check if next and past compare values are 32 ticks apart
    assert(DIFF(pending_match_val, last_match_val) == 32);
    assert((pending_match_val % 0x20) == 0);
}

// Override the timer setup in FreeRTOS port.c (by enabling
// configOVERRIDE_DEFAULT_TICK_CONFIGURATION in FreeRTOSConfig.h)
// Note that FreeRTOS wants it's tick interrupt to be provided at
// the lowest possible interrupt priority.
void vPortSetupTimerInterrupt( void )
{
    LL_LPTIM_InitTypeDef lptim2_config;

    /* Enable the peripheral clock for the LPTIM2 timer */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_LPTIM2);

    /* Stop and clear the OS timer. */
    LL_LPTIM_Disable(LPTIM2);

    LL_LPTIM_DeInit(LPTIM2);            // Performs a peripheral reset
    LL_LPTIM_StructInit(&lptim2_config);
    LL_LPTIM_Init(LPTIM2, &lptim2_config);

    /* Enable compare match interrupt */
    LL_LPTIM_EnableIT_CMPM(LPTIM2);
    LL_LPTIM_DisableIT_ARRM(LPTIM2);
    
    /* Set up compare match value to be in 1 FREERTOS tick */
    input_clock_rate = LL_RCC_GetLPTIMClockFreq(LL_RCC_LPTIM2_CLKSOURCE);
    
    /* Calculate the number of LPTIM2 Counts per FreeRTOS tick */
    ulTimerCountsForOneTick = (input_clock_rate / configTICK_RATE_HZ);
    
    assert(get_lptim_value() == 0);
    assert(ulTimerCountsForOneTick == 32);
    last_match_val = 0;
    pending_match_val = ulTimerCountsForOneTick;
    
    // Check if next and past compare values are 32 ticks apart
    assert(DIFF(pending_match_val, last_match_val) == 32);

    LL_LPTIM_Enable(LPTIM2);
    
    LL_LPTIM_ClearFlag_CMPOK(LPTIM2);

    //Do both of these on the same side of "Enable"
    LL_LPTIM_SetCompare( LPTIM2, pending_match_val );
    LL_LPTIM_SetAutoReload(LPTIM2, auto_reload_value);
    
    /* LPTIM2_IRQn interrupt configuration, with lowest priority possible */
    HAL_NVIC_SetPriority(LPTIM2_IRQn, ((1 << configPRIO_BITS) - 1) , 0);
    HAL_NVIC_EnableIRQ(LPTIM2_IRQn);

    //Start the count
    LL_LPTIM_StartCounter(LPTIM2, LL_LPTIM_OPERATING_MODE_CONTINUOUS);
}

void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{       
    // nCS_ACCEL
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_5);
    
    /* 2047 ticks x 32 = 65504. This is equivalent to 0xFFE0, or the largest
       tick count that's a multiple of 32 before the LPTIM2 Autoreload occurs. */
    if (xExpectedIdleTime > 2047)
    {
        xExpectedIdleTime = 2047;
    }
    
    /* Calculate the number of LPTIM2 counts from the number of FreeRTOS ticks*/
    expected_idle_counts = xExpectedIdleTime * ulTimerCountsForOneTick;
    
    /* Enter a critical section but don't use the taskENTER_CRITICAL()
    method as that will mask interrupts that should exit sleep mode. */
    __disable_irq();
    __dsb( portSY_FULL_READ_WRITE );
    __isb( portSY_FULL_READ_WRITE );

    /* If a context switch is pending or a task is waiting for the scheduler
    to be unsuspended then abandon the low power entry. */
    if( eTaskConfirmSleepModeStatus() == eAbortSleep )
    {
        /* Re-enable interrupts - see comments above __disable_irq() call
        above. */
        __enable_irq();
        
        // Check if next and past compare values are 32 ticks apart
        assert(DIFF(pending_match_val, last_match_val) == 32);
        return;
    }
 
    // Confirm that the compare value set matches the value in pending_match_val
    assert(LL_LPTIM_GetCompare(LPTIM2) == pending_match_val);
    assert((pending_match_val % 0x20) == 0);
    
    // Get initial LPTIM value as we enter vPortSuppressTicksAndSleep
    vPSTAS_start = get_lptim_value();
    
    // Too close to pending compare match interrupt. Exit vPortSuppressTicksAndSleep    
    if( DIFF(pending_match_val, vPSTAS_start) <= 5)
    {
        __force_stores();
        __WFI();
        __enable_irq();
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_5);

        // Check if next and past compare values are 32 ticks apart
        assert(DIFF(pending_match_val, last_match_val) == 32);
        return;
    }
    
    /* Set next compare match value to be "Expected Idle" LPTIM counts away from the
       past compare match value. */
    temp_match_val = last_match_val + expected_idle_counts;

    set_compare_match(temp_match_val);
    //LL_LPTIM_SetCompare( LPTIM2, temp_match_val );
    now_debug = get_lptim_value();
    
    /* Make sure the value in the Compare Match Register matches the value we THINK we just wrote */
    assert(temp_match_val == LL_LPTIM_GetCompare(LPTIM2));
    assert((pending_match_val % 0x20) == 0);
    
    isr_snapshot_before = LPTIM2->ISR;
    
    /* Enter STOP1 Mode */
    sysclk_driver_enter_sleep();
    
    isr_snapshot_after = LPTIM2->ISR;
    assert((pending_match_val % 0x20) == 0);
    
    /* Determine if the LPTIM2 Compare Match Interrupt fired or if an interrupt other than the LPTIM2 CM
    must have brought the system out of sleep mode. */
    if( LL_LPTIM_IsActiveFlag_CMPM(LPTIM2) )
    {
        // Check to make sure NVIC has a pending LPTIM2 IRQ
        assert( NVIC_GetPendingIRQ(LPTIM2_IRQn) && LL_LPTIM_IsActiveFlag_CMPM(LPTIM2) );
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_9);
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_9);
        
        /* Check if the match occured when we think it should have 
           THIS IS WHERE THE BUG IS. */
        now_match = get_lptim_value();
        assert(DIFF( now_match ,temp_match_val) <= 2);
        
        /* Debugging */
//        elapsed_ticks_buf[buf_counter_1++] =  xExpectedIdleTime;
//        if (buf_counter_1 >= 100)
//        {
//            buf_counter_1 = 0;
//        }
        
        /* Only now update the pending_match_val to the temporary match value */
        pending_match_val = temp_match_val;
        vTaskStepTick( xExpectedIdleTime - 1UL );         
    }
    else
    {
        // ENC_LOW
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14);
        
        /* Get current LPTIM2 count and align it with FreeRTOS Tick (multiple of 32).
           We choose to floor the count to the lower value Tick*/
        now_async = get_lptim_value();
        now_async_aligned = (now_async - (now_async % ulTimerCountsForOneTick));
        
        elapsed_timer_counts = DIFF(now_async_aligned, last_match_val);
        
        /* The complete Tick periods is the elapsed timer counts divided by the number of LPTIM2
           counts per FreeRTOS tick */
        ulCompleteTickPeriods = (elapsed_timer_counts/ulTimerCountsForOneTick);
        
        //debugging
        expected_idle_time = xExpectedIdleTime;
        assert(ulCompleteTickPeriods <= expected_idle_time);
        assert((pending_match_val % 0x20) == 0);

        /* The next compare match is set to the next FreeRTOS tick */
        temp_match_val = now_async_aligned + ulTimerCountsForOneTick;

        if( DIFF(temp_match_val, now_async) <= 4 )
        {
            if ( ulCompleteTickPeriods >= ( xExpectedIdleTime - 1 ) )
            {
                last_match_val = temp_match_val - ulTimerCountsForOneTick;
                pending_match_val = temp_match_val;
                vTaskStepTick( ulCompleteTickPeriods );
                LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14); 
                LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_5);
                __enable_irq();
                
                // Check if next and past compare values are 32 ticks apart
                assert(DIFF(pending_match_val, last_match_val) == 32);
                
                return;
                
            }
            else
            {
                temp_match_val += ulTimerCountsForOneTick;
                ulCompleteTickPeriods += 1;
            }
        }
        
        last_match_val = temp_match_val - ulTimerCountsForOneTick;
        pending_match_val = temp_match_val;
        //LL_LPTIM_SetCompare( LPTIM2, pending_match_val );
        set_compare_match(pending_match_val);
        
//        async_elapsed_ticks_buf[buf_counter_2++] =  ulCompleteTickPeriods;
//        if (buf_counter_2 >= 100)
//        {
//            buf_counter_2 = 0;
//        }
        
        //assert(ulCompleteTickPeriods <= 2047);
        vTaskStepTick( ulCompleteTickPeriods );
        
        // ENC_LOW
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14); 
    }
    
    // nCS_ACCEL
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_5);
          
    /* Exit with interrpts enabled. */
    __enable_irq();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
    while(1) { /* SPIN */ }
}
void vApplicationIdleHook( void )
{
    __WFI();
}

//-------------------- the application ------------------------------------------


void tickless_1_task(void *params)
{
    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(100UL) );
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_0);
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);
    }
}

void tickless_2_task(void *params)
{
    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(90UL) );
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);
    }
}

static QueueHandle_t _task3_event_queue;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    uint8_t dummy;
    if(GPIO_Pin == GPIO_PIN_1)
    {
        //notify task3
        if(_task3_event_queue)
        {
            xQueueSendFromISR(_task3_event_queue, &dummy, &higher_priority_task_woken);
        }
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void tickless_3_task(void *params)
{
    _task3_event_queue = xQueueCreate(10, sizeof(uint8_t ));

    while (1)
    {
        uint8_t dummy;
        BaseType_t res = xQueueReceive(_task3_event_queue, (void *) &dummy, portMAX_DELAY);
        LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_11);
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_11);
    }
}

// ----------------- entry point ----------------------
void tickless_testapp(void)
{
    //Enable clocks needed
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_LPTIM2_CLK_ENABLE();
    
    
    //Initialize pins needed
    config_pin_output_pp(GPIOB, LL_GPIO_PIN_0);
    config_pin_output_pp(GPIOB, LL_GPIO_PIN_1);
    config_pin_output_pp(GPIOB, LL_GPIO_PIN_15);
    config_pin_output_pp(GPIOB, LL_GPIO_PIN_9);
    config_pin_output_pp(GPIOA, LL_GPIO_PIN_1);
    config_pin_output_pp(GPIOC, LL_GPIO_PIN_11);
    config_pin_output_pp(GPIOB, LL_GPIO_PIN_14);
    config_pin_output_pp(GPIOC, LL_GPIO_PIN_5);
    HAL_NVIC_SetPriority(EXTI1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1 , 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    config_pin_it_rising(GPIOD, LL_GPIO_PIN_1);
    

    //Create the FreeRTOS tasks
    xTaskCreate(tickless_1_task, NULL, 256, NULL, configMAX_PRIORITIES - 3, NULL );
    xTaskCreate(tickless_2_task, NULL, 256, NULL, configMAX_PRIORITIES - 3, NULL );
    xTaskCreate(tickless_3_task, NULL, 256, NULL, configMAX_PRIORITIES - 3, NULL );

    __enable_irq();
    // Start FreeRTOS
    vTaskStartScheduler();
}
