
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"
#include "string.h"

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
void SystemClock_Config(void);

#define WS_LEDS                         28      /*!< Number of leds in a strip row */

uint8_t leds_colors[3 * WS_LEDS];               /*!< Declare array of 3x number of leds (R, G, B colors) */

/**
 * \brief           Temporary array for single LED with extracted PWM duty cycles
 * 
 * We need 24 bytes for PWM setup to send all bits.
 * Before we can send data for first led, we have to send reset pulse, which must be 50us long.
 * PWM frequency is 800kHz, to achieve 50us, we need to send 40 pulses with 0 duty cycle = make array size MAX(24, 40)
 */
uint32_t tmp_led_data[48];

uint8_t is_reset_pulse;                         /*!< Status if we are sending reset pulse or led data */
volatile uint8_t is_updating;                   /*!< Is updating in progress? */
uint32_t current_led;                           /*!< Current LED number we are sending */

uint8_t     ws_update(uint8_t block);

uint8_t     ws_set_color(uint32_t index, uint8_t r, uint8_t g, uint8_t b);
uint8_t     ws_set_color_rgb(uint32_t index, uint32_t rgb);
uint8_t     ws_set_color_all(uint8_t r, uint8_t g, uint8_t b);
uint8_t     ws_set_color_all_rgb(uint32_t rgb);

uint8_t     ws_is_update_finished(void);
uint8_t     ws_start_reset_pulse(uint8_t num);

/**
 * \brief           The application entry point.
 */
int
main(void) {
    size_t i;
    volatile uint32_t timeout;
    
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    LL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM2_Init();

    /* At this point, timer is ready and CC1 is enabled */
    
    /* Infinite loop */
    while (1) {
        for (i = 0; i < WS_LEDS; i++) {
            ws_set_color((i + 0) % WS_LEDS, 0x1F, 0, 0);
            ws_set_color((i + 1) % WS_LEDS, 0x1F, 0, 0);
            ws_set_color((i + 2) % WS_LEDS, 0, 0x1F, 0);
            ws_set_color((i + 3) % WS_LEDS, 0, 0x1F, 0);
            ws_set_color((i + 4) % WS_LEDS, 0, 0, 0x1F);
            ws_set_color((i + 5) % WS_LEDS, 0, 0, 0x1F);
            ws_update(1);
            ws_set_color_all(0, 0, 0);
            
            timeout = 0x7FFFF;
            while (timeout--);
        }
    }
}

/**
 * \brief           Set R,G,B color for specific LED
 * \param[in]       index: LED index in array, starting from `0`
 * \param[in]       r,g,b: Red, Green, Blue values
 * \return          `1` on success, `0` otherwise
 */
uint8_t
ws_set_color(uint32_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index < WS_LEDS) {
        leds_colors[index * 3 + 0] = r;
        leds_colors[index * 3 + 1] = g;
        leds_colors[index * 3 + 2] = b;
        return 1;
    }
    return 0;
}

uint8_t
ws_set_color_rgb(uint32_t index, uint32_t rgb) {
    if (index < WS_LEDS) {
        leds_colors[index * 3 + 0] = (rgb >> 16) & 0xFF;
        leds_colors[index * 3 + 1] = (rgb >> 8) & 0xFF;
        leds_colors[index * 3 + 2] = rgb & 0xFF;
        return 1;
    }
    return 0;
}

uint8_t
ws_set_color_all(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t index;
    for (index = 0; index < WS_LEDS; index++) {
        leds_colors[index * 3 + 0] = r;
        leds_colors[index * 3 + 1] = g;
        leds_colors[index * 3 + 2] = b;
    }
    return 1;
}

uint8_t
ws_set_color_all_rgb(uint32_t rgb) {
    uint32_t index;
    for (index = 0; index < WS_LEDS; index++) {
        leds_colors[index * 3 + 0] = (rgb >> 16) & 0xFF;
        leds_colors[index * 3 + 1] = (rgb >> 8) & 0xFF;
        leds_colors[index * 3 + 2] = rgb & 0xFF;
    }
    return 1;
}

/**
 * \brief           Check if update procedure is finished
 * \return          `1` if not updating, `0` if updating process is in progress
 */
uint8_t
ws_is_update_finished(void) {
    return !is_updating;                        /* Return updating flag status */
}

/**
 * \brief           Start LEDs update procedure
 * \param[in]       block: Set to `1` to block for update process until finished
 * \return          `1` if update started, `0` otherwise
 */
uint8_t
ws_update(uint8_t block) {
    if (is_updating) {                          /* Check if update in progress already */
        return 0;
    }
    is_updating = 1;                            /* We are now updating */

    ws_start_reset_pulse(1);                    /* Start reset pulse */
    if (block) {
        while (!ws_is_update_finished());       /* Wait to finish */
    }
    return 1;
}

/**
 * \brief           Prepares data from memory for PWM output for timer
 * \note            Memory is in format R,G,B, while PWM must be configured in G,R,B
 * \param[in]       ledx: LED index to set the color
 * \param[out]      ptr: Output array with at least 24-words of memory
 */
static uint8_t
ws_fill_led_pwm_data(uint32_t ledx, uint32_t* ptr) {
    size_t i;
    
    if (ledx < WS_LEDS) {
        for (i = 0; i < 8; i++) {
            ptr[i] =        (leds_colors[3 * ledx + 1] & (1 << (7 - i))) ? (2 * TIM2->ARR / 3) : (TIM2->ARR / 3);
            ptr[8 + i] =    (leds_colors[3 * ledx + 0] & (1 << (7 - i))) ? (2 * TIM2->ARR / 3) : (TIM2->ARR / 3);
            ptr[16 + i] =   (leds_colors[3 * ledx + 2] & (1 << (7 - i))) ? (2 * TIM2->ARR / 3) : (TIM2->ARR / 3);
        }
        return 1;
    }
    return 0;
}

/**
 * \brief           Update sequence function, called on each DMA transfer complete or half-transfer complete events
 * \param[in]       tc: Transfer complete flag. Set to `1` on TC event, or `0` on HT event
 *
 * \note            TC = Transfer-Complete event, called at the end of block
 * \note            HT = Half-Transfer-Complete event, called in the middle of elements transfered by DMA
 *                  If block is 48 elements (our case),
 *                      HT is called when first 24 elements are transfered,
 *                      TC is called when second 24 elements are transfered.
 *
 * \note            This function must be called from DMA interrupt
 */
static void
ws_update_sequence(uint8_t tc) {    
    tc = !!tc;                                  /* Convert to 1 or 0 value only */
    
    /* 
     * Check for reset pulse at the end of PWM stream
     */
    if (is_reset_pulse == 2) {                  /* Check for reset pulse at the end */
        LL_TIM_CC_DisableChannel(TIM2, LL_TIM_CHANNEL_CH1); /* Disable channel */
        LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_5);    /* Disable DMA stream */
        
        is_updating = 0;                        /* We are not updating anymore */
        return;
    }
    
    /*
     * Check for reset pulse on beginning of PWM stream
     */
    if (is_reset_pulse == 1) {                  /* Check if we finished with reset pulse */
        /*
         * When reset pulse is active, we have to wait full DMA response,
         * before we can start modifying array which is shared with DMA and PWM
         */
        if (!tc) {                              /* We must wait for transfer complete */
            return;                             /* Return and wait to finish */
        }
        
        /* Disable timer output and disable DMA stream */
        LL_TIM_CC_DisableChannel(TIM2, LL_TIM_CHANNEL_CH1); /* Disable channel */
        LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_5);
        
        is_reset_pulse = 0;                     /* Not in reset pulse anymore */
        current_led = 0;                        /* Reset current led */
    } else {
        /*
         * When we are not in reset mode,
         * go to next led and process data for it
         */
        current_led++;                          /* Go to next LED */
    }
    
    /**
     * This part is used to prepare data for "next" led,
     * for which update will start once current transfer stops in circular mode
     */
    if (current_led < WS_LEDS) {
        /*
         * If we are preparing data for first time (current_led == 0)
         * or if there was no TC event (it was HT):
         *
         *  - Prepare first part of array, because either there is no transfer
         *      or second part (from HT to TC) is now in process for PWM transfer
         *
         * In other case (TC = 1)
         */
        if (current_led == 0 || !tc) {
            ws_fill_led_pwm_data(current_led, &tmp_led_data[0]);
        } else {
            ws_fill_led_pwm_data(current_led, &tmp_led_data[24]);
        }
        
        /*
         * If we are preparing first led (current_led = 0), then:
         * 
         *  - We setup first part of array for first led,
         *  - We have to prepare second part for second led to have one led prepared in advance
         *  - Set DMA to circular mode and start the transfer + PWM output
         */
        if (current_led == 0) {
            current_led++;                      /* Go to next LED */
            ws_fill_led_pwm_data(current_led, &tmp_led_data[24]);   /* Prepare second LED too */
            
            /* Set DMA to circular mode and set length to 48 elements for 2 leds */
            LL_DMA_SetMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MODE_CIRCULAR);  /* Go to non-circular mode */
            LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)tmp_led_data);
            LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, 48);
            
            /* Clear DMA flags */
            LL_DMA_ClearFlag_TC5(DMA1);
            LL_DMA_ClearFlag_HT5(DMA1);
            LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_5);
            LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);
            LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);  /* Enable channel */
        }
        
    /*
     * When we reached all leds, we have to wait to transmit data for all leds before we can disable DMA and PWM:
     *
     *  - If TC event is enabled and we have EVEN number of LEDS (2, 4, 6, ...)
     *  - If HT event is enabled and we have ODD number of LEDS (1, 3, 5, ...)
     */
    } else if ((!tc && (WS_LEDS & 0x01)) || (tc && !(WS_LEDS & 0x01))) {
        LL_TIM_CC_DisableChannel(TIM2, LL_TIM_CHANNEL_CH1); /* Disable channel */
        LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_5);
        
        /* It is time to send final reset pulse, 50us at least */
        ws_start_reset_pulse(2);                /* Start reset pulse at the end */
    }
}

/**
 * \brief           Start reset pulse sequence
 * \param[in]       num: Number indicating pulse is for beginning (1) or end (2) of PWM data stream
 */
static uint8_t
ws_start_reset_pulse(uint8_t num) {
    is_reset_pulse = num;                       /* Set reset pulse flag */
    
    memset(tmp_led_data, 0, sizeof(tmp_led_data));   /* Set all bytes to 0 to achieve 50us pulse */
    
    if (num == 1) {
        tmp_led_data[0] = TIM2->ARR / 2;
    }
    
    /* Set DMA to normal mode, set memory to beginning of data and length to 40 elements */
    /* 800kHz PWM x 40 samples = ~50us pulse low */
    LL_DMA_SetMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);  /* Go to non-circular mode */
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)tmp_led_data);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, 40);

    /* Reset DMA configuration and enable stream */
    LL_DMA_ClearFlag_TC5(DMA1);
    LL_DMA_ClearFlag_HT5(DMA1);
    LL_DMA_DisableIT_HT(DMA1, LL_DMA_STREAM_5);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_5);
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);
    
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);  /* Enable channel for timer */
    LL_TIM_EnableCounter(TIM2);                 /* Start timer counter */
    
    return 1;
}

/**
 * \brief           DMA1 Stream5 global interrupt
 */
void
DMA1_Stream5_IRQHandler(void) {
    if (LL_DMA_IsActiveFlag_HT5(DMA1)) {        /* Check for HT event */
        LL_DMA_ClearFlag_HT5(DMA1);
        ws_update_sequence(0);                  /* Call update sequence as HT event */
    } else if (LL_DMA_IsActiveFlag_TC5(DMA1)) { /* Check for TC event */
        LL_DMA_ClearFlag_TC5(DMA1);
        ws_update_sequence(1);                  /* Call update sequence as TC event */
    } 
}


static void
LL_Init(void) {
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* System interrupt init*/
    /* MemoryManagement_IRQn interrupt configuration */
    NVIC_SetPriority(MemoryManagement_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    /* BusFault_IRQn interrupt configuration */
    NVIC_SetPriority(BusFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    /* UsageFault_IRQn interrupt configuration */
    NVIC_SetPriority(UsageFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    /* SVCall_IRQn interrupt configuration */
    NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    /* DebugMonitor_IRQn interrupt configuration */
    NVIC_SetPriority(DebugMonitor_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    /* PendSV_IRQn interrupt configuration */
    NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    /* SysTick_IRQn interrupt configuration */
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void
SystemClock_Config(void) {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

    if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {
        Error_Handler();  
    }
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
    LL_RCC_HSI_SetCalibTrimming(16);
    LL_RCC_HSI_Enable();

    /* Wait till HSI is ready */
    while (LL_RCC_HSI_IsReady() != 1) {

    }
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 84, LL_RCC_PLLP_DIV_2);
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1) {

    }
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {

    }
    LL_Init1msTick(84000000);
    LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
    LL_SetSystemCoreClock(84000000);
    LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);

    /* SysTick_IRQn interrupt configuration */
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    LL_SYSTICK_EnableIT();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
