#include "main.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

/**
 * \brief           Number of elements in statically allocated array
 */
#define ARRAYSIZE(x)                    (sizeof(x) / sizeof((x)[0]))

/**
 * \brief           Number of LEDs (or LED drivers in case of WS2811) connected to strip
 *
 * Set this value for your use case
 */
#define LED_CFG_COUNT                           240 /* 20 */

/**
 * \brief           Number of bytes for one LED color description
 */
#define LED_CFG_BYTES_PER_LED                   3

/**
 * \brief           Defines how many times is "1-led-transmission-time" sent before DMA TC or HT interrupts are triggered
 * 
 * \note            `1 unit` represents `30us` at `800 kHz and 24-bit of LED data`
 * 
 * This value defines:
 * - Greater it is, less frequent DMA interrupts get fired
 * - Greater it is, lower is the reset-sequence time granularity
 * - Greater it is, larger is the raw dma buffer as it has to accomodate for 2*value number of elements for led's data
 * 
 * \note            Values should be greater-or-equal than `1` and (advise) not higher than `8˙
 * 
 * \note            Value set to `6` means DMA TC or HT interrupts are triggered at each `180us at 800kHz tim update rate`
 */
#define LED_CFG_LEDS_PER_DMA_IRQ                6

/**
 * \brief           Defines minimum number of "1-led-transmission-time" blocks
 *                  to send logical `0` to the bus, indicating reset before data transmission starts.
 * 
 * \note            Few conditions apply to the value, there must be AND match
 *                      - Must be multiplier of \ref LED_CFG_LEDS_PER_DMA_IRQ
 *                      - Must be at least `2 * LED_CFG_LEDS_PER_DMA_IRQ` because first led data starts only after first DMA TC transfer
 *                      - Must be at least value to support minimum reset time, `300us`
 */
#define LED_RESET_PRE_MIN_IRQ_COUNT             (2 * LED_CFG_LEDS_PER_DMA_IRQ)

/**
 * \brief           Number of "1-led-transmission-time" units to send all zeros
 *                  after last led has been sent out
 *
 * \note            Few conditions apply to the value, there must be AND match
 *                      - Must be multiplier of \ref LED_CFG_LEDS_PER_DMA_IRQ
 */
#define LED_RESET_POST_MIN_IRQ_COUNT            (1 * LED_CFG_LEDS_PER_DMA_IRQ)

/**
 * \brief           Application array to store led colors
 *                      Data in format R,G,B,R,G,B,... 
 * 
 * Array used by user application to store data to
 */
static uint8_t leds_color_data[LED_CFG_BYTES_PER_LED * LED_CFG_COUNT];

/**
 * \brief           This buffer acts as raw buffer for DMA to transfer data from memory to timer compare
 * \note            DMA must have access to this variable (memory location)
 * 
 * It is a multi-dimentional array:
 * - First part represents 2 parts, one before half-transfer, second after half-transfer complete
 * - Second part is number of LEDs to transmit before DMA HT/TC interrupts get fired
 * - Third part are entries for raw timer compare register to send logical 1 and 0 to the bus
 * 
 * Type of variable should be unsigned 32-bit, to satisfy TIM2 that is 32-bit timer
 */
static uint32_t dma_raw_buffer[2][LED_CFG_LEDS_PER_DMA_IRQ][LED_CFG_BYTES_PER_LED * 8];

/* Control variables for transfer */
static volatile uint8_t     is_updating = 0;            /* Set to `1` when update is in progress */
static volatile uint32_t    irq_count;                  /* Counts how many "1-led-time" have been transmitted so far */
static volatile uint8_t     brightness = 0xFF;          /* Brightness between 0 and 0xFF */
static volatile uint32_t    color_counter = 1;          /* Color, being increased each fade reaching 0 */

/* Application variables for fading effect */
int8_t      fade_step;
int16_t     fade_value;

/* Start data transfer */
static uint8_t  led_start_transfer(void);
void            SystemClock_Config(void);
static void     tim2_init(void);
static void     gpio_init(void);
static void     led_fill_led_pwm_data(size_t ledx, uint32_t* ptr);

/* Debug control pins */
#define DBG_PIN_UPDATING_HIGH                   LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_0)
#define DBG_PIN_UPDATING_LOW                    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_0)
#define DBG_PIN_IRQ_HIGH                        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1)
#define DBG_PIN_IRQ_LOW                         LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1)

/**
 * \brief           Calculate Bezier curve (ease-in, ease-out) for input value
 * 
 * \note            This is experimental function, bezier is not approproate curve
 *                  to fade-in-out LEDs
 *
 * \param[in]       t: Input value between 0 and 1, indicating start to stop position
 * \return          Value between 0 and 1, respecting minimum and maximum
 */
float
bezier_calc(float t) {
    return t * t * (3.0f - 2.0f * t);
}

/**
 * \brief           Calculate cub of an input for lightning approx curve
 * 
 * \param[in]       t: Input value between 0 and 1, indicating start to stop position
 * \return          Value between 0 and 1, respecting minimum and maximum
 */
float
quad_calc(float t) {
    return t * t * t * t;
}

/**
 * \brief           The application entry point
 */
int
main(void) {
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    /* Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral */
    LL_SYSCFG_DisableDBATT(LL_SYSCFG_UCPD1_STROBE);

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    tim2_init();
    gpio_init();

    /* Set up the first leds with dummy color */
    for (size_t i = 0; i < LED_CFG_COUNT; ++i) {
        leds_color_data[i * LED_CFG_BYTES_PER_LED + 0] = i * 0x10;
        leds_color_data[i * LED_CFG_BYTES_PER_LED + 1] = i * 0x10;
        leds_color_data[i * LED_CFG_BYTES_PER_LED + 2] = i * 0x10;
    }

    /* Define fade init values */
    fade_step = 0x02;
    fade_value = 0;

    /* Enable systick interrupt generation */
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

    /*
     * Everything is happening in SysTick_Handler function,
     * defined later in this file
     */

    /* Infinite loop */
    while (1) {}
    return 0;
}

/**
 * \brief           Update sequence function,
 *                  called on each DMA transfer complete or half-transfer complete events
 * \param[in]       tc: Set to `1` when function is called from DMA TC event, `0` when from HT event
 */
static void
led_update_sequence(uint8_t tc) {
    DBG_PIN_IRQ_HIGH;
    tc = !!tc;
    if (!is_updating) {
        DBG_PIN_IRQ_LOW;
        return;
    }

    /*
     * Each time this function gets called,
     * this value indicates number of "1-led-time" sequences to be transmitted
     */
    irq_count += LED_CFG_LEDS_PER_DMA_IRQ;

    if (irq_count < LED_RESET_PRE_MIN_IRQ_COUNT) {
        /* We are still in reset sequence mode */
    } else if (irq_count < (LED_RESET_PRE_MIN_IRQ_COUNT + LED_CFG_COUNT)) {
        /*
         * This is where LED data gets prepared to be transmitted in the next cycle
         *
         * It is important to have irq_count aligned with minimum reset cycle
         */
        uint32_t next_led = irq_count - LED_RESET_PRE_MIN_IRQ_COUNT;
#if LED_CFG_LEDS_PER_DMA_IRQ == 1
        led_fill_led_pwm_data(next_led, &dma_raw_buffer[tc][0][0]);
#endif /* LED_CFG_LEDS_PER_DMA_IRQ == 1 */
#if LED_CFG_LEDS_PER_DMA_IRQ > 1
        uint32_t counter = 0;
        for (; counter < LED_CFG_LEDS_PER_DMA_IRQ && next_led < LED_CFG_COUNT; ++counter, ++next_led) {
            led_fill_led_pwm_data(next_led, &dma_raw_buffer[tc][counter][0]);
        }
        if (counter < LED_CFG_LEDS_PER_DMA_IRQ) {
            memset(&dma_raw_buffer[tc][counter][0], 0x00, (LED_CFG_LEDS_PER_DMA_IRQ - counter) * sizeof(dma_raw_buffer[0][0]));
        }
#endif /* LED_CFG_LEDS_PER_DMA_IRQ > 1 */
    } else if (irq_count <= (LED_RESET_PRE_MIN_IRQ_COUNT + LED_CFG_COUNT + LED_RESET_POST_MIN_IRQ_COUNT)) {
        /*
         * This is post-reset circuitry and must be set to at least 1 level in size
         *
         * It sends all-zero to the all leds
         */
        memset(&dma_raw_buffer[tc], 0x00, sizeof(dma_raw_buffer[0]));
    } else {
        /*
         * We are now ready to stop DMA and TIM channel,
         * otherwise transfers will continue to occur (circular mode).
         *
         * Disable interrupts prior disabling DMA transfer.
         *
         * Some STM32 (F2, F4, F7) may generate TC or HT interrupts when DMA is manually disabled,
         * and since it is not necessary to receive these interrupts from now-on,
         * it's better to simply disable them
         */
        LL_DMA_DisableIT_TC(DMA2, LL_DMA_CHANNEL_5);
        LL_DMA_DisableIT_HT(DMA2, LL_DMA_CHANNEL_5);
        LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_5);
        LL_TIM_CC_DisableChannel(TIM2, LL_TIM_CHANNEL_CH4);
        is_updating = 0;
        DBG_PIN_UPDATING_LOW;
    }
    DBG_PIN_IRQ_LOW;
}

/**
 * \brief           DMA2 channel 5 interrupt handler
 */
void
DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQHandler(void) {
    if (LL_DMA_IsEnabledIT_HT(DMA2, LL_DMA_CHANNEL_5) && LL_DMA_IsActiveFlag_HT5(DMA2)) {
        LL_DMA_ClearFlag_HT5(DMA2);
        led_update_sequence(0);
    }
    if (LL_DMA_IsEnabledIT_TC(DMA2, LL_DMA_CHANNEL_5) && LL_DMA_IsActiveFlag_TC5(DMA2)) {
        LL_DMA_ClearFlag_TC5(DMA2);
        led_update_sequence(1);
    }
}

/**
 * \brief           Prepares data from memory for PWM output for timer
 * \note            Memory is in format R,G,B, while PWM must be configured in G,R,B[,W]
 * \param[in]       ledx: LED index to set the color
 * \param[out]      ptr: Output array with at least LED_CFG_RAW_BYTES_PER_LED-words of memory
 */
static void
led_fill_led_pwm_data(size_t ledx, uint32_t* ptr) {
    const uint32_t arr = TIM2->ARR + 1;
    const uint32_t pulse_high = (3 * arr / 4) - 1;
    const uint32_t pulse_low = (1 * arr / 4) - 1;

    if (ledx < LED_CFG_COUNT) {
        uint32_t r, g, b;

        r = (uint8_t)(((uint32_t)leds_color_data[ledx * LED_CFG_BYTES_PER_LED + 0] * (uint32_t)brightness) / (uint32_t)0xFF);
        g = (uint8_t)(((uint32_t)leds_color_data[ledx * LED_CFG_BYTES_PER_LED + 1] * (uint32_t)brightness) / (uint32_t)0xFF);
        b = (uint8_t)(((uint32_t)leds_color_data[ledx * LED_CFG_BYTES_PER_LED + 2] * (uint32_t)brightness) / (uint32_t)0xFF);
        for (size_t i = 0; i < 8; i++) {
            ptr[i] =        (g & (1 << (7 - i))) ? pulse_high : pulse_low;
            ptr[8 + i] =    (r & (1 << (7 - i))) ? pulse_high : pulse_low;
            ptr[16 + i] =   (b & (1 << (7 - i))) ? pulse_high : pulse_low;
        }
    }
}

/**
 * \brief           Start with transfer process to update LED on the strip
 * \return          `1` if transfer has started, `0` otherwise
 */
static uint8_t
led_start_transfer(void) {
    if (is_updating) {
        return 0;
    }

    /* Set initial values */
    is_updating = 1;
    irq_count = LED_CFG_LEDS_PER_DMA_IRQ;

    /* Set all bytes to 0 to achieve start reset pulse = all low */
    memset(dma_raw_buffer, 0x00, sizeof(dma_raw_buffer));

    /*
     * This is where DMA gets configured for data transfer
     *
     * - Circular mode, continuous transmission
     * - Memory length (number of elements) for 2 LEDs of data
     */
    LL_DMA_SetMode(DMA2, LL_DMA_CHANNEL_5, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_CHANNEL_5, (uint32_t)dma_raw_buffer);
    LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_5, sizeof(dma_raw_buffer) / sizeof(dma_raw_buffer[0][0][0]));

    /* Clear flags, enable interrupts */
    LL_DMA_ClearFlag_TC5(DMA2);
    LL_DMA_ClearFlag_HT5(DMA2);
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_5);
    LL_DMA_EnableIT_HT(DMA2, LL_DMA_CHANNEL_5);

    /* Enable DMA, TIM channel and TIM counter */
    LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_5);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4);
    LL_TIM_EnableCounter(TIM2);

    DBG_PIN_UPDATING_HIGH;

    /* All the rest is happening in DMA interrupt from now on */
    return 1;
}

/**
 * \brief           TIM2 Initialization Function
 */
static void
tim2_init(void) {
    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

    /*
     * TIM2 GPIO Configuration
     *
     * PA3   ------> TIM2_CH4
     */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* DMA interrupts */
    NVIC_SetPriority(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn);

    /* DMA setup */
    LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_5, LL_DMAMUX_REQ_TIM2_CH4);
    LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetChannelPriorityLevel(DMA2, LL_DMA_CHANNEL_5, LL_DMA_PRIORITY_MEDIUM);
    LL_DMA_SetMode(DMA2, LL_DMA_CHANNEL_5, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_CHANNEL_5, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_CHANNEL_5, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA2, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_WORD);
    LL_DMA_SetMemorySize(DMA2, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_WORD);
    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_CHANNEL_5, (uint32_t)&TIM2->CCR4);

    /* 
     * Set basic timer settings
     * 
     * - Set no prescaler for max resolution
     * - Set autoreload for 800kHz refresh rate (64 MHz TIM2 kernel clock)
     */
    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 79;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM2, &TIM_InitStruct);
    LL_TIM_EnableARRPreload(TIM2);
    LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM2);

    /* Channel settings */
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = 0;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH4);
    LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH4);

    /* Enable channel DMA requests */
    LL_TIM_EnableDMAReq_CC4(TIM2);
    LL_TIM_OC_SetCompareCH4(TIM2, 0);
}

/**
 * \brief           Initialize debug GPIOs
 */
static void
gpio_init(void) {
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

    /*
     * GPIO configuration
     *
     * PA0   ------> Pin set to high when transmission is on-going
     * PA1   ------> Pin toggled on each DMA interrupt
     * PA2   ------> Reserved for future use
     */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * \brief           This function handles System tick timer
 */
void
SysTick_Handler(void) {
    static uint32_t counter = 0;

    /* Perform strip color update every 10 ms */
    if (++counter % 10 == 0) {
        if (is_updating) {
            return;
        }
#if 0
        /* Calculate fading efect */
        fade_value += fade_step;
        if (fade_value > 0xFF) {
            fade_value = 0xFF;
            fade_step = -fade_step;
        } else if (fade_value < 0) {
            fade_value = 0;
            fade_step = -fade_step;
        }

        /* Check if we need to change the color */
        if (fade_value == 0) {
            for (size_t i = 0; i < LED_CFG_COUNT; ++i) {
                leds_color_data[i * LED_CFG_BYTES_PER_LED + 0] = color_counter & 0x01 ? 0xFF : 0x00;
                leds_color_data[i * LED_CFG_BYTES_PER_LED + 1] = color_counter & 0x02 ? 0xFF : 0x00;
                leds_color_data[i * LED_CFG_BYTES_PER_LED + 2] = color_counter & 0x04 ? 0xFF : 0x00;
            }
            color_counter++;
        }

        /* Calculate new brightness */
        brightness = (uint8_t)(quad_calc((float)fade_value / (float)0xFF) * (float)0x3F);
#endif

        /* Start data transfer in non-blocking mode */
        led_start_transfer();
    }
}

/**
 * \brief           System Clock Configuration
 */
void
SystemClock_Config(void) {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {}

    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while (!LL_RCC_HSI_IsReady()) {}

    /* Main PLL configuration and activation */
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_Enable();
    LL_RCC_PLL_EnableDomain_SYS();
    while (!LL_RCC_PLL_IsReady()) {}

    /* Set AHB prescaler */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

    /* Sysclk activation on the main PLL */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {}

    /* Set APB1 prescaler */
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_Init1msTick(64000000);

    /* Enable SysTick interrupt */
    NVIC_SetPriority(SysTick_IRQn, 3);
    NVIC_EnableIRQ(SysTick_IRQn);

    /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
    LL_SetSystemCoreClock(64000000);
}

/**
 * \brief           This function is executed in case of error occurrence
 */
void
Error_Handler(void) {
    __disable_irq();
    while (1) {

    }
}