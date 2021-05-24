/** \file insertIntoMain.c
 * Central hub of user-defined functions.
 * To be #include "" into the STM32CubeMX generated main.c file.
 */
/*
 * Copyright (c)
 *
 *     Meson Dynamics
 *
 * All rights reserved.
 */
#include "cmdinterp.h"
/** usb receive buffer */
extern volatile uint8_t usb_recv_buf[]; // extern must declare array here.
/** Processor clock and timer clock frequency. */
#define STM32_FCLK 48000000
/** PWM frequency. */
#define STM32_PWM_FREQ 40000
/** PWM counter max value.  Equal is not allowed. */
#define STM32_PWM_CNT_MAX (STM32_FCLK/STM32_PWM_FREQ)
/** ADC DMA buffer. */
#define ADC_BUF_LEN 72*72 /* should be multiple of 32. */
static volatile __attribute__((section(".dma_buf")))
ALIGN_32BYTES(uint32_t adc_buf[ADC_BUF_LEN]);
/* Aligned to 32-byte (0x20) is important for SCB_InvalidateDCache_by_Addr() to work. */
/** DAC DMA data buffer. */
#define DAC_BUF_LEN 32
static volatile __attribute__((section(".dma_buf")))
ALIGN_32BYTES(uint16_t dac_buf[DAC_BUF_LEN]);  /* 12-bit, right(lsb)-aligned. */
const uint16_t dac_sine_wave[DAC_BUF_LEN] = {
    2047, 2446, 2831, 3185, 3495, 3749, 3939, 4055, 4095, 4055, 3939, 3749, 3495, 3185, 2831, 2446, 2047, 1648, 1263, 909, 599, 345, 155, 39, 0, 39, 155, 345, 599, 909, 1263, 1648
};

/*oo00OO00oo GPIO input oo00OO00oo<*/
static volatile int button_pushed = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
    case USER_Btn_Pin: /* GPIOC, GPIO_PIN_13 in STM32H743ZITx NUCLEO-LQFP144 */
        button_pushed = 1;
        if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)) {
            // __HAL_TIM_ENABLE(&htim4);
            // TIM4->CR1 |= (TIM_CR1_CEN);
        } else {
            // TIM4->CR1 &= ~(TIM_CR1_CEN);
        }
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
        if (button_pushed) {
            GPIOE->BSRR = GPIO_PIN_10 << 16; /* Clear (zero) output. */
            while ((TIM8->CNT & 0xffff)>2) {;
                /* Sync TIM4_CEN to TIM8 more deterministically. */
                /* !=0 seems to hang forever. */
                /* This seems to reliably align TIM4 outputs to the rising edge of pixel clock. */
            }
            TIM4->CR1 |= (TIM_CR1_CEN);
            button_pushed = 0;
        }
        break;
    case GPIO_PIN_7:
    default:
        // GPIOE->BSRR = GPIO_PIN_10; /* Set high. */
        break;
    }
}
/*oo00OO00oo GPIO input oo00OO00oo>*/

/*oo00OO00oo Timer oo00OO00oo<*/
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    int i;
    if (htim == &htim4) {
        /* Speak already high, which is an error condition.  But the
         * following test doesn't seem to work. */
        /*
        if (GPIOD->ODR & GPIO_PIN_13) {
            return;
        }
        */
        for (i=0; i<40; i++)
            asm("nop");

        GPIOE->BSRR = GPIO_PIN_10;
        /* Start ADC/DMA acquisition. */
    }
}
/*oo00OO00oo Timer oo00OO00oo<*/

/*oo00OO00oo PWM control oo00OO00oo<*/
void stm32_pwm_update_ccr(uint16_t v)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, v);
}
/*oo00OO00oo PWM control oo00OO00oo>*/

/*oo00OO00oo ADC oo00OO00oo<*/
void adc_init()
{
    // Calibration must be done when ADC is disabled.
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }
    /*
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN) != HAL_OK) {
        Error_Handler();
    }
    */
/* Start ADC1 in interrupt mode. */
//    HAL_ADC_Start_IT(&hadc1);
}
/** ADC values are ready.
 * This one is called after DMA has transferred the data.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    SCB_InvalidateDCache_by_Addr((uint32_t *)(&adc_buf[ADC_BUF_LEN/2]), sizeof(adc_buf)/2);

    int i;
    static int j;
    j++;
    // if(j%5000==0) {
    // if (hadc->Instance == ADC1) {
    /*
    printf("val:");
    for(i=0; i<4; i++) {
        printf(" %4lu", adc_buf[i]);
    }
    printf("\n");
    */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
//    }
/* Read in interrupt mode, then restart. */
//    ADCvalues[0] = HAL_ADC_GetValue(hadc1);
//    HAL_ADC_Start_IT(hadc1);
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
    SCB_InvalidateDCache_by_Addr((uint32_t *)adc_buf, sizeof(adc_buf)/2);
}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    /*
    printf("ADC %p, ErrorCode: 0x%02lx.\n", hadc->Instance, hadc->ErrorCode);
    printf("HAL_ADC_ERROR_OVR: 0x%02x, HAL_ADC_ERROR_DMA: 0x%02x\n",
           HAL_ADC_ERROR_OVR, HAL_ADC_ERROR_DMA);
    */
    if (hadc->ErrorCode == HAL_ADC_ERROR_DMA)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}
/*oo00OO00oo ADC oo00OO00oo>*/

/*oo00OO00oo DAC oo00OO00oo<*/
void dac_init()
{
    int i;
    for (i=0; i<DAC_BUF_LEN; i++)
        dac_buf[i] = dac_sine_wave[i];

    /* TIM runs at 200MHz */
    /* After HAL_TIM_Base_Init(&htim6), one has to directly manipulate registers. */
    TIM6->PSC = 0; /* counter freq = f / (PSC + 1). */
    TIM6->ARR = 199; /* i.e. period is val+1 */

    HAL_TIM_Base_Start(&htim6);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,
                      (uint32_t*)dac_sine_wave, DAC_BUF_LEN, DAC_ALIGN_12B_R);
}

/*oo00OO00oo DAC oo00OO00oo>*/

/*oo00OO00oo Command interpreter oo00OO00oo<*/
static int cmdinterp_nextc(void *s)
{
    int c = getchar();
    putchar(c);
    return c;
}
static cmdinterp_data_t cmdinterp_cmderr(cmdinterp_t *hdl)
{
    int i;
    cmdinterp_data_t ret = {1};
    printf("\ncmd: %s\n", hdl->cmd);
    for(i=0; i<hdl->nparam; i++) {
        switch(hdl->paramdt[i]) {
        case CMDINTERP_DTYPE_CHAR:
            printf("Param %2d: %c\n", i, hdl->param[i].c);
            break;
        case CMDINTERP_DTYPE_INT:
            printf("Param %2d: %d\n", i, hdl->param[i].i);
            break;
        case CMDINTERP_DTYPE_FLOAT:
            printf("Param %2d: %f\n", i, hdl->param[i].f);
            break;
        }
    }
    printf("msg: %s\n", hdl->msg);
    return ret;
}
static cmdinterp_data_t cmdinterp_adc(cmdinterp_t *hdl)
{
    cmdinterp_data_t ret = {1};
    uint16_t v = (uint16_t)hdl->param[0].i;

    volatile uint16_t *val = (uint16_t*)adc_buf;
    printf("\n");
    int i, j, k=0;
    switch (v) {
    case 0:
        for (j=0; j<23; j++) {
            for (i=0; i<16; i++)
                printf("%04x ", val[k++]);
            printf("\n");
        }
        break;
    case 1:
        for (j=0; j<ADC_BUF_LEN*sizeof(adc_buf[0])/2; j++)
            printf("%-6d\n", val[k++]);
        break;
    }
    return ret;
}
static cmdinterp_data_t cmdinterp_chc(cmdinterp_t *hdl)
{
    cmdinterp_data_t ret = {1};
    int ch = hdl->param[0].i;
    int c = hdl->param[1].i;
    GPIO_PinState s;
    if(hdl->nparam < 2) {
        printf("\nCh and Stat expected.\n");
        ret.i = 0;
        return ret;
    }
    if(c) {s = GPIO_PIN_SET;} else {s = GPIO_PIN_RESET;}
    switch(ch) {
    case 0:
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, s);
        break;
    case 1:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, s);
        break;
    default:
        printf("\nPin/Value specified out of range.\n");
        ret.i = 0;
        return ret;
        break;
    }
    printf("\nPin %d (0==LED) set to %d.\n", ch, c);
    return ret;
}
static cmdinterp_data_t cmdinterp_drv(cmdinterp_t *hdl)
{
    cmdinterp_data_t ret = {1};
    int c = hdl->param[0].i;
    GPIO_PinState s;
    if (c) {s = GPIO_PIN_SET;} else {s = GPIO_PIN_RESET;}
    if (hdl->nparam < 1) {
        printf("\nDrvEN is %d\n", HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1));
        return ret;
    }
    switch (c) {
    case 1:
        /*
          if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN) != HAL_OK) {
          Error_Handler();
          }
        */
        if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUF_LEN) != HAL_OK) {
            Error_Handler();
        }
        break;
    case 0:
    default:
        if (HAL_ADCEx_MultiModeStop_DMA(&hadc1) != HAL_OK) {
            Error_Handler();
        }
        break;
    }

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, s);
    printf("\nDrvEN set to %d.\n", c);
    return ret;
}
static cmdinterp_data_t cmdinterp_pwm(cmdinterp_t *hdl)
{
    cmdinterp_data_t ret = {1};
    uint16_t v = (uint16_t)hdl->param[0].i;
    if(hdl->nparam < 1) {
        printf("\npwm value is expected.\n");
        ret.i = 0;
        return ret;
    }
    if(v>=STM32_PWM_CNT_MAX) {
        printf("\npwm (%u) greater than STM32_PWM_CNT_MAX (%u)!", v, STM32_PWM_CNT_MAX);
        v = 0;
    }
    printf("\npwm value set to %u.\n", v);
    stm32_pwm_update_ccr(v);
    return ret;
}
static cmdinterp_data_t cmdinterp_statq(cmdinterp_t *hdl)
{
    cmdinterp_data_t ret = {1};

    printf("\nLED (LD2): %d\n",
           HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1));

    /* Temporary variable to retrieve RCC clock configuration */
    RCC_ClkInitTypeDef clk_init_struct = {0};
    /* Temporary variable to retrieve Flash Latency */
    uint32_t latency;
    /* Timer clock frequency */
    uint32_t timer_clock_frequency = 0;
    /* Retrieve timer clock source frequency */
    HAL_RCC_GetClockConfig(&clk_init_struct, &latency);
    /* If APB1 prescaler is different of 1, timers have a factor x2 on their    */
    /* clock source.                                                            */
    if (clk_init_struct.APB1CLKDivider == RCC_HCLK_DIV1) {
        timer_clock_frequency = HAL_RCC_GetPCLK1Freq();
    } else {
        timer_clock_frequency = HAL_RCC_GetPCLK1Freq() *2;
    }
    printf("APB1 PCLK1 freq = %ld, timer freq = %ld\n",
           HAL_RCC_GetPCLK1Freq(), timer_clock_frequency);
    printf("TIM4_CR1: 0x%04lx, TIM4_CR2: 0x%08lx, TIM4_SMCR: 0x%08lx, TIM4_SR: 0x%08lx\n",
           TIM4->CR1, TIM4->CR2, TIM4->SMCR, TIM4->SR);

    fflush(stdout);
    return ret;
}
static cmdinterp_data_t cmdinterp_tim(cmdinterp_t *hdl)
{
    cmdinterp_data_t ret = {1};
    int id = (int)hdl->param[0].i;

    switch (id) {
    case 1:
        // TIM1->RCR  = 0;
        TIM1->ARR  = hdl->param[1].i;
        TIM1->CCR1 = hdl->param[2].i;
        TIM1->CCR2 = hdl->param[3].i;
        TIM1->CCR3 = hdl->param[4].i;
        TIM1->CCR4 = hdl->param[5].i;
    case 8:
        // TIM8->RCR  = 0;
        TIM8->ARR  = hdl->param[1].i;
        TIM8->CCR1 = hdl->param[2].i;
        TIM8->CCR2 = hdl->param[3].i;
        TIM8->CCR3 = hdl->param[4].i;
        TIM8->CCR4 = hdl->param[5].i;
    default:
        ret.i = 0;
        break;
    }
    return ret;
}
static const cmdinterp_funcmap_t cmdfuncmap[] = {
    {"err", (cmdinterp_cmdfunc_t)cmdinterp_cmderr, 1},
    {"adc", (cmdinterp_cmdfunc_t)cmdinterp_adc, 1},
    {"chc", (cmdinterp_cmdfunc_t)cmdinterp_chc, 2},
    {"drv", (cmdinterp_cmdfunc_t)cmdinterp_drv, 1},
    {"pwm", (cmdinterp_cmdfunc_t)cmdinterp_pwm, 1},
    {"tim", (cmdinterp_cmdfunc_t)cmdinterp_tim, 6},
    {"stat?", (cmdinterp_cmdfunc_t)cmdinterp_statq, 1},
    {NULL, NULL, 0}
};
static cmdinterp_t interp;
/*oo00OO00oo Command interpreter oo00OO00oo>*/

void stm32_init()
{
    /* setup timer TIM4.  TIM4 generates reset and speak. */
    // HAL_TIM_Base_Start(&htim4);
    /* Stop counter */
    TIM4->CR1 &= ~(TIM_CR1_CEN);
    HAL_TIM_OnePulse_Start_IT(&htim4, TIM_CHANNEL_1);
    HAL_TIM_OnePulse_Start_IT(&htim4, TIM_CHANNEL_2);
    TIM4->ARR  = 4;
    TIM4->CCR1 = 1;
    TIM4->CCR2 = 1;
    // TIM4->ARR = 1024;  /* in one-pulse mode, pulse stays high until CNT reaches this value. */
    // TIM4->CCR1 = 512;  /* in one-pulse mode, delay after trigger event, must >0. */

    /* Enable timer TIM8.  TIM8 generates pixel clock (CH1) and ADC sampling trigger (CH3). */
    HAL_TIM_Base_Start(&htim8);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    /* In center-aligned mode (up-down counting), number of taps in a
     * cycle is ARR*2.  ARR=28 would mean 200MHz/(28*2) = 3.571MHz. */
    TIM8->ARR  = 28; /* PWM counter would INC to this value then start to DEC back to 0. */
    TIM8->CCR1 = 0;  /* go low at up-counting, Asymmetric PWM1 mode. */
    TIM8->CCR2 = 28; /* go high at down-counting, Asymmetric PWM1 mode. */
    TIM8->CCR3 = 14; /* go high at up-counting, Asymmetric PWM2 mode. */
    TIM8->CCR4 = 14; /* go low at down-counting, Asymmetric PWM2 mode. */

    /* Initialize interpreter */
    cmdinterp_init(&interp, NULL, cmdinterp_nextc, cmdfuncmap);
    /* Turn off stdin buffer. */
    setvbuf(stdin, NULL, _IONBF, 0);
    /* Initialize ADC */
    adc_init();
    /* Initialize DAC */
    dac_init();
}

void stm32_main_loop()
{
    cmdinterp_run(&interp);

    printf("Hello.\n");
    for(int i=0; i<64; i++) {
        printf("% 3d", usb_recv_buf[i]);
    }
    printf("\n");

    HAL_Delay(500);
    /* These are slow. */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
    /* These are fast.  LED turns on or off in 2 CLK cycles: 48MHz CLK
     * results in f=12MHz square wave output. */
    GPIOE->BSRR = GPIO_PIN_1;
    GPIOE->BSRR = GPIO_PIN_1 << 16;
}

/*oo00OO00oo To make printf and getchar work oo00OO00oo<*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
}
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
  #define GETCHAR_PROTOTYPE int __io_getchar(void)
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define GETCHAR_PROTOTYPE int fgetc(FILE *f)
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
GETCHAR_PROTOTYPE
{
    int ch, i;
    /* usart */
    /*
    HAL_StatusTypeDef status;
    status = HAL_UART_Receive(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    if(status != HAL_OK) {
        return EOF;
    }
    */
    /*
    while((status = HAL_UART_Receive_IT(&huart1, (uint8_t*)&ch, 1)) != HAL_OK);
    */
    /* usb */
    while(usb_recv_buf[0]==0);
    i=strlen((const char*)usb_recv_buf)-1;
    ch = usb_recv_buf[i];
    usb_recv_buf[i] = 0;
    return ch;
}
/**
  * @brief  Retargets the C library printf function through USART or USB.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
    /* usart */
    /*
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    */
    /* usb */
    while(CDC_Transmit_FS((uint8_t *)&ch, 1) == USBD_BUSY);
    return ch;
}
/*oo00OO00oo To make printf and getchar work oo00OO00oo>*/
