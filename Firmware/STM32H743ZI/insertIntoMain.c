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
#define ADC_BUF_LEN 72*72
static volatile __attribute__((section(".dma_buf"))) __attribute__((aligned(0x20)))
uint32_t adc_buf[ADC_BUF_LEN];

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
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
    // HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);
/* Start ADC1 in interrupt mode. */
//    HAL_ADC_Start_IT(&hadc1);
}
/** ADC values are ready.
 * This one is called after DMA has transferred the data.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
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
    SCB_InvalidateDCache_by_Addr((uint32_t *)adc_buf, ADC_BUF_LEN);
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
    if(c) {s = GPIO_PIN_SET;} else {s = GPIO_PIN_RESET;}
    if(hdl->nparam < 1) {
        printf("\nDrvEN is %d\n", HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1));
        return ret;
    }
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN) != HAL_OK) {
        Error_Handler();
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
static cmdinterp_data_t cmdinterp_adc(cmdinterp_t *hdl)
{
    cmdinterp_data_t ret = {1};
    uint16_t v = (uint16_t)hdl->param[0].i;

    volatile uint16_t *val = (uint16_t*)adc_buf;
    printf("\n");
    int i, j, k=0;
    for (j=0; j<23; j++) {
        for (i=0; i<16; i++)
            if (v)
                printf("%4d ", (int16_t)val[k++]);
            else
                printf("%04x ", val[k++]);
        printf("\n");
    }
    return ret;
}
static cmdinterp_data_t cmdinterp_statq(cmdinterp_t *hdl)
{
    cmdinterp_data_t ret = {1};
    printf("\nLED: %d\n",
           HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1));
    fflush(stdout);
    return ret;
}
static const cmdinterp_funcmap_t cmdfuncmap[] = {
    {"err", (cmdinterp_cmdfunc_t)cmdinterp_cmderr, 1},
    {"chc", (cmdinterp_cmdfunc_t)cmdinterp_chc, 2},
    {"drv", (cmdinterp_cmdfunc_t)cmdinterp_drv, 1},
    {"pwm", (cmdinterp_cmdfunc_t)cmdinterp_pwm, 1},
    {"adc", (cmdinterp_cmdfunc_t)cmdinterp_adc, 1},
    {"stat?", (cmdinterp_cmdfunc_t)cmdinterp_statq, 1},
    {NULL, NULL, 0}
};
static cmdinterp_t interp;
/*oo00OO00oo Command interpreter oo00OO00oo>*/

void stm32_init()
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* Initialize TIM1/PWM */
    htim1.Init.Period = (STM32_FCLK/STM32_PWM_FREQ)-1;
    if(HAL_TIM_Base_Init(&htim1) != HAL_OK)
        Error_Handler();
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if(HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();
    /* Enable timer TIM1 */
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    /* Initialize interpreter */
    cmdinterp_init(&interp, NULL, cmdinterp_nextc, cmdfuncmap);
    /* Turn off stdin buffer. */
    setvbuf(stdin, NULL, _IONBF, 0);
    /* Initialize ADC */
    adc_init();
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
