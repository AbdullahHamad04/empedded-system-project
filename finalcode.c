/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdio.h>      // for snprintf

/* Private define ------------------------------------------------------------*/
#define LCD_ADDR    (0x27<<1)
#define LCD_BL      0x08
#define LCD_EN      0x04
#define LCD_RS      0x01

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line);
#endif

/* Register‐level inits and helpers ------------------------------------------*/
void GPIO_regs_init(void);
void I2C1_regs_init(void);
void TIM2_regs_init(void);
void LCD_init(void);
void HC_SR04_ReadAndDisplay(void);

static void I2C1_wait_tx(void);
static void I2C1_start_tx(void);
static void I2C1_write_byte(uint8_t d);
static void I2C1_stop(void);
static void LCD_write_nibble(uint8_t nibble);
static void LCD_send(uint8_t b, uint8_t flags);

#define LCD_cmd(c)  LCD_send((c), 0)
#define LCD_data(c) LCD_send((c), LCD_RS)

/*---------------------------------------------------------------------------*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    GPIO_regs_init();   // configure PB6/PB7 for I2C, PA0/PA1 for HC-SR04
    I2C1_regs_init();   // set up I2C1 @100kHz
    LCD_init();         // init the LCD in 4-bit mode
    TIM2_regs_init();   // configure TIM2 channel 2 for input capture

    while (1)
    {
        HC_SR04_ReadAndDisplay();
        for (volatile int d = 0; d < 1000000; d++);  // ~1 s delay
    }
}

/** Configure GPIO:
 *  PB6=SCL, PB7=SDA (AF4 open-drain)
 *  PA0 = trigger output
 *  PA1 = echo input (TIM2_CH2, AF1)
 */
void GPIO_regs_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    /* PB6/PB7 = AF4, open-drain, very high speed */
    GPIOB->MODER   &= ~((3<<12)|(3<<14));
    GPIOB->MODER   |=  (2<<12)|(2<<14);
    GPIOB->OTYPER  |=  (1<<6)|(1<<7);
    GPIOB->OSPEEDR |=  (3<<12)|(3<<14);
    GPIOB->PUPDR   &= ~((3<<12)|(3<<14));
    GPIOB->AFR[0]  &= ~((0xF<<24)|(0xF<<28));
    GPIOB->AFR[0]  |=  (4<<24)|(4<<28);

    /* PA0 = push-pull output (trigger) */
    GPIOA->MODER   &= ~(3<<0);
    GPIOA->MODER   |=  (1<<0);
    GPIOA->OTYPER  &= ~(1<<0);
    GPIOA->OSPEEDR |=  (3<<0);
    GPIOA->PUPDR   &= ~(3<<0);

    /* PA1 = AF1 (TIM2_CH2), push-pull */
    GPIOA->MODER   &= ~(3<<2);
    GPIOA->MODER   |=  (2<<2);
    GPIOA->OTYPER  &= ~(1<<1);
    GPIOA->OSPEEDR |=  (3<<2);
    GPIOA->PUPDR   &= ~(3<<2);
    GPIOA->AFR[0]  &= ~(0xF<<4);
    GPIOA->AFR[0]  |=  (1<<4);
}

/** Initialize I2C1 for 100 kHz @ PCLK1=42 MHz */
void I2C1_regs_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    I2C1->CR1   &= ~I2C_CR1_PE;
    I2C1->CR2    = 42;      // APB1 clock in MHz
    I2C1->CCR    = 210;     // CCR = Fpclk1/(2*Fscl)
    I2C1->TRISE  = 43;      // max rise time
    I2C1->CR1   |=  I2C_CR1_PE;
}

static void I2C1_wait_tx(void) {
    while (!(I2C1->SR1 & I2C_SR1_TXE));
}
static void I2C1_start_tx(void) {
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    I2C1->DR = LCD_ADDR & ~1;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
}
static void I2C1_write_byte(uint8_t d) {
    I2C1_wait_tx();
    I2C1->DR = d;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
}
static void I2C1_stop(void) {
    I2C1->CR1 |= I2C_CR1_STOP;
}

/** Low-level LCD 4-bit routines over I2C */
static void LCD_write_nibble(uint8_t nibble) {
    uint8_t data = (nibble & 0xF0) | LCD_BL;
    I2C1_start_tx();
      I2C1_write_byte(data | LCD_EN);
      I2C1_write_byte(data);
    I2C1_stop();
    for (volatile int i = 0; i < 10000; i++);
}

static void LCD_send(uint8_t b, uint8_t flags) {
    uint8_t hi = (b & 0xF0) | flags | LCD_BL;
    uint8_t lo = ((b << 4) & 0xF0) | flags | LCD_BL;
    I2C1_start_tx();
      I2C1_write_byte(hi | LCD_EN);
      I2C1_write_byte(hi);
      I2C1_write_byte(lo | LCD_EN);
      I2C1_write_byte(lo);
    I2C1_stop();
    for (volatile int i = 0; i < 10000; i++);
}

void LCD_init(void) {
    for (int i = 0; i < 3; i++) {
        LCD_write_nibble(0x30);
        for (volatile int d = 0; d < 50000; d++);
    }
    LCD_write_nibble(0x20);
    LCD_cmd(0x28);  // 4-bit, 2 lines
    LCD_cmd(0x08);  // display off
    LCD_cmd(0x01);  // clear
    for (volatile int d = 0; d < 200000; d++);
    LCD_cmd(0x06);
    LCD_cmd(0x0C);  // display on
}

/** Configure TIM2 CH2 as input capture on rising/falling edges */
void TIM2_regs_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC   = SystemCoreClock/1000000 - 1;  // 1 MHz timer
    TIM2->ARR   = 0xFFFF;
    TIM2->CCMR1 = (1<<8);           // CC2 = TI2
    TIM2->CCMR1 &= ~TIM_CCMR1_IC2F;
    TIM2->CCER  = TIM_CCER_CC2E;     // capture rising
    TIM2->CR1   = TIM_CR1_CEN;
}

/** Trigger HC-SR04, capture pulse, compute & display distance allowing >100 cm */
void HC_SR04_ReadAndDisplay(void) {
    uint16_t rise, fall;

    /* Trigger 10 µs */
    GPIOA->BSRR = (1<<0);
    for (volatile int i = 0; i < 160; i++);
    GPIOA->BSRR = (1<<(0+16));

    /* Capture rising edge */
    TIM2->SR   &= ~TIM_SR_CC2IF;
    while (!(TIM2->SR & TIM_SR_CC2IF));
    rise = TIM2->CCR2;

    /* Capture falling edge */
    TIM2->CCER |= TIM_CCER_CC2P;
    TIM2->SR   &= ~TIM_SR_CC2IF;
    while (!(TIM2->SR & TIM_SR_CC2IF));
    fall = TIM2->CCR2;
    TIM2->CCER &= ~TIM_CCER_CC2P;

    /* Compute mm */
    uint32_t mm = ((uint32_t)(fall - rise) * 343 + 1000) / 2000;
    /* Clamp at 4000 mm (400 cm) if desired */
    if (mm > 4000) mm = 4000;

    /* Split into cm + 1 decimal */
    uint32_t whole = mm / 10;    // e.g. 120
    uint32_t frac  = mm % 10;    // e.g. 3 → “120.3”

    char buf[16];
    snprintf(buf, sizeof(buf), "%u.%u cm", whole, frac);

    /* Display on LCD */
    LCD_cmd(0x01);
    for (volatile int d = 0; d < 200000; d++);
    for (int i = 0; buf[i]; i++) {
        LCD_data(buf[i]);
    }
}

/* System Clock Configuration (from CubeMX) */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 336;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK
                                     | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1
                                     | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can printf or log here */
}
#endif
