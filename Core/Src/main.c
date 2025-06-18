#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

// Cấu hình chân
#define MQ2_CHANNEL     0
#define RELAY_PIN       5
#define BUZZER_PIN      6
#define LED_BLUE_PIN    7
#define LED_YELLOW_PIN  0
#define LED_RED_PIN     1
#define LED_GREEN_PIN   10
#define SW1_PIN         13
#define SW2_PIN         12
#define LCD_I2C_ADDR    0x27
#define LED_CATHODE_COMMON 1
#define BUZZER_ACTIVE_HIGH 1
#define UART1_TX_PIN    9
#define UART1_RX_PIN    10
#define UART1_BAUDRATE  9600

uint8_t system_state = 1;
uint8_t alert_state = 0;
volatile uint32_t tick_count = 0;
volatile uint8_t tim2_blink_state = 0;
volatile uint8_t relay_blink_state = 0;

void SystemClock_Config(void);
void UART_Transmit(const char *data);
void delay_ms(uint32_t ms) {
    uint32_t start = tick_count;
    while (tick_count - start < ms) __NOP();
}
void delay_us(uint32_t us) {
    for (uint32_t i = 0; i < us * 8; i++) __NOP();
}

void SystemClock_Config(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    RCC->PLLCFGR = (8 << 0) | (336 << 6) | (1 << 16);
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;
}

void GPIO_Config(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    GPIOA->MODER |= (3 << (MQ2_CHANNEL * 2));
    GPIOA->MODER &= ~((3 << (RELAY_PIN * 2)) | (3 << (BUZZER_PIN * 2)) | (3 << (LED_BLUE_PIN * 2)));
    GPIOA->MODER |= ((1 << (RELAY_PIN * 2)) | (1 << (BUZZER_PIN * 2)) | (1 << (LED_BLUE_PIN * 2)));
    GPIOA->ODR &= ~(1 << RELAY_PIN | 1 << BUZZER_PIN | 1 << LED_BLUE_PIN);
    GPIOB->MODER &= ~((3 << (LED_YELLOW_PIN * 2)) | (3 << (LED_RED_PIN * 2)) | (3 << (LED_GREEN_PIN * 2)));
    GPIOB->MODER |= ((1 << (LED_YELLOW_PIN * 2)) | (1 << (LED_RED_PIN * 2)) | (1 << (LED_GREEN_PIN * 2)));
    GPIOB->ODR &= ~(1 << LED_YELLOW_PIN | 1 << LED_RED_PIN | 1 << LED_GREEN_PIN);
    GPIOC->MODER &= ~((3 << (SW1_PIN * 2)) | (3 << (SW2_PIN * 2)));
    GPIOC->PUPDR &= ~((3 << (SW1_PIN * 2)) | (3 << (SW2_PIN * 2)));
    GPIOC->PUPDR |= ((1 << (SW1_PIN * 2)) | (1 << (SW2_PIN * 2)));
}

void I2C_Config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
    GPIOB->MODER |= ((2 << (8 * 2)) | (2 << (9 * 2)));
    GPIOB->OTYPER |= (1 << 8) | (1 << 9);
    GPIOB->PUPDR |= ((1 << (8 * 2)) | (1 << (9 * 2)));
    GPIOB->AFR[1] |= ((4 << ((8 - 8) * 4)) | (4 << ((9 - 8) * 4)));
    I2C1->CR1 = 0;
    I2C1->CR2 = 42;
    I2C1->CCR = 210;
    I2C1->TRISE = 43;
    I2C1->CR1 |= I2C_CR1_PE;
}

uint8_t I2C_WriteByte(uint8_t addr, uint8_t data, const char *context) {
    uint32_t timeout = 100000;
    while (I2C1->SR2 & I2C_SR2_BUSY && timeout--);
    if (!timeout) return 1;
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout--);
    if (!timeout) return 2;
    I2C1->DR = (addr << 1);
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout--);
    if (!timeout) return 3;
    (void)I2C1->SR2;
    while (!(I2C1->SR1 & I2C_SR1_TXE) && timeout--);
    if (!timeout) return 4;
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF) && timeout--);
    if (!timeout) return 5;
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0;
}

void LCD_SendCommand(uint8_t cmd) {
    uint8_t data_u = (cmd & 0xF0);
    uint8_t data_l = ((cmd << 4) & 0xF0);
    uint8_t data_t[4] = { data_u | 0x0C, data_u | 0x08, data_l | 0x0C, data_l | 0x08 };
    for (int i = 0; i < 4; i++) {
        I2C_WriteByte(LCD_I2C_ADDR, data_t[i], "CMD");
        delay_us(50);
    }
}

void LCD_SendData(uint8_t data) {
    uint8_t data_u = (data & 0xF0);
    uint8_t data_l = ((data << 4) & 0xF0);
    uint8_t data_t[4] = { data_u | 0x0D, data_u | 0x09, data_l | 0x0D, data_l | 0x09 };
    for (int i = 0; i < 4; i++) {
        I2C_WriteByte(LCD_I2C_ADDR, data_t[i], "DATA");
        delay_us(50);
    }
}

uint8_t LCD_Init(void) {
    uint8_t retry = 3;
    while (retry--) {
        delay_ms(50);
        if (I2C_WriteByte(LCD_I2C_ADDR, 0x00, "INIT") == 0) {
            LCD_SendCommand(0x33);
            delay_ms(5);
            LCD_SendCommand(0x32);
            delay_us(50);
            LCD_SendCommand(0x28);
            delay_us(50);
            LCD_SendCommand(0x0C);
            delay_us(50);
            LCD_SendCommand(0x06);
            delay_us(50);
            LCD_SendCommand(0x01);
            delay_ms(2);
            return 0;
        }
        delay_ms(50);
    }
    return 1;
}

void LCD_SendString(const char *str) {
    while (*str && *str != '\0') LCD_SendData(*str++);
}

void ADC_Config(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->CR2 = 0;
    ADC1->SQR3 = MQ2_CHANNEL;
    ADC1->SMPR2 |= (7 << (MQ2_CHANNEL * 3));
    ADC1->CR2 |= ADC_CR2_ADON;
    delay_us(10);
}

uint16_t ADC_Read(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}

void UART_Config(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    // Cấu hình PA9 (TX) và PA10 (RX)
    GPIOA->MODER &= ~((3U << (UART1_TX_PIN * 2)) | (3U << (UART1_RX_PIN * 2)));
    GPIOA->MODER |= ((2U << (UART1_TX_PIN * 2)) | (2U << (UART1_RX_PIN * 2)));
    GPIOA->OTYPER &= ~((1U << UART1_TX_PIN) | (1U << UART1_RX_PIN));
    GPIOA->OSPEEDR |= ((3U << (UART1_TX_PIN * 2)) | (3U << (UART1_RX_PIN * 2)));
    GPIOA->AFR[1] |= ((7U << ((UART1_TX_PIN - 8) * 4)) | (7U << ((UART1_RX_PIN - 8) * 4)));
    // Cấu hình USART1
    USART1->CR1 = 0;
    USART1->BRR = SystemCoreClock / UART1_BAUDRATE; // SystemCoreClock = 84MHz
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void UART_Transmit(const char *data) {
    while (*data && *data != '\0') {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = *data++;
    }
    while (!(USART1->SR & USART_SR_TC)); // Đợi truyền xong
}

void LED_Set(uint8_t r, uint8_t y, uint8_t b, uint8_t g) {
    if (LED_CATHODE_COMMON) {
        GPIOA->ODR = (GPIOA->ODR & ~(1 << LED_BLUE_PIN)) | (b << LED_BLUE_PIN);
        GPIOB->ODR = (GPIOB->ODR & ~((1 << LED_RED_PIN) | (1 << LED_YELLOW_PIN) | (1 << LED_GREEN_PIN))) |
                     (r << LED_RED_PIN) | (y << LED_YELLOW_PIN) | (g << LED_GREEN_PIN);
    } else {
        GPIOA->ODR = (GPIOA->ODR & ~(1 << LED_BLUE_PIN)) | ((b ? 0 : 1) << LED_BLUE_PIN);
        GPIOB->ODR = (GPIOB->ODR & ~((1 << LED_RED_PIN) | (1 << LED_YELLOW_PIN) | (1 << LED_GREEN_PIN))) |
                     ((r ? 0 : 1) << LED_RED_PIN) | ((y ? 0 : 1) << LED_YELLOW_PIN) | ((g ? 0 : 1) << LED_GREEN_PIN);
    }
}

void Buzzer_Set(uint8_t state) {
    if (BUZZER_ACTIVE_HIGH) {
        GPIOA->ODR = (GPIOA->ODR & ~(1 << BUZZER_PIN)) | (state << BUZZER_PIN);
    } else {
        GPIOA->ODR = (GPIOA->ODR & ~(1 << BUZZER_PIN)) | ((state ? 0 : 1) << BUZZER_PIN);
    }
}

static uint32_t last_arr = 0;
void TIM2_Reconfig(uint32_t arr) {
    if (arr != last_arr) {
        TIM2->CR1 &= ~TIM_CR1_CEN;
        TIM2->PSC = 4199;
        TIM2->ARR = arr;
        TIM2->CNT = 0;
        TIM2->DIER |= TIM_DIER_UIE;
        TIM2->SR &= ~TIM_SR_UIF;
        TIM2->CR1 |= TIM_CR1_CEN;
        last_arr = arr;
    }
}

void TIM2_Config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler(void) {
	if (TIM2->SR & TIM_SR_UIF) {
	        TIM2->SR &= ~TIM_SR_UIF;
	        if (TIM2->CR1 & TIM_CR1_CEN) {
	            tim2_blink_state = !tim2_blink_state;
	            relay_blink_state = tim2_blink_state; // Đồng bộ trạng thái relay
	            Relay_Set(relay_blink_state); // Bật/tắt relay
	        }
	}
}

float MQ2_GetPPM(uint16_t adc_value) {
    float voltage = (adc_value / 4096.0f) * 3.3f;
    float k = 500.0f;
    float c = 0.0f;
    float ppm = k * voltage + c;
    if (ppm < 0) ppm = 0;
    return ppm;
}

void SysTick_Initialize(void) {
    SysTick->LOAD = 84000000 / 1000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    NVIC_EnableIRQ(SysTick_IRQn);
}

void SysTick_Handler(void) {
    tick_count++;
}

void Relay_Set(uint8_t state) {
    GPIOA->ODR = (GPIOA->ODR & ~(1 << RELAY_PIN)) | (state << RELAY_PIN);
}

int main(void) {
    SystemClock_Config();
    GPIO_Config();
    ADC_Config();
    I2C_Config();
    UART_Config();
    TIM2_Config();
    SysTick_Initialize();

    if (LCD_Init()) {
        UART_Transmit("Try LCD_I2C_ADDR 0x3F\r\n");
    }

    LED_Set(1, 0, 0, 0); delay_ms(100);
    LED_Set(0, 0, 0, 1); delay_ms(100);
    LED_Set(0, 1, 0, 0); delay_ms(100);
    LED_Set(0, 0, 1, 0); delay_ms(100);
    LED_Set(0, 0, 0, 0);

    LCD_SendCommand(0x01);
    LCD_SendCommand(0x80);
    LCD_SendString("Gas Detector");
    LCD_SendCommand(0xC0);
    LCD_SendString("Starting...");
    delay_ms(1000);

    LCD_SendCommand(0x01);
    LCD_SendCommand(0x80);
    LCD_SendString("PPM:----");
    LCD_SendCommand(0xC0);
    LCD_SendString("SYS:ON Alert:-");
    delay_ms(50);

    char lcd_buf[17];
    char debug_buf[80];
    uint8_t last_system_state = system_state;
    uint8_t last_alert_state = 255;
    uint32_t last_ppm = 0xFFFFFFFF;
    uint32_t last_transmit = 0;
    uint32_t last_lcd_update = 0;
    float last_ppm_for_freq = 0;

    while (1) {
        static uint8_t last_sw1 = 1, last_sw2 = 1;
        uint8_t sw1 = (GPIOC->IDR & (1 << SW1_PIN)) ? 1 : 0;
        uint8_t sw2 = (GPIOC->IDR & (1 << SW2_PIN)) ? 1 : 0;

        if (last_sw1 && !sw1) {
            system_state = !system_state;
            snprintf(debug_buf, sizeof(debug_buf), "SW1 Pressed: System State=%u\r\n", system_state);
            UART_Transmit(debug_buf);
            delay_ms(10);
        }
        last_sw1 = sw1;

        if (last_sw2 && !sw2) {
            system_state = 1;
            alert_state = 0;
            tim2_blink_state = 0;
            LED_Set(0, 0, 0, 0);
            Relay_Set(0);
            Buzzer_Set(0);
            TIM2->CR1 &= ~TIM_CR1_CEN;
            LCD_SendCommand(0x01);
            LCD_SendCommand(0x80);
            LCD_SendString("Gas Detector");
            LCD_SendCommand(0xC0);
            LCD_SendString("Starting...");
            UART_Transmit("SW2 Pressed: System Reset\r\n");
            delay_ms(1000);
            LCD_SendCommand(0x01);
            LCD_SendCommand(0x80);
            LCD_SendString("PPM:----");
            LCD_SendCommand(0xC0);
            LCD_SendString("SYS:ON Alert:-");
            last_system_state = system_state;
            last_alert_state = 255;
            last_ppm = 0xFFFFFFFF;
            last_ppm_for_freq = 0;
            last_lcd_update = 0;
        }
        last_sw2 = sw2;

        if (!system_state) {
            if (last_system_state != system_state) {
                LED_Set(0, 0, 0, 1);
                Relay_Set(0);
                Buzzer_Set(0);
                TIM2->CR1 &= ~TIM_CR1_CEN;
                LCD_SendCommand(0x01);
                LCD_SendCommand(0x80);
                LCD_SendString("SYS:OFF");
                LCD_SendCommand(0xC0);
                LCD_SendString("PPM:--- Alert:0");
                UART_Transmit("System: OFF, PPM: ---\r\n");
                delay_ms(10);
                last_system_state = system_state;
                last_alert_state = 255;
                last_ppm = 0xFFFFFFFF;
                last_ppm_for_freq = 0;
                last_lcd_update = 0;
            }
            delay_ms(50);
            continue;
        }

        if (last_system_state != system_state) {
            LCD_SendCommand(0x01);
            LCD_SendCommand(0x80);
            LCD_SendString("PPM:----");
            LCD_SendCommand(0xC0);
            LCD_SendString("SYS:ON Alert:-");
            last_system_state = system_state;
            last_alert_state = 255;
            last_ppm = 0xFFFFFFFF;
            last_ppm_for_freq = 0;
            last_lcd_update = 0;
        }

        uint16_t adc_val = ADC_Read();
        float ppm = MQ2_GetPPM(adc_val);
        uint32_t ppm_int = (uint32_t)ppm;

        if (tick_count - last_transmit >= 250) {
            snprintf(debug_buf, sizeof(debug_buf), "%.1f\n", ppm);
            UART_Transmit(debug_buf);
            delay_ms(10);
            last_transmit = tick_count;
        }

        if (ppm > 800) alert_state = 3;
        else if (ppm > 500) alert_state = 2;
        else if (ppm > 300) alert_state = 1;
        else alert_state = 0;

        if (tick_count - last_lcd_update >= 500 || last_alert_state != alert_state) {
            if (ppm_int != last_ppm) {
                LCD_SendCommand(0x80 + 4);
                snprintf(lcd_buf, sizeof(lcd_buf), "%4lu", ppm_int);
                LCD_SendString(lcd_buf);
                last_ppm = ppm_int;
            }
            if (last_alert_state != alert_state) {
                LCD_SendCommand(0xC0 + 13);
                snprintf(lcd_buf, sizeof(lcd_buf), "%d", alert_state);
                LCD_SendString(lcd_buf);
                last_alert_state = alert_state;
            }
            last_lcd_update = tick_count;
        }

        switch (alert_state) {
            case 0:
                LED_Set(0, 0, 1, 0);
                Relay_Set(0);
                Buzzer_Set(0);
                TIM2->CR1 &= ~TIM_CR1_CEN;
                last_ppm_for_freq = 0;
                break;
            case 1:
                LED_Set(0, 1, 0, 0);
                Relay_Set(0);
                Buzzer_Set(0);
                TIM2->CR1 &= ~TIM_CR1_CEN;
                last_ppm_for_freq = 0;
                break;
            case 2:
            	if (last_alert_state != 2) {
            	    tim2_blink_state = 0;
            	    relay_blink_state = 0;
            	    TIM2_Reconfig(4999); // 1 Hz
            	    last_ppm_for_freq = 0;
            	}
            	LED_Set(tim2_blink_state, 0, 0, 0);
            	Buzzer_Set(tim2_blink_state);
            	break;
            case 3:
            	if (last_alert_state != 3 || fabs(ppm - last_ppm_for_freq) > 50) {
            	    tim2_blink_state = 0;
            	    relay_blink_state = 0;
            	    float freq = (ppm >= 2000 ? 10.0f : 2.0f + (ppm - 800.0f) * 8.0f / 1200.0f);
            	    uint32_t arr = (uint32_t)((42000000.0f / (2.0f * freq * 4200.0f)) - 1);
            	    TIM2_Reconfig(arr);
            	    last_ppm_for_freq = ppm;
            	}
            	LED_Set(tim2_blink_state, 0, 0, 0);
            	Buzzer_Set(tim2_blink_state);
            	break;
        }

        delay_ms(50);
    }
}
