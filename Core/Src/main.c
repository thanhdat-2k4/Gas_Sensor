#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// === Cấu hình chân ===
#define MQ2_CHANNEL     0       // PA0 - ADC1_IN0
#define RELAY_PIN       5       // PA5
#define BUZZER_PIN      6       // PA6
#define LED_BLUE_PIN    7       // PA7 - Xanh dương
#define LED_YELLOW_PIN  0       // PB0 - Vàng
#define LED_RED_PIN     1       // PB1 - Đỏ
#define LED_GREEN_PIN   10      // PB10 - Xanh lá
#define SW1_PIN         13      // PC13
#define SW2_PIN         12      // PC12
#define LCD_I2C_ADDR    0x27    // Địa chỉ I2C LCD (thử 0x3F nếu lỗi)

// LED cathode-common (1 = ON)
#define LED_CATHODE_COMMON 1
// Buzzer active-high (1 = ON)
#define BUZZER_ACTIVE_HIGH 1

// === Biến toàn cục ===
uint8_t system_state = 1;       // Hệ thống bắt đầu ON
uint8_t alert_state = 0;        // Trạng thái cảnh báo
volatile uint32_t tick_count = 0; // Đếm tick từ SysTick (1ms)
volatile uint8_t tim2_blink_state = 0; // Trạng thái nhấp nháy TIM2

// === Khai báo hàm trước khi sử dụng ===
void UART_Transmit(char *data);

// === Hàm delay sử dụng SysTick ===
void delay_ms(uint32_t ms) {
    uint32_t start_tick = tick_count;
    while (tick_count - start_tick < ms) __NOP();
}

void delay_us(uint32_t us) {
    for (uint32_t i = 0; i < us * 8; i++) __NOP();
}

// === Cấu hình GPIO ===
void GPIO_Config(void) {
    // Bật clock cho GPIOA, GPIOB, GPIOC
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    // PA0 (MQ-2) - Analog
    GPIOA->MODER |= (3 << (MQ2_CHANNEL * 2));

    // PA5 (Relay), PA6 (Buzzer), PA7 (LED Blue) - Output
    GPIOA->MODER &= ~((3 << (RELAY_PIN * 2)) | (3 << (BUZZER_PIN * 2)) | (3 << (LED_BLUE_PIN * 2)));
    GPIOA->MODER |= ((1 << (RELAY_PIN * 2)) | (1 << (BUZZER_PIN * 2)) | (1 << (LED_BLUE_PIN * 2)));
    GPIOA->ODR &= ~(1 << RELAY_PIN); // Relay OFF
    GPIOA->ODR &= ~(1 << BUZZER_PIN); // Buzzer OFF
    GPIOA->ODR &= ~(1 << LED_BLUE_PIN); // Blue LED OFF

    // PB0 (Yellow), PB1 (Red), PB10 (Green) - Output
    GPIOB->MODER &= ~((3 << (LED_YELLOW_PIN * 2)) | (3 << (LED_RED_PIN * 2)) | (3 << (LED_GREEN_PIN * 2)));
    GPIOB->MODER |= ((1 << (LED_YELLOW_PIN * 2)) | (1 << (LED_RED_PIN * 2)) | (1 << (LED_GREEN_PIN * 2)));
    GPIOB->ODR &= ~((1 << LED_YELLOW_PIN) | (1 << LED_RED_PIN) | (1 << LED_GREEN_PIN)); // LEDs OFF

    // PC12, PC13 (SW1, SW2) - Input Pull-up
    GPIOC->MODER &= ~((3 << (SW1_PIN * 2)) | (3 << (SW2_PIN * 2)));
    GPIOC->PUPDR &= ~((3 << (SW1_PIN * 2)) | (3 << (SW2_PIN * 2)));
    GPIOC->PUPDR |= ((1 << (SW1_PIN * 2)) | (1 << (SW2_PIN * 2)));
}

// === Cấu hình I2C1 (PB8 - SCL, PB9 - SDA) ===
void I2C_Config(void) {
    // Bật clock cho I2C1 và GPIOB
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // PB8 (SCL), PB9 (SDA) - Alternate function, open-drain
    GPIOB->MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
    GPIOB->MODER |= ((2 << (8 * 2)) | (2 << (9 * 2)));
    GPIOB->OTYPER |= (1 << 8) | (1 << 9); // Open-drain
    GPIOB->PUPDR |= ((1 << (8 * 2)) | (1 << (9 * 2))); // Pull-up
    GPIOB->AFR[1] |= ((4 << ((8 - 8) * 4)) | (4 << ((9 - 8) * 4))); // AF4 cho I2C1

    // Cấu hình I2C1
    I2C1->CR1 = 0; // Reset
    I2C1->CR2 = 84; // APB1 clock 84MHz
    I2C1->CCR = 420; // 100kHz
    I2C1->TRISE = 85; // TRISE
    I2C1->CR1 |= I2C_CR1_PE; // Bật I2C
}

uint8_t I2C_WriteByte(uint8_t addr, uint8_t data, const char *context) {
    uint32_t timeout = 100000;
    char debug_buf[32];
    while (I2C1->SR2 & I2C_SR2_BUSY && timeout--) {
        if (timeout == 0) {
            snprintf(debug_buf, sizeof(debug_buf), "%s I2C Busy\r\n", context);
            UART_Transmit(debug_buf);
            return 1;
        }
    }
    I2C1->CR1 |= I2C_CR1_START; // Phát start
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout--) {
        if (timeout == 0) {
            snprintf(debug_buf, sizeof(debug_buf), "%s I2C Start timeout\r\n", context);
            UART_Transmit(debug_buf);
            return 2;
        }
    }
    I2C1->DR = (addr << 1); // Gửi địa chỉ
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout--) {
        if (timeout == 0) {
            snprintf(debug_buf, sizeof(debug_buf), "%s I2C Addr timeout\r\n", context);
            UART_Transmit(debug_buf);
            return 3;
        }
    }
    (void)I2C1->SR2; // Xóa cờ ADDR
    while (!(I2C1->SR1 & I2C_SR1_TXE) && timeout--) {
        if (timeout == 0) {
            snprintf(debug_buf, sizeof(debug_buf), "%s I2C TXE timeout\r\n", context);
            UART_Transmit(debug_buf);
            return 4;
        }
    }
    I2C1->DR = data; // Gửi dữ liệu
    while (!(I2C1->SR1 & I2C_SR1_BTF) && timeout--) {
        if (timeout == 0) {
            snprintf(debug_buf, sizeof(debug_buf), "%s I2C BTF timeout\r\n", context);
            UART_Transmit(debug_buf);
            return 5;
        }
    }
    I2C1->CR1 |= I2C_CR1_STOP; // Phát stop
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
    char debug_buf[32];
    uint8_t retry = 3;
    while (retry--) {
        delay_ms(50);
        if (I2C_WriteByte(LCD_I2C_ADDR, 0x00, "INIT") == 0) {
            LCD_SendCommand(0x33); // Khởi tạo 4-bit
            delay_ms(5);
            LCD_SendCommand(0x32);
            delay_us(50);
            LCD_SendCommand(0x28); // 4-bit, 2 dòng
            delay_us(50);
            LCD_SendCommand(0x0C); // Bật hiển thị, không con trỏ
            delay_us(50);
            LCD_SendCommand(0x06); // Chế độ nhập
            delay_us(50);
            LCD_SendCommand(0x01); // Xóa màn hình
            delay_ms(2);
            snprintf(debug_buf, sizeof(debug_buf), "LCD Init OK\r\n");
            UART_Transmit(debug_buf);
            return 0;
        }
        snprintf(debug_buf, sizeof(debug_buf), "LCD Init Retry %d\r\n", retry);
        UART_Transmit(debug_buf);
        delay_ms(50);
    }
    snprintf(debug_buf, sizeof(debug_buf), "LCD Init Failed\r\n");
    UART_Transmit(debug_buf);
    return 1;
}

void LCD_SendString(char *str) {
    char debug_buf[32];
    snprintf(debug_buf, sizeof(debug_buf), "LCD Send: %s\r\n", str);
    UART_Transmit(debug_buf);
    while (*str && *str != '\0') LCD_SendData(*str++);
}

// === Cấu hình ADC1 cho PA0 ===
void ADC_Config(void) {
    // Bật clock cho ADC1
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Cấu hình ADC
    ADC1->CR2 = 0;
    ADC1->SQR3 = MQ2_CHANNEL; // Kênh 0
    ADC1->SMPR2 |= (7 << (MQ2_CHANNEL * 3)); // 480 chu kỳ
    ADC1->CR2 |= ADC_CR2_ADON; // Bật ADC
    delay_us(10);
}

uint16_t ADC_Read(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART; // Bắt đầu chuyển đổi
    while (!(ADC1->SR & ADC_SR_EOC)); // Chờ hoàn tất
    return ADC1->DR;
}

// === Cấu hình UART2 (PA2 - TX, PA3 - RX) ===
void UART_Config(void) {
    // Bật clock cho USART2 và GPIOA
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // PA2 (TX), PA3 (RX) - Alternate function
    GPIOA->MODER &= ~((3 << (2 * 2)) | (3 << (3 * 2)));
    GPIOA->MODER |= ((2 << (2 * 2)) | (2 << (3 * 2)));
    GPIOA->AFR[0] |= ((7 << (2 * 4)) | (7 << (3 * 4))); // AF7
    // Baud rate 115200
    USART2->BRR = 84000000 / 115200; // Clock 84MHz
    // Bật UART, TX, RX
    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void UART_Transmit(char *data) {
    while (*data && *data != '\0') {
        while (!(USART2->SR & USART_SR_TXE)); // Chờ TXE
        USART2->DR = *data++;
    }
}

// === Điều khiển LED ===
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
    char debug_buf[80];
    snprintf(debug_buf, sizeof(debug_buf), "LED Set: R=%u, Y=%u, B=%u, G=%u, ODR: PA7=%lu, PB0=%lu, PB1=%lu, PB10=%lu\r\n",
             r, y, b, g,
             (GPIOA->ODR >> LED_BLUE_PIN) & 1,
             (GPIOB->ODR >> LED_YELLOW_PIN) & 1,
             (GPIOB->ODR >> LED_RED_PIN) & 1,
             (GPIOB->ODR >> LED_GREEN_PIN) & 1);
    UART_Transmit(debug_buf);
}

// === Điều khiển Buzzer ===
void Buzzer_Set(uint8_t state) {
    if (BUZZER_ACTIVE_HIGH) {
        GPIOA->ODR = (GPIOA->ODR & ~(1 << BUZZER_PIN)) | (state << BUZZER_PIN);
    } else {
        GPIOA->ODR = (GPIOA->ODR & ~(1 << BUZZER_PIN)) | ((state ? 0 : 1) << BUZZER_PIN);
    }
    char debug_buf[32];
    snprintf(debug_buf, sizeof(debug_buf), "Buzzer: %u, PA6=%lu\r\n", state, (GPIOA->ODR >> BUZZER_PIN) & 1);
    UART_Transmit(debug_buf);
}

// === Cấu hình TIM2 (nhấp nháy, tần số thay đổi) ===
void TIM2_Config(void) {
    // Bật clock cho TIM2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    // Prescaler = 8400 - 1, ARR sẽ được cập nhật động
    TIM2->PSC = 8399;
    TIM2->ARR = 999; // Mặc định 1Hz (500ms chu kỳ)
    TIM2->DIER |= TIM_DIER_UIE; // Bật ngắt update
    TIM2->CR1 |= TIM_CR1_CEN; // Bật timer
    NVIC_EnableIRQ(TIM2_IRQn);
}

// === Cập nhật tần số TIM2 dựa trên PPM ===
void TIM2_UpdateFrequency(float ppm) {
    // Tần số: 2Hz (PPM=800) đến 10Hz (PPM=2000)
    float freq;
    if (ppm >= 2000) {
        freq = 10.0f; // 10Hz
    } else if (ppm <= 800) {
        freq = 2.0f; // 2Hz
    } else {
        // Ánh xạ tuyến tính: 800->2Hz, 2000->10Hz
        freq = 2.0f + (ppm - 800.0f) * (10.0f - 2.0f) / (2000.0f - 800.0f);
    }
    // Tính ARR: t = (ARR + 1) * (PSC + 1) / 84MHz
    // Chu kỳ t = 1 / (2 * freq) (vì nhấp nháy cần bật/tắt)
    uint32_t arr = (uint32_t)((84000000.0f / (2.0f * freq * 8400.0f)) - 1);
    TIM2->ARR = arr;
    char debug_buf[32];
    snprintf(debug_buf, sizeof(debug_buf), "TIM2: Freq=%.2fHz, ARR=%lu\r\n", freq, arr);
    UART_Transmit(debug_buf);
}

// === Xử lý ngắt TIM2 ===
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF; // Xóa cờ
        tim2_blink_state = !tim2_blink_state; // Đảo trạng thái
        char debug_buf[32];
        snprintf(debug_buf, sizeof(debug_buf), "TIM2: Blink=%u\r\n", tim2_blink_state);
        UART_Transmit(debug_buf);
    }
}

// === Tính PPM cho MQ-2 ===
float MQ2_GetPPM(uint16_t adc_value) {
    float voltage = (adc_value / 4096.0f) * 3.3f; // Điện áp 3.3V
    float k = 500.0f; // Ánh xạ 0-3.3V -> 0-990ppm
    float c = 0.0f;
    float ppm = k * voltage + c;
    if (ppm < 0) ppm = 0;
    return ppm;
}

// === Cấu hình SysTick (1ms) ===
void SysTick_Initialize(void) {
    SysTick->LOAD = 84000000 / 1000 - 1; // 1ms (84MHz)
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    NVIC_EnableIRQ(SysTick_IRQn);
}

void SysTick_Handler(void) {
    tick_count++;
}

// === Main ===
int main(void) {
    // Cấu hình
    GPIO_Config();
    ADC_Config();
    I2C_Config();
    UART_Config();
    TIM2_Config();
    SysTick_Initialize();

    // Khởi tạo LCD
    if (LCD_Init()) {
        char debug_buf[32];
        snprintf(debug_buf, sizeof(debug_buf), "Try LCD_I2C_ADDR 0x3F\r\n");
        UART_Transmit(debug_buf);
    }

    // Test LED
    LED_Set(1, 0, 0, 0); // Đỏ
    delay_ms(100);
    LED_Set(0, 0, 0, 1); // Xanh lá
    delay_ms(100);
    LED_Set(0, 1, 0, 0); // Vàng
    delay_ms(100);
    LED_Set(0, 0, 1, 0); // Xanh dương
    delay_ms(100);
    LED_Set(0, 0, 0, 0); // Tắt

    // Hiển thị khởi động LCD
    LCD_SendCommand(0x01); // Xóa màn
    LCD_SendCommand(0x80); // Dòng 1
    LCD_SendString("Gas Detector");
    LCD_SendCommand(0xC0); // Dòng 2
    LCD_SendString("Starting...");
    delay_ms(1000);

    // Hiển thị giao diện chính
    LCD_SendCommand(0x01); // Xóa màn
    LCD_SendCommand(0x80); // Dòng 1
    LCD_SendString("PPM:----");
    LCD_SendCommand(0xC0); // Dòng 2
    LCD_SendString("SYS:ON Alert:-");
    delay_ms(50);

    char lcd_buf[17];
    char debug_buf[80];
    uint8_t last_system_state = system_state;
    uint8_t last_alert_state = 255; // Ép cập nhật lần đầu
    uint32_t last_ppm = 0xFFFFFFFF; // Ép cập nhật lần đầu
    uint32_t last_transmit = 0; // Thời gian gửi UART

    while (1) {
        // Đọc trạng thái nút nhấn
        static uint8_t last_sw1 = 1, last_sw2 = 1;
        uint8_t sw1 = (GPIOC->IDR & (1 << SW1_PIN)) ? 1 : 0;
        uint8_t sw2 = (GPIOC->IDR & (1 << SW2_PIN)) ? 1 : 0;

        // SW1: Bật/tắt hệ thống
        if (last_sw1 && !sw1) {
            system_state = !system_state;
            snprintf(debug_buf, sizeof(debug_buf), "SW1 Pressed: System State=%u\r\n", system_state);
            UART_Transmit(debug_buf);
        }
        last_sw1 = sw1;

        // SW2: Reset hệ thống
        if (last_sw2 && !sw2) {
            system_state = 1;
            alert_state = 0;
            tim2_blink_state = 0;
            LED_Set(0, 0, 0, 0); // Tắt LED
            GPIOA->ODR &= ~(1 << RELAY_PIN); // Relay OFF
            Buzzer_Set(0); // Buzzer OFF
            TIM2->CR1 &= ~TIM_CR1_CEN; // Tắt TIM2
            LCD_SendCommand(0x01); // Xóa màn
            LCD_SendCommand(0x80); // Dòng 1
            LCD_SendString("Gas Detector");
            LCD_SendCommand(0xC0); // Dòng 2
            LCD_SendString("Starting...");
            snprintf(debug_buf, sizeof(debug_buf), "SW2 Pressed: System Reset\r\n");
            UART_Transmit(debug_buf);
            delay_ms(1000);
            LCD_SendCommand(0x01); // Xóa màn
            LCD_SendCommand(0x80); // Dòng 1
            LCD_SendString("PPM:----");
            LCD_SendCommand(0xC0); // Dòng 2
            LCD_SendString("SYS:ON Alert:-");
            last_system_state = system_state;
            last_alert_state = 255;
            last_ppm = 0xFFFFFFFF;
        }
        last_sw2 = sw2;

        // Hệ thống tắt
        if (!system_state) {
            if (last_system_state != system_state) {
                LED_Set(0, 0, 0, 1); // Xanh lá
                GPIOA->ODR &= ~(1 << RELAY_PIN); // Relay OFF
                Buzzer_Set(0);
                TIM2->CR1 &= ~TIM_CR1_CEN; // Tắt TIM2
                LCD_SendCommand(0x01); // Xóa màn
                LCD_SendCommand(0x80); // Dòng 1
                LCD_SendString("SYS:OFF");
                LCD_SendCommand(0xC0); // Dòng 2
                LCD_SendString("PPM:--- Alert:0");
                snprintf(debug_buf, sizeof(debug_buf), "System: OFF, PPM: ---\r\n");
                UART_Transmit(debug_buf);
                last_system_state = system_state;
                last_alert_state = 255;
                last_ppm = 0xFFFFFFFF;
            }
            delay_ms(50);
            continue;
        }

        // Cập nhật trạng thái hệ thống
        if (last_system_state != system_state) {
            LCD_SendCommand(0x01); // Xóa màn
            LCD_SendCommand(0x80); // Dòng 1
            LCD_SendString("PPM:----");
            LCD_SendCommand(0xC0); // Dòng 2
            LCD_SendString("SYS:ON Alert:-");
            last_system_state = system_state;
            last_alert_state = 255;
            last_ppm = 0xFFFFFFFF;
        }

        // Đọc và tính PPM
        uint16_t adc_val = ADC_Read();
        float ppm = MQ2_GetPPM(adc_val);
        uint32_t ppm_int = (uint32_t)ppm;

        // Gửi dữ liệu qua UART mỗi 500ms
        if (tick_count - last_transmit >= 500) {
            snprintf(debug_buf, sizeof(debug_buf), "ADC: %u, V: %.2f, PPM: %.0f, Alert: %d\r\n",
                     adc_val, (adc_val / 4096.0f) * 3.3f, ppm, alert_state);
            UART_Transmit(debug_buf);
            last_transmit = tick_count;
        }

        // Xác định trạng thái cảnh báo
        if (ppm > 980) alert_state = 3; // Nguy hiểm
        else if (ppm > 500) alert_state = 2; // Cao
        else if (ppm > 300) alert_state = 1; // Thấp
        else alert_state = 0; // Bình thường

        // Cập nhật PPM trên LCD
        if (ppm_int != last_ppm) {
            LCD_SendCommand(0x80 + 4); // Sau "PPM:"
            snprintf(lcd_buf, sizeof(lcd_buf), "%4lu", ppm_int);
            LCD_SendString(lcd_buf);
            last_ppm = ppm_int;
        }

        // Cập nhật Alert trên LCD
        if (last_alert_state != alert_state) {
            LCD_SendCommand(0xC0 + 13); // Sau "Alert:"
            snprintf(lcd_buf, sizeof(lcd_buf), "%d", alert_state);
            LCD_SendString(lcd_buf);
            last_alert_state = alert_state;
        }

        // Điều khiển LED, Relay, Buzzer
        switch (alert_state) {
            case 0: // Bình thường
                LED_Set(0, 0, 1, 0); // Xanh dương
                GPIOA->ODR &= ~(1 << RELAY_PIN); // Relay OFF
                Buzzer_Set(0);
                TIM2->CR1 &= ~TIM_CR1_CEN; // Tắt TIM2
                break;
            case 1: // Thấp
                LED_Set(0, 1, 0, 0); // Vàng
                GPIOA->ODR &= ~(1 << RELAY_PIN); // Relay OFF
                Buzzer_Set(0);
                TIM2->CR1 &= ~TIM_CR1_CEN; // Tắt TIM2
                break;
            case 2: // Cao (500 < PPM ≤ 800)
                TIM2->CR1 |= TIM_CR1_CEN; // Bật TIM2
                TIM2->ARR = 999; // 1Hz (500ms chu kỳ)
                LED_Set(tim2_blink_state, 0, 0, 0); // Đỏ nhấp nháy
                GPIOA->ODR |= (1 << RELAY_PIN); // Relay ON
                Buzzer_Set(1);
                break;
            case 3: // Nguy hiểm (PPM > 800)
                TIM2->CR1 |= TIM_CR1_CEN; // Bật TIM2
                TIM2_UpdateFrequency(ppm); // Cập nhật tần số dựa trên PPM
                LED_Set(tim2_blink_state, 0, 0, 0); // Đỏ nhấp nháy
                GPIOA->ODR |= (1 << RELAY_PIN); // Relay ON
                Buzzer_Set(1);
                break;
        }

        delay_ms(50);
    }
}
