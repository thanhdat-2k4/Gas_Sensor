//
//#include "stm32f4xx.h"
//#include <stdint.h>
//#include <stdio.h>
//#include <string.h>
//
//// === Cấu hình chân ===
//#define MQ2_CHANNEL     0       // PA0 - ADC1_IN0
//#define RELAY_PIN       5       // PA5
//#define BUZZER_PIN      6       // PA6
//#define LED_BLUE_PIN    7       // PA7 - Xanh dương
//#define LED_YELLOW_PIN  0       // PB0 - Vàng
//#define LED_RED_PIN     1       // PB1 - Đỏ
//#define LED_GREEN_PIN   10      // PB10 - Xanh lá
//#define SW1_PIN         13      // PC13
//#define SW2_PIN         12      // PC12
//#define LCD_I2C_ADDR    0x27    // Địa chỉ I2C LCD (thử 0x3F nếu lỗi)
//
//// LED cathode-common (1 = ON)
//#define LED_CATHODE_COMMON 1
//// Buzzer active-high (1 = ON)
//#define BUZZER_ACTIVE_HIGH 1
//
//uint8_t system_state = 1; // Bắt đầu ON
//uint8_t alert_state = 0;
//volatile uint32_t tick_count = 0; // Biến toàn cục cho SysTick
//volatile uint8_t tim2_blink_state = 0; // Trạng thái nhấp nháy cho TIM2
//
//// === Khai báo hàm trước khi sử dụng ===
//void UART_Transmit(char *data);
//
//// === Hàm delay ===
//void delay_ms(uint32_t ms) {
//    for (uint32_t i = 0; i < ms * 8000; i++) __NOP();
//}
//
//void delay_us(uint32_t us) {
//    for (uint32_t i = 0; i < us * 8; i++) __NOP();
//}
//
//// === Cấu hình GPIO ===
//void GPIO_Config(void) {
//    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
//
//    // PA0 (MQ2) - Analog
//    GPIOA->MODER |= (3 << (MQ2_CHANNEL * 2));
//
//    // PA5 (Relay), PA6 (Buzzer), PA7 (LED Blue) - Output
//    GPIOA->MODER &= ~((3 << (RELAY_PIN * 2)) | (3 << (BUZZER_PIN * 2)) | (3 << (LED_BLUE_PIN * 2)));
//    GPIOA->MODER |= ((1 << (RELAY_PIN * 2)) | (1 << (BUZZER_PIN * 2)) | (1 << (LED_BLUE_PIN * 2)));
//    GPIOA->ODR &= ~(1 << RELAY_PIN); // Relay OFF
//    GPIOA->ODR &= ~(1 << BUZZER_PIN); // Buzzer OFF
//    GPIOA->ODR &= ~(1 << LED_BLUE_PIN); // Blue LED OFF
//
//    // PB0 (Yellow), PB1 (Red), PB10 (Green) - Output
//    GPIOB->MODER &= ~((3 << (LED_YELLOW_PIN * 2)) | (3 << (LED_RED_PIN * 2)) | (3 << (LED_GREEN_PIN * 2)));
//    GPIOB->MODER |= ((1 << (LED_YELLOW_PIN * 2)) | (1 << (LED_RED_PIN * 2)) | (1 << (LED_GREEN_PIN * 2)));
//    GPIOB->ODR &= ~((1 << LED_YELLOW_PIN) | (1 << LED_RED_PIN) | (1 << LED_GREEN_PIN)); // LEDs OFF
//
//    // PC12, PC13 - Input Pull-up
//    GPIOC->MODER &= ~((3 << (SW1_PIN * 2)) | (3 << (SW2_PIN * 2)));
//    GPIOC->PUPDR &= ~((3 << (SW1_PIN * 2)) | (3 << (SW2_PIN * 2)));
//    GPIOC->PUPDR |= ((1 << (SW1_PIN * 2)) | (1 << (SW2_PIN * 2)));
//}
//
//// === Cấu hình I2C1 (PB8 - SCL, PB9 - SDA) ===
//void I2C_Config(void) {
//    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
//    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
//
//    GPIOB->MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
//    GPIOB->MODER |= ((2 << (8 * 2)) | (2 << (9 * 2)));
//    GPIOB->OTYPER |= (1 << 8) | (1 << 9);
//    GPIOB->PUPDR |= (1 << (8 * 2)) | (1 << (9 * 2));
//    GPIOB->AFR[1] |= (4 << ((8 - 8) * 4)) | (4 << ((9 - 8) * 4));
//
//    I2C1->CR1 = 0;
//    I2C1->CR2 = 84; // 84MHz APB1
//    I2C1->CCR = 420; // 100kHz
//    I2C1->TRISE = 85;
//    I2C1->CR1 |= I2C_CR1_PE;
//}
//
//uint8_t I2C_WriteByte(uint8_t addr, uint8_t data, const char *context) {
//    uint32_t timeout = 100000;
//    char debug_buf[32];
//    while (I2C1->SR2 & I2C_SR2_BUSY && timeout--) {
//        if (timeout == 0) {
//            snprintf(debug_buf, sizeof(debug_buf), "%s I2C Busy\r\n", context);
//            UART_Transmit(debug_buf);
//            return 1;
//        }
//    }
//    I2C1->CR1 |= I2C_CR1_START;
//    timeout = 100000;
//    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout--) {
//        if (timeout == 0) {
//            snprintf(debug_buf, sizeof(debug_buf), "%s I2C Start\r\n", context);
//            UART_Transmit(debug_buf);
//            return 2;
//        }
//    }
//    I2C1->DR = (addr << 1);
//    timeout = 100000;
//    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout--) {
//        if (timeout == 0) {
//            snprintf(debug_buf, sizeof(debug_buf), "%s I2C Addr\r\n", context);
//            UART_Transmit(debug_buf);
//            return 3;
//        }
//    }
//    (void)I2C1->SR2;
//    timeout = 100000;
//    while (!(I2C1->SR1 & I2C_SR1_TXE) && timeout--) {
//        if (timeout == 0) {
//            snprintf(debug_buf, sizeof(debug_buf), "%s I2C TXE\r\n", context);
//            UART_Transmit(debug_buf);
//            return 4;
//        }
//    }
//    I2C1->DR = data;
//    timeout = 100000;
//    while (!(I2C1->SR1 & I2C_SR1_BTF) && timeout--) {
//        if (timeout == 0) {
//            snprintf(debug_buf, sizeof(debug_buf), "%s I2C BTF\r\n", context);
//            UART_Transmit(debug_buf);
//            return 5;
//        }
//    }
//    I2C1->CR1 |= I2C_CR1_STOP;
//    return 0;
//}
//
//void LCD_SendCommand(uint8_t cmd) {
//    uint8_t data_u = (cmd & 0xF0);
//    uint8_t data_l = ((cmd << 4) & 0xF0);
//    uint8_t data_t[4] = { data_u | 0x0C, data_u | 0x08, data_l | 0x0C, data_l | 0x08 };
//    for (int i = 0; i < 4; i++) {
//        I2C_WriteByte(LCD_I2C_ADDR, data_t[i], "CMD");
//        delay_us(200);
//    }
//}
//
//void LCD_SendData(uint8_t data) {
//    uint8_t data_u = (data & 0xF0);
//    uint8_t data_l = ((data << 4) & 0xF0);
//    uint8_t data_t[4] = { data_u | 0x0D, data_u | 0x09, data_l | 0x0D, data_l | 0x09 };
//    for (int i = 0; i < 4; i++) {
//        I2C_WriteByte(LCD_I2C_ADDR, data_t[i], "DATA");
//        delay_us(200);
//    }
//}
//
//uint8_t LCD_Init(void) {
//    char debug_buf[32];
//    uint8_t retry = 3;
//    while (retry--) {
//        delay_ms(50);
//        if (I2C_WriteByte(LCD_I2C_ADDR, 0x00, "INIT") == 0) {
//            LCD_SendCommand(0x33);
//            delay_ms(5);
//            LCD_SendCommand(0x32);
//            delay_us(50);
//            LCD_SendCommand(0x28); // 4-bit, 2 dòng
//            delay_us(50);
//            LCD_SendCommand(0x0C); // Bật hiển thị
//            delay_us(50);
//            LCD_SendCommand(0x06); // Chế độ nhập
//            delay_us(50);
//            LCD_SendCommand(0x01); // Xóa màn
//            delay_ms(2);
//            snprintf(debug_buf, sizeof(debug_buf), "LCD Init OK\r\n");
//            UART_Transmit(debug_buf);
//            return 0;
//        }
//        snprintf(debug_buf, sizeof(debug_buf), "LCD Init Retry %d\r\n", retry);
//        UART_Transmit(debug_buf);
//        delay_ms(50);
//    }
//    snprintf(debug_buf, sizeof(debug_buf), "LCD Init Failed\r\n");
//    UART_Transmit(debug_buf);
//    return 1;
//}
//
//void LCD_SendString(char *str) {
//    char debug_buf[32];
//    snprintf(debug_buf, sizeof(debug_buf), "LCD Send: %s\r\n", str);
//    UART_Transmit(debug_buf);
//    while (*str && *str != '\0') LCD_SendData(*str++);
//}
//
//// === Cấu hình ADC1 cho PA0 ===
//void ADC_Config(void) {
//    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
//    ADC1->CR2 = 0;
//    ADC1->SQR3 = MQ2_CHANNEL;
//    ADC1->SMPR2 |= (7 << (MQ2_CHANNEL * 3)); // 480 chu kỳ
//    ADC1->CR2 |= ADC_CR2_ADON;
//    delay_us(10);
//}
//
//uint16_t ADC_Read(void) {
//    ADC1->CR2 |= ADC_CR2_SWSTART;
//    while (!(ADC1->SR & ADC_SR_EOC));
//    return ADC1->DR;
//}
//
//// === Cấu hình UART2 (PA2 - TX, PA3 - RX) ===
//void UART_Config(void) {
//    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
//    GPIOA->MODER &= ~((3 << (2 * 2)) | (3 << (3 * 2)));
//    GPIOA->MODER |= ((2 << (2 * 2)) | (2 << (3 * 2)));
//    GPIOA->AFR[0] |= (7 << (2 * 4)) | (7 << (3 * 4));
//    USART2->BRR = 84000000 / 115200;
//    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
//}
//
//void UART_Transmit(char *data) {
//    while (*data && *data != '\0') {
//        while (!(USART2->SR & USART_SR_TXE));
//        USART2->DR = *data++;
//    }
//}
//
//// === Điều khiển LED ===
//void LED_Set(uint8_t r, uint8_t y, uint8_t b, uint8_t g) {
//    if (LED_CATHODE_COMMON) {
//        GPIOA->ODR = (GPIOA->ODR & ~(1 << LED_BLUE_PIN)) | (b << LED_BLUE_PIN);
//        GPIOB->ODR = (GPIOB->ODR & ~((1 << LED_RED_PIN) | (1 << LED_YELLOW_PIN) | (1 << LED_GREEN_PIN))) |
//                     (r << LED_RED_PIN) | (y << LED_YELLOW_PIN) | (g << LED_GREEN_PIN);
//    } else {
//        GPIOA->ODR = (GPIOA->ODR & ~(1 << LED_BLUE_PIN)) | ((b ? 0 : 1) << LED_BLUE_PIN);
//        GPIOB->ODR = (GPIOB->ODR & ~((1 << LED_RED_PIN) | (1 << LED_YELLOW_PIN) | (1 << LED_GREEN_PIN))) |
//                     ((r ? 0 : 1) << LED_RED_PIN) | ((y ? 0 : 1) << LED_YELLOW_PIN) | ((g ? 0 : 1) << LED_GREEN_PIN);
//    }
//    char debug_buf[80];
//    snprintf(debug_buf, sizeof(debug_buf), "LED Set: R=%u, Y=%u, B=%u, G=%u, ODR: PA7=%lu, PB0=%lu, PB1=%lu, PB10=%lu\r\n",
//             r, y, b, g,
//             (GPIOA->ODR >> LED_BLUE_PIN) & 1,
//             (GPIOB->ODR >> LED_YELLOW_PIN) & 1,
//             (GPIOB->ODR >> LED_RED_PIN) & 1,
//             (GPIOB->ODR >> LED_GREEN_PIN) & 1);
//    UART_Transmit(debug_buf);
//}
//
//// === Điều khiển Buzzer ===
//void Buzzer_Set(uint8_t state) {
//    if (BUZZER_ACTIVE_HIGH) {
//        GPIOA->ODR = (GPIOA->ODR & ~(1 << BUZZER_PIN)) | (state << BUZZER_PIN);
//    } else {
//        GPIOA->ODR = (GPIOA->ODR & ~(1 << BUZZER_PIN)) | ((state ? 0 : 1) << BUZZER_PIN);
//    }
//    char debug_buf[32];
//    snprintf(debug_buf, sizeof(debug_buf), "Buzzer: %u, PA6=%lu\r\n", state, (GPIOA->ODR >> BUZZER_PIN) & 1);
//    UART_Transmit(debug_buf);
//}
//
//// === Cấu hình TIM2 cho nhấp nháy 5Hz ===
//void TIM2_Config(void) {
//    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Bật clock cho TIM2
//    TIM2->PSC = 8399; // Prescaler = 8400 - 1
//    TIM2->ARR = 999; // Auto-reload = 1000 - 1 (100ms)
//    TIM2->DIER |= TIM_DIER_UIE; // Bật ngắt update
//    TIM2->CR1 |= TIM_CR1_CEN; // Bật timer
//    NVIC_EnableIRQ(TIM2_IRQn); // Bật ngắt TIM2
//}
//
//// === Tính PPM cho MQ-2 ===
//float MQ2_GetPPM(uint16_t adc_value) {
//    float voltage = (adc_value / 4096.0f) * 3.3f; // Điện áp tham chiếu 3.3V
//    float k = 300.0f; // Ánh xạ 0-3.3V thành 0-990ppm
//    float c = 0.0f;
//    float ppm = k * voltage + c;
//    if (ppm < 0) ppm = 0;
//    return ppm;
//}
//
//// === Main ===
//int main(void) {
//    GPIO_Config();
//    ADC_Config();
//    I2C_Config();
//    UART_Config();
//    TIM2_Config(); // Cấu hình TIM2
//    if (LCD_Init()) {
//        char debug_buf[32];
//        snprintf(debug_buf, sizeof(debug_buf), "Try LCD_I2C_ADDR 0x3F\r\n");
//        UART_Transmit(debug_buf);
//    }
//
//    // Test LED để kiểm tra phần cứng
//    LED_Set(1, 0, 0, 0); // Bật LED đỏ
//    delay_ms(100);
//    LED_Set(0, 0, 0, 1); // Bật LED xanh lá
//    delay_ms(100);
//    LED_Set(0, 1, 0, 0); // Bật LED vàng
//    delay_ms(100);
//    LED_Set(0, 0, 1, 0); // Bật LED xanh dương
//    delay_ms(100);
//    LED_Set(0, 0, 0, 0); // Tắt tất cả LED
//
//    char debug_buf[80];
//    snprintf(debug_buf, sizeof(debug_buf), "Init: PA7=%lu, PB0=%lu, PB1=%lu, PB10=%lu, PA6=%lu\r\n",
//             (GPIOA->ODR >> LED_BLUE_PIN) & 1,
//             (GPIOB->ODR >> LED_YELLOW_PIN) & 1,
//             (GPIOB->ODR >> LED_RED_PIN) & 1,
//             (GPIOB->ODR >> LED_GREEN_PIN) & 1,
//             (GPIOA->ODR >> BUZZER_PIN) & 1);
//    UART_Transmit(debug_buf);
//
//    // Khởi tạo hiển thị LCD ban đầu
//    LCD_SendCommand(0x01); // Xóa màn hình
//    LCD_SendCommand(0x80); // Đặt con trỏ dòng 1
//    LCD_SendString("Gas Detector");
//    LCD_SendCommand(0xC0); // Đặt con trỏ dòng 2
//    LCD_SendString("Starting...");
//    delay_ms(1000); // Chờ 1 giây để hiển thị thông báo khởi động
//
//    // Hiển thị giao diện chính
//    LCD_SendCommand(0x01); // Xóa màn hình
//    LCD_SendCommand(0x80); // Đặt con trỏ dòng 1
//    LCD_SendString("PPM:----"); // Chuỗi tĩnh cho dòng 1
//    LCD_SendCommand(0xC0); // Đặt con trỏ dòng 2
//    LCD_SendString("SYS:ON Alert:-"); // Chuỗi tĩnh cho dòng 2
//    delay_ms(50);
//
//    char lcd_buf[17];
//    uint8_t last_system_state = system_state; // Lưu trạng thái hệ thống trước đó
//    uint8_t last_alert_state = 255; // Giá trị không hợp lệ để ép cập nhật lần đầu
//    uint32_t last_ppm = 0xFFFFFFFF; // Giá trị không hợp lệ để ép cập nhật lần đầu
//
//    SysTick->LOAD = 84000000 / 1000 - 1; // 1ms
//    SysTick->VAL = 0;
//    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
//
//    NVIC_EnableIRQ(SysTick_IRQn);
//
//    while (1) {
//        static uint8_t last_sw1 = 1, last_sw2 = 1;
//        uint8_t sw1 = (GPIOC->IDR & (1 << SW1_PIN)) ? 1 : 0;
//        uint8_t sw2 = (GPIOC->IDR & (1 << SW2_PIN)) ? 1 : 0;
//
//        // Xử lý SW1: Chuyển đổi trạng thái hệ thống
//        if (last_sw1 && !sw1) {
//            system_state = !system_state;
//            snprintf(debug_buf, sizeof(debug_buf), "SW1 Pressed: System State=%u\r\n", system_state);
//            UART_Transmit(debug_buf);
//        }
//        last_sw1 = sw1;
//
//        // Xử lý SW2: Reset hệ thống
//        if (last_sw2 && !sw2) {
//            system_state = 1;
//            alert_state = 0;
//            tim2_blink_state = 0;
//            LED_Set(0, 0, 0, 0); // Tắt tất cả LED
//            GPIOA->ODR &= ~(1 << RELAY_PIN); // Relay OFF
//            Buzzer_Set(0); // Buzzer OFF
//            TIM2->CR1 &= ~TIM_CR1_CEN; // Tắt TIM2
//            LCD_SendCommand(0x01); // Xóa màn hình
//            LCD_SendCommand(0x80); // Đặt con trỏ dòng 1
//            LCD_SendString("Gas Detector");
//            LCD_SendCommand(0xC0); // Đặt con trỏ dòng 2
//            LCD_SendString("Starting...");
//            snprintf(debug_buf, sizeof(debug_buf), "SW2 Pressed: System Reset\r\n");
//            UART_Transmit(debug_buf);
//            delay_ms(1000); // Chờ 1 giây
//            LCD_SendCommand(0x01); // Xóa màn hình
//            LCD_SendCommand(0x80); // Đặt con trỏ dòng 1
//            LCD_SendString("PPM:----"); // Chuỗi tĩnh
//            LCD_SendCommand(0xC0); // Đặt con trỏ dòng 2
//            LCD_SendString("SYS:ON Alert:-"); // Chuỗi tĩnh
//            last_system_state = system_state;
//            last_alert_state = 255; // Ép cập nhật alert
//            last_ppm = 0xFFFFFFFF; // Ép cập nhật PPM
//        }
//        last_sw2 = sw2;
//
//        // Trạng thái hệ thống dừng
//        if (!system_state) {
//            if (last_system_state != system_state) {
//                LED_Set(0, 0, 0, 1); // Xanh lá
//                GPIOA->ODR &= ~(1 << RELAY_PIN); // Relay OFF
//                Buzzer_Set(0);
//                TIM2->CR1 &= ~TIM_CR1_CEN; // Tắt TIM2
//                LCD_SendCommand(0x01); // Xóa màn hình
//                LCD_SendCommand(0x80); // Đặt con trỏ dòng 1
//                LCD_SendString("SYS:OFF");
//                LCD_SendCommand(0xC0); // Đặt con trỏ dòng 2
//                LCD_SendString("PPM:--- Alert:0");
//                snprintf(debug_buf, sizeof(debug_buf), "System: OFF, PPM: ---\r\n");
//                UART_Transmit(debug_buf);
//                last_system_state = system_state;
//                last_alert_state = 255; // Ép cập nhật alert
//                last_ppm = 0xFFFFFFFF; // Ép cập nhật PPM
//            }
//            delay_ms(50);
//            continue;
//        }
//
//        // Cập nhật trạng thái hệ thống nếu thay đổi
//        if (last_system_state != system_state) {
//            LCD_SendCommand(0x01); // Xóa màn hình
//            LCD_SendCommand(0x80); // Đặt con trỏ dòng 1
//            LCD_SendString("PPM:----"); // Chuỗi tĩnh
//            LCD_SendCommand(0xC0); // Đặt con trỏ dòng 2
//            LCD_SendString("SYS:ON Alert:-"); // Chuỗi tĩnh
//            last_system_state = system_state;
//            last_alert_state = 255; // Ép cập nhật alert
//            last_ppm = 0xFFFFFFFF; // Ép cập nhật PPM
//        }
//
//        // Đọc và tính toán nồng độ khí
//        uint16_t adc_val = ADC_Read();
//        float ppm = MQ2_GetPPM(adc_val);
//        uint32_t ppm_int = (uint32_t)ppm; // Chuyển đổi sang số nguyên để so sánh
//
//        snprintf(debug_buf, sizeof(debug_buf), "ADC: %u, V: %.2f, PPM: %.0f, Alert: %d\r\n",
//                 adc_val, (adc_val / 4096.0f) * 3.3f, ppm, alert_state);
//        UART_Transmit(debug_buf);
//
//        // Xác định trạng thái cảnh báo
//        if (ppm > 800) {
//            alert_state = 3; // Nguy hiểm
//        } else if (ppm > 500) {
//            alert_state = 2; // Cao
//        } else if (ppm > 300) {
//            alert_state = 1; // Thấp
//        } else {
//            alert_state = 0; // Bình thường
//        }
//
//        // Cập nhật giá trị PPM trên LCD nếu thay đổi
//        if (ppm_int != last_ppm) {
//            LCD_SendCommand(0x80 + 4); // Đặt con trỏ sau "PPM:"
//            snprintf(lcd_buf, sizeof(lcd_buf), "%4lu", ppm_int); // Định dạng 4 ký tự
//            LCD_SendString(lcd_buf);
//            last_ppm = ppm_int;
//        }
//
//        // Cập nhật trạng thái Alert trên LCD nếu thay đổi
//        if (last_alert_state != alert_state) {
//            LCD_SendCommand(0xC0 + 13); // Đặt con trỏ sau "Alert:"
//            snprintf(lcd_buf, sizeof(lcd_buf), "%d", alert_state);
//            LCD_SendString(lcd_buf);
//            last_alert_state = alert_state;
//        }
//
//        // Điều khiển LED, Relay, Buzzer theo trạng thái
//        switch (alert_state) {
//            case 0: // Bình thường
//                LED_Set(0, 0, 1, 0); // Xanh dương
//                GPIOA->ODR &= ~(1 << RELAY_PIN); // Relay OFF
//                Buzzer_Set(0);
//                TIM2->CR1 &= ~TIM_CR1_CEN; // Tắt TIM2
//                break;
//            case 1: // Thấp
//                LED_Set(0, 1, 0, 0); // Vàng
//                GPIOA->ODR &= ~(1 << RELAY_PIN); // Relay OFF
//                Buzzer_Set(0);
//                TIM2->CR1 &= ~TIM_CR1_CEN; // Tắt TIM2
//                break;
//            case 2: // Cao
//                TIM2->CR1 |= TIM_CR1_CEN; // Bật TIM2
//                LED_Set(tim2_blink_state, 0, 0, 0); // Chỉ LED đỏ nhấp nháy
//                GPIOA->ODR |= (1 << RELAY_PIN); // Relay ON
//                Buzzer_Set(0);
//                break;
//            case 3: // Nguy hiểm
//                TIM2->CR1 |= TIM_CR1_CEN; // Bật TIM2
//                LED_Set(tim2_blink_state, 0, 0, 0); // Chỉ LED đỏ nhấp nháy
//                GPIOA->ODR |= (1 << RELAY_PIN); // Relay ON
//                Buzzer_Set(1);
//                break;
//        }
//
//        delay_ms(50);
//    }
//}
//
//// === Xử lý ngắt TIM2 ===
//void TIM2_IRQHandler(void) {
//    if (TIM2->SR & TIM_SR_UIF) { // Kiểm tra cờ update
//        TIM2->SR &= ~TIM_SR_UIF; // Xóa cờ
//        tim2_blink_state = !tim2_blink_state; // Đảo trạng thái
//        char debug_buf[32];
//        snprintf(debug_buf, sizeof(debug_buf), "TIM2: Blink=%u\r\n", tim2_blink_state);
//        UART_Transmit(debug_buf);
//    }
//}
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

uint8_t system_state = 1; // Bắt đầu ON
uint8_t alert_state = 0;
volatile uint32_t tick_count = 0; // Biến toàn cục cho SysTick
volatile uint8_t tim2_blink_state = 0; // Trạng thái nhấp nháy cho TIM2

// === Khai báo hàm trước khi sử dụng ===
void UART1_Transmit(char *data); // Debug
void UART2_Transmit(char *data); // Gửi PPM tới ESP32

// === Hàm delay ===
void delay_ms(uint32_t ms) {
    uint32_t start = tick_count;
    while ((tick_count - start) < ms) {}
}

void delay_us(uint32_t us) {
    for (uint32_t i = 0; i < us * 8; i++) __NOP(); // Tạm giữ, có thể thay bằng SysTick nếu cần
}

// === Cấu hình clock 84 MHz ===
void SystemClock_Config(void) {
    // Bật HSE
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    // Cấu hình PLL: PLLM = 8, PLLN = 336, PLLP = 4 -> 84 MHz
    RCC->PLLCFGR = (8 << 0) | (336 << 6) | (1 << 16); // PLLP = 4
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // Chọn PLL làm nguồn clock hệ thống
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // Cấu hình bus clock: AHB = 84 MHz, APB1 = 42 MHz, APB2 = 84 MHz
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;
}

// === Cấu hình GPIO ===
void GPIO_Config(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    // PA0 (MQ2) - Analog
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

    // PC12, PC13 - Input Pull-up
    GPIOC->MODER &= ~((3 << (SW1_PIN * 2)) | (3 << (SW2_PIN * 2)));
    GPIOC->PUPDR &= ~((3 << (SW1_PIN * 2)) | (3 << (SW2_PIN * 2)));
    GPIOC->PUPDR |= ((1 << (SW1_PIN * 2)) | (1 << (SW2_PIN * 2)));
}

// === Cấu hình I2C1 (PB8 - SCL, PB9 - SDA) ===
void I2C_Config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    GPIOB->MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
    GPIOB->MODER |= ((2 << (8 * 2)) | (2 << (9 * 2)));
    GPIOB->OTYPER |= (1 << 8) | (1 << 9);
    GPIOB->PUPDR |= (1 << (8 * 2)) | (1 << (9 * 2));
    GPIOB->AFR[1] |= (4 << ((8 - 8) * 4)) | (4 << ((9 - 8) * 4));

    I2C1->CR1 = 0;
    I2C1->CR2 = 42; // APB1 = 42 MHz
    I2C1->CCR = 210; // 100 kHz
    I2C1->TRISE = 43; // TRISE = (1000 ns / (1/42 MHz)) + 1
    I2C1->CR1 |= I2C_CR1_PE;
}

uint8_t I2C_WriteByte(uint8_t addr, uint8_t data, const char *context) {
    uint32_t timeout = 100000;
    char debug_buf[32];
    while (I2C1->SR2 & I2C_SR2_BUSY && timeout--) {
        if (timeout == 0) {
            snprintf(debug_buf, sizeof(debug_buf), "%s I2C Busy\r\n", context);
            UART1_Transmit(debug_buf);
            return 1;
        }
    }
    I2C1->CR1 |= I2C_CR1_START;
    timeout = 100000;
    while (!(I2C1->SR1 & I2C_SR1_SB) && timeout--) {
        if (timeout == 0) {
            snprintf(debug_buf, sizeof(debug_buf), "%s I2C Start\r\n", context);
            UART1_Transmit(debug_buf);
            return 2;
        }
    }
    I2C1->DR = (addr << 1);
    timeout = 100000;
    while (!(I2C1->SR1 & I2C_SR1_ADDR) && timeout--) {
        if (timeout == 0) {
            snprintf(debug_buf, sizeof(debug_buf), "%s I2C Addr\r\n", context);
            UART1_Transmit(debug_buf);
            return 3;
        }
    }
    (void)I2C1->SR2;
    timeout = 100000;
    while (!(I2C1->SR1 & I2C_SR1_TXE) && timeout--) {
        if (timeout == 0) {
            snprintf(debug_buf, sizeof(debug_buf), "%s I2C TXE\r\n", context);
            UART1_Transmit(debug_buf);
            return 4;
        }
    }
    I2C1->DR = data;
    timeout = 100000;
    while (!(I2C1->SR1 & I2C_SR1_BTF) && timeout--) {
        if (timeout == 0) {
            snprintf(debug_buf, sizeof(debug_buf), "%s I2C BTF\r\n", context);
            UART1_Transmit(debug_buf);
            return 5;
        }
    }
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0;
}

void LCD_SendCommand(uint8_t cmd) {
    uint8_t data_u = (cmd & 0xF0);
    uint8_t data_l = ((cmd << 4) & 0xF0);
    uint8_t data_t[4] = { data_u | 0x0C, data_u | 0x08, data_l | 0x0C, data_l | 0x08 };
    for (int i = 0; i < 4; i++) {
        I2C_WriteByte(LCD_I2C_ADDR, data_t[i], "CMD");
        delay_us(200);
    }
}

void LCD_SendData(uint8_t data) {
    uint8_t data_u = (data & 0xF0);
    uint8_t data_l = ((data << 4) & 0xF0);
    uint8_t data_t[4] = { data_u | 0x0D, data_u | 0x09, data_l | 0x0D, data_l | 0x09 };
    for (int i = 0; i < 4; i++) {
        I2C_WriteByte(LCD_I2C_ADDR, data_t[i], "DATA");
        delay_us(100);
    }
}

uint8_t LCD_Init(void) {
    char debug_buf[32];
    uint8_t retry = 3;
    while (retry--) {
        delay_ms(50);
        if (I2C_WriteByte(LCD_I2C_ADDR, 0x00, "INIT") == 0) {
            LCD_SendCommand(0x33);
            delay_ms(5);
            LCD_SendCommand(0x32);
            delay_us(50);
            LCD_SendCommand(0x28); // 4-bit, 2 dòng
            delay_us(50);
            LCD_SendCommand(0x0C); // Bật hiển thị
            delay_us(50);
            LCD_SendCommand(0x06); // Chế độ nhập
            delay_us(50);
            LCD_SendCommand(0x01); // Xóa màn hình
            delay_ms(2);
            snprintf(debug_buf, sizeof(debug_buf), "LCD Init OK\r\n");
            UART1_Transmit(debug_buf);
            return 0;
        }
        snprintf(debug_buf, sizeof(debug_buf), "LCD Init Retry %d\r\n", retry);
        UART1_Transmit(debug_buf);
        delay_ms(50);
    }
    snprintf(debug_buf, sizeof(debug_buf), "LCD Init Failed\r\n");
    UART1_Transmit(debug_buf);
    return 1;
}

void LCD_SendString(char *str) {
    char debug_buf[32];
    snprintf(debug_buf, sizeof(debug_buf), "LCD Send: %s\r\n", str);
    UART1_Transmit(debug_buf);
    while (*str && *str != '\0') LCD_SendData(*str++);
}

// === Cấu hình ADC1 cho PA0 ===
void ADC_Config(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->CR2 = 0;
    ADC1->SQR3 = MQ2_CHANNEL;
    ADC1->SMPR2 |= (7 << (MQ2_CHANNEL * 3)); // 480 chu kỳ
    ADC1->CR2 |= ADC_CR2_ADON;
    delay_us(10);
}

uint16_t ADC_Read(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}

// === Cấu hình UART1 (PA9 - TX) cho debug ===
void UART1_Config(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(3 << (9 * 2));
    GPIOA->MODER |= (2 << (9 * 2));
    GPIOA->AFR[1] |= (7 << ((9 - 8) * 4));
    USART1->BRR = 84000000 / 115200;
    USART1->CR1 |= USART_CR1_UE | USART_CR1_TE;
}

void UART1_Transmit(char *data) {
    while (*data && *data != '\0') {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = *data++;
    }
}

// === Cấu hình UART2 (PA2 - TX) cho ESP32 ===
void UART2_Config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(3 << (2 * 2));
    GPIOA->MODER |= (2 << (2 * 2));
    GPIOA->AFR[0] |= (7 << (2 * 4));
    USART2->BRR = 84000000 / 9600; // Baud rate 9600
    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE; // Chỉ TX
}

void UART2_Transmit(char *data) {
    char debug_buf[32];
    snprintf(debug_buf, sizeof(debug_buf), "UART2 Sending: %s", data);
    UART1_Transmit(debug_buf);
    while (*data && *data != '\0') {
        while (!(USART2->SR & USART_SR_TXE));
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
}

// === Điều khiển Buzzer ===
void Buzzer_Set(uint8_t state) {
    if (BUZZER_ACTIVE_HIGH) {
        GPIOA->ODR = (GPIOA->ODR & ~(1 << BUZZER_PIN)) | (state << BUZZER_PIN);
    } else {
        GPIOA->ODR = (GPIOA->ODR & ~(1 << BUZZER_PIN)) | ((state ? 0 : 1) << BUZZER_PIN);
    }
}

// === Cấu hình TIM2 cho nhấp nháy 5Hz ===
void TIM2_Config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 4199; // Prescaler = 4200 - 1 (APB1 = 42 MHz)
    TIM2->ARR = 999; // Auto-reload = 1000 - 1 (100ms)
    TIM2->DIER |= TIM_DIER_UIE; // Bật ngắt update
    TIM2->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM2_IRQn);
}

// === Tính PPM cho MQ-2 ===
float MQ2_GetPPM(uint16_t adc_value) {
    float voltage = (adc_value / 4096.0f) * 3.3f; // Điện áp tham chiếu 3.3V
    float k = 300.0f; // Ánh xạ 0-3.3V thành 0-990ppm
    float c = 0.0f;
    float ppm = k * voltage + c;
    if (ppm < 0) ppm = 0;
    return ppm;
}

// === Xử lý ngắt SysTick ===
void SysTick_Handler(void) {
    tick_count++;
}

// === Xử lý ngắt TIM2 ===
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        tim2_blink_state = !tim2_blink_state;
    }
}

// === Main ===
int main(void) {
    SystemClock_Config();
    SysTick->LOAD = 84000000 / 1000 - 1; // 1ms
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    NVIC_EnableIRQ(SysTick_IRQn);

    GPIO_Config();
    UART1_Config();
    UART2_Config();
    ADC_Config();
    I2C_Config();
    TIM2_Config();

    char debug_buf[32];
    if (LCD_Init()) {
        snprintf(debug_buf, sizeof(debug_buf), "Try LCD_I2C_ADDR 0x3F\r\n");
        UART1_Transmit(debug_buf);
    }

    // Test LED
    LED_Set(1, 0, 0, 0); delay_ms(100);
    LED_Set(0, 0, 0, 1); delay_ms(100);
    LED_Set(0, 1, 0, 0); delay_ms(100);
    LED_Set(0, 0, 1, 0); delay_ms(100);
    LED_Set(0, 0, 0, 0);

    // Khởi tạo LCD
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

    char lcd_buf[17];
    char uart_buf[16];
    uint8_t last_system_state = system_state;
    uint8_t last_alert_state = 255;
    uint32_t last_ppm = 0xFFFFFFFF;

    while (1) {
        static uint8_t last_sw1 = 1, last_sw2 = 1;
        uint8_t sw1 = (GPIOC->IDR & (1 << SW1_PIN)) ? 1 : 0;
        uint8_t sw2 = (GPIOC->IDR & (1 << SW2_PIN)) ? 1 : 0;

        // Xử lý SW1: Chuyển trạng thái hệ thống
        if (last_sw1 && !sw1) {
            system_state = !system_state;
        }
        last_sw1 = sw1;

        // Xử lý SW2: Reset hệ thống
        if (last_sw2 && !sw2) {
            system_state = 1;
            alert_state = 0;
            tim2_blink_state = 0;
            LED_Set(0, 0, 0, 0);
            GPIOA->ODR &= ~(1 << RELAY_PIN);
            Buzzer_Set(0);
            TIM2->CR1 &= ~TIM_CR1_CEN;
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
            last_system_state = system_state;
            last_alert_state = 255;
            last_ppm = 0xFFFFFFFF;
        }
        last_sw2 = sw2;

        // Trạng thái hệ thống dừng
        if (!system_state) {
            if (last_system_state != system_state) {
                LED_Set(0, 0, 0, 1);
                GPIOA->ODR &= ~(1 << RELAY_PIN);
                Buzzer_Set(0);
                TIM2->CR1 &= ~TIM_CR1_CEN;
                LCD_SendCommand(0x01);
                LCD_SendCommand(0x80);
                LCD_SendString("SYS:OFF");
                LCD_SendCommand(0xC0);
                LCD_SendString("PPM:--- Alert:0");
                last_system_state = system_state;
                last_alert_state = 255;
                last_ppm = 0xFFFFFFFF;
                snprintf(uart_buf, sizeof(uart_buf), "0.0\n");
                UART2_Transmit(uart_buf);
            }
            delay_ms(50);
            continue;
        }

        // Cập nhật trạng thái hệ thống
        if (last_system_state != system_state) {
            LCD_SendCommand(0x01);
            LCD_SendCommand(0x80);
            LCD_SendString("PPM:----");
            LCD_SendCommand(0xC0);
            LCD_SendString("SYS:ON Alert:-");
            last_system_state = system_state;
            last_alert_state = 255;
            last_ppm = 0xFFFFFFFF;
        }

        // Đọc và tính toán PPM
        uint16_t adc_val = ADC_Read();
        float ppm = MQ2_GetPPM(adc_val);
        uint32_t ppm_int = (uint32_t)ppm;

        // Gửi PPM qua UART2
        snprintf(uart_buf, sizeof(uart_buf), "%.1f\n", ppm);
        UART2_Transmit(uart_buf);

        // Xác định trạng thái cảnh báo
        if (ppm > 800) {
            alert_state = 3; // Nguy hiểm
        } else if (ppm > 500) {
            alert_state = 2; // Cao
        } else if (ppm > 300) {
            alert_state = 1; // Thấp
        } else {
            alert_state = 0; // Bình thường
        }

        // Cập nhật PPM trên LCD
        if (ppm_int != last_ppm) {
            LCD_SendCommand(0x80 + 4);
            snprintf(lcd_buf, sizeof(lcd_buf), "%4lu", ppm_int);
            LCD_SendString(lcd_buf);
            last_ppm = ppm_int;
        }

        // Cập nhật Alert trên LCD
        if (last_alert_state != alert_state) {
            LCD_SendCommand(0xC0 + 13);
            snprintf(lcd_buf, sizeof(lcd_buf), "%d", alert_state);
            LCD_SendString(lcd_buf);
            last_alert_state = alert_state;
        }

        // Điều khiển LED, Relay, Buzzer
        switch (alert_state) {
            case 0: // Bình thường
                LED_Set(0, 0, 1, 0);
                GPIOA->ODR &= ~(1 << RELAY_PIN);
                Buzzer_Set(0);
                TIM2->CR1 &= ~TIM_CR1_CEN;
                break;
            case 1: // Thấp
                LED_Set(0, 1, 0, 0);
                GPIOA->ODR &= ~(1 << RELAY_PIN);
                Buzzer_Set(0);
                TIM2->CR1 &= ~TIM_CR1_CEN;
                break;
            case 2: // Cao
                TIM2->CR1 |= TIM_CR1_CEN;
                LED_Set(tim2_blink_state, 0, 0, 0);
                GPIOA->ODR |= (1 << RELAY_PIN);
                Buzzer_Set(1);
                break;
            case 3: // Nguy hiểm
                TIM2->CR1 |= TIM_CR1_CEN;
                LED_Set(tim2_blink_state, 0, 0, 0);
                GPIOA->ODR |= (1 << RELAY_PIN);
                Buzzer_Set(1);
                break;
        }

        delay_ms(50);
    }
}
