#define BLYNK_TEMPLATE_ID "TMPL6GKYgpKOq"
#define BLYNK_TEMPLATE_NAME "cảm biến rò rỉ khí gas"
#define BLYNK_AUTH_TOKEN "SMPsiZ4DgQGrZ1YKi-QBEiqg0Rffhzod"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// UART2 pins
#define RXD2 16  // ESP32 RX từ STM32 TX
#define TXD2 17  // Không dùng, nhưng vẫn cần khai báo

// UART2
HardwareSerial mySerial(2); // UART2

// WiFi credentials
char ssid[] = "VIETTEL_2.4";
char pass[] = "12345679";

// Biến nhận dữ liệu
float co2 = 0.0;
String inputBuffer = "";

BlynkTimer timer;

// Gửi dữ liệu lên Blynk
void sendToBlynk() {
  Blynk.virtualWrite(V4, co2); // Gửi CO2 (PPM) đến V4

  // Kiểm tra ngưỡng CO2 và gửi email cảnh báo
  if (co2 > 800.0) {
    Blynk.logEvent("co2_warning", "Nồng độ CO2 vượt quá 800 ppm: " + String(co2) + " ppm");
  }
}

void setup() {
  Serial.begin(115200); // Serial monitor
  mySerial.begin(9600, SERIAL_8N1, RXD2, TXD2); // UART2 RX từ STM32

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  timer.setInterval(1000L, sendToBlynk); // Gửi mỗi 1s

  Serial.println("ESP32 đã sẵn sàng. Đợi dữ liệu từ STM32...");
}

void loop() {
  Blynk.run();
  timer.run();

  while (mySerial.available()) {
    char c = mySerial.read();
    if (c == '\n') {
      // Đọc giá trị float từ chuỗi, ví dụ: "400.0\n"
      co2 = inputBuffer.toFloat();

      Serial.print("Đã nhận: ");
      Serial.print(inputBuffer);
      Serial.print("CO2: ");
      Serial.println(co2);

      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}