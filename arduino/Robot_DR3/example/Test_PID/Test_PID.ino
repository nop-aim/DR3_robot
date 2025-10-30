#include <PID.h>

// ----- CÁC BIẾN TOÀN CỤC -----
PID pid1(2.0, 0.5, 0.1);
PID pid2(2.0, 0.5, 0.1);
PID pid3(2.0, 0.5, 0.1);

float setpoint[3] = {0, 0, 0};
float measured[3] = {0, 0, 0};
int pwmPin[3] = {3, 5, 6};
unsigned long lastTime = 0;

// ----- HÀM SETUP -----
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 3; i++) pinMode(pwmPin[i], OUTPUT);
  // Cài đặt encoder/motor driver ở đây...
}

// ----- HÀM LOOP -----
void loop() {
  // 1. Nhận lệnh mới từ Python
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    int idx1 = data.indexOf(',');
    int idx2 = data.lastIndexOf(',');
    if (idx1 > 0 && idx2 > idx1) {
      setpoint[0] = data.substring(0, idx1).toFloat();
      setpoint[1] = data.substring(idx1 + 1, idx2).toFloat();
      setpoint[2] = data.substring(idx2 + 1).toFloat();
    }
  }

  // 2. Đọc encoder hoặc cảm biến góc thực tế
  measured[0] = readEncoder1();
  measured[1] = readEncoder2();
  measured[2] = readEncoder3();

  // 3. Tính PID
  float dt = (millis() - lastTime) / 1000.0;
  lastTime = millis();

  float output1 = pid1.compute(setpoint[0], measured[0], dt);
  float output2 = pid2.compute(setpoint[1], measured[1], dt);
  float output3 = pid3.compute(setpoint[2], measured[2], dt);

  // 4. Xuất PWM
  analogWrite(pwmPin[0], constrain(output1, 0, 255));
  analogWrite(pwmPin[1], constrain(output2, 0, 255));
  analogWrite(pwmPin[2], constrain(output3, 0, 255));

  // 5. Gửi phản hồi về Python
  Serial.print(measured[0]);
  Serial.print(",");
  Serial.print(measured[1]);
  Serial.print(",");
  Serial.println(measured[2]);
}