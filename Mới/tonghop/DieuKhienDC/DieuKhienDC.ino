const int IN1 = 4;
const int IN2 = 5;
const int encoderA = 2;  // Pha A - có ngắt
const int encoderB = 8;  // Pha B - đọc trong ISR

volatile long encoderCount = 0;
int targetPulses = 0;

float pulsesPerRevolution = 1848.0; // 11 xung * tỉ số truyền 18
float pulsesPerDegree = pulsesPerRevolution / 360.0;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(encoderA), encoderISR, RISING);
  
  Serial.begin(9600);
  Serial.println("Nhap goc muon quay (do):");
}

void loop() {
  if (Serial.available() > 0) {
    float deg = Serial.parseFloat();
    targetPulses = abs(deg * pulsesPerDegree);
    encoderCount = 0;
    
    Serial.print("Quay ");
    Serial.print(deg);
    Serial.println(" do...");

    if (deg > 0) {
      quayThuan();
    } else {
      quayNguoc();
    }

    // Chờ đạt số xung mục tiêu
    while (abs(encoderCount) < targetPulses);

    // Dừng motor
    dungMotor();

    Serial.println("Hoan thanh!");
    Serial.println("Nhap goc moi:");
  }
}

// --- Ngắt đếm xung encoder ---
void encoderISR() {
  int b = digitalRead(encoderB); // đọc pha B để xác định chiều
  if (b == HIGH)
    encoderCount++;
  else
    encoderCount--;
}

// --- Điều khiển motor ---
void quayThuan() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void quayNguoc() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void dungMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}
