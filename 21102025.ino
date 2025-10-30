// Định nghĩa các chân cho 3 động cơ và nam châm
#define dirPin1 24
#define stepPin1 22
#define homePin1 34

#define dirPin2 26
#define stepPin2 28
#define homePin2 36

#define dirPin3 30
#define stepPin3 32
#define homePin3 38

#define namcham 40 

String inputString = "";
bool stringComplete = false;

float stepAngle = 0.1125; // Độ mỗi bước (1/16 microstep cho động cơ 1.8 độ)
long currentAngle[4] = {0, 0, 0, 0}; // Vị trí góc hiện tại (sử dụng index 1, 2, 3)
long targetAngle[4] = {0, 0, 0, 0};  // Vị trí góc mục tiêu

void setup() {
  Serial.begin(9600);

  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(homePin1, INPUT); // Sử dụng điện trở kéo lên nội bộ cho ổn định

  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(homePin2, INPUT);

  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(homePin3, INPUT);

  pinMode(namcham, OUTPUT);
  digitalWrite(namcham, LOW); // Tắt nam châm khi khởi động

  // Đặt hướng mặc định (CCW) để homing
  digitalWrite(dirPin1, HIGH);
  digitalWrite(dirPin2, HIGH);
  digitalWrite(dirPin3, HIGH);

  // ======== Trình tự homing đồng bộ ========
  Serial.println("Bat dau homing dong bo...");
  bool motor1_homed = false;
  bool motor2_homed = false;
  bool motor3_homed = false;

  while (!motor1_homed || !motor2_homed || !motor3_homed) {
    // Khi công tắc được nhấn, chân sẽ được kéo xuống LOW
    if (digitalRead(homePin1) == HIGH) motor1_homed = true;
    if (digitalRead(homePin2) == HIGH) motor2_homed = true;
    if (digitalRead(homePin3) == HIGH) motor3_homed = true;

    // Phát xung cho các motor chưa về home
    if (!motor1_homed) digitalWrite(stepPin1, HIGH);
    if (!motor2_homed) digitalWrite(stepPin2, HIGH);
    if (!motor3_homed) digitalWrite(stepPin3, HIGH);
    delayMicroseconds(1200);

    if (!motor1_homed) digitalWrite(stepPin1, LOW);
    if (!motor2_homed) digitalWrite(stepPin2, LOW);
    if (!motor3_homed) digitalWrite(stepPin3, LOW);
    delayMicroseconds(1200);
  }

  // Sau khi homing, tất cả các góc đều là 0
  currentAngle[1] = 0;
  currentAngle[2] = 0;
  currentAngle[3] = 0;

  Serial.println("Homing hoan tat. Arduino san sang.");
}

// Di chuyển cả 3 motor gần như đồng thời (dùng cho lệnh SET và HOME)
void moveAllMotors() {
  long stepsRaw[4] = {0, 0, 0, 0};
  
  for (int m = 1; m <= 3; m++) {
    long delta = targetAngle[m] - currentAngle[m];
    long steps = abs((long)(delta / stepAngle));
    stepsRaw[m] = (long)(steps * 3.75);
    
    int dir = (delta >= 0) ? LOW : HIGH; // LOW = CW, HIGH = CCW
    if (m == 1) digitalWrite(dirPin1, dir);
    if (m == 2) digitalWrite(dirPin2, dir);
    if (m == 3) digitalWrite(dirPin3, dir);
  }

  long maxSteps = max(stepsRaw[1], max(stepsRaw[2], stepsRaw[3]));
  if (maxSteps <= 0) {
    Serial.println("Khong can di chuyen");
    return;
  }

  for (long i = 0; i < maxSteps; i++) {
    if (i < stepsRaw[1]) digitalWrite(stepPin1, HIGH);
    if (i < stepsRaw[2]) digitalWrite(stepPin2, HIGH);
    if (i < stepsRaw[3]) digitalWrite(stepPin3, HIGH);
    delayMicroseconds(1000);

    if (i < stepsRaw[1]) digitalWrite(stepPin1, LOW);
    if (i < stepsRaw[2]) digitalWrite(stepPin2, LOW);
    if (i < stepsRaw[3]) digitalWrite(stepPin3, LOW);
    delayMicroseconds(1000);
  }

  for (int m = 1; m <= 3; m++) currentAngle[m] = targetAngle[m];

  Serial.print("Da di chuyen dong thoi xong -> ");
  Serial.print("t1=");
  Serial.print(currentAngle[1]);
  Serial.print(" t2=");
  Serial.print(currentAngle[2]);
  Serial.print(" t3=");
  Serial.println(currentAngle[3]);
}

// ======= MAIN LOOP =======
void loop() {
  if (stringComplete) {
    inputString.trim();

    if (inputString.length() > 0) {
      
      // --- LENH SET ---
      if (inputString.startsWith("SET")) {
        inputString.replace("SET ", "");
        char buffer[50];
        inputString.toCharArray(buffer, 50);
        char *token = strtok(buffer, " ");
        
        int count = 1;
        while(token != NULL && count <= 3) {
           float val = atof(token);
           targetAngle[count] = (int)round(val);
           targetAngle[count] = constrain(targetAngle[count], -180, 180);
           token = strtok(NULL, " ");
           count++;
        }

        if (count > 3) {
          moveAllMotors();
        } else {
          Serial.println("Lenh SET sai dinh dang hoac thieu goc");
        }
      }

      // --- LENH HOME ---
      else if (inputString.equalsIgnoreCase("HOME")) {
        Serial.println("Dang ve HOME dong bo...");
        targetAngle[1] = 0;
        targetAngle[2] = 0;
        targetAngle[3] = 0;
        moveAllMotors();
        Serial.println("Da ve HOME (0,0,0)");
      }

      // --- LENH HUT ---
      else if (inputString.equalsIgnoreCase("HUT")) {
        digitalWrite(namcham, HIGH);
        Serial.println("Nam cham: HUT (ON)");
      }

      // --- LENH THA ---
      else if (inputString.equalsIgnoreCase("THA")) {
        digitalWrite(namcham, LOW);
        Serial.println("Nam cham: THA (OFF)");
      }

      // --- LENH BOX ---
      else if (inputString.equalsIgnoreCase("BOX")) {
        Serial.println("Di chuyen toi BOX...");
        moveAllMotors();
        Serial.println("Da toi vi tri BOX");
      }

      else {
        Serial.print("Lenh khong hieu: ");
        Serial.println(inputString);
      }
    }

    inputString = "";
    stringComplete = false;
  }
}

// ======= NHẬN DỮ LIỆU SERIAL =======
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}
