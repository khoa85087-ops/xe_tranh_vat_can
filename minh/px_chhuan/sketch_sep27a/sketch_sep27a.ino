// ===== Cấu hình tần số xung (ĐẶT Ở ĐẦU CODE) =====
const float PULSE_FREQUENCY_HZ = 1000.0;   // Tần số xung mong muốn (Hz)

// Dùng micro giây (µs) để hỗ trợ tần số cao
const unsigned long pulseIntervalUs = (unsigned long)(1000000.0 / (2.0 * PULSE_FREQUENCY_HZ));

// ===== Cấu hình chân =====
const int pulsePin     = 4;   // chân PHÁT xung
const int lowPin       = 3;   // chân LÀM MASS (giữ mức LOW cố định)
const int pulseReadPin = 2;   // chân NHẬN xung (hỗ trợ ngắt ngoài INT0)

unsigned long lastPulseTime = 0;   // tính theo micros()
bool pulseState = false;
unsigned long sentPulseCount = 0;  // số xung đã PHÁT ra (đếm liên tục, không dừng)

volatile unsigned long pulseCount = 0;   // số xung ĐỌC được (đếm liên tục, không dừng)
volatile bool counting = false;

String inputBuffer = "";
unsigned long lastShown = 0;   // để in Serial mỗi khi pulseCount thay đổi

// ===== ISR: đếm cạnh lên trên pulseReadPin =====
void onPulseRising() {
  if (counting) pulseCount++;
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "st") {
    noInterrupts();
    counting = true;
    interrupts();
    Serial.println(">> Bat dau dem xung (lien tuc).");
  }
  else if (cmd == "stop") {
    counting = false;
    Serial.println(">> Da dung dem xung.");
  }
  else if (cmd == "ret") {
    noInterrupts();
    pulseCount = 0;
    interrupts();
    sentPulseCount = 0;
    lastShown = 0;
    Serial.println(">> Da reset bo dem ve 0.");
  }
  else if (cmd.length() > 0) {
    Serial.print(">> Lenh khong hop le: ");
    Serial.println(cmd);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(pulsePin, OUTPUT);
  digitalWrite(pulsePin, LOW);

  pinMode(lowPin, OUTPUT);
  digitalWrite(lowPin, LOW);   // giữ mass cố định

  pinMode(pulseReadPin, INPUT);   // đổi INPUT_PULLUP nếu tín hiệu ngoài không có pull-up
  attachInterrupt(digitalPinToInterrupt(pulseReadPin), onPulseRising, RISING);

  Serial.println("San sang phat va doc xung LIEN TUC...");
  Serial.print("Tan so phat: ");
  Serial.print(PULSE_FREQUENCY_HZ);
  Serial.println(" Hz");
  Serial.println("Lenh: st = bat dau dem | stop = dung dem | ret = reset bo dem");
}

void loop() {
  unsigned long now = micros();

  // ---- Đọc lệnh từ Serial ----
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        handleCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }

  // ---- Phát xung LIÊN TỤC, không có điểm dừng ----
  if (now - lastPulseTime >= pulseIntervalUs) {
    lastPulseTime = now;
    pulseState = !pulseState;
    digitalWrite(pulsePin, pulseState);
    if (pulseState == HIGH) sentPulseCount++;
  }

  // ---- In số đếm mỗi khi thay đổi (đọc biến volatile an toàn) ----
  noInterrupts();
  unsigned long safeCount = pulseCount;
  interrupts();

  if (safeCount != lastShown) {
    lastShown = safeCount;
    Serial.print("Da dem xung: ");
    Serial.print(safeCount);
    Serial.print(" | Da phat: ");
    Serial.println(sentPulseCount);
  }
}