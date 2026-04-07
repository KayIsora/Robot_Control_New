#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

// Khai báo SoftwareSerial kết nối với SIM800A
SoftwareSerial SIM800A(4, 3); // RX, TX
int maxRetries = 3;           // Số lần gọi lại tối đa
String phoneNumbers[] = {"+84915831727", "+84367531168"}; // Danh sách số điện thoại
int currentPhoneIndex = 0;    // Chỉ số của số điện thoại hiện tại
int callAttempts = 0;         // Số lần gọi lại cho số hiện tại

// Chân analog đọc từ cảm biến ACS712 
const int sensorPin = A7; 
const float sensitivity = 0.1;          // Độ nhạy cảm biến (100mV/A)
const int numReadings = 50;             // Số lần lấy mẫu để tính trung bình
float vOffset = 0;                      // Biến lưu giá trị offset trung bình

// Khai báo các hàm
void receiveData(int byteCount);                    // Hàm nhận dữ liệu từ I2C
void readEncoder();                                 // Hàm đọc tín hiệu từ encoder
void controlMotors(int leftSpeed, int rightSpeed);  // Hàm điều khiển động cơ
// void sendData();
// void countLeftPulses();
// void countRightPulses();  // Hàm gửi dữ liệu vận tốc lên master
// void initializeModule();
// bool sendSMS(String phoneNumber, String message);
// bool makeCall(String phoneNumber);
// bool checkModule();
// bool sendATCommand(String command, String expectedResponse, int timeout);
String readResponse(int timeout);
// Khai báo các chân điều khiển động cơ
#define IN1 A0  // Điều khiển hướng động cơ 1
#define IN2 A1
#define ENA 9   // PWM động cơ 1
#define IN3 A2  // Điều khiển hướng động cơ 2
#define IN4 A3
#define ENB 6  // PWM động cơ 2
//HC-SR04
const int trig = 13;  // chân trig của HC-SR04
const int echo = 12;  // chân echo của HC-SR04
// Khai báo các chân encoder
#define ENCODER_LEFT_A 5
#define ENCODER_LEFT_B 2
#define ENCODER_RIGHT_A 7
#define ENCODER_RIGHT_B 8

// Biến lưu trạng thái trước đó của tín hiệu
int lastLeftSignalA = HIGH;
int lastLeftSignalB = HIGH;
int lastRightSignalA = HIGH;
int lastRightSignalB = HIGH;
int Tien = 170;
bool fallDetected = false;  // 0: không té ngã, 1: té ngã

// Biến đếm xung
volatile long leftPulseCount = 0;
volatile long rightPulseCount = 0;

// Biến tốc độ động cơ nhận từ I2C
volatile int leftSpeed = 0;
volatile int rightSpeed = 0;
int distance;

// Thông số động cơ và bánh xe
#define PPR 330              // Số xung mỗi vòng quay (Pulses Per Revolution) sau hộp giảm tốc
#define WHEEL_RADIUS 0.0325  // Bán kính bánh xe (m)
#define GEAR_RATIO 30        // Tỉ số truyền hộp số
float leftSpeedCalculated = 0.0;
float rightSpeedCalculated = 0.0;
// Thời gian đo tốc độ (ms)
unsigned long lastTime = 0;
unsigned long interval = 1000;  // 1 giây

#define SLAVE_ADDRESS 0x08  // Địa chỉ I2C của Arduino Nano
#define FALL_SIGNAL 255   // Tín hiệu té ngã

void readDistance() {
  unsigned long duration;  // biến đo thời gian
                           // biến lưu khoảng cách

  /* Phát xung từ chân trig */
  digitalWrite(trig, 0);  // tắt chân trig
  delayMicroseconds(2);
  digitalWrite(trig, 1);  // phát xung từ chân trig
  delayMicroseconds(5);   // xung có độ dài 5 microSeconds
  digitalWrite(trig, 0);  // tắt chân trig


  /* Tính toán thời gian */
  // Đo độ rộng xung HIGH ở chân echo.
  duration = pulseIn(echo, HIGH);
  // Tính khoảng cách đến vật.
  distance = int(duration / 2 / 29.412);


  /* In kết quả ra Serial Monitor */
  Serial.print(distance);
  Serial.println("cm");
  delay(200);
}
void countLeftPulses() {
  leftPulseCount++;
}

void countRightPulses() {
  rightPulseCount++;
}
void setup() {
  
  // Cấu hình chân encoder
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  // Thiết lập các chân điều khiển động cơ
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Khởi tạo Serial để theo dõi trạng thái
  Serial.begin(9600);
  Serial.println("Chuong trinh bat dau...");
  
  // Khởi tạo I2C slave
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);  // Gắn hàm xử lý khi nhận dữ liệu từ master
  Wire.onRequest(requestEvent);
  //attachInterrupt(0, countLeftPulses, RISING);
  //attachInterrupt(0, countRightPulses, RISING);
  
// KHỞI ĐỘNG
  Serial.begin(9600);         // Serial debug
  // SIM800A.begin(9600);        // Serial cho SIM800A
  delay(2000);                // Chờ module khởi động
  Serial.println("BAT DAU KHOI DONG...");
 //initCurrentSensor(); // Gọi hàm khởi tạo cảm biến
//  initializeModule();
}
void loop() {
  //initCurrentSensor(); // Gọi hàm khởi tạo cảm biến
  //float current = readCurrent();
  //delay(1000);
  //Serial.print("Dòng điện (A): ");
  //Serial.println(current, 3);  // In giá trị dòng điện với 3 chữ số
  // Đọc tín hiệu từ encoder và tính toán tốc độ
  readEncoder();

  // Tính toán tốc độ thực tế mỗi giây
  if (millis() - lastTime >= interval) {
    lastTime = millis();

    // Tính tốc độ và in ra Serial Monitor
    float leftPRM = (leftPulseCount / 330) * 60;
    float rightPRM = (rightPulseCount / 330) * 60;
    leftSpeedCalculated = float(leftPulseCount / 330) * float(PI * WHEEL_RADIUS);
    rightSpeedCalculated = float(rightPulseCount / 330) * float(PI * WHEEL_RADIUS);

    // Serial.print("Left Speed (m/s): ");
    // Serial.println(leftSpeedCalculated);
    // Serial.print("Right Speed (m/s): ");
    // Serial.println(rightSpeedCalculated);

    // Reset pulse count
    leftPulseCount = 0;
    rightPulseCount = 0;

    delay(100);
  }
  // Điều khiển động cơ dựa trên tốc độ nhận từ I2C
  readDistance();
  controlMotors(leftSpeed, rightSpeed);
    // digitalWrite(IN1, LOW);
    // digitalWrite(IN2, HIGH);
    // digitalWrite(IN3, LOW);
    // digitalWrite(IN4, HIGH);
    // analogWrite(ENA, 255);
    // analogWrite(ENB, 255);
  // Xử lý tín hiệu té ngã
  // if (fallDetected) {
  //   Serial.println("TE NGA! DUNG DONG CO. BAT DAU GUI THONG BAO...");
  //   // controlMotors(0, 0);
  //   // Thực hiện cuộc gọi
  //   if (makeCall(phoneNumbers[currentPhoneIndex])) {
  //   //Serial.println("NGUOI NHAN DA NHAN CUOC GOI.");
  //   while (true) {} // Dừng chương trình khi có người nhận
  // } else {
  //   callAttempts++;
  //   if (callAttempts >= maxRetries) {
  //     //Serial.println("KHONG CO PHAN HOI SAU " + String(maxRetries) + " LAN. CHUYEN SANG SO TIEP THEO...");
  //     currentPhoneIndex++;
  //     callAttempts = 0; // Reset số lần gọi lại khi chuyển số

  //     if (currentPhoneIndex >= sizeof(phoneNumbers) / sizeof(phoneNumbers[0])) {
  //       //Serial.println("HOAN THANH MOT VONG. THUC HIEN GOI LAI DANH SACH...");
  //       currentPhoneIndex = 0; // Quay lại số đầu tiên
  //     }
  //   } else {
  //     //Serial.println("THU GOI LAI LAN " + String(callAttempts) + "...");
  //   }
  // }
  // }
}

// Hàm điều khiển động cơ dựa trên tốc độ nhận được từ I2C
void controlMotors(int leftSpeed, int rightSpeed) {
  Serial.println(leftSpeed);
  Serial.println(rightSpeed);

  if (leftSpeed == 1 && rightSpeed == 1) {
    fallDetected = false;
    Serial.println("clear");
  }
  if (fallDetected) {
    // Dừng động cơ khi té ngã
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    Serial.println("Te");
    return;
  }
  if ((leftSpeed == 175 && rightSpeed == 175)) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, Tien);
    analogWrite(ENB, Tien);
    Serial.println("Tien");
  }  
  else if(leftSpeed == 220 && rightSpeed == 0){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 220);
    analogWrite(ENB, rightSpeed);
    Serial.println("Qua phai");
  }
  else if(leftSpeed == 0 && rightSpeed == 220){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    analogWrite(ENB, 220);
    analogWrite(ENA, 0);
    Serial.println("Qua trai");
    } 
  else if (leftSpeed == 174 && rightSpeed == 174) {
    if(distance >= 50)
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENA, 170);
      analogWrite(ENB, 170);
      Serial.println("Lui");
    }
    else
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
      Serial.println("K Lui");
    }
  }
  else if ((leftSpeed == 0 && rightSpeed == 0)) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    Serial.println("Dung");
  }

}

// Hàm đọc tín hiệu từ encoder và đếm xung
void readEncoder() {
  // Đọc trạng thái tín hiệu từ encoder trái
  int currentLeftSignalA = digitalRead(ENCODER_LEFT_A);
  int currentLeftSignalB = digitalRead(ENCODER_LEFT_B);

  // Kiểm tra sự thay đổi tín hiệu từ kênh A và B của encoder trái
  if (currentLeftSignalA != lastLeftSignalA || currentLeftSignalB != lastLeftSignalB) {
    leftPulseCount++;  // Tăng đếm xung
    lastLeftSignalA = currentLeftSignalA;
    lastLeftSignalB = currentLeftSignalB;
  }

  // Đọc trạng thái tín hiệu từ encoder phải
  int currentRightSignalA = digitalRead(ENCODER_RIGHT_A);
  int currentRightSignalB = digitalRead(ENCODER_RIGHT_B);

  // Kiểm tra sự thay đổi tín hiệu từ kênh A và B của encoder phải
  if (currentRightSignalA != lastRightSignalA || currentRightSignalB != lastRightSignalB) {
    rightPulseCount++;  // Tăng đếm xung
    lastRightSignalA = currentRightSignalA;
    lastRightSignalB = currentRightSignalB;
  }
}
// Hàm gửi dữ liệu vận tốc về master qua I2C
void requestEvent() {
  byte leftSpeedBytes[4];
  byte rightSpeedBytes[4];

  // Chuyển đổi giá trị float sang mảng byte
  memcpy(leftSpeedBytes, &leftSpeedCalculated, sizeof(leftSpeedCalculated));
  memcpy(rightSpeedBytes, &rightSpeedCalculated, sizeof(rightSpeedCalculated));

  // Gửi 4 byte dữ liệu qua I2C
  Wire.write(leftSpeedBytes, 4);
  Wire.write(rightSpeedBytes, 4);
}
// Hàm xử lý khi nhận tín hiệu qua I2C
void receiveData(int byteCount) {
  if (byteCount < 3) return; // Đảm bảo nhận đủ 3 byte

  uint8_t packetType = Wire.read(); // Đọc byte đầu tiên để xác định loại gói tin

  if (packetType == 0x01) {
    // Serial.println(Wire.read());
    leftSpeed = Wire.read();  // Đọc tốc độ động cơ trái
    rightSpeed = Wire.read(); // Đọc tốc độ động cơ phải
    // Serial.println(leftSpeed);
    packetType = 0;
  // Kiểm tra tín hiệu té ngã
  } else if (packetType == 0x02) { // Gói tin báo té ngã
    fallDetected = true; // Đặt trạng thái tín hiệu té ngã
    packetType = 0;
    Serial.println(">>> NHAN DUOC TIN HIEU TE NGA TU JETSON!");
  }
}
// Hàm khởi động module
// void initializeModule() {
//   if (!checkModule()) {    // Lặp đến khi module khởi động thành công
//     Serial.println("KHOI DONG THAT BAI. KHOI DONG LAI...");
//     delay(3000);
//   }
//   if (!checkSIM()) {    // Lặp đến khi module khởi động thành công
//     Serial.println("KHOI DONG THAT BAI. KHOI DONG LAI...");
//     delay(3000);
//   }
//   Serial.println("KHOI DONG THANH CONG.");
// }

// Hàm gửi tin nhắn
// bool sendSMS(String phoneNumber, String message) {
//   SIM800A.println("AT+CMGF=1"); // Chuyển sang chế độ text mode
//   delay(1000);
//   SIM800A.println("AT+CMGS=\"" + phoneNumber + "\""); // Số điện thoại nhận
//   delay(1000);
//   SIM800A.println(message); // Nội dung tin nhắn
//   delay(100);
//   SIM800A.write(26); // Gửi ký tự CTRL+Z
//   delay(3000);

//   String response = readResponse(5000);
//   if (response.indexOf("OK") != -1) {
//     //Serial.println("TIN NHAN GUI THANH CONG DEN SO " + phoneNumber);
//     return true;
//   } else {
//     //Serial.println("THAT BAI KHI GUI TIN NHAN DEN SO " + phoneNumber);
//     return false;
//   }
// }

// Hàm thực hiện cuộc gọi
// bool makeCall(String phoneNumber) {
//   if (!sendSMS(phoneNumber, "PHAT HIEN TE NGA")) {
//     //Serial.println("KHONG GUI DUOC TIN NHAN. HUY CUOC GOI.");
//     return false;
//   } else {
//     //Serial.println("TIN NHAN GUI THANH CONG DEN SO " + phoneNumber);
// }

//   SIM800A.println("ATD" + phoneNumber + ";"); // Bắt đầu cuộc gọi
//   delay(2000);
//   //Serial.println("DANG THUC HIEN CUOC GOI DEN " + phoneNumber);

//   for (int i = 0; i < 15; i++) { // Kiểm tra trạng thái trong 10 giây
//     SIM800A.println("AT+CLCC");
//     delay(1000);

//     String response = readResponse(2000);
//     // Serial.println("TRANG THAI CUOC GOI: " + response);

//     if (response.indexOf("+CLCC: 1,0,0,0,0") >= 0) { 
//       //Serial.println("CUOC GOI DANG KET NOI.");
//       delay(5000); // Giữ cuộc gọi trong 5 giây
//       SIM800A.println("ATH"); // Ngắt cuộc gọi
//       delay(1000);
//       return true;
//     } else if (response.indexOf("+CLCC: 1,0,2,0,0") >= 0) { 
//       Serial.println("DANG THUC HIEN CUOC GOI.");
//     } else if (response.indexOf("+CLCC: 1,0,3,0,0") >= 0) { 
//       Serial.println("DANG DO CHUONG.");
//     } else if (response.indexOf("BUSY") >= 0) { 
//       Serial.println("NGUOI NHAN TU CHOI CUOC GOI HOAC MAY BAN.");
//       break; // Thoát khỏi vòng lặp khi nhận "BUSY"
//     } else if (response.indexOf("NO CARRIER") >= 0) { 
//       //Serial.println("CUOC GOI BI NGAT.");
//       break;
//     }
// }
//   SIM800A.println("ATH"); // Ngắt cuộc gọi nếu không thành công
//   delay(2000);
//   return false;
// }

// Hàm kiểm tra module SIM800A
// bool checkModule1() {
//   if (!sendATCommand("AT", "OK", 5000)) {
//     return false;
//   }
//   Serial.println("MODULE SAN SANG.");
// //Kiểm tra trạng thái SIM
//   if (!sendATCommand("AT+CPIN?", "READY", 2000)) {
//     Serial.println("SIM KHONG SAN SANG.");
//     return false;
//   }
//   Serial.println("SIM SAN SANG.");
//   return true;
// }
// bool checkModule() {
//   Serial.println("Dang kiem tra ket noi voi SIM800A...");

//   // Bước 1: Kiểm tra kết nối cơ bản
//   if (!sendATCommand("AT", "OK", 5000)) {
//     Serial.println("LOI: KHONG NHAN DUOC PHAN HOI 'AT'.");
//     return false;
//   }
//   Serial.println("MODULE SAN SANG.");
// }
// bool checkSIM(){
//   // Bước 2: Kiểm tra trạng thái SIM
//   if (!sendATCommand("AT+CPIN?", "READY", 2000)) {
//     Serial.println("LOI: SIM KHONG SAN SANG HOAC CO MA PIN.");
//     // Nếu SIM yêu cầu mã PIN, cố gắng vô hiệu hóa yêu cầu mã PIN
//     Serial.println("Dang vo hieu hoa ma PIN...");
//     if (!sendATCommand("AT+CLCK=\"SC\",0", "OK", 5000)) {  // Vô hiệu hóa mã PIN
//       Serial.println("Khong the vo hieu hoa ma PIN.");
      
//       // Nếu không thể vô hiệu hóa mã PIN, thử gửi mã PIN
//       Serial.println("Dang gui ma PIN...");
//       if (!sendATCommand("AT+CPIN=\"1234\"", "OK", 5000)) {  // Thay "1234" bằng mã PIN của bạn
//         Serial.println("Khong the mo khoa SIM.");
//         return false;  // Nếu không thể mở khóa SIM, trả về false
//       }
//     }
//   }
//   if (!sendATCommand("AT+CSQ", "OK", 2000) ){
// //String response = readResponse(2000);  // Đọc phản hồi từ SIM800A
//   //Serial.println(response);
//     Serial.println("LOI: SIM khong song.");
//     return false;
//   }
//   Serial.println("SIM SAN SANG.");
// }

// Hàm gửi lệnh AT và kiểm tra phản hồi
// bool sendATCommand(String command, String expectedResponse, int timeout) {
//   SIM800A.println(command);
//   String response = readResponse(timeout);
//   return response.indexOf(expectedResponse) != -1;
// }

// Hàm đọc phản hồi từ module SIM800A
String readResponse(int timeout) {
  String response = "";
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    while (SIM800A.available()) {
      response += (char)SIM800A.read();
    }
  }
  if (response == "") {
    Serial.println("KHONG NHAN DUOC PHAN HOI TU SIM800A.");
  }
  return response;
}

void initCurrentSensor() {
  float totalVoltage = 0;
  for (int i = 0; i < numReadings; i++) {
    int analogValue = analogRead(sensorPin);           // Đọc giá trị từ cảm biến
    float voltage = analogValue * (5.0 / 1023.0);      // Chuyển đổi giá trị sang điện áp
    totalVoltage += voltage;                           // Cộng dồn để tính trung bình
    delay(250);                                         // Chờ giữa các lần đọc
  }
  vOffset = totalVoltage / numReadings;                // Tính giá trị offset trung bình
  //Serial.print("Giá trị offset (V): ");
  //Serial.println(vOffset, 3);
}

// Hàm đọc giá trị dòng điện
float readCurrent() {
  int analogValue = analogRead(sensorPin);             // Đọc giá trị từ cảm biến
  float voltage = analogValue * (5.0 / 1023.0);        // Chuyển đổi sang điện áp
  float current = (voltage - vOffset) / sensitivity;   // Tính dòng điện
  return abs(current);                                 // Trả về giá trị dòng điện tuyệt đối
}