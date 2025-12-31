const int flexPin = 36;
const int numReadings = 10; // Số mẫu để lấy trung bình

int readings[numReadings];  // Mảng lưu các giá trị đọc
int readIndex = 0;          // Chỉ số hiện tại
int total = 0;              // Tổng các giá trị
int average = 0;            // Giá trị trung bình

void setup() {
  Serial.begin(115200);
  
  // Khởi tạo tất cả giá trị về 0
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }
}

void loop() {
  // Loại bỏ giá trị cũ khỏi tổng
  total = total - readings[readIndex];
  
  // Đọc giá trị mới
  readings[readIndex] = analogRead(flexPin);
  
  // Thêm giá trị mới vào tổng
  total = total + readings[readIndex];
  
  // Chuyển đến vị trí tiếp theo
  readIndex = (readIndex + 1) % numReadings;
  
  // Tính giá trị trung bình
  average = total / numReadings;
  
  Serial.println(average);
  
  delay(50);
}