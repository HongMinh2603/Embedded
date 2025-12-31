const int flexPin = 35;
const int filterSize = 5; // Kích thước bộ lọc (số lẻ)

int readings[filterSize];
int sorted[filterSize];

void setup() {
  Serial.begin(115200);
  
  // Khởi tạo mảng
  for (int i = 0; i < filterSize; i++) {
    readings[i] = analogRead(flexPin);
  }
}

void loop() {
  // Dịch các giá trị cũ
  for (int i = filterSize - 1; i > 0; i--) {
    readings[i] = readings[i - 1];
  }
  
  // Đọc giá trị mới
  readings[0] = analogRead(flexPin);
  
  // Sao chép mảng để sắp xếp
  for (int i = 0; i < filterSize; i++) {
    sorted[i] = readings[i];
  }
  
  // Sắp xếp nổi bọt
  for (int i = 0; i < filterSize - 1; i++) {
    for (int j = i + 1; j < filterSize; j++) {
      if (sorted[i] > sorted[j]) {
        int temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
    }
  }
  
  // Lấy giá trị trung vị
  int median = sorted[filterSize / 2];
  
  Serial.println(median);
  
  delay(50);
}