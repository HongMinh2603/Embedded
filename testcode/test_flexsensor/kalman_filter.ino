const int flexPin = 36;

float kalmanFilter(float measurement) {
  static float estimate = 0; // Ước lượng hiện tại
  static float estimateError = 1; // Sai số ước lượng
  static float processNoise = 0.01; // Nhiễu quá trình
  static float measurementNoise = 10; // Nhiễu đo lường
  static float kalmanGain;
  
  // Dự đoán
  estimate = estimate;
  estimateError = estimateError + processNoise;
  
  // Cập nhật
  kalmanGain = estimateError / (estimateError + measurementNoise);
  estimate = estimate + kalmanGain * (measurement - estimate);
  estimateError = (1 - kalmanGain) * estimateError;
  
  return estimate;
}

void setup() {
  Serial.begin(115200);
}

void loop() {
  int raw = analogRead(flexPin);
  float filtered = kalmanFilter(raw);
  
  Serial.print(raw);
  Serial.print(",");
  Serial.println(filtered);
  
  delay(50);
}