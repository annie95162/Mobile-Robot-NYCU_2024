// 定義腳位
int enA = 6;  // 馬達 A 速度控制
int in1 = 10; // 馬達 A 方向控制
int in2 = 11; // 馬達 A 方向控制

int enB = 5;  // 馬達 B 速度控制
int in3 = 8;  // 馬達 B 方向控制
int in4 = 9;  // 馬達 B 方向控制

void setup() {
  // 設置所有腳位為輸出模式
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // 設置馬達 A 的初始狀態
  analogWrite(enA, 255);  // 設置馬達 A 的速度（0~255，255 為最大速度）
  digitalWrite(in1, HIGH); // 設置馬達 A 正轉
  digitalWrite(in2, LOW);  // 設置馬達 A 正轉
  
  // 設置馬達 B 的初始狀態
  analogWrite(enB, 255);  // 設置馬達 B 的速度
  digitalWrite(in3, HIGH); // 設置馬達 B 正轉
  digitalWrite(in4, LOW);  // 設置馬達 B 正轉
}

void loop() {
  // 在這裡你可以更改馬達的速度或方向來測試
  // 可以用 analogWrite 改變速度，例如:
  // analogWrite(enA, 150); // 速度降低
  // digitalWrite(in1, LOW);  // 改變方向
  // digitalWrite(in2, HIGH); // 改變方向
}

