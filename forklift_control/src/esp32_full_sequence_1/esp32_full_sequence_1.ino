#include <WiFi.h>

// 핀 정의
const int IN1 = 18;  // 포크 상승 방향 제어
const int IN2 = 19;  // 포크 하강 방향 제어

// WiFi 설정
const char* WIFI_SSID = "pinky_d8f7";
const char* WIFI_PASS = "pinkyros2";

// TCP 서버 포트
const uint16_t ESP_PORT = 8888;
WiFiServer server(ESP_PORT);

// 상태 정의
enum ForkState {
  BOTTOM = 1,           // 최하단
  PALLET_FLOOR,         // 바닥 팔레트
  PALLET_FLOOR_PLUS,    // 바닥 팔레트 +1000ms
  RACK1,                // 랙 1층
  RACK1_PLUS,           // 랙 1층 +1000ms
  RACK2,                // 랙 2층
  RACK2_PLUS            // 랙 2층 +1000ms
};

// 각 상태까지 **올라가는** 데 걸리는 시간(ms)
const int upTimesMs[] = {
  0,      // 인덱스 맞추기용
  0,      // BOTTOM
  800,    // PALLET_FLOOR
  1800,   // PALLET_FLOOR_PLUS
  2200,   // RACK1
  3200,   // RACK1_PLUS
  16900,  // RACK2
  17900   // RACK2_PLUS
};

// 각 상태에서 **바닥까지 내리는** 데 걸리는 시간(ms)
const int downTimesMs[] = {
  0,      // 인덱스 맞추기용
  0,      // BOTTOM
  800,    // PALLET_FLOOR
  1800,   // PALLET_FLOOR_PLUS
  2200,   // RACK1
  3200,   // RACK1_PLUS
  16900,  // RACK2
  17900   // RACK2_PLUS
};

ForkState currentState = BOTTOM;

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
    if (millis() - start > 20000) {
      start = millis();
      // WiFi.begin(WIFI_SSID, WIFI_PASS);
      // break;
    }
  }

  server.begin();
  Serial.printf("   • IP  : %s\n\n", WiFi.localIP().toString().c_str());
  Serial.println("✅ WiFi connected, TCP server started");
}

void loop() {
  WiFiClient client = server.available();
  if (!client) return;

  while (client.connected()) {
    if (!client.available()) continue;
    String line = client.readStringUntil('\n');
    line.trim();

    int sp = line.indexOf(' ');
    String cmd = sp > 0 ? line.substring(0, sp) : line;
    int dur = sp > 0 ? line.substring(sp + 1).toInt() : 0;

    if (cmd == "FORCE_DOWN") {
      // 바닥으로 강제 하강
      returnToBottom();
      client.println("✅ FORCE_DOWN COMPLETE");
    }
    else if (cmd == "UP" && dur > 0) {
      moveUp(dur);
      client.println("✅ UP COMPLETE");
    }
    else if (cmd == "DOWN" && dur > 0) {
      moveDown(dur);
      client.println("✅ DOWN COMPLETE");
    }
    else if (cmd == "STOP") {
      stopMotor();
      client.println("🛑 STOPPED");
    }
    else if (cmd == "RETURN_HOME") {
      returnToBottom();
      client.println("🏠 RETURNED TO BOTTOM");
    }
    else if (cmd == "ZERO") {
      // 상태 초기화만 할 경우
      currentState = BOTTOM;
      client.println("⚙️ STATE RESET TO BOTTOM");
    }
    else {
      client.println("❓ UNKNOWN CMD: " + cmd);
    }
  }

  client.stop();
}

// ms만큼 올리고, 딱 맞는 상태가 있으면 currentState 갱신
void moveUp(int ms) {
  digitalWrite(IN2, LOW);
  digitalWrite(IN1, HIGH);
  delay(ms);
  digitalWrite(IN1, LOW);

  // 상태 매핑
  for (int i = 1; i <= 7; i++) {
    if (ms == upTimesMs[i]) {
      currentState = static_cast<ForkState>(i);
      break;
    }
  }
}

// ms만큼 내리고, 바닥 도달 여부 혹은 중간 상태 갱신
void moveDown(int ms) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  delay(ms);
  digitalWrite(IN2, LOW);

  // 충분히 많이 내리면 바닥
  if (ms >= downTimesMs[currentState] + 50) {
    currentState = BOTTOM;
  } else {
    // 남은 높이 계산
    int rem = downTimesMs[currentState] - ms;
    bool found = false;
    for (int i = 1; i <= 7; i++) {
      if (downTimesMs[i] == rem) {
        currentState = static_cast<ForkState>(i);
        found = true;
        break;
      }
    }
    if (!found) {
      currentState = BOTTOM;
    }
  }
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

// 현재 위치에서 바닥까지 내리기
void returnToBottom() {
  int t = downTimesMs[currentState];
  if (t <= 0) return;  // 이미 바닥
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  delay(t + 50);       // 안전 여유분 50ms
  digitalWrite(IN2, LOW);
  currentState = BOTTOM;
}
