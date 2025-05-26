#include <WiFi.h>

// 핀 정의
const int IN3 = 22;  // 포크 상승 방향 제어
const int IN4 = 23;  // 포크 하강 방향 제어

// WiFi 설정
const char* WIFI_SSID = "pinky_e92b";
const char* WIFI_PASS = "pinkyros2";

// TCP 서버 포트
const uint16_t ESP_PORT = 8888;
WiFiServer server(ESP_PORT);

// 상태 정의
enum ForkState {
  BOTTOM = 1,           // 최하단
  PALLET_FLOOR,
  PALLET_FLOOR_PLUS,
  RACK1,
  RACK1_PLUS,
  RACK2,
  RACK2_PLUS
};

// 각 상태까지 **올라가는** 데 걸리는 시간(ms)
const int upTimesMs[] = {
  0, 0, 800, 1800, 1500, 2500, 9600, 10600
};

// 각 상태에서 **바닥까지 내리는** 데 걸리는 시간(ms)
const int downTimesMs[] = {
  0, 0, 800, 1800, 1600, 2600, 9700, 10700
};

ForkState currentState = BOTTOM;

void setup() {
  Serial.begin(115200);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
    if (millis() - start > 20000) {
      start = millis();
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
      returnToBottom();
      client.println("✅ FORCE_DOWN COMPLETE");
    }
    else if (cmd == "UP" && dur > 0) {
      moveUp(dur);
      client.println("✅ UP COMPLETE");
      client.println("📦 CURRENT_STATE=" + String(currentState));
    }
    else if (cmd == "DOWN" && dur > 0) {
      moveDown(dur);
      client.println("✅ DOWN COMPLETE");
      client.println("📦 CURRENT_STATE=" + String(currentState));
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
      currentState = BOTTOM;
      client.println("⚙️ STATE RESET TO BOTTOM");
    }
    else if (cmd == "GET_STATE") {
      client.println(String(currentState));
    }
    else {
      client.println("❓ UNKNOWN CMD: " + cmd);
    }
  }
  client.stop();
}

// ms만큼 올리고, 딱 맞는 상태가 있으면 currentState 갱신
void moveUp(int ms) {
  digitalWrite(IN4, LOW);
  digitalWrite(IN3, HIGH);
  delay(ms);
  digitalWrite(IN3, LOW);

  for (int i = 1; i <= 7; i++) {
    if (ms == upTimesMs[i]) {
      currentState = static_cast<ForkState>(i);
      break;
    }
  }
  Serial.printf("📈 UP %dms → New state: %d\n", ms, currentState);
}

// DOWN은 상태를 유지한 채 동작만 수행
void moveDown(int ms) {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(ms);
  digitalWrite(IN4, LOW);

  Serial.printf("📉 DOWN %dms executed. State remains as %d\n", ms, currentState);
}

void stopMotor() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// 현재 위치에서 바닥까지 내리기 → 상태를 BOTTOM으로 리셋
void returnToBottom() {
  int t = downTimesMs[currentState];
  if (t <= 0) return;
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(t + 50);
  digitalWrite(IN4, LOW);
  currentState = BOTTOM;
  Serial.println("🏠 Returned to BOTTOM");
}
