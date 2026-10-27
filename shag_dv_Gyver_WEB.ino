#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <Preferences.h>
#define DRIVER_STEP_TIME 1
#include <GyverStepper2.h>

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Update.h>

#define SCREEN_WIDTH 128     // ширина OLED дисплея в пикселах
#define SCREEN_HEIGHT 64     // высота OLED дисплея в пикселах
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define CONTACT_PIN_1 34  // Пин, к которому подключен контакт обратной связи 1
#define CONTACT_PIN_2 35  // Пин, к которому подключен контакт обратной связи 2


const int N = 16; //выставляется на плате драйвера (1-1-1)
const int DIR = 12;
const int STEP = 13;
const int EN = 14;
const int steps = 200; //характеристика шд, имп/об

const String FIRMWARE_VERSION = "2.0"; // версия прошивки

int targetDeg = 145;      //первоначальный поворот выключателя, градусы
int stabTimeMs = 1000;    //время стабилизации, мс
int cyclesPerHour = 600;  //циклов в час


GStepper2<STEPPER2WIRE> stepper(steps* N, STEP, DIR, EN);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define BUTTON_PIN 33 //кнопка управления
WebServer server(80);
DNSServer dnsServer;

enum State {
  WAITING_FOR_ACTION,
  MOVING,
  STABILIZING
};

State currentState = WAITING_FOR_ACTION;
unsigned long lastActionTime = 0;
Preferences preferences;
unsigned int firstPosCounter = 0;

unsigned long lastDisableTime = 0; // Время, когда stepper был отключён
bool currentDirection = false; // Текущее направление движения
bool isFirstMove = true; // Флаг первого шага после запуска

bool contact1StatePrev;
bool contact2StatePrev;
bool condition;
String strState = "Lindas";

int testMode = 0; // 0 = sw1,sw2 test, 1 = sw-proh test
int dirMode = 2; // 0 = CW, 1 = CCW, 2 = Random
bool isRunning = false;
bool isWifiEnabled = false;
uint32_t pressStart = 0;
bool wifiTriggered = false;
bool lastRawBtn = false;
bool currentBtn = false;
uint32_t debounceTimer = 0;

// Проверка изменения состояния контактов 00-10-11-01 для двухклавишного выключателя
bool checkContactChange2w(bool contact1State, bool contact2State, bool contact1StatePrev, bool contact2StatePrev) {
  if (contact1State != contact1StatePrev && contact2State == contact2StatePrev) {
    return true;
  } else if (contact1State == contact1StatePrev && contact2State != contact2StatePrev) {
    return true;
  }
  return false;
}

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="ru">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Настройки устройства</title>
  <style>
    body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; background: #121212; color: #ffffff; margin: 0; padding: 20px; display: flex; justify-content: center; }
    .card { background: #1e1e1e; padding: 25px; border-radius: 16px; box-shadow: 0 10px 30px rgba(0,0,0,0.5); max-width: 400px; width: 100%; }
    h1 { text-align: center; font-size: 24px; color: #bb86fc; margin-top: 0; }
    h2 { font-size: 16px; color: #03dac6; border-bottom: 1px solid #333; padding-bottom: 5px; margin-top: 20px; }
    .radio-group { display: flex; flex-direction: column; gap: 10px; margin-bottom: 15px; }
    .radio-label { display: flex; align-items: center; cursor: pointer; font-size: 15px; background: #2c2c2c; padding: 12px; border-radius: 8px; transition: background 0.2s; }
    .radio-label:hover { background: #3c3c3c; }
    input[type="radio"] { margin-right: 10px; accent-color: #bb86fc; transform: scale(1.2); }
    .tuning-box { border: 1px solid #333; border-radius: 12px; padding: 15px; margin: 20px 0; background: #1a1a1a; }
    .tuning-box legend { color: #03dac6; font-size: 14px; font-weight: bold; padding: 0 8px; }
    .input-row { display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; }
    .input-row:last-child { margin-bottom: 0; }
    .input-row label { font-size: 14px; color: #ccc; }
    input[type="number"] { width: 80px; padding: 8px; border-radius: 6px; border: 1px solid #444; background: #2c2c2c; color: white; text-align: right; font-size: 14px; }
    input[type="number"]:focus { outline: none; border-color: #bb86fc; }
    .btn { display: block; width: 100%; background: #bb86fc; color: #000; font-weight: bold; font-size: 16px; padding: 15px; border: none; border-radius: 8px; cursor: pointer; margin-top: 25px; transition: transform 0.1s, background 0.2s; }
    .btn:active { transform: scale(0.98); }
    .btn:hover { background: #d0a8ff; }
  </style>
</head>
<body>
  <div class="card">
    <h1>Настройки</h1>
    <form id="modeForm">
      <h2>Режим тестирования</h2>
      <div class="radio-group">
        <label class="radio-label"><input type="radio" name="mode" value="0" %MODE0%> Выключатель 1кл</label>
        <label class="radio-label"><input type="radio" name="mode" value="2" %MODE2%> Выключатель 2кл</label>
        <label class="radio-label"><input type="radio" name="mode" value="1" %MODE1%> Выключатель проходной</label>
        <label class="radio-label"><input type="radio" name="mode" value="3" %MODE3%> Без обратной связи</label>
      </div>

      <h2>Направление вращения</h2>
      <div class="radio-group">
        <label class="radio-label"><input type="radio" name="dir" value="0" %DIR0%> По часовой</label>
        <label class="radio-label"><input type="radio" name="dir" value="1" %DIR1%> Против часовой</label>
        <label class="radio-label"><input type="radio" name="dir" value="2" %DIR2%> Случайно</label>
      </div>

      <div style="background:#2c2c2c; padding:12px; border-radius:8px; margin-bottom:20px;">
        <div class="input-row" style="margin-bottom:8px;">
          <label for="cycles" style="color:white; font-weight:bold;">Циклов в час:</label>
          <input type="number" id="cycles" value="%CYCLES%" style="width:100px;">
        </div>
        <div style="font-size:11px; color:#888; text-align:left; line-height:1.4;">
          Макс. скорость (stab=0):<br>
          • ~11800 (одно направление)<br>
          • ~9500 (случайное)<br>
          При stab=1с: ~2800 / ~2700
        </div>
      </div>

      <fieldset class="tuning-box">
        <legend>Тонкая настройка</legend>
        <div class="input-row">
          <label for="targetDeg">Начальный угол (°):</label>
          <input type="number" id="targetDeg" value="%TARGETDEG%">
        </div>
        <div class="input-row">
          <label for="stabTime">Стабилизация (мс)*:</label>
          <input type="number" id="stabTime" value="%STABTIME%">
        </div>
      </fieldset>
      <div style="font-size:11px; color:#888; text-align:center; margin-top:-10px; margin-bottom:15px;">* игнорируется в режиме "Без обратной связи"</div>

      <div class="input-row" style="background:#2c2c2c; padding:12px; border-radius:8px; margin-top:20px;">
        <label for="counter" style="color:white; font-weight:bold;">Счетчик циклов:</label>
        <input type="number" id="counter" value="%COUNTER%" style="width:100px;">
      </div>

      <button type="button" class="btn" onclick="submitForm()">Сохранить и перегрузить</button>
    </form>
    <hr style="border:0; height:1px; background:#444; margin:25px 0;">
    <button type="button" class="btn" style="background:#555;" onclick="window.location.href='/update'">OTA прошивка</button>
    <div style="text-align:center; margin-top:20px; font-size:12px; color:#555;">Версия: %VERSION%</div>
  </div>
  <script>
    function submitForm() {
      var mode = document.querySelector('input[name="mode"]:checked').value;
      var dir = document.querySelector('input[name="dir"]:checked').value;
      var targetDeg = document.getElementById('targetDeg').value;
      var stabTime = document.getElementById('stabTime').value;
      var counter = document.getElementById('counter').value;
      var cycles = document.getElementById('cycles').value;
      
      var btn = document.querySelector('.btn');
      btn.innerText = 'Сохранение...';
      btn.style.background = '#03dac6';
      
      fetch('/setMode', {
        method: 'POST', 
        headers: {'Content-Type': 'application/x-www-form-urlencoded'}, 
        body: 'mode=' + mode + '&dir=' + dir + '&targetDeg=' + targetDeg + '&stabTime=' + stabTime + '&counter=' + counter + '&cycles=' + cycles
      }).then(function() {
        document.body.innerHTML = '<div class="card" style="text-align:center;"><h1>Успешно!</h1><p style="color:#03dac6; font-size:16px;">Настройки сохранены.<br>Устройство перезагружается...</p></div>';
      });
    }
  </script>
</body>
</html>
)rawliteral";

  html.replace("%MODE0%", testMode == 0 ? "checked" : "");
  html.replace("%MODE2%", testMode == 2 ? "checked" : "");
  html.replace("%MODE1%", testMode == 1 ? "checked" : "");
  html.replace("%MODE3%", testMode == 3 ? "checked" : "");

  html.replace("%DIR0%", dirMode == 0 ? "checked" : "");
  html.replace("%DIR1%", dirMode == 1 ? "checked" : "");
  html.replace("%DIR2%", dirMode == 2 ? "checked" : "");

  html.replace("%CYCLES%", String(cyclesPerHour));
  html.replace("%TARGETDEG%", String(targetDeg));
  html.replace("%STABTIME%", String(stabTimeMs));
  html.replace("%COUNTER%", String(firstPosCounter));
  html.replace("%VERSION%", FIRMWARE_VERSION);

  server.send(200, "text/html", html);
}

void handleSetMode() {
  if (server.hasArg("mode")) {
    testMode = server.arg("mode").toInt();
    preferences.putInt("testMode", testMode);
    
    if (testMode == 0) {
      strState = "SW 1-key";
    } else if (testMode == 2) {
      strState = "SW 2-key";
    } else if (testMode == 3) {
      strState = "SW no feedback";
    } else {
      strState = "SW pass-through";
    }
  }
  
  if (server.hasArg("counter")) {
    firstPosCounter = server.arg("counter").toInt();
    preferences.putUInt("counter", firstPosCounter);
  }
  
  if (server.hasArg("dir")) {
    dirMode = server.arg("dir").toInt();
    preferences.putInt("dirMode", dirMode);
  }
  
  if (server.hasArg("targetDeg")) {
    targetDeg = server.arg("targetDeg").toInt();
    preferences.putInt("targetDeg", targetDeg);
  }
  
  if (server.hasArg("stabTime")) {
    stabTimeMs = server.arg("stabTime").toInt();
    preferences.putInt("stabTimeMs", stabTimeMs);
  }
  
  if (server.hasArg("cycles")) {
    cyclesPerHour = server.arg("cycles").toInt();
    if (cyclesPerHour <= 0) cyclesPerHour = 1; // Защита от деления на 0
    preferences.putInt("cyclesPerHour", cyclesPerHour);
  }
  
  server.send(200, "text/plain", "OK");
  
  delay(1000);
  ESP.restart();
}

void setup() {
  Serial.begin(115200);
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(CONTACT_PIN_1, INPUT);
  pinMode(CONTACT_PIN_2, INPUT);

  contact1StatePrev = digitalRead(CONTACT_PIN_1);  // Предыдущее состояние контакта 1
  contact2StatePrev = digitalRead(CONTACT_PIN_2);  // Предыдущее состояние контакта 2

  randomSeed(analogRead(36));

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();

  preferences.begin("stepper", false);
  firstPosCounter = preferences.getUInt("counter", 0);
  testMode = preferences.getInt("testMode", 0);
  dirMode = preferences.getInt("dirMode", 2);
  targetDeg = preferences.getInt("targetDeg", 145);
  stabTimeMs = preferences.getInt("stabTimeMs", 1000);
  cyclesPerHour = preferences.getInt("cyclesPerHour", 600);
  
  if (testMode == 0) {
    strState = "SW 1-key";
  } else if (testMode == 2) {
    strState = "SW 2-key";
  } else if (testMode == 3) {
    strState = "SW no feedback";
  } else {
    strState = "SW pass-through";
  }

  stepper.setMaxSpeed(6000);
  stepper.setAcceleration(40000);

  Serial.println("Старт: " + String(firstPosCounter));

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Проверяем состояние кнопки сброса при старте
  if (!digitalRead(BUTTON_PIN)) {
    firstPosCounter = 0;
    preferences.putUInt("counter", firstPosCounter);
    Serial.println("Счетчик сброшен");
  }
  
  display.clearDisplay();
  display.setCursor(0, 10);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("click-start/stop");
  display.println("long(5s)-setting");
  display.println("");
  display.print("cnt: ");
  display.println(firstPosCounter);
  display.display();
  
  stepper.disable();
}

void loop() {
  bool rawBtn = !digitalRead(BUTTON_PIN);
  if (rawBtn != lastRawBtn) {
    debounceTimer = millis();
    lastRawBtn = rawBtn;
  }

  if ((millis() - debounceTimer) > 100) {
    if (rawBtn != currentBtn) {
      currentBtn = rawBtn;
      if (currentBtn) {
        // Нажатие кнопки
        pressStart = millis();
        wifiTriggered = false;
      } else {
        // Отпускание кнопки
        if (millis() - pressStart < 1000 && !wifiTriggered) {
          if (!isWifiEnabled) {
            isRunning = !isRunning;
            if (!isRunning) {
              stepper.stop();
              stepper.disable();
              display.clearDisplay();
              display.setCursor(0, 10);
              display.setTextSize(1);
              display.setTextColor(WHITE);
              display.println("click-start/stop");
              display.println("long(5s)-setting");
              display.println("");
              display.print("cnt: ");
              display.println(firstPosCounter);
              display.display();
              currentState = WAITING_FOR_ACTION;
            } else {
              isFirstMove = true;
            }
          }
        }
      }
    }
  }

  // Удержание > 5 сек
  if (currentBtn && !wifiTriggered) {
    if (millis() - pressStart >= 5000) {
      wifiTriggered = true;
      
      if (!isWifiEnabled) {
        isWifiEnabled = true;
        WiFi.softAP("LindasSW");
        dnsServer.start(53, "*", WiFi.softAPIP());
        server.on("/", handleRoot);
        server.on("/setMode", handleSetMode);
        server.on("/update", HTTP_GET, []() {
          server.sendHeader("Connection", "close");
          server.send(200, "text/html", "<!DOCTYPE html><html lang='ru'><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>OTA Обновление</title><style>body{background:#121212;color:#fff;font-family:sans-serif;text-align:center;padding:50px;}input[type=submit]{background:#03dac6;border:none;padding:10px 20px;border-radius:5px;cursor:pointer;margin-top:20px;}</style></head><body><h2>Обновление прошивки</h2><form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update' style='margin-bottom:20px;'><br><input type='submit' value='Загрузить и обновить'></form></body></html>");
        });
        server.on("/update", HTTP_POST, []() {
          server.sendHeader("Connection", "close");
          server.send(200, "text/plain", (Update.hasError()) ? "ОШИБКА ОБНОВЛЕНИЯ" : "УСПЕШНО! Перезагрузка...");
          delay(1000);
          ESP.restart();
        }, []() {
          HTTPUpload& upload = server.upload();
          if (upload.status == UPLOAD_FILE_START) {
            Serial.printf("Update: %s\n", upload.filename.c_str());
            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
              Update.printError(Serial);
            }
          } else if (upload.status == UPLOAD_FILE_WRITE) {
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
              Update.printError(Serial);
            }
          } else if (upload.status == UPLOAD_FILE_END) {
            if (Update.end(true)) {
              Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
            } else {
              Update.printError(Serial);
            }
          }
        });
        server.onNotFound([]() {
          server.sendHeader("Location", "/", true);
          server.send(302, "text/plain", "");
        });
        server.begin();
        Serial.println("Wi-Fi AP started: LindasSW (192.168.4.1)");
      }
      
      display.clearDisplay();
      display.setCursor(0, 10);
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.println("Wi-Fi AP: LindasSW");
      display.println("IP: 192.168.4.1");
      display.println("");
      display.print("cnt: ");
      display.println(firstPosCounter);
      display.display();
      
      // Отключаем шаговик при входе в настройки
      isRunning = false;
      stepper.disable();
    }
  }

  // Если Wi-Fi включен, обрабатываем запросы
  if (isWifiEnabled) {
    dnsServer.processNextRequest();
    server.handleClient();
  }

  // Основная логика работы двигателя
  if (isRunning) {
    switch (currentState) {
      case WAITING_FOR_ACTION:
        {
          if (millis() - lastActionTime >= (3600000UL / cyclesPerHour)) { 
          bool nextDirection;
          if (dirMode == 0) {
            nextDirection = false;
          } else if (dirMode == 1) {
            nextDirection = true;
          } else {
            nextDirection = random(2);
          }
          stepper.reverse(nextDirection);
          
          int moveDeg;
          if (isFirstMove) {
            moveDeg = targetDeg;
            isFirstMove = false;
            stepper.setMaxSpeed(6000); // стандартная скорость для первого хода
          } else {
            if (nextDirection == currentDirection) {
              moveDeg = 90;
              stepper.setMaxSpeed(6000); // 90° — короткий ход, стандартная скорость
            } else {
              moveDeg = (2 * targetDeg) - 90;
              stepper.setMaxSpeed(9000); // 200° — длинный ход, повышенная скорость
            }
          }
          
          stepper.setTargetDeg(moveDeg, RELATIVE);
          currentDirection = nextDirection;
          currentState = MOVING;
          lastActionTime = millis(); // Засекаем старт цикла
          stepper.enable();
          }
        }
        break;

      case MOVING:
        if (!stepper.tick()) {
          stepper.disable();
          lastDisableTime = millis();
          currentState = STABILIZING;
        }
        break;

      case STABILIZING:
        if (millis() - lastDisableTime >= ((testMode == 3) ? 0 : stabTimeMs)) {
            bool contact1State = digitalRead(CONTACT_PIN_1);
            bool contact2State = digitalRead(CONTACT_PIN_2);

            if (testMode == 3) {
                condition = true; // Без обратной связи - всегда успешно
            } else if (testMode == 0 || testMode == 2) {
                condition = checkContactChange2w(contact1State, contact2State, contact1StatePrev, contact2StatePrev);
            } else if (testMode == 1) {
                condition = (contact1State != contact1StatePrev && contact2State != contact2StatePrev);
            }

            if (condition) {
                contact1StatePrev = contact1State;
                contact2StatePrev = contact2State;
                
                firstPosCounter++;
                preferences.putUInt("counter", firstPosCounter);
                Serial.println("Клац №" + String(firstPosCounter));
                display.setCursor(0, 10);
                display.clearDisplay();
                display.setTextSize(1);
                display.setTextColor(WHITE);
                display.println(strState);
                display.println("");
                display.println("");
                display.setTextSize(2);
                display.println("  " + String(firstPosCounter));
                display.display();
                
                currentState = WAITING_FOR_ACTION;
            } else {
                stepper.stop();
                stepper.disable();
                isRunning = false;
                
                display.setCursor(0, 10);
                display.clearDisplay();
                display.setTextSize(1);
                display.setTextColor(WHITE);
                display.println(strState);
                display.println("");
                display.setTextSize(2);
                display.println("ERROR");
                display.println("  " + String(firstPosCounter));
                display.display();
                
                currentState = WAITING_FOR_ACTION;
            }
        }
        break;
    }
  }
}
