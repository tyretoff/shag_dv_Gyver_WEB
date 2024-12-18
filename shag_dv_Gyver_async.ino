#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <Preferences.h>
#define DRIVER_STEP_TIME 1
#include <GyverStepper2.h>
#include <EncButton.h>

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

const int targetDeg = 145;      //первоначальный поворот выключателя, градусы
const int minSwitchPeriod = 2; // пауза между переключениями, min max, сек
const int maxSwitchPeriod = 5;  // 


GStepper2<STEPPER2WIRE> stepper(steps* N, STEP, DIR, EN);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Button btn(33); //сюда черная кнопка подключена

enum State {
  WAITING,
  MOVING_TO_FIRST_POS,
  MOVING_TO_SECOND_POS
};

State currentState = WAITING;
unsigned long lastActionTime = 0;
Preferences preferences;
unsigned int firstPosCounter = 0;

unsigned long lastDisableTime = 0; // Время, когда stepper был отключён
bool waitingForStabilization = false; // Флаг ожидания стабилизации сигналов

bool contact1StatePrev;
bool contact2StatePrev;
bool condition;
int countClick = 0;
String strState = "Lindas";

// Проверка изменения состояния контактов 00-10-11-01 для двухклавишного выключателя
bool checkContactChange2w(bool contact1State, bool contact2State, bool contact1StatePrev, bool contact2StatePrev) {
  if (contact1State != contact1StatePrev && contact2State == contact2StatePrev) {
    return true;
  } else if (contact1State == contact1StatePrev && contact2State != contact2StatePrev) {
    return true;
  }
  return false;
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

  stepper.setMaxSpeed(6000);
  stepper.setAcceleration(10000);

  Serial.println("Старт: " + String(firstPosCounter));

  // Проверяем состояние кнопки сброса при старте
  if (btn.read()) {
    firstPosCounter = 0;
    preferences.putUInt("counter", firstPosCounter);
    Serial.println("Счетчик сброшен");
  }
}

void loop() {
  btn.tick();
  if (btn.hasClicks()) {
    countClick = btn.getClicks();
  }
  if (btn.hold()) {
    countClick = 3;
  }
  switch (countClick) {
    case 1:
      strState = "Lindas sw1,sw2 test";
      break;
    case 2:
      strState = "Lindas sw-proh test";
      break;
  }

  if (countClick == 0 || countClick == 3) {
    stepper.disable();
    display.setCursor(0, 0);
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.println("");
    display.println("");
    display.println("Waiting ");
    display.display();
  } else {

    switch (currentState) {
      case WAITING:
        if (millis() - lastActionTime > random(minSwitchPeriod, maxSwitchPeriod) * 1000) { //время между переключениями, случайно, в диапазоне
          stepper.reverse(random(2));
          stepper.setTargetDeg(targetDeg, RELATIVE);
          currentState = MOVING_TO_FIRST_POS;
          stepper.enable();
        }
        break;

      case MOVING_TO_FIRST_POS:
        if (!stepper.tick()) {
        
          lastActionTime = millis();
          currentState = WAITING;
          
          stepper.setTargetDeg((90 - targetDeg), RELATIVE);
          currentState = MOVING_TO_SECOND_POS;
          firstPosCounter++;
          preferences.putUInt("counter", firstPosCounter);
          Serial.println("Клац №" + String(firstPosCounter));
          display.setCursor(0, 0);
          display.clearDisplay();
          display.setTextSize(1);
          display.setTextColor(WHITE);
          display.println(strState);
          display.println("");
          display.println("");
          display.setTextSize(2);
          display.println("  " + String(firstPosCounter));
          display.display();
        }
        break;

      case MOVING_TO_SECOND_POS:
    if (!stepper.tick()) {
        if (!waitingForStabilization) {
            stepper.disable();
            lastDisableTime = millis(); // Сохраняем время начала паузы
            waitingForStabilization = true; // Устанавливаем флаг ожидания
        }

        // Проверяем, прошло ли 2 секунды
        if (waitingForStabilization && millis() - lastDisableTime >= 2000) {
            bool contact1State = digitalRead(CONTACT_PIN_1);
            bool contact2State = digitalRead(CONTACT_PIN_2);

            if (countClick == 1) {
                condition = checkContactChange2w(contact1State, contact2State, contact1StatePrev, contact2StatePrev);
            }
            if (countClick == 2) {
                condition = (contact1State != contact1StatePrev && contact2State != contact2StatePrev);
            }

            if (condition) {
                // Сохранение текущего состояния для следующей итерации
                contact1StatePrev = contact1State;
                contact2StatePrev = contact2State;
                lastActionTime = millis();   // вот почему добавляется время стабилизации к времени между переключениями. Исправлять ( переносить после !stepper.tick())не буду, уменьшу диапазон рандома на время стабилизации
                currentState = WAITING;
            } else {
                // Остановить двигатель
                stepper.stop();
                stepper.disable();
                // Вывести сообщение об ошибке и значение firstPosCounter

                display.setCursor(0, 0);
                display.clearDisplay();
                display.setTextSize(1);
                display.setTextColor(WHITE);
                display.println(strState);
                display.println("");
                display.setTextSize(2);
                display.println("ERROR");
                display.println("  " + String(firstPosCounter));
                display.display();
                while (true) {  // бесконечный цикл
                    delay(100);   // задержка для уменьшения нагрузки на контроллер
                }
            }
            waitingForStabilization = false; // Сбрасываем флаг ожидания
        }
    }
    break;
    }
  }
}
