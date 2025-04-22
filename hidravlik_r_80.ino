#include <Arduino.h>

// Пины подключения
const int joystickXPin = A1; // Ось X джойстика
const int potPin = A0;        // Потенциометр
const int in1Pin = 9;         // IN1 IBT-2
const int in2Pin = 10;        // IN2 IBT-2
const int buttonPin = 2;      // Пин для кнопки
const int pin3 = 3;           // АОГ пин для проверки 3.3 В
const int pin4 = 4;           // АОГ пин для проверки 3.3 В
const int ledPin6 = 6;           // пин светодиода
const int ledPin7 = 7;           // пин светодиода
const int ledPin8 = 8;           // пин светодиода

// Фиксированные положения актуатора
const int fixedPosition1 = 497; // Первое фиксированное положение нейтральное
const int fixedPosition2 = 422; // Второе фиксированное положение подъём
const int fixedPosition3 = 580; // Третье фиксированное положение опускание
const int fixedPosition4 = 620; // Четвёртое фиксированное положение плавающее
const int tolerance = 5;        // Допустимая погрешность для фиксированного положения

// Переменные для обработки кнопки
bool lastButtonState = HIGH; // Предыдущее состояние кнопки
bool currentButtonState;      // Текущее состояние кнопки
bool buttonPressed = false;   // Флаг нажатия кнопки
bool actuatorPosition4 = false; // Флаг для отслеживания положения актуатора

// Переменная для отслеживания текущего положения актуатора
int currentActuatorPosition = fixedPosition1; // Начальное положение актуатора

void setup() {
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); // Используем внутренний подтягивающий резистор
  pinMode(pin3, INPUT); // Настраиваем пин 3 как вход
  pinMode(pin4, INPUT); // Настраиваем пин 4 как вход
  pinMode(ledPin6, OUTPUT); // пин 6 выход
  pinMode(ledPin7, OUTPUT); // пин 7 выход
  pinMode(ledPin8, OUTPUT); // пин 8 выход
  Serial.begin(9600);
}

void controlActuator(int targetPosition, int potNormalized, int power) {
  if (potNormalized > targetPosition + tolerance) {
    analogWrite(in1Pin, power); // Двигаться вперед
    digitalWrite(in2Pin, LOW);
    currentActuatorPosition = targetPosition; // Обновляем текущее положение
  } else if (potNormalized < targetPosition - tolerance) {
    digitalWrite(in1Pin, LOW);
    analogWrite(in2Pin, power); // Двигаться назад
    currentActuatorPosition = targetPosition; // Обновляем текущее положение
  } else {
    // Остановить актуатор, если он в пределах допустимой погрешности
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
}

// Функция для управления светодиодами
void controlLEDs(int potValue) {
  if (potValue >= fixedPosition1 + tolerance && potValue <= fixedPosition1 - tolerance) { // Нейтральное
    digitalWrite(ledPin6, LOW);
    digitalWrite(ledPin7, LOW);
    digitalWrite(ledPin8, LOW);
  } else if (potValue >= fixedPosition2  && potValue <= fixedPosition1 - tolerance) { // Подъём ledPin6
    digitalWrite(ledPin6, HIGH);
    digitalWrite(ledPin7, LOW);
    digitalWrite(ledPin8, LOW);
  } else if (potValue <= fixedPosition3 + (4*tolerance) && potValue >= fixedPosition1 + (4*tolerance)) { // Опускание ledPin7
    digitalWrite(ledPin6, LOW);
    digitalWrite(ledPin7, HIGH);
    digitalWrite(ledPin8, LOW);
  } else if (potValue <= fixedPosition4 + (4*tolerance) && potValue >= fixedPosition3 + (4*tolerance)) { // Плавающее ledPin8
    digitalWrite(ledPin6, LOW);
    digitalWrite(ledPin7, LOW);
    digitalWrite(ledPin8, HIGH);
  } else {
    // Если значение не попадает ни в один диапазон, выключаем все светодиоды
    digitalWrite(ledPin6, LOW);
    digitalWrite(ledPin7, LOW);
    digitalWrite(ledPin8, LOW);
  }
}

void loop() {
  int joystickXValue = analogRead(joystickXPin);
  int potValue = analogRead(potPin);
  int potNormalized = map(potValue, 0, 1023, 0, 1000); // Пример нормализации для актуатора
   // Считываем текущее состояние кнопки
  currentButtonState = digitalRead(buttonPin);

  // Проверка нажатия кнопки (переход с HIGH на LOW)
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    buttonPressed = !buttonPressed; // Переключаем состояние кнопки
    actuatorPosition4 = !actuatorPosition4; // Переключаем флаг положения актуатора
  }
  Serial.print("Флаг кнопки: ");
  Serial.println(buttonPressed ? "Нажата" : "Не нажата");

  
  // Управление актуатором
  if (actuatorPosition4) { // Если актуатор должен быть в позиции 4
    controlActuator(fixedPosition4, potNormalized, 255); // Перемещение в позицию 4
    controlLEDs( potValue);
    if (digitalRead(pin3) == HIGH) { //если пин 3 активен
    actuatorPosition4 = false; // условие не выполняется,нормализация джойстика
    controlLEDs( potValue);              
    }
  } else if (digitalRead(pin3) == HIGH) { // Если пин 3 активен
    controlActuator(fixedPosition2, potNormalized, 255); // Перемещение в позицию 2
    controlLEDs( potValue);
  } else if (digitalRead(pin4) == HIGH) { // Если пин 4 активен
    controlActuator(fixedPosition3, potNormalized, 255); // Перемещение в позицию 3
    controlLEDs( potValue);    
         if (buttonPressed) { // если флаг кнопки активен
         actuatorPosition4 = true ; // актуатор движется в позицию 4
         controlLEDs( potValue);
         }
    }
    
   else {
    // Нормализация значений джойстика
    float joystickXNormalized = map(joystickXValue, 0, 1023, -100, 100);

    // Управление актуатором с помощью джойстика
    if (abs(joystickXNormalized) > 10) { // Если джойстик отклонен
      int power = map(abs(joystickXNormalized), 10, 100, 0, 255); // Нормализация мощности

      if (joystickXNormalized > 10) { // Движение вперед к позиции 3
        controlActuator(fixedPosition3, potNormalized, power);
        controlLEDs( potValue);
      } else { // Движение назад к позиции 2
        controlActuator(fixedPosition2, potNormalized, power);
        controlLEDs( potValue);
      }
    } else { // Если джойстик в нейтральном положении
      controlActuator(fixedPosition1, potNormalized, 255);
      controlLEDs( potValue);
    }
  }

  // Обновляем предыдущее состояние кнопки
  lastButtonState = currentButtonState;

  // Отладочная информация
  Serial.print("Joystick X: ");
  Serial.print(joystickXValue);
  Serial.print(" | Potentiometer: ");
  Serial.println(potValue);
  
  delay(100); //
}
