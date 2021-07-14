#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <Servo.h> 

Servo servo; 

#define DHTPIN A1
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial PISerial(2, 3);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// 미세먼지 측정 
int Vo = A0;            
int dust_led = 10;      
float Vo_value = 0;     
float dust_voltage = 0;

#define AIRPIN 13       // 에어컨 led
#define BOILPIN 12      // 보일러 led
#define HUMIDIFPIN 11   // 가습기 led
#define DEHUMIFPIN 9    // 제습기 led
#define FANPIN 8        // 환풍기 dc모터
#define FANPIN1 6       // 환풍기 dc모터
#define valvePin 7      // 가스밸브 서보모터
#define LED3 A3         // 화장실 led
#define LED2 5          // 방 led
#define LED1 4          // 거실 led

void setup() {
  //시리얼 통신속도
  Serial.begin(9600);
  PISerial.begin(9600);
  PISerial.print("\n\n\n\ninit success!\n\n\n\n");
  servo.attach(valvePin);
  
  // 각 종 센서 설정
  pinMode(Vo, INPUT);
  pinMode(dust_led, OUTPUT);
  pinMode(valvePin, OUTPUT);
  pinMode(AIRPIN, OUTPUT);
  pinMode(BOILPIN, OUTPUT);
  pinMode(HUMIDIFPIN, OUTPUT);
  pinMode(DEHUMIFPIN, OUTPUT);
  pinMode(FANPIN, OUTPUT);
  pinMode(FANPIN1, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  digitalWrite(AIRPIN, LOW);
  digitalWrite(BOILPIN, LOW);
  digitalWrite(HUMIDIFPIN, LOW);
  digitalWrite(DEHUMIFPIN, LOW);
  digitalWrite(FANPIN, LOW);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  
  //LCD intro & settings
  lcd.init();
  lcd.backlight();
  lcd.setCursor(2, 0);
  lcd.print("SMART");
  lcd.setCursor(4, 1);
  lcd.print("HOME");
  delay(500);
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("STARTING...");
  delay(1000);
  lcd.clear();
}

unsigned long c_micros = 0;
unsigned long c_millis = 0;
unsigned long p_millis = 0;

int temperature = 0;                      // 온도
bool aircon = false;                      // 에어컨 on/off
bool boiler = false;                      // 보일러 on/off
int humidity = 0;                         // 습도
bool humidifier = false;                  // 가습기 on/off
bool dehumidifier = false;                // 제습기 on/off
bool fan = false;                         // 환풍기 on/off
bool gas_servo = false;                   // 가스밸브 on/off
bool room_led[3] = {false, false, false}; // 거실, 방 화장실

void loop() {
  c_micros = micros();
  c_millis = millis();

  //======================  미세먼지 센서 ===========================

  if (Serial.available())
  {
     char cmd = Serial.read();

    if (cmd == 'A'){
      digitalWrite(AIRPIN, HIGH);
      aircon = true;
    }else if (cmd == 'a'){
      digitalWrite(AIRPIN, LOW);
      aircon = false;
    }else if (cmd == 'B'){
      digitalWrite(BOILPIN, HIGH);
      boiler = true;
    }else if (cmd == 'b'){
      digitalWrite(BOILPIN, LOW);
      boiler = false;
    }else if (cmd == 'D'){
      digitalWrite(DEHUMIFPIN, HIGH);
      dehumidifier = true;
    }else if (cmd == 'd'){
      digitalWrite(DEHUMIFPIN, LOW);
      dehumidifier = false;
    }else if (cmd == 'H'){
      digitalWrite(HUMIDIFPIN, HIGH);
      humidifier = true;
    }else if (cmd == 'h'){
      digitalWrite(HUMIDIFPIN, LOW);
      humidifier = false;
    }else if (cmd == 'F'){    //FAN
      digitalWrite(6, HIGH);
      digitalWrite(8, LOW);
      fan = true;
    }else if (cmd == 'f'){
      digitalWrite(6, LOW);
      digitalWrite(8, LOW);
      fan = false;
    }else if (cmd == 'G'){      
      servo.write(90);
      gas_servo = true;
    }else if (cmd == 'g'){
      servo.write(0);
      gas_servo = false;
    }else if (cmd == 'L'){
      digitalWrite(LED1, HIGH);
      room_led[0] = true;
    }else if (cmd == 'l'){
      digitalWrite(LED1, LOW);
      room_led[0] = false;
    }else if (cmd == 'R'){
      digitalWrite(LED2, HIGH);
      room_led[1] = true;
    }else if (cmd == 'r'){
      digitalWrite(LED2, LOW);
      room_led[1] = false;
    }else if (cmd == 'T'){
       analogWrite(LED3, 255);
      room_led[2] = true;
    }else if (cmd == 't'){
      analogWrite(LED3, 0);
      room_led[2] = false;
    }

  }

  if (c_millis - p_millis >= 1000)
  {
    p_millis = c_millis;

    // 센서 데이터 읽기
    dust_read();
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();

    send_data();

    // lcd 출력
    //먼지
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("dust : ");
    lcd.print(dust_voltage);
    lcd.print(" ug");
    //습도
    lcd.setCursor(0, 1);
    lcd.print("humi:");
    lcd.print(humidity);
    lcd.print("%");
    //온도
    lcd.print("temp:");
    lcd.print(temperature);
    lcd.print("C");

  }
}

void send_data() // 파이쪽으로 보내는 데이터 관리
{
  Serial.write('T'); // temperature
  Serial.write((byte*)&temperature, 2);
  Serial.write('a'); // aircon
  Serial.write((byte*)&aircon, 1);
  Serial.write('b'); // boiler
  Serial.write((byte*)&boiler, 1);
  Serial.write('H'); // humidity
  Serial.write((byte*)&humidity, 2);
  Serial.write('h'); // humidifier
  Serial.write((byte*)&humidifier, 1);
  Serial.write('d'); // dehumidifier
  Serial.write((byte*)&dehumidifier, 1);
  Serial.write('A'); // air
  Serial.write((byte*)&dust_voltage, 4);
  Serial.write('f'); // fan
  Serial.write((byte*)&fan, 1);
  Serial.write('g'); // gasvalve
  Serial.write((byte*)&gas_servo, 1);
  Serial.write('l'); // livingroom led
  Serial.write((byte*)&room_led[0], 1);
  Serial.write('r'); // room led
  Serial.write((byte*)&room_led[1], 1);
  Serial.write('t'); // toilet led
  Serial.write((byte*)&room_led[2], 1);
  Serial.write('\n');
}

void dust_read() //미세먼지 값 받아오기
{
  digitalWrite(dust_led, LOW);
  delayMicroseconds(280);
  Vo_value = analogRead(Vo);
  digitalWrite(dust_led, HIGH);
  delayMicroseconds(9680);

  dust_voltage = (0.172 * (Vo_value * (5 / 1024.0)) - 0.0999) * 1000; //표현값 voltage로
}
