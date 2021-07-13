#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

#define DHTPIN A1
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial PISerial(2, 3);
LiquidCrystal_I2C lcd(0x27, 16, 2);

int Vo = A0;
int dust_led = 10;
float Vo_value = 0;
float dust_voltage = 0;

#define AIRPIN 13
#define BOILPIN 12
#define HUMIDIFPIN 11
#define DEHUMIFPIN 9
#define FANPIN 8
#define valvePin 7
#define LED3 6
#define LED2 5
#define LED1 4

int valve_angle = 0;

void setup() {
  //시리얼 통신속도
  Serial.begin(9600);
  PISerial.begin(9600);
  PISerial.print("\n\n\n\ninit success!\n\n\n\n");
  
  //미세먼지 센서 핀 모드 설정
  pinMode(Vo, INPUT);
  pinMode(dust_led, OUTPUT);

  //가스벨브 세팅
  pinMode(valvePin, OUTPUT);
  
  pinMode(AIRPIN, OUTPUT);
  pinMode(BOILPIN, OUTPUT);
  pinMode(HUMIDIFPIN, OUTPUT);
  pinMode(DEHUMIFPIN, OUTPUT);
  pinMode(FANPIN, OUTPUT);
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

int temperature = 0;
bool aircon = false;
bool boiler = false;
int humidity = 0;
bool humidifier = false;
bool dehumidifier = false;
bool fan = false;
bool gas_servo = false;
int gas_servo_count = 0;
unsigned long p_gas_servo_micros = 0;
bool room_led[3] = {false, false, false}; // 거실, 방 화장실

void loop() {
  c_micros = micros();
  c_millis = millis();

  //======================  미세먼지 센서 ===========================

  if (PISerial.available())
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
    }else if (cmd == 'F'){
      digitalWrite(FANPIN, HIGH);
      fan = true;
    }else if (cmd == 'f'){
      digitalWrite(FANPIN, LOW);
      fan = false;
    }else if (cmd == 'G'){      
      // 가스밸브 on
      gas_servo = true;
      valve_angle = 1;
    }else if (cmd == 'g'){
      // 가스밸브 off
      gas_servo = true;
      valve_angle = 0;
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
      digitalWrite(LED3, HIGH);
      room_led[2] = true;
    }else if (cmd == 't'){
      digitalWrite(LED3, LOW);
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

    //새로고침 빈도
    Serial.println("\n\n\n\n\n\n\n\n\n\n");
  }
  
  if (gas_servo)
  {
    if (c_micros - p_gas_servo_micros >= 100)
    {
      p_gas_servo_micros = c_micros;

      if(gas_servo_count == 2000)
      {
        digitalWrite(valvePin, HIGH);
        gas_servo_count = 0;
      }
      
      if (valve_angle == 0)
      {
        if(gas_servo_count >= 7) digitalWrite(valvePin, LOW);
      }
      else if (valve_angle == 1)
      {
        if(gas_servo_count >= 23) digitalWrite(valvePin, LOW);
      }
      
      gas_servo_count++;
    }
  }
  check_flag();
}

void check_flag()
{
  if(aircon) digitalWrite(AIRPIN, HIGH);
  else digitalWrite(AIRPIN, LOW);
  if(boiler) digitalWrite(BOILPIN, HIGH);
  else digitalWrite(BOILPIN, LOW);
  if(humidifier) digitalWrite(HUMIDIFPIN, HIGH);
  else digitalWrite(HUMIDIFPIN, LOW);
  if(dehumidifier) digitalWrite(DEHUMIFPIN, HIGH);
  else digitalWrite(DEHUMIFPIN, LOW);
  if(fan) digitalWrite(FANPIN, HIGH);
  else digitalWrite(FANPIN, LOW);
  
  for(int i = 0; i < 3; i++)
  {
    if(room_led[i]) digitalWrite(LED1 + i, HIGH);
    else digitalWrite(LED1 + i, LOW);
  }
}

void send_data() // 파이쪽으로 보내는 데이터 관리
{
  PISerial.print("temperarture : ");
  PISerial.print(temperature);
  PISerial.println("");
  PISerial.print("aircon : ");
  PISerial.print(aircon);
  PISerial.println("");
  PISerial.print("boiler : ");
  PISerial.print(boiler);
  PISerial.println("");
  PISerial.print("humidity : ");
  PISerial.print(humidity);
  PISerial.println("");
  PISerial.print("humidifier : ");
  PISerial.print(humidifier);
  PISerial.println("");
  PISerial.print("dehumidifier : ");
  PISerial.print(dehumidifier);
  PISerial.println("");
  PISerial.print("air : ");
  PISerial.print(dust_voltage);
  PISerial.println("");
  PISerial.print("fan : ");
  PISerial.print(fan);
  PISerial.println("");
  PISerial.print("gasvalve : ");
  PISerial.print(gas_servo);
  PISerial.println("");
  PISerial.print("livingroom led : ");
  PISerial.print(room_led[0]);
  PISerial.println("");
  PISerial.print("room led : ");
  PISerial.print(room_led[1]);
  PISerial.println("");
  PISerial.print("toilet led : ");
  PISerial.print(room_led[2]);
  PISerial.println("");
}

void dust_read()
{
  digitalWrite(dust_led, LOW);
  delayMicroseconds(280);
  Vo_value = analogRead(Vo);
  digitalWrite(dust_led, HIGH);
  delayMicroseconds(9680);

  dust_voltage = (0.172 * (Vo_value * (5 / 1024.0)) - 0.0999) * 1000; //표현값 voltage로
}
