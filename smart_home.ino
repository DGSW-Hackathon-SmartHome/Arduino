#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

//DHT11 관련 핀 활성화
#define DHTPIN A1   //인풋 핀
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial PISerial(2, 3);

//LCD관련 세팅
LiquidCrystal_I2C lcd(0x27, 16, 2);

//미세먼지 센서관련 핀 활성화, 변수 선언
int Vo = A0;          //적외선 핀 번호
int dust_led = 10;
float Vo_value = 0;
float dust_voltage = 0;

//환풍기 관련 핀, 변수 설정
#define fanPin  6             //모터 디지털 핀
int fanCondition = 0;       //fan ON(1) / OFF(0) 상태
int fanAuto = 0;            //fan auto mode 상태  0 = semi / 1 = auto

#define valvePin 7          //서보모터 핀
int valve_angle = 0;

void setup() {
  //시리얼 통신속도
  Serial.begin(9600);
  PISerial.begin(9600);
  PISerial.print("\n\n\n\ninit success!\n\n\n\n");

  //fan settings
  pinMode(fanPin, OUTPUT);

  //미세먼지 센서 핀 모드 설정
  pinMode(Vo, INPUT);
  pinMode(dust_led, OUTPUT);

  //가스벨브 세팅
  pinMode(valvePin, OUTPUT);

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
bool air = false;
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
      // 에어컨 on
      aircon = true;
    }else if (cmd == 'a'){
      // 에어컨 off
      aircon = false;
    }else if (cmd == 'B'){
      // 보일러 on
      boiler = true;
    }else if (cmd == 'b'){
      // 보일러 off
      boiler = false;
    }else if (cmd == 'D'){
      // 제습기 on
      dehumidifier = true;
    }else if (cmd == 'd'){
      // 제습기 off
      dehumidifier = false;
    }else if (cmd == 'H'){
      // 가습기 on
      humidifier = true;
    }else if (cmd == 'h'){
      // 가습기 off
      humidifier = false;
    }else if (cmd == 'F'){
      // 환풍기 on
      fan = true;
    }else if (cmd == 'f'){
      // 환풍기 off
      fan = false;
    }else if (cmd == 'G'){      
      // 가스밸브 on
      gas_servo = true;
      gas_angle = 1;
    }else if (cmd == 'g'){
      // 가스밸브 off
      gas_servo = true;
      gas_angle = 0;
    }else if (cmd == 'L'){
      // 거실 led on
      room_led[0] = true;
    }else if (cmd == 'l'){
      // 거실 led off
      room_led[0] = false;
    }else if (cmd == 'R'){
      // 방 led on
      room_led[1] = true;
    }else if (cmd == 'r'){
      // 방 led off
      room_led[1] = false;
    }else if (cmd == 'T'){
      // 화장실 led on
      room_led[2] = true;
    }else if (cmd == 't'){
      // 화장실 led off
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

    //====================  환풍기  ==================================

    //    if (fanAuto == 1) {               //fan auto / semi 판단
    //      Serial.println("auto fan mode : ON");
    //      if (dust_voltage >= 50) {       // fan auto 모드시, dust >= 50 이라면 fan ON
    //        fanCondition = 1;
    //      }
    //      else {                         // fan auto 모드시, dust >= 50 이 아니라면 fan OFF
    //        fanCondition = 0;
    //      }
    //    }
    //    else {
    //      Serial.println("auto fan mode : OFF");
    //    }
    //
    //    if (fanCondition == 1) {           //fan ON / OFF 여부 판단 & 상태 시리얼 출력
    //      digitalWrite(fanPin, HIGH);
    //      Serial.println("fan : ON");
    //    }
    //    else {
    //      digitalWrite(fanPin, LOW);
    //      Serial.println("fan : OFF");
    //    }

    //===============================================================
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

}

void send_data() // 파이쪽으로 보내는 데이터 관리
{
  PISerial.print("temperarture : ");
  PISerial.print(temperature);
  PISerial.println("");
  PISerial.print("aircon : ");
  PISerial.print(aircon); //
  PISerial.println("");
  PISerial.print("boiler : ");
  PISerial.print(boiler); //
  PISerial.println("");
  PISerial.print("humidity : ");
  PISerial.print(humidity);
  PISerial.println("");
  PISerial.print("humidifier : ");
  PISerial.print(humidifier); //
  PISerial.println("");
  PISerial.print("dehumidifier : ");
  PISerial.print(dehumidifier); //
  PISerial.println("");
  PISerial.print("air : ");
  PISerial.print(dust_voltage);
  PISerial.println("");
  PISerial.print("fan : ");
  PISerial.print(fan); //
  PISerial.println("");
  PISerial.print("gasvalve : ");
  PISerial.print(gas_servo); //
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
