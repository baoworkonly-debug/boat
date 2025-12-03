#include <Arduino_FreeRTOS.h>
#include <EEPROM.h>
#include <queue.h>
QueueHandle_t queue_anable_gps;
QueueHandle_t queue_handle_gps;
// define two tasks for Blink & Serial
void LORA(void* pvParameters);
void GPS(void* pvParameters);
void handle_gps(void* pvParameters);
#include <Servo.h>
#include <SoftwareSerial.h>
//#include <L298NX2.h>

Servo servo1;
Servo servo2;

Servo servo3;
volatile uint8_t check_gps = 0, check_ = 0;
volatile float value1 = 0, value2 = 0;
const unsigned int EN_A = 46;
const unsigned int IN1_A = 48;
const unsigned int IN2_A = 50;

const unsigned int IN1_B = 42;
const unsigned int IN2_B = 40;
const unsigned int EN_B = 44;

//L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);
//L298NX2 motors(IN1_A, IN2_A, IN1_B, IN2_B);
// epprom
const int EEPROM_START_ADDRESS = 0;

uint8_t state_dc1 = 0;

const int EEPROM_ADDRESS1 = 0;
const int EEPROM_ADDRESS2 = 10;

String gps[7] = { "AT", "AT+MGPSC=1", "AT+MGPSC=0", "AT+GPSINIT=1", "AT+GTPOS=1", "AT$HTTPOPEN", "AT+GTPOS=2" };
// String answer_1[10] = { "AT", "AT+MGPSC=1", "AT+MGPSC=0", "CONNECT OK" };
// String answer_2[10] = { "OK", "OK" };
// String err[3] = { "CONNECT FAILED", "+CME ERROR: 4" };

// +GTPOS:106.575669,10.787549

// OK
//                enable              off gps
void control_DC(uint8_t anal1, uint8_t anal2);
void check_gohome(bool a);



}
// ------------------------ lora + ln982 + servo----------------------------------------
uint8_t handle_string(String c, uint8_t anable_gps) {
  uint8_t d[6];

  char a[8] = { '@', '#', '$', '%', '&', '*', '/' };
  uint8_t b[6];
  for (int i = 0; i < 6; i++) {
    b[i] = c.indexOf(a[i]);
  }

  if (b[0] != 255 && b[6] != 255) {
    for (int i = 0; i < 6; i++) {
      d[i] = c.substring(b[i] + 1, b[i + 1]).toInt();
      //Serial.println(d[i]);
    }

    if (d[4] == 0) {
      Serial.println(d[4]);
      servo3.write(180);
    } else {
      servo3.write(90);
    }
    if (d[5] == 0) {

      check_ = 0;

      Serial2.println("9");
    } else if (d[5] == 1) {
      check_ = 1;

      Serial2.println("8");
    }

    // Serial.println(asd);
    control_DC(d[0], d[1]);

    servo1.write(map(d[2], 0, 255, 0, 180));
    servo2.write(map(d[3], 0, 255, 0, 180));
  }
}
// ------------------------ lora + ln982 _servo----------------------------------------

void setup() {

  pinMode(22, OUTPUT);
  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);
  pinMode(IN1_B, OUTPUT);
  pinMode(IN2_B, OUTPUT);
  Serial.begin(115200);
  Serial3.begin(115200);
  Serial2.begin(9600);
  //motors.setSpeed(60);
  //  motors.setSpeedA(map(anal2, 136, 255, 0, 255));
  // // motors.forwardA();
  // motors.forward();
  // delay(1000);

  //queue_anable_gps = xQueueCreate(1, sizeof(int));
  // queue_handle_gps = xQueueCreate(1, sizeof(int));
  ;
  check_gps == 0;


  xTaskCreate(
    GPS, "GPS", 128  // Stack size
    ,
    NULL  //Parameters passed to the task function
    ,
    1  // Priority
    ,
    NULL);  //Task handle
  xTaskCreate(
    handle_gps, "handle_gps", 128  // Stack size
    ,
    NULL, 1  // Priority
    ,
    NULL);
  vTaskStartScheduler();
}




void loop() {


  // ----- esp32cam --- ////
  // if (Serial1.available()) {
  //   //  a1b2c3d
  //   //  http://192.168.1.9/data320x240.45
  //   //  http://192.168.1.9/320x240.mjpeg
  //   String c = Serial1.readString();
  //   uint8_t d[6];
  //   char a[4] = { 'z', 'x', 'c', 'v' };
  //   uint8_t b[4];
  //   for (int i = 0; i < 4; i++) {
  //     b[i] = c.indexOf(a[i]);
  //   }
  //   if (b[0] != 255 && b[1] != 255 && b[2] != 255 && b[3] != 255) {

  //     for (int i = 0; i < 3; i++) {
  //       d[i] = c.substring(b[i] + 1, b[i + 1]).toInt();
  //       Serial.println(d[i]);
  //     }
  //     servo1.write(d[1]);





  //     servo2.write(d[2]);
  //     servo3.write(d[3]);
  //   }
  // }
}
void handle_gps(void* pvParameters)  // This is a task.
{
  (void)pvParameters;
  uint8_t check_ = 0, handle_gps;
  int led = 0;
  for (;;)  // A Task shall never return or exit.
  {
    Serial.println(11);


    for (uint8_t i = 0; i < 7; i++) {
      digitalWrite(22, led);
      led = !led;

      Serial3.println(gps[i]);
      Serial.print("gui:");
      Serial.println(gps[i]);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelay(60000 / portTICK_PERIOD_MS);
  }
}


void GPS(void* pvParameters)  // This is a task.
{


  // float value1 = 106.1321231;           // giá trị float cần lưu
  // EEPROM.put(EEPROM_ADDRESS1, value1);  // lưu giá trị vào EEPROM
  // Serial.print("Đã lưu giá trị: ");
  // Serial.println(value1);
  // delay(1000);
  // float value2 = 10.1321231;            // giá trị float cần lưu
  // EEPROM.put(EEPROM_ADDRESS2, value2);  // lưu giá trị vào EEPROM
  // Serial.print("Đã lưu giá trị: ");
  // Serial.println(value2);
  // delay(1000);



  uint8_t anable_gps = 0;
  uint8_t state_speed_dc1 = 0;
  uint8_t state_speed_dc2 = 0;
  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(4);
  float check_a = 0, check_b = 0, home_L = 0, home_R = 0, a = 0, b = 0;
  bool anable_gohome = false;
  uint8_t handle_gps = 0;
  unsigned long time;
  (void)pvParameters;

  for (;;)  // A Task shall never return or exit.
  {
    if (Serial2.available() > 0) {
      //Serial.println("inputReceive");

      String inputReceive = Serial2.readStringUntil('\n');

      if (inputReceive != "") {
        inputReceive.trim();
        Serial.println(inputReceive);
        handle_string(inputReceive, anable_gps);

        inputReceive = "";
        //  time = millis();
      }
    }


    if (Serial3.available()) {

      String ser2 = Serial3.readString();
      //Serial.print("ser:");
      //Serial.println(ser2);

      String gtpos = ser2.substring(13, 20);
      // Serial.println(gtpos );
      if (gtpos == "+GTPOS:") {


        String x = ser2.substring(20, 30);
        String y = ser2.substring(31, 40);
        a = x.toFloat();
        b = y.toFloat();

        String send = "AT$HTTPPARA=******************************************************************************************/exec?value3=" + x + "&value4=" + y + ",443,1,0";
        Serial3.println(send);
        //Serial.println(send);

        Serial3.println("AT$HTTPACTION=0");
        vTaskDelay(10 / portTICK_PERIOD_MS);
        anable_gohome = true;
      }
    }
    // if (((unsigned long)(millis() - time) >= 10000) && anable_gohome == true) {
    //   Serial.print("go home start");

    //   if (a > 1 && b > 1) {
    //     float retrievedValue1;
    //     EEPROM.get(EEPROM_ADDRESS1, retrievedValue1);  // đọc giá trị từ EEPROM
    //     Serial.print("Giá trị đã lấy ra: ");
    //     Serial.println(retrievedValue1, 9);
    //     vTaskDelay(100 / portTICK_PERIOD_MS);
    //     float retrievedValue2;
    //     EEPROM.get(EEPROM_ADDRESS2, retrievedValue2);  // đọc giá trị từ EEPROM
    //     Serial.print("Giá trị đã lấy ra: ");
    //     Serial.println(retrievedValue2, 9); //
    //     vTaskDelay(100 / portTICK_PERIOD_MS);
    //     if (abs(home_L - a) < check_a) {
    //       check_a = abs(home_L - a);
    //       check_gohome(1);


    //     } else {
    //       check_a = abs(home_L - a);
    //       check_gohome(0);
    //     }
    //     if (abs(home_R > b) > check_b) {
    //       check_b = abs(home_R - b);
    //       check_gohome(1);
    //     } else {
    //       check_b = abs(home_R - b);
    //       check_gohome(0);
    //     }
    //   }
    // }
  }
}
void check_gohome(bool a) {

  if (a == true) {
    control_DC(100, 130);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    control_DC(100, 100);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  } else {
    control_DC(140, 130);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    control_DC(140, 140);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}
void control_DC(uint8_t anal1, uint8_t anal2) {
  //motors.setSpeed(90);


  if (anal1 > 120 && anal1 < 135) {

    if (anal2 > 135) {
      digitalWrite(IN1_A, 0);
      digitalWrite(IN2_A, 1);
      digitalWrite(IN1_B, 0);
      digitalWrite(IN2_B, 1);

    } else if (anal2 < 120) {
      digitalWrite(IN1_A, 1);
      digitalWrite(IN2_A, 0);
      digitalWrite(IN1_B, 1);
      digitalWrite(IN2_B, 0);
  
    } else if (anal2 < 135 && anal2 > 120) {
      // motors.stopB();
      // motors.stopA();

      digitalWrite(IN1_A, 0);
      digitalWrite(IN2_A, 0);
      digitalWrite(IN1_B, 0);
      digitalWrite(IN2_B, 0);
    }
  }


  if (anal2 > 120 && anal2 < 135) {

    // motors.setSpeedA(90);
    // motors.forwardA();

    if (anal1 < 120) {
      digitalWrite(IN1_B, 0);
      digitalWrite(IN2_B, 1);
      ///motors.setSpeedB(90);
      // motors.forwardB();
    } else {
      digitalWrite(IN1_B, 0);
      digitalWrite(IN2_B, 0);
    }

    if (anal1 > 135) {
      digitalWrite(IN1_A, 0);
      digitalWrite(IN2_A, 1);
    } else {
      // motors.forwardA();
      digitalWrite(IN1_A, 0);
      digitalWrite(IN2_A, 0);
    }
  }
