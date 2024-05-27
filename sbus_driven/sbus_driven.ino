/*
 * This code is a simple UART hacking on "MN2 WSDC/3+7F&G-X XD" ESC
 * It will work only on this model number.
 * 
 * SBUS values come from Futaba receiver with UART ISR implemented
 * 
 * The SBUS value ch1 is to drive left wheel,
 * and ch2 is to drive right wheel.
 * 
 * Made by: Rasheed Kittinanthapanya
 * Date: 2024-5-24
 */

//////////////////////////
// For SBUS UART setup //
/////////////////////////
#include "driver/uart.h"

#define NUMERO_PORTA_SERIALE UART_NUM_2
#define BUF_SIZE (1024 * 2)
#define RD_BUF_SIZE (1024)

static QueueHandle_t uart2_queue;
static const char * TAG = "";
#define U2RXD 16
#define U2TXD 17

#define SBUS_MIN 368
#define SBUS_MID 1024
#define SBUS_MAX 1680
#define SBUS_DB 10

uint16_t ch[16];
uint16_t checksum = 0;
uint16_t sbus_ch[16];

uint16_t sbus_max_db = SBUS_MID + SBUS_DB;
uint16_t sbus_min_db = SBUS_MID - SBUS_DB;

////////////////////
// For ESC setup //
///////////////////
#define RX_PIN 12
#define TX_PIN 13
bool ESC_setupDone = false;

///////////////////////////
// For sending commands //
//////////////////////////
unsigned int RawInt1;
unsigned int RawInt2;
unsigned int rawInts[2];
unsigned char Motor1SpeedByte[2];
unsigned char Motor2SpeedByte[2];
unsigned long last_drive_stamp = millis();

////////////////////////////
// For ESC reply packets //
///////////////////////////
unsigned char reply_byte;
const int reply_size = 10;
unsigned char reply_packets[reply_size];
int reply_counter = 0;
bool got_reply_header = false;
unsigned long last_got_reply_stamp = millis();

////////////////////////
// For screen logging //
////////////////////////
unsigned long last_log_stamp = millis();



void setup() {

  setupESC();

  Serial.begin(115200);

  /// Config UART ISR for SBUS ///
  uart_config_t Configurazione_UART2 = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(NUMERO_PORTA_SERIALE, &Configurazione_UART2);
  esp_log_level_set(TAG, ESP_LOG_INFO);
  uart_set_pin(NUMERO_PORTA_SERIALE, U2TXD, U2RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(NUMERO_PORTA_SERIALE, BUF_SIZE, BUF_SIZE, 20, &uart2_queue, 0);
  xTaskCreate(UART_ISR_ROUTINE, "UART_ISR_ROUTINE", 2048, NULL, 12, NULL);


}

void loop() {

  memcpy(sbus_ch, ch, sizeof(ch));

  /////////////////////
  /// SBUS Failsafe ///
  ////////////////////
  if ((sbus_ch[0] == 0) || (sbus_ch[1] == 0)) {

    Serial.println("SBUS Failsafe");
    sbus_ch[0] = 1024;
    sbus_ch[1] = 1024;

  }

  ////////////////////////////////////
  /// Once setup is done,          ///
  /// start sending command to ESC ///
  ////////////////////////////////////
  if (ESC_setupDone) {
    sbusToMotorRawInt(sbus_ch[0], sbus_ch[1], rawInts);

    Int16ToByteData(rawInts[0], Motor1SpeedByte);
    Int16ToByteData(rawInts[1], Motor2SpeedByte);

    driveLeftRightByBytes(Motor1SpeedByte, Motor2SpeedByte);

    /// Read reply from ESC ///
    if (Serial1.available() > 0) {
      reply_byte = Serial1.read();

      // header byte 0x82 (130)
      if (reply_byte == 130) {
        reply_counter = 0;
        got_reply_header = true;
      }

      if (got_reply_header) {
        reply_packets[reply_counter] = reply_byte;
        reply_counter++;

        if (reply_counter == reply_size) {
          reply_counter = 0;
          got_reply_header = false;
        }

        last_got_reply_stamp = millis();

      }

    }

    /// If not getting reply from ESC
    /// reset UART and try setup one more time 
    if ((millis() - last_got_reply_stamp) > 500) {
      Serial.println("no reply from ESC");
      ESC_setupDone = false;
      Serial1.end();
    }
  } else {

    /// re setup ESC
    setupESC();


  }


  //  for (int i = 0; i < reply_size; i++) {
  //    Serial.print(reply_packets[i], HEX);
  //    Serial.print(" ");
  //  }

  //////////////////////////////////////
  /// Log on screen every at 0.1 sec ///
  //////////////////////////////////////
  if ((millis() - last_log_stamp) > 100) {

    Serial.print("ch1 ");
    Serial.print(sbus_ch[0]);
    Serial.print(" ch2 ");
    Serial.print(sbus_ch[1]);
    Serial.print(" raw_left ");
    Serial.print(rawInts[0]);
    Serial.print(" raw_right ");
    Serial.print(rawInts[1]);
    Serial.println();
    last_log_stamp = millis();

  }

}
////////////////////
/// Maths Helper ///
////////////////////

void Int16ToByteData(unsigned int Data, unsigned char StoreByte[2]){
  /*
   * Just convert uint16_t to hi-byte and lo-byte
   * 
   */
  // unsigned int can store 16 bit int
  StoreByte[0] = (Data & 0xFF00) >> 8;                  //High byte, most right of HEX
  StoreByte[1] = (Data & 0x00FF);                       //Low byte, most left of HEX
}

//////////////
/// ATCART ///
//////////////
void setupESC() {

  /*
   * The step to setup UART for ATCart ESC
   * 
   */

  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  /// Start init ///
  Serial.println("wait four zeros");
  waitFourZeros();
  ESC_Handshake();
  ESC_init();

  /// try sending zero speed for few times ///
  //  for (int i = 0; i < 10; i++) {
  //    idle();
  //  }

  ESC_setupDone = true;
  last_got_reply_stamp = millis();
  Serial.println("ESC setup done");
}

void waitFourZeros() {
  /*
   * This function will block the code until it received 4 zeros,
   * If need to keep running other code while checking, 
   * this loop need to change later...
   */

  bool startTick = true;
  bool ReadOK = false;
  unsigned char ReadByte;
  unsigned char Reply[6];
  int i = 0;

  while (startTick == true) {

    while (Serial1.available() > 0) {

      ReadByte = Serial1.read();
      Reply[i] = ReadByte;
      i++;
      ReadOK = true;
      //delay(273);                      // This delay will wait for incoming byte to complete 6 bytes as
    }

    if (ReadOK == true) {
      if ( (Reply[0] == 0) && (Reply[1] == 0) && (Reply[2] == 0) && (Reply[3] == 0 )) {
        startTick = false;
      }
      i = 0;    // reset i
      //memset(Reply, 0, sizeof(Reply));
      ReadOK = false;

    }

  }

  delay(273);

}

void send_init_bytes() {
  /*
   * The init bytes has 32 bytes and has to be exact same with this.
   * There is a lot of unknown parameters which I don't know the meaning,
   * but with this setup, the ATCart wheels could run same as before.
   * 
   * I though the last byte was checksum, but it's not...
   */
  Serial1.write(0x01);
  Serial1.write(0x20);
  Serial1.write(0xC0);
  Serial1.write(0xA8);
  Serial1.write(0x01);
  Serial1.write(0x01);
  Serial1.write(0x02);
  Serial1.write(0x64);
  Serial1.write(0x64);
  Serial1.write(byte(0x0));
  Serial1.write(0x5A);
  Serial1.write(0x64);
  Serial1.write(0x0E); //0x0E  //0x09
  Serial1.write(0x32);
  Serial1.write(0x64);
  Serial1.write(0x64);
  Serial1.write(0x64);
  Serial1.write(0x05);
  Serial1.write(0x96);
  Serial1.write(0x01);
  Serial1.write(0x03);
  Serial1.write(0x01);
  Serial1.write(0x12);
  Serial1.write((byte)0x0);
  Serial1.write(0x32);
  Serial1.write(0x14);
  Serial1.write(0x5A);
  Serial1.write(0x14);
  Serial1.write(0x02);
  Serial1.write(0x32);
  Serial1.write((byte)0x0);

  Serial1.write(0x19); //0x19  //0x14
  Serial1.flush();
}

void ESC_Handshake() {

  /*
   * The first handshake packets is to send this 53 bytes,
   * 0xFF and 20 times of 0x00,
   * after that the init bytes
   */

  Serial1.write(0xFF);
  for (int i = 0; i < 20; i++) {
    Serial1.write((byte)0x0);
  }
  send_init_bytes();

  delay(5);
}

void ESC_init() {

  /*
   * After handshake we need to send 2 times of init bytes
   * 
   */
  for (int i = 0; i < 2; i++) {
    send_init_bytes();
    delay(7);
  }

  delay(486);


}


uint16_t map_with_limit(uint16_t val, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
  /*
   * Similar to Arduino's map function but this will limit the output as well
   * 
   */

  float m;
  float out;

  m = (out_max - out_min) / (in_max - in_min);
  out = m * (val - in_min) + out_min;

  if (out_min > out_max) {
    if (out > out_min) {
      out = out_min;
    } else if (out < out_max) {
      out = out_max;
    }
  } else if (out_max > out_min) {
    if (out > out_max) {
      out = out_max;
    }
    else if (out < out_min) {
      out = out_min;
    }
  }

  return (uint16_t)out;

}

void sbusToMotorRawInt(uint16_t sbus_left, uint16_t sbus_right, unsigned int motorsRaw[2]) {

  /*
   * Convert input SBUS of left/right wheels to Raw int command 
   * 
   */

  unsigned int left_raw;
  unsigned int right_raw;

  if (sbus_left >= sbus_max_db) {
    left_raw = map_with_limit(sbus_left, sbus_max_db, SBUS_MAX, 33000, 55000);
  } else if (sbus_left <= sbus_min_db) {
    left_raw = map_with_limit(sbus_left, SBUS_MIN, sbus_min_db, 22000, 0);
  } else {
    left_raw = 0;
  }

  if (sbus_right >= sbus_max_db) {
    right_raw = map_with_limit(sbus_right, sbus_max_db, SBUS_MAX, 0, 22000);
  } else if (sbus_right <= sbus_min_db) {
    right_raw = map_with_limit(sbus_right, SBUS_MIN, sbus_min_db, 55000, 33000);
  } else {
    right_raw = 0;
  }

  motorsRaw[0] = left_raw;
  motorsRaw[1] = right_raw;


}

void driveLeftRightByBytes(unsigned char leftMotorBytes[2], unsigned char rightMotorBytes[2]) {

  /*
   * Send the command bytes of left/right wheel to ESC 
   * Instead of delay(21), we are measuring the time of last command instead,
   * then send the next command once the time has passed 21ms.
   */


  byte header1 = 0x02;
  byte header2 = 0x0B;
  byte r_hibyte = rightMotorBytes[0];
  byte r_lobyte = rightMotorBytes[1];
  byte l_hibyte = leftMotorBytes[0];
  byte l_lobyte = leftMotorBytes[1];

  byte CheckSum = header1 + header2 + r_hibyte + r_lobyte + l_hibyte + l_lobyte;

  /// instead of delay between command
  /// we are measuring time, so it won't block others process.
  if ((millis() - last_drive_stamp) >= 21) {
    Serial1.write(header1);
    Serial1.write(header2);

    Serial1.write(r_hibyte);
    Serial1.write(r_lobyte);
    Serial1.write(l_hibyte);
    Serial1.write(l_lobyte);

    Serial1.write((byte)0x0);
    Serial1.write((byte)0x0);
    Serial1.write((byte)0x0);
    Serial1.write((byte)0x0);

    Serial1.write(CheckSum);
    Serial1.flush();

    last_drive_stamp = millis();
  }

  //delayMicroseconds(21000);

}

void driveByBytes(unsigned char hi_byte, unsigned char lo_byte) {

  
  /*
   * Just for hacking step, no need for production.
   * 
   */

  byte header1 = 0x02;
  byte header2 = 0x0B;
  byte r_hibyte = hi_byte;
  byte r_lobyte = lo_byte;
  byte l_hibyte = hi_byte;
  byte l_lobyte = lo_byte;

  byte CheckSum = header1 + header2 + r_hibyte + r_lobyte + l_hibyte + l_lobyte;

  Serial1.write(header1);
  Serial1.write(header2);

  Serial1.write(r_hibyte);
  Serial1.write(r_lobyte);
  Serial1.write(l_hibyte);
  Serial1.write(l_lobyte);

  Serial1.write((byte)0x0);
  Serial1.write((byte)0x0);
  Serial1.write((byte)0x0);
  Serial1.write((byte)0x0);

  Serial1.write(CheckSum);
  Serial1.flush();
  delayMicroseconds(21000);



}

////////////////
/// SBUS ISR ///
////////////////
static void UART_ISR_ROUTINE(void *pvParameters){
  /*
   * ISR to receive SBUS signal on UART2
   * I have got this code from ESP32 dev forum.
   * try searching "ESP32 UART interrupt routine"....
   */
  uart_event_t event;
  size_t buffered_size;
  bool exit_condition = false;

  //Infinite loop to run main bulk of task
  while (1) {

    //Loop will continually block (i.e. wait) on event messages from the event queue
    if (xQueueReceive(uart2_queue, (void * )&event, (portTickType)portMAX_DELAY)) {

      //Handle received event
      if (event.type == UART_DATA) {

        uint8_t UART2_data[128];
        //uint8_t buf[35];
        //uint16_t ch[16];
        int UART2_data_length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&UART2_data_length));
        UART2_data_length = uart_read_bytes(UART_NUM_2, UART2_data, UART2_data_length, 100);

        //Serial.println("LEN= ");Serial.println(UART2_data_length);

        //Serial.print("DATA= ");
        //for(byte i=0; i<UART2_data_length;i++) Serial.print(UART2_data[i]);
        //Serial.println("");

        for (int i = 0; i < 16; i++) {
          ch[i] = ((uint16_t)UART2_data[(2 * i) + 1] << 8 | ((uint16_t)UART2_data[(2 * i) + 2]));

          checksum ^= UART2_data[(2 * i) + 1];
          checksum ^= UART2_data[(2 * i) + 2];

        }

      }

      //Handle frame error event
      else if (event.type == UART_FRAME_ERR) {
        //TODO...
      }

      //Final else statement to act as a default case
      else {
        //TODO...
      }
    }

    //If you want to break out of the loop due to certain conditions, set exit condition to true
    if (exit_condition) {
      break;
    }
  }

  //Out side of loop now. Task needs to clean up and self terminate before returning
  vTaskDelete(NULL);
}
