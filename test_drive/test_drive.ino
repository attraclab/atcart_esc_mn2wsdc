#include <SoftwareSerial.h>

#define RX_PIN 11
#define TX_PIN 10

SoftwareSerial _ser(RX_PIN, TX_PIN);

void setup() {

  //pinMode(RX_PIN, INPUT);
  //pinMode(TX_PIN, OUTPUT);

  //Serial.begin(115200);
  _ser.begin(9600);

  //Serial.println("start waitFourZeros");
  //waitFourZeros();
  delay(406);
  //Serial.println("start handshake");
  ESC_Handshake();
  //Serial.println("start esc init");
  ESC_init();

  Serial.println("finished init");

  for(int i=0; i<20; i++){
    idle();
  }

  
}

void loop() {


  fullSpeed();

  
}

void waitFourZeros() {

  bool startTick = true;
  bool ReadOK = false;
  unsigned char ReadByte;
  unsigned char Reply[6];
  int i = 0;

  while (startTick == true) {

    while (_ser.available() > 0) {

      ReadByte = _ser.read();
      Reply[i] = ReadByte;
      i++;
      ReadOK = true;
      delay(273);                      // This delay will wait for incoming byte to complete 6 bytes as
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

}

void ESC_Handshake() {

  // send 53 bytes

  _ser.write(0xFF);
  for (int i = 0; i < 20; i++) {
    _ser.write((byte)0x0);
  }
  _ser.write(0x01);
  _ser.write(0x20);
  _ser.write(0xC0);
  _ser.write(0xA8);
  _ser.write(0x01);
  _ser.write(0x01);
  _ser.write(0x02);
  _ser.write(0x46);
  _ser.write(0x17);
  _ser.write(0x14);
  _ser.write(0x5A);
  _ser.write(0x32);
  _ser.write(0x0E);
  _ser.write(0x32);
  _ser.write(0x50);
  _ser.write(0x28);
  _ser.write(0x64);
  _ser.write(0x46);
  _ser.write(0x96);
  _ser.write(0x01);
  _ser.write(0x03);
  _ser.write(0x01);
  _ser.write(0x12);
  _ser.write((byte)0x0);
  _ser.write(0x32);
  _ser.write(0x14);
  _ser.write(0x5A);
  _ser.write(0x14);
  _ser.write(0x02);
  _ser.write(0x32);
  _ser.write((byte)0x0);
  _ser.write(0x81);
  _ser.flush();

  delay(5);
}

void ESC_init() {

  // send 32 bytes
  for (int i = 0; i < 2; i++) {
    _ser.write(0x01);
    _ser.write(0x20);
    _ser.write(0xC0);
    _ser.write(0xA8);
    _ser.write(0x01);
    _ser.write(0x01);
    _ser.write(0x02);
    _ser.write(0x46);
    _ser.write(0x17);
    _ser.write(0x14);
    _ser.write(0x5A);
    _ser.write(0x32);
    _ser.write(0x0E);
    _ser.write(0x32);
    _ser.write(0x50);
    _ser.write(0x28);
    _ser.write(0x64);
    _ser.write(0x46);
    _ser.write(0x96);
    _ser.write(0x01);
    _ser.write(0x03);
    _ser.write(0x01);
    _ser.write(0x12);
    _ser.write((byte)0x0);
    _ser.write(0x32);
    _ser.write(0x14);
    _ser.write(0x5A);
    _ser.write(0x14);
    _ser.write(0x02);
    _ser.write(0x32);
    _ser.write((byte)0x0);
    _ser.write(0x81);
    _ser.flush();
    delay(7);
  }

  delay(484);

}

void idle(){
  _ser.write(0x02);
  _ser.write(0x0B);
  _ser.write((byte)0x0);
  _ser.write((byte)0x0);
  _ser.write((byte)0x0);
  _ser.write((byte)0x0);
  _ser.write((byte)0x0);
  _ser.write((byte)0x0);
  _ser.write((byte)0x0);
  _ser.write((byte)0x0);
  _ser.write(0x0D);
  _ser.flush();
  delay(21);
}


void fullSpeed(){
  _ser.write(0x02);
  _ser.write(0x0B);
  _ser.write(0x33);
  _ser.write(0xEA);
  _ser.write(0xBE);
  _ser.write(0xEA);
  _ser.write(0xA7);
  _ser.write(0x81);
  _ser.write((byte)0x0);
  _ser.write((byte)0x0);
  _ser.write(0xEF);
  _ser.flush();
  delay(21);
}
