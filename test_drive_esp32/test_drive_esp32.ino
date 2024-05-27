
#define RX_PIN 12
#define TX_PIN 13

void setup() {

  //pinMode(RX_PIN, INPUT);
  //pinMode(TX_PIN, OUTPUT);

//  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  //Serial.println("start waitFourZeros");
  // waitFourZeros();
  delay(370); //406
  //Serial.println("start handshake");
  ESC_Handshake();
  //Serial.println("start esc init");
  ESC_init();

  //Serial.println("finished init");

  for(int i=0; i<10; i++){
    //idle2();
    idle();
  }

  
}

void loop() {


  //fullSpeed();
  //fullLeftSpeed();
  sendBytesSpeed();

  
}

void waitFourZeros() {

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

  Serial1.write(0xFF);
  for (int i = 0; i < 20; i++) {
    Serial1.write((byte)0x0);
  }
  Serial1.write(0x01);
  Serial1.write(0x20);
  Serial1.write(0xC0);
  Serial1.write(0xA8);
  Serial1.write(0x01);
  Serial1.write(0x01);
  Serial1.write(0x02);
  Serial1.write(0x46);
  Serial1.write(0x17);
  Serial1.write(0x14);
  Serial1.write(0x5A);
  Serial1.write(0x32);
  Serial1.write(0x0E);
  Serial1.write(0x32);
  Serial1.write(0x50);
  Serial1.write(0x28);
  Serial1.write(0x64);
  Serial1.write(0x46);
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
  Serial1.write(0x81);
  Serial1.flush();

  delay(5);
}

void ESC_init() {

  // send 32 bytes
  for (int i = 0; i < 2; i++) {
    Serial1.write(0x01);
    Serial1.write(0x20);
    Serial1.write(0xC0);
    Serial1.write(0xA8);
    Serial1.write(0x01);
    Serial1.write(0x01);
    Serial1.write(0x02);
    Serial1.write(0x46);
    Serial1.write(0x17);
    Serial1.write(0x14);
    Serial1.write(0x5A);
    Serial1.write(0x32);
    Serial1.write(0x0E);
    Serial1.write(0x32);
    Serial1.write(0x50);
    Serial1.write(0x28);
    Serial1.write(0x64);
    Serial1.write(0x46);
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
    Serial1.write(0x81);
    Serial1.flush();
    delay(7);
  }

  delay(486);


}

void idle(){
  Serial1.write(0x02);
  Serial1.write(0x0B);
  Serial1.write((byte)0x0);
  Serial1.write((byte)0x0);
  Serial1.write((byte)0x0);
  Serial1.write((byte)0x0);
  Serial1.write((byte)0x0);
  Serial1.write((byte)0x0);
  Serial1.write((byte)0x0);
  Serial1.write((byte)0x0);
  Serial1.write(0x0D);
  Serial1.flush();
  //delay(21);
  delayMicroseconds(21000);
}

void idle2(){
  Serial1.write(0x02);
  Serial1.write(0x0B);
  Serial1.write((byte)0x0);
  Serial1.write((byte)0x0);
  Serial1.write((byte)0x0);
  Serial1.write((byte)0x0);
  Serial1.write((byte)0x9C);
  Serial1.write((byte)0x0);
  Serial1.write((byte)0x0);
  Serial1.write((byte)0x0);
  Serial1.write(0xA9);
  Serial1.flush();
  //delay(21);
  delayMicroseconds(21000);
}


void fullSpeed(){
  Serial1.write(0x02);
  Serial1.write(0x0B);
  Serial1.write(0x33);
  Serial1.write(0xEA);
  Serial1.write(0xB3);
  Serial1.write(0xEA);
  Serial1.write(0xB1);
  Serial1.write(0x81);
  Serial1.write((byte)0x0);
  Serial1.write((byte)0x0);
  Serial1.write(0xF9);
  Serial1.flush();
  delayMicroseconds(21000);
}

void fullLeftSpeed(){
  Serial1.write(0x02);
  Serial1.write(0x0B);
  
  Serial1.write((byte)0x0);
  
  Serial1.write(0xEA);
  Serial1.write(0xBE);
  Serial1.write((byte)0x0);
  Serial1.write((byte)0x0);
  
  Serial1.write((byte)0x0);
  
  Serial1.write((byte)0x0);
  Serial1.write((byte)0x0);
  
  Serial1.write(0xB5);
  Serial1.flush();
  delayMicroseconds(21000);
}

void sendBytesSpeed(){
  byte header1 = 0x02;
  byte header2 = 0x0B;

  // right speed
  byte byte3 = 0x90;
  byte byte4 = (byte)0x0;

  // left speed
  byte byte5 = 0x90; //(byte)0x0;
  byte byte6 = 0xFF;  //EA

  
  byte byte7 = 0x0;  //BE

  byte byte8 = 0x0;

  byte CheckSum = header1 + header2 + byte3 + byte4 + byte5 + byte6 + byte7 + byte8;
  
  Serial1.write(header1);
  Serial1.write(header2);
  
  Serial1.write(byte3);
  Serial1.write(byte4);
  Serial1.write(byte5);
  Serial1.write(byte6);
  
  Serial1.write(byte7);
 
  Serial1.write(byte8);
  
  Serial1.write((byte)0x0);
  Serial1.write((byte)0x0);
  Serial1.write(CheckSum);
  Serial1.flush();
  delayMicroseconds(21000);
}
