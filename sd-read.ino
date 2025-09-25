#include <SPI.h>

// #pragma pack(1)

// #define DEBUG

#define SD_CS   26
#define SD_MOSI 14
#define SD_SCLK 27
#define SD_MISO 12

#ifndef CONFIG_IDF_TARGET_ESP32
  #define VSPI FSPI
#endif

#define BLOCK_SIZE 512

#define READ_BLOCK  17
#define WRITE_BLOCK 24

SPIClass *vspi = NULL;
SPISettings spiset(250000, MSBFIRST, SPI_MODE0);

struct dataPoint{
  uint32_t valueX;
  uint32_t valueY;

  uint8_t theRest[56];
};

void setup(){
  Serial.begin(115200);

  while(!Serial);

  start:

  Serial.println("waiting for input");
  while(!Serial.available()){}
  Serial.read();

  vspi = new SPIClass(VSPI);
  vspi -> begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  pinMode(vspi -> pinSS(), OUTPUT);
  digitalWrite(vspi -> pinSS(), HIGH);
  #ifdef DEBUG
    Serial.println("spi inited");
  #endif
  
  if(!cardInit()){
  #ifdef DEBUG
    Serial.println("failed to init");
  #endif
    vspi -> end();
    goto start;
  }

  uint32_t initial = 0;
  if(!writeBlock(0, (uint8_t*)&initial, 4)){
  #ifdef DEBUG
    Serial.println("failed to reset first block");
  #endif
    vspi -> end();
    goto start;
  }

  dataPoint firstData {};

  for(uint32_t ang = 0; ang < 10; ang += 1){
    Serial.print("Writing 2x 5y: " + String(ang));
    Serial.print("\t" + String(ang * 2));
    Serial.println("\t" + String(ang * 5));

    firstData.valueX = ang * 2;
    firstData.valueY = ang * 5;

    if(!saveData(&firstData)){
    #ifdef DEBUG
        Serial.println("failed to save data");
    #endif
      vspi -> end();
      goto start;
    }
  }
  Serial.println("Have written successfully");

  uint8_t buf[8];
  for(int ang = 2; ang <= 11; ang++){
    if (!readBlock(ang, (uint8_t*)&buf, 8)){
      #ifdef DEBUG
          Serial.println("failed to read data");
      #endif
      vspi -> end();
      goto start;
    };

    memcpy(&firstData.valueX, &buf,sizeof(uint32_t));
    memcpy(&firstData.valueY, &buf[4],sizeof(uint32_t));

    // firstData.valueX = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0]
    // firstData.valueY = (buf[7] << 24) | (buf[6] << 16) | (buf[5] << 8) | buf[4]


    //firstData.valueX = *((double*) buf[0]);
    //firstData.valueY = *((double*) buf[4]);

    Serial.print("data:\t");
    Serial.print(firstData.valueX);
    Serial.print("\t");
    Serial.println(firstData.valueY);
  }
  Serial.println("Have read successfully");

}

void sendCMD(uint8_t cmd, uint32_t arg, uint8_t crc){
  #ifdef DEBUG
    Serial.print("sending cmd\t");
    Serial.print(cmd);
    Serial.print("\t");
    Serial.print(arg >> 24 & 0xFF, HEX);
    Serial.print(arg >> 16 & 0xFF, HEX);
    Serial.print(arg >> 8  & 0xFF, HEX);
    Serial.print(arg & 0xFF, HEX);
    Serial.println();
  #endif
  waitCard();

  vspi -> beginTransaction(spiset);

  vspi -> transfer(0x40 | cmd);

  vspi -> transfer(arg >> 24);
  vspi -> transfer(arg >> 16);
  vspi -> transfer(arg >> 8);
  vspi -> transfer(arg);

  vspi -> transfer(crc);

  vspi -> endTransaction();
}

uint8_t readCard(){
  vspi -> beginTransaction(spiset);

  for (uint8_t i = 0; i < 10; i++){
    uint8_t resp = vspi -> transfer(0xFF);
    if(resp != 0xFF){
      vspi -> endTransaction();
      return resp;
    }
  }
  vspi -> endTransaction();
  return 0xFF;
}

bool waitCard(){
  auto start = millis();

  while(1){
    auto r = vspi ->transfer(0xFF);

    if(r == 0xFF){ return true; }
    if (millis() - start > 300){ return false; }
  }
}

bool cardInit(){
  delay(300);

  digitalWrite(vspi -> pinSS(), HIGH);

  vspi -> beginTransaction(spiset);
  for(uint8_t i = 0; i < 10; i++){
    vspi -> transfer(0xFF);
  }
  vspi -> endTransaction();

  digitalWrite(vspi -> pinSS(), LOW);
  sendCMD(0, 0, 0x95);
  if(readCard() != 0x01){
    #ifdef DEBUG
      Serial.println("failed to idle");
    #endif
    digitalWrite(vspi -> pinSS(), HIGH);
    return false;
  }

  sendCMD(8, 0x01AA, 0x87);
  if(readCard() != 0x01){
    #ifdef DEBUG
      Serial.println("failed to confirm version");
    #endif
    digitalWrite(vspi -> pinSS(), HIGH);
    return false;
  }

  uint8_t tries = 10;
  for(uint8_t i = 0; i < tries; i++){
    #ifdef DEBUG
      Serial.print("attempt to init n"); Serial.println(i);
    #endif
    sendCMD(55, 0, 0x01);
    if(readCard() != 0x01){
    #ifdef DEBUG
        Serial.println("failed to ack");
    #endif
      digitalWrite(vspi -> pinSS(), HIGH);
      return false;
    }

    sendCMD(41, 0x40000000, 0x01);

    if(readCard() == 0x00){ break; }
  }

  digitalWrite(vspi -> pinSS(), HIGH);
  vspi -> transfer(0xFF);
  return true;
}

bool readBlock(uint32_t adr, uint8_t* buf, uint16_t buf_size){
    #ifdef DEBUG
    Serial.print("reading block\t");
    Serial.print(String(adr) + "\t");
    for(uint16_t i = 0; i < buf_size; i++)
      Serial.print(((uint8_t*)buf)[i], HEX);
    Serial.print("\t");
    Serial.println(buf_size);
  #endif
  digitalWrite(vspi -> pinSS(), LOW);

  sendCMD(17, adr, 0x01);
  if(readCard() != 0x00){
    #ifdef DEBUG
      Serial.println("failed to send read cmd");
    #endif
    digitalWrite(vspi -> pinSS(), HIGH);
    return false;
  }

  vspi -> beginTransaction(spiset);

  while(vspi -> transfer(0xFF) != 0xFE);

  for(uint16_t i = 0; i < BLOCK_SIZE; i++){
    uint8_t r = vspi -> transfer(0xFF);

    if(i < buf_size){ buf[i] = r; }
  }

  vspi -> transfer(0xFF);
  vspi -> transfer(0xFF);
  vspi -> endTransaction();
  digitalWrite(vspi -> pinSS(), HIGH);
  vspi -> transfer(0xFF);

  return true;
}

bool writeBlock(uint32_t adr, const void* buf, uint16_t buf_size){
  #ifdef DEBUG
    Serial.print("writing block\t");
    Serial.print(String(adr) + "\t");
    for(uint16_t i = 0; i < buf_size; i++)
      Serial.print(((uint8_t*)buf)[i], HEX);
    Serial.print("\t");
    Serial.println(buf_size);
  #endif
  digitalWrite(vspi -> pinSS(), LOW);

  sendCMD(24, adr, 0x01);
  if(readCard() != 0x00){
    #ifdef DEBUG
      Serial.println("failed to send cmd");
    #endif
    digitalWrite(vspi -> pinSS(), HIGH);
    return false;
  }

  vspi -> beginTransaction(spiset);
  vspi -> transfer(0xFE);
  for(uint16_t i = 0; i < BLOCK_SIZE; i++){
    if (i < buf_size){ vspi -> transfer(((uint8_t*)buf)[i]); }
    else {vspi -> transfer(0x00);}
  }

  vspi -> transfer(0xFF);
  vspi -> transfer(0xFF);

  uint8_t resp = vspi -> transfer(0xFF);

  if((resp & 0x1F) != 0x05){
    vspi -> endTransaction();
    digitalWrite(vspi -> pinSS(), HIGH);
    #ifdef DEBUG
      Serial.println("data wasn't accepted");
    #endif
    return false;
  }

  while(vspi -> transfer(0xFF) == 0x00);
  vspi -> endTransaction();
  digitalWrite(vspi -> pinSS(), HIGH);
  vspi -> transfer(0xFF);

  return true;
}

bool saveData(const dataPoint* data){
  #ifdef DEBUG
    Serial.println("saving");
  #endif

  uint32_t count = 0;
  if(!readBlock(0,(uint8_t*)&count, 4)){ 
    #ifdef DEBUG
      Serial.println("failed to read first block");
    #endif
    return false; 
  }

  count++;
  if(!writeBlock(0,(uint8_t*)&count, 4)){ 
    #ifdef DEBUG
      Serial.println("failed to write first block");
    #endif
    return false;
  }
  if(!writeBlock(count+1, data, sizeof(dataPoint))) {
    #ifdef DEBUG
      Serial.println("failed to write block");
    #endif
    return false; 
  }
  return true;
}

void loop(){

}
