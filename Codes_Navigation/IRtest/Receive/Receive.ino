#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

// Creating datapack structure containing LDR values
struct datapack{
  int IR3;
  int IR4;
  int IR5;
};

// Creating object of datapack type
datapack data;

 RF24 radio(6,A0);//CE,SCN
//RF24 radio(7,8);//CE,SCN
const byte address[] = "node1";

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0,address);
  radio.startListening();
}

void loop() {

  while(radio.available()){
    radio.read(&data, sizeof(data));
    Serial.print(data.IR3);
    Serial.print("\t");
    Serial.print(data.IR4);
    Serial.print("\t");
    Serial.println(data.IR5);
  }
}
