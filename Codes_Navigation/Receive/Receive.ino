#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

// Creating datapack structure containing LDR values
struct datapack{
  int Count;
  unsigned long int t;
  unsigned long int Tr;
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
    Serial.print(data.Count);
    Serial.print("\t");
    Serial.print(data.Tr);
    Serial.print("\t");
    Serial.println(data.t);
  }
}
