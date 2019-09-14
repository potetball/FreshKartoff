#include <Arduino.h>
#include <SPI.h>              // include libraries
#include <LoRa.h>

void sendMessage(String message);
void onReceive(int packetSize);
void init_lora();
