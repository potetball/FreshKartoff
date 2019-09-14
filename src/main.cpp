#include <main.h>
#include <pinconfig.h>

/*
 * Arbeidskrav 1
 * 
 *  Author: Erik Alvarez
 *  Description: Transmitter of potato sensordata using
 *    Long Range 866Mhz Band Lora TTGO 
 *  Based on:

  LoRa Duplex communication

  Sends a message every half second, and polls continually
  for new incoming messages. Implements a one-byte addressing scheme,
  with 0xFF as the broadcast address.

  Uses readString() from Stream class to read payload. The Stream class'
  timeout may affect other functuons, like the radio's callback. For an

  created 28 April 2017
  by Tom Igoe
*/

byte msgCount = 0;        // count of outgoing messages
byte localAddress = 0xEA; // address of this device
byte destination = 0xFF;  // destination to send to
long lastSendTime = 0;    // last send time
int interval = 5000;      // interval between sends

#define DEBUGS

#define LED 2

void setup()
{
  Serial.begin(9600); // initialize serial and wait for someone to read
#ifdef DEBUGS
  while (!Serial) //wait for debugger to attach so we can read the output
    ;
#endif

  init_lora();
  pinMode(LED, OUTPUT);
}
void init_lora()
{
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI0);

  if (!LoRa.begin(FREQ))
  { // initialize radio at 866 Mhz
    Serial.println("LoRa init failed. Check your connections.");
    while (true)
      ; // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");
  Serial.print("My address: 0x");
  Serial.println(localAddress, HEX);
}
void loop()
{

  // Make sure we don't spam the GW with signals
  // using a specified interval to keep the messages
  // apart, we'll use the last measured values as
  // payload.
  if (millis() - lastSendTime > interval)
  {
    String message = "20/12";
    digitalWrite(LED, HIGH);
    Serial.println("Sending sensordata: " + message);
    sendMessage(message);
    lastSendTime = millis();        // timestamp the message
    interval = random(2000) + 6000; // 2-3 seconds

    delay(500);
    digitalWrite(LED, LOW);
  }

  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
}

void sendMessage(String outgoing)
{
  LoRa.beginPacket();            // start packet
  LoRa.write(destination);       // add destination address
  LoRa.write(localAddress);      // add sender address
  LoRa.write(msgCount);          // add message ID
  LoRa.write(outgoing.length()); // add payload length
  LoRa.print(outgoing);          // add payload
  LoRa.endPacket();              // finish packet and send it
  msgCount++;                    // increment message ID
}

void onReceive(int packetSize)
{
  if (packetSize == 0)
    return; // if there's no packet, return

  // read packet header bytes:
  byte recipient = LoRa.read();      // recipient address
  byte sender = LoRa.read();         // sender address
  byte incomingMsgId = LoRa.read();  // incoming msg ID
  byte incomingLength = LoRa.read(); // incoming msg length

  String incoming = "";

  while (LoRa.available())
  {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length())
  { // check length for error
    Serial.println("error: message length does not match length");
    return; // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient == localAddress && sender == destination)
  {
    // if message is for this device, or broadcast, print details:
    Serial.println("Received from: 0x" + String(sender, HEX));
    Serial.println("Sent to: 0x" + String(recipient, HEX));
    Serial.println("Message ID: " + String(incomingMsgId));
    Serial.println("Message length: " + String(incomingLength));
    Serial.println("Message: " + incoming);
    Serial.println("RSSI: " + String(LoRa.packetRssi()));
    Serial.println("Snr: " + String(LoRa.packetSnr()));
    Serial.println();
  }
  else
  {
    Serial.print("Received from: 0x" + String(sender, HEX));
    Serial.print("RSSI: " + String(LoRa.packetRssi()));
    Serial.println(" - This message is not for me.");
    return; // skip rest of function
  }
}