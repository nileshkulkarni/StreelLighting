#include <XBee.h>
#include <SoftwareSerial.h>
XBee xbee = XBee();

uint8_t payload[] = { 'H', 'i' };


SoftwareSerial mySerial(10, 11);
XBeeAddress64 addr64 = XBeeAddress64(0x0, 0xFFFF);
//Tx16Request tx64 = Tx16Request(0xFFFF, payload, sizeof(payload));
Tx64Request tx64 = Tx64Request(addr64, payload, sizeof(payload));
TxStatusResponse txStatus = TxStatusResponse();
void setup()
{
   // xbee.begin(9600);
    mySerial.begin(9600);
    xbee.setSerial(mySerial);
}

void loop()
{
    xbee.send(tx64);
    delay(50);
}
