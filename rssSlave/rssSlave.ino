#include <XBee.h>
#include <SoftwareSerial.h>
XBee xbee = XBee();

Rx64Response rx64 = Rx64Response();
SoftwareSerial mySerial(10, 11);
void setup()
{
    //xbee.begin(9600);
    Serial.begin(9600);
    mySerial.begin(9600);
    xbee.setSerial(mySerial);
}

void loop()
{
  Serial.println("HI");
    xbee.readPacket(100);
    if (xbee.getResponse().isAvailable())
    {
        Serial.println("hello");
        if (xbee.getResponse().getApiId() == RX_64_RESPONSE)
        {
            Serial.println("bye");
            xbee.getResponse().getRx64Response(rx64);
            Serial.print( rx64.getRssi() );
        }
    }
}
