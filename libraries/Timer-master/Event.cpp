/*
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2 of the License, or
 *      (at your option) any later version.
 *
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with this program; if not, write to the Free Software
 *      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *      MA 02110-1301, USA.
 */

/*  * * * * * * * * * * * * * * * * * * * * * * * * * * *
 Code by Simon Monk
 http://www.simonmonk.org
* * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// For Arduino 1.0 and earlier
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Event.h"

Event::Event(void)
{
	eventType = EVENT_NONE;
}

void Event::update(void)
{
    unsigned long now = millis();
    update(now);
}

void Event::update(unsigned long now)
{
	/*
	if(repeatCount==1){
		Serial.println("Before executing IF");
		Serial.print("Now : ");
		Serial.println((long)now);
		Serial.print("LastEventTime : ");
		Serial.println((long)lastEventTime);
	}
	*/
	if (now - lastEventTime >= period)
	{
	/*	
		if(repeatCount==1){
			Serial.print("Event time ");
			Serial.println(now);
			Serial.print("Last Event time ");
			Serial.println(lastEventTime);
			Serial.print("Period ");
			Serial.println(period);

		}
	*/
		switch (eventType)
		{
			case EVENT_EVERY:
				(*callback)();
				break;

			case EVENT_OSCILLATE:
				pinState = ! pinState;
				digitalWrite(pin, pinState);
				break;
		}
		lastEventTime = now;
		count++;
	}
	if (repeatCount > -1 && count >= repeatCount)
	{
		eventType = EVENT_NONE;
	}
}
