/*Created 26 Sept. 2005 by Tom Igoe
 modified 24 Apr 2012 by Tom Igoe and Scott Fitzgerald
 
 modified 24 Feb 2015 by Alejandro Daniel Noel
 
 This example code is in the public domain.
 http://www.arduino.cc/en/Tutorial/SerialCallResponseASCII
 */

String inString = "";  // incoming serial string

void setup()
{
  // start serial port at 9600 bps and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  establishContact();  // send a byte to establish contact until receiver responds 
}

void loop()
{
  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) {
    // get incoming byte:
    inString = Serial.readString();
    String outString = "Hi computer, I'm Arduino!, your message was: " + inString;
    Serial.println(outString.c_str());
  }
}

void establishContact() {
  while (Serial.available() <= 0) {
    Serial.println("Someone there?");   // send an initial string
    delay(300);
  }
}
