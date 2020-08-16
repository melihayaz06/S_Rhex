#include <SoftwareSerial.h>
SoftwareSerial mySerial(4, 5); // RX, TX

char gelen;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
    while (!Serial) {
  }
  Serial.println("Xbee'den Arduino'ya baglanti kuruldu");
  mySerial.begin(9600);
  mySerial.println("Arduino'dan Xbee'ye baglanti kuruldu");

}

void loop() {
  if (mySerial.available()) {
    
    gelen = mySerial.read();
    Serial.write(gelen);
  }
  if (Serial.available()) {
    mySerial.write(Serial.read());
  }
}
