#include <Arduino.h>


#define BOARD_LED_PIN  PA5


char input;

void setup() {
    pinMode(BOARD_LED_PIN, OUTPUT);
    //Serial.begin(9600); 
    Serial.begin(115200);
    Serial1.begin(9600);
    delay(2000);  
 
    Serial.println("Type something!");

}

void loop() {
       if(Serial.available()){
        input = Serial.read();
        Serial.print(": " );
        Serial.println(input);
       }
    Serial1.println("S");
    Serial.println("H");
    digitalWrite(BOARD_LED_PIN, !digitalRead(BOARD_LED_PIN));    
    delay(500);
}



#if 0
__attribute__((constructor)) void premain() {
    init();
}


int main(void) {
    setup();

    while (true) {
        loop();
    }
    return 0;
}
#endif