#include <Arduino.h>
#include <robot.h>
#include <Chassis.h>
#include <ir_codes.h>
#include <IRdecoder.h>

#define IR_PIN 14
IRDecoder decoder(IR_PIN);

void setup() 
{
    Serial.begin(115200);
    delay(500);

    decoder.init();
    chassis.init();
    initialize();
}

void loop() 
{
    chassis.loop();

    int16_t keyCode = decoder.getKeyCode();
    if(keyCode != -1) handleKeyCode(keyCode);

    // our state machine
    wallie();

}
