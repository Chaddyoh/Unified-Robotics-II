/**
 * 
 * initializes everything and 
 * calls the state machine function in the loop.
 * 
 **/

#include <Arduino.h>
#include <robot.h>
#include <Chassis.h>
#include <ir_codes.h>
#include <IRdecoder.h>
#define IR_PIN 14
IRDecoder decoder(IR_PIN);

void setup() // initializer
{
    Serial.begin(115200);
    delay(500);

    Serial.println("setup()");

    decoder.init();
    chassis.init();

    initialize();

  Serial.println("/setup()");
}

void loop() // looper
{
    chassis.loop();

    int16_t keyCode = decoder.getKeyCode();
    if(keyCode != -1) handleKeyCode(keyCode);

    pallas(); // state machine

}
