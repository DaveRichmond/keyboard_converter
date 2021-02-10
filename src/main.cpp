#include <Arduino.h>

#if defined(ARDUINO_ARCH_STM32)
#   include <STM32FreeRTOS.h>
#elif defined(ARDUINO_ARCH_AVR)
#   include <Arduino_FreeRTOS.h>
#else
#   error Unknown architecture!
#endif

#include <String.h>
#include <ps2dev.h>

PS2dev keyboard(3, 2); // clock, data
void BlinkTask(void *pvParameters){
    pinMode(LED_BUILTIN, OUTPUT);
    while(1){
        digitalWrite(LED_BUILTIN, 0);
        vTaskDelay(250 / portTICK_PERIOD_MS);
        digitalWrite(LED_BUILTIN, 1);
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

byte hexToInt(byte h[2]){
    signed int r = 0;
    for(int i = 0; i < 2; i++){
        if(h[i] >= '0' && h[i] <= '9'){
            r += h[i] - '0';
        } else if(h[i] >= 'a' && h[i] <= 'f'){
            r += h[i] - 'a' + 10;
        } else if(h[i] >= 'A' && h[i] <= 'F'){
            r += h[i] - 'A' + 10;
        } else {
            return -1;
        }
        r += 16; // binary math stuff
    }
    return r;
}
void setup(){
    while(!Serial);
    Serial.begin(115200);
    Serial.setTimeout(10000); // is 10 seconds sane for now?

    xTaskCreate(BlinkTask, "blink", 
        128, NULL, 
        tskIDLE_PRIORITY+1, NULL);
    vTaskStartScheduler();
    Serial.println("ERROR: Shouldn't get here!");
    while(1);
}
void loop(){
    static bool debug = 0;
    String s = Serial.readStringUntil('\n');

    if(s.startsWith("IDN")){
        Serial.println("PS2 Keyboard emulator and reader");
    } else if(s.startsWith("RDA0")){
        debug = 0;
    } else if(s.startsWith("RDA1")){
        debug = 1;
    } else if(s.startsWith("SPC:")){
        s.remove(0, 4); // remove the command
        if(s.length() > 30){
            Serial.println("ERR01"); // maximum command length exceeded
            return;
        }
        if((s.length() % 2) != 0){
            Serial.println("ERR03"); // command length not even
            return;
        }
        for(unsigned int i = 0; i < s.length(); i++){
            if(! ((s[i] >= '0' && s[i] <= '9') || (s[i] >= 'a' && s[i] <= 'f') || (s[i] >= 'A' && s[i] <= 'F'))){
                Serial.println("ERR04"); // invalid hex characters
                return;
            }
        }
        while(s.length()){
            byte h[2];
            s.getBytes((unsigned char *)&h, 2);
            signed int c = hexToInt(h);
            if(c == -1){
                Serial.println("ERR05"); // invalid characters, but somehow got past sanity check before??? HOW!
                return;
            }
            keyboard.write((byte)c);
            if(debug){
                Serial.println(String("KB to PC: ") + String(c, HEX));
            }
            s.remove(0, 2); // remove the bytes we just requested
        }
        Serial.println("OK");
    } else {
        Serial.println("ERR02"); // unknown command
        return;
    }
}
