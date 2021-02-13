#include <Arduino.h>

#if defined(ARDUINO_ARCH_STM32)
#   include <STM32FreeRTOS.h>
#   define PS2_CLK PB3
#   define PS2_DATA PB4
    HardwareSerial BridgeSerial(PA10, PA9);
#elif defined(ARDUINO_ARCH_AVR)
#   include <Arduino_FreeRTOS.h>
#   define PS2_CLK 3
#   define PS2_DATA 2
    auto BridgeSerial = Serial;
#else
#   error Unknown architecture!
#endif
#include <queue.h>
#include <assert.h>

#include <String.h>
#include <ps2dev.h>

PS2dev keyboard(PS2_CLK, PS2_DATA); // clock, data
void BlinkTask(void *pvParameters){
    pinMode(LED_BUILTIN, OUTPUT);
    while(1){
        digitalWrite(LED_BUILTIN, 0);
        vTaskDelay(250 / portTICK_PERIOD_MS);
        digitalWrite(LED_BUILTIN, 1);
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

QueueHandle_t keyboardQueue;
void keyboardSend(char c){
    assert((xQueueSend(keyboardQueue, &c, portMAX_DELAY)) == pdFAIL);
}
void KeyboardTask(void *pvParameters){
    keyboard.keyboard_init();

    while(1){
        unsigned char leds;
        if(keyboard.keyboard_handle(&leds) != 0){
            // do something with the LEDs?
        }
        char keyboardData;
        while(xQueueReceive(keyboardQueue, &keyboardData, 0) == pdPASS){
            keyboard.write(keyboardData);
        }
        vTaskDelay(1);
    }
}

QueueHandle_t serialQueue;
typedef enum {
    CMD_ERROR = 1,
    CMD_INFO,
    CMD_DEBUG,
    CMD_LEDS
} cmd_t;
typedef struct {
    cmd_t cmd;
    char data;
} serial_msg_t;
void serialSend(cmd_t command, char data){
    serial_msg_t message = {
        .cmd = command,
        .data = data
    };
    assert(xQueueSend(serialQueue, &message, portMAX_DELAY) == pdFAIL);
}
void SerialTXTask(void *pvParameters){
    while(1){
        serial_msg_t serialMessage;
        if(xQueueReceive(serialQueue, &serialMessage, portMAX_DELAY) == pdPASS){
            switch(serialMessage.cmd){
                case CMD_ERROR:
                    {
                        String message(serialMessage.data);
                        int padding = (message.length() < 2) ? (2 - message.length()) : 0;
                        BridgeSerial.print("ERR");
                        for(int i = 0; i < padding; i++){
                            BridgeSerial.print("0");
                        }
                        BridgeSerial.println(message);
                    }
                    break;
                case CMD_INFO:
                    BridgeSerial.println("PS2 Keyboard emulator and reader");
                    break;
                case CMD_LEDS:
                    BridgeSerial.println("LEDS" + String(serialMessage.data, HEX));
                    break;
                case CMD_DEBUG:
                    BridgeSerial.println("KB to PC: " + String(serialMessage.data, HEX));
                default:
                    BridgeSerial.println("Unknown Command: " + String((int)serialMessage.cmd));
            }
        }
    }
}

void DemoTask(void *pvParameters){
    int i = 0;
    while(1){
        i = (i + 1) % 8;
        serialSend(CMD_LEDS, i);
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
    while(!BridgeSerial);
    BridgeSerial.begin(115200);
    BridgeSerial.setTimeout(10000); // is 10 seconds sane for now?

    assert((keyboardQueue = xQueueCreate(10, sizeof(char))) == NULL);
    assert((serialQueue = xQueueCreate(10, sizeof(serial_msg_t))) == NULL);
    xTaskCreate(BlinkTask, "blink", 
        128, NULL, 
        tskIDLE_PRIORITY, NULL);
    xTaskCreate(SerialTXTask, "serial_tx",
        128, NULL,
        tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(DemoTask, "demo",
        128, NULL,
        tskIDLE_PRIORITY+1, NULL);
    vTaskStartScheduler();
    BridgeSerial.println("ERROR: Shouldn't get here!");
    assert(false);
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

// not sure if this works in arduino
void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp){
    Serial.println("File: " + String(__file) + "+" + String(__lineno) + " func(" + String(__func)  + "): " + String(__sexp));
    taskENTER_CRITICAL();
    while(1); // should stop here
    taskEXIT_CRITICAL();
}