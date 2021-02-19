#include <Arduino.h>

// configuration
#if defined(ARDUINO_ARCH_STM32)
#   include <STM32FreeRTOS.h>
#   if defined(ARDUINO_NUCLEO_F411RE)
#       define PS2_CLK PB3
#       define PS2_DATA PB4
#       define LED_DATA PB0
#       define LED_CLK PC0
#       define LED_CS PC1
        auto BridgeSerial = Serial; 
#   elif defined(ARDUINO_BLACKPILL_F411CE)
#       define PS2_CLK PB9
#       define PS2_DATA PB8
        HardwareSerial BridgeSerial(PA10, PA9);
#   endif
    
#elif defined(ARDUINO_ARCH_AVR)
#   include <Arduino_FreeRTOS.h>
#   define PS2_CLK 3
#   define PS2_DATA 2
#   define LED_DATA 4
#   define LED_CLK 5
#   define LED_CS 6
    auto BridgeSerial = Serial;
#else
#   error Unknown architecture!
#endif
#include <queue.h>
#include <assert.h>

//#include <String.h>
#include <vector>
#include <ps2dev.h>
#include <LedController.hpp>

typedef enum {
    CMD_ERROR = 1,
    CMD_INFO,
    CMD_DEBUG,
    CMD_OK,
    CMD_KEYBOARD,
    CMD_LEDS
} cmd_t;
typedef struct {
    cmd_t cmd;
    char data;
} serial_msg_t;
void serialSend(cmd_t cmd, char data);
void keyboardSend(char c);
void debugSend(char c);

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

void serialService(){
    serialEventRun();
}
QueueHandle_t keyboardQueue;
void keyboardSend(char c){
    assert((xQueueSend(keyboardQueue, &c, portMAX_DELAY)) != pdFAIL);
}
int keyboard_write(unsigned char d){
    //taskENTER_CRITICAL();
    int r = keyboard.write(d);
    //taskEXIT_CRITICAL();
    return r;
}
int keyboard_handle(unsigned char *leds){
    //taskENTER_CRITICAL();
    int r = keyboard.keyboard_handle(leds);
    //taskEXIT_CRITICAL();
    return r;
}
void KeyboardTask(void *pvParameters){
    while(1){
        while(keyboard_write(0xA0) != 0){
            serialSend(CMD_KEYBOARD, 0);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        serialSend(CMD_KEYBOARD, 1);

        int idle = 0;
        const int timeout = 10000; // approximately 10 seconds
        do {

            unsigned char leds;
            if(keyboard_handle(&leds) != 0){
                serialSend(CMD_LEDS, leds);
            }
            char keyboardData;
            while(xQueueReceive(keyboardQueue, &keyboardData, 1) == pdPASS){
                serialSend(CMD_KEYBOARD, 2);
                keyboard_write(keyboardData);
                idle = 0;
            }
            idle++;
            vTaskDelay(1/portTICK_PERIOD_MS);
        } while(idle < timeout);
    }
}

QueueHandle_t serialQueue;
void serialSend(cmd_t command, char data){
    serial_msg_t message = {
        .cmd = command,
        .data = data
    };
    assert(xQueueSend(serialQueue, &message, portMAX_DELAY) != pdFAIL);
}
void SerialTXTask(void *pvParameters){
    while(1){
        serial_msg_t serialMessage;
        if(xQueueReceive(serialQueue, &serialMessage, 0) == pdPASS){
            switch(serialMessage.cmd){
                case CMD_ERROR:
                    {
                        String message((int)serialMessage.data);
                        int padding = (message.length() < 2) ? (2 - message.length()) : 0;
                        BridgeSerial.print("ERR");
                        for(int i = 0; i < padding; i++){
                            BridgeSerial.print("0");
                        }
                        BridgeSerial.println(message);
                    }
                    break;
                case CMD_INFO:
                    if(serialMessage.data == 0){
                        BridgeSerial.println("PS2 Keyboard emulator and reader");
                    } else {
                        BridgeSerial.println("PS2 Keyboard Emulator - DEBUG MODE");
                    }
                    break;
                case CMD_LEDS:
                    BridgeSerial.println("LEDS" + String((int)serialMessage.data, HEX));
                    break;
                case CMD_DEBUG:
                    BridgeSerial.println("KB to PC: " + String((int)serialMessage.data, HEX));
                    break;
                case CMD_OK:
                    //BridgeSerial.println("OK" + String((int)serialMessage.data, DEC));
                    BridgeSerial.println("OK");
                    break;
                case CMD_KEYBOARD:
                    /*
                    if(serialMessage.data == 0){
                        BridgeSerial.println("Keyboard disconnected, attempting connect!");
                    } else if(serialMessage.data == 1){
                        BridgeSerial.println("Keyboard connected!");
                    } else if(serialMessage.data == 2){
                        BridgeSerial.println("Sending keycode");
                    }
                    // */
                    break;
                default:
                    BridgeSerial.println("Unknown Command: " + String((int)serialMessage.cmd));
                    break;
            }
            BridgeSerial.flush();
        }
        serialService();
        vTaskDelay(1);

        //if(xTaskGetTickCount() > next){
        //    char buf[200];
        //    vTaskGetRunTimeStats((char *)&buf);
        //    BridgeSerial.println(buf);
        //    next = next + (1000 / portTICK_PERIOD_MS);
        //}
    }
}

int hexToInt(char c){
    if(c >= '0' && c <= '9'){
        return c - '0';
    }
    if(c >= 'A' && c <= 'F'){
        return 10 + (c - 'A');
    }
    if(c >= 'a' && c <= 'f'){
        return 10 + (c - 'a');
    }
    return 0;
}
int keyboardHexParse(String &s, bool debug){
        if((s.length() % 2) != 0){
            serialSend(CMD_ERROR, 3); // uneven number of arguments
            return 0;
        }
        if(s.length() > 30){
            serialSend(CMD_ERROR, 1); // too many keys, the input loop checks this too so we probably shouldn't hit this one. Doesn't hurt to check again
            return 0;
        }   
        for(signed int i = 0; i < s.length(); i++){
            if(!isHexadecimalDigit(s[0])){
                serialSend(CMD_ERROR, 4); // invalid character
                return 0;
            }
        }
        
        // now we've sanity checked our input, send it through the queue
        int sent = 0;
        for(signed int i = 0; i < s.length(); i += 2){
            char a = hexToInt(s[i]);
            char b = hexToInt(s[i+1]);
            char c = (a << 4) | b;
            //BridgeSerial.println("a:" + String((int)a, HEX) + " b:" + String((int)b, HEX) + " c:" + String((int)c, HEX));
            if(debug){
                serialSend(CMD_DEBUG, c);
            }
            debugSend(c);
            keyboardSend(c);
            sent++;
        }
        return sent;
}
String readLine(std::vector<char> terminators){
    String line;
    char c = (char)NULL;
    int length = 0;

    bool terminate = false;
    do {
        while(BridgeSerial.available() && !terminate){
            c = BridgeSerial.read();
            //BridgeSerial.println("Received: " + String((int)c, HEX)); // for when dealing with a terminal program that doesn't work as expected
            line += String((char)c);
            length++;
            if(length >= 34){
                serialSend(CMD_ERROR, 1);
                return String("NULL");
            }

            for(char terminator : terminators){
                if(c == terminator){
                    terminate = true;
                }
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    } while(!terminate);
    return line;
}
void SerialRXTask(void *pvParameters){
    bool debug = false;
    while(1){
        //String line = BridgeSerial.readStringUntil('\n'); // this is very CPU heavy and needs some vTaskDelays to not just sit there chewing the whole cpu up with nothing happening
        String line = readLine({'\r', '\n'});
        line.trim(); // remove excess whitespace
        if(line.length() > 0){
            //BridgeSerial.println(String("Line: ") + line);
            if(line.startsWith("NULL")){
                /// do nothing, the reader encountered an issue so we'll just continue on
            } else if(line.startsWith("IDN")){
                serialSend(CMD_INFO, (int)debug);
            } else if(line.startsWith("RDA0")){
                debug = false;
            } else if(line.startsWith("RDA1")){
                debug = true;
            } else if(line.startsWith("SPC:")){
                line = line.substring(line.indexOf(":")+1); // give me everything after the ":"
                int sent = keyboardHexParse(line, debug);
                serialSend(CMD_OK, sent);
            } else {
                serialSend(CMD_ERROR, 2); // unknown command!
            }
        }
    }
}

// For bringing up the project, and to ensure the basic serial IO is working
// we'll make a task that periodically fires off an LED update
void DemoTask(void *pvParameters){
    int i = 0;
    while(1){
        i = (i + 1) % 8;
        serialSend(CMD_LEDS, i);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

QueueHandle_t debugQueue = NULL; 
void debugSend(char msg){
    xQueueSend(debugQueue, (void *)&msg, 0); // don't care if this fails
}
char toHex(int i){
    String s = String(i, HEX);
    return s.c_str()[0];
}

#ifdef USE_DEBUG_LED
void DebugTask(void *pvParameters){
    LedController led(
        LED_DATA, // Data
        LED_CLK, // Clk
        LED_CS, // Load/CS
        1 // Number of MAX7219 Devices
    );
    led.activateAllSegments();
    led.setIntensity(8);
    led.clearMatrix();
    while(1){
        int msg;
        if(xQueueReceive(debugQueue, &msg, portMAX_DELAY) == pdPASS){            
            led.setChar(0, 1, toHex((msg >> 4) & 0x0F), false); // indexed to the right of the display
            led.setChar(0, 0, toHex(msg & 0x0F), false);
        }
    }
}
#else
void DebugTask(void *pvParameters){
    vTaskSuspend(NULL); // just stop this task from running
}
#endif

// This never worked on STM32FreeRTOS, needs some extra config and I'm too lazy for that
void StatsTask(void *pvParameters){
    char buf[200];
    while(1){
        //vTaskList((char *)&buf);
        //Serial.println(buf);
        vTaskDelay(1000 * portTICK_PERIOD_MS);
    }
}

void setup(){
    while(!BridgeSerial);
    BridgeSerial.begin(115200);
    //BridgeSerial.setTimeout(10000); // is 10 seconds sane for now?
    BridgeSerial.println("Initialising");

    pinMode(PS2_CLK, INPUT_PULLUP);
    pinMode(PS2_DATA, INPUT_PULLUP);

    assert((keyboardQueue = xQueueCreate(32, sizeof(char))) != NULL);
    assert((serialQueue = xQueueCreate(10, sizeof(serial_msg_t))) != NULL);
    assert((debugQueue  = xQueueCreate(16, sizeof(char))) != NULL);
    xTaskCreate(BlinkTask, "blink", 
        128, NULL, 
        tskIDLE_PRIORITY+1, NULL);
    xTaskCreate(SerialTXTask, "serial_tx",
        1024, NULL,
        tskIDLE_PRIORITY+2, NULL);
    xTaskCreate(SerialRXTask, "serial_rx",
        1024, NULL,
        tskIDLE_PRIORITY+1, NULL);
    xTaskCreate(KeyboardTask, "keyboard",
        1024, NULL,
        tskIDLE_PRIORITY+2, NULL);
    xTaskCreate(DebugTask, "debug_output", 
        1024, NULL,
        tskIDLE_PRIORITY+1, NULL);
    //xTaskCreate(DemoTask, "demo",
    //    256, NULL,
    //    tskIDLE_PRIORITY+1, NULL);
    vTaskStartScheduler();
    BridgeSerial.println("ERROR: Shouldn't get here!");
    assert(false);
}
void loop(){
    static int idle = 0;
    // Shouldn't get here!
    //assert(false);
    idle++;
}

// not sure if this works in arduino
void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp){
    Serial.println("File: " + String(__file) + "+" + String(__lineno) + " func(" + String(__func)  + "): " + String(__sexp));
    taskENTER_CRITICAL();
    while(1); // should stop here
    taskEXIT_CRITICAL();
}