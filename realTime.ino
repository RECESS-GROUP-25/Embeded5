#include <Wire.h>
#include <DS3231.h>
#include <IRremote.hpp>

DS3231 clock;
RTCDateTime dt;

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//for the remote reciever
int IR_RECEIVE_PIN = 2;

int ldr = A0;
int ldr_value;
int motion_sensor = 4;
int motion_value = 0;
int led1 = 9;//pin for led 1
int led2 = 10;//pin for led 2
int led3 = 6;//pin for led 3
int led1_state = LOW;
int led2_state = LOW;
int led3_state = LOW;
bool night = false;
bool auto_intensity = false;
bool motion_detected = false;
bool motion_activated = false;
bool system_activated = true;
int brightness;
String rain_falling = "";


// lowest and highest sensor readings rain sensor:
const int sensorMin = 0;     // sensor minimum
const int sensorMax = 1024;  // sensor maximum
int sensorReading;
int rainThreshold;

void setup() {
  Serial.begin(9600);
  pinMode(ldr, INPUT);
  pinMode(motion_sensor, INPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  //start the reciever
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  // Initialize DS3231
  Serial.println("Initialize DS3231");;
  clock.begin();

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Set sketch compiling time
  clock.setDateTime(__DATE__, __TIME__);
}

void loop() {
  oled();
  myRemote();
  ldr_reading();
  rainSensor();
  motion();
  if (system_activated == true) {
    if (rain_falling != "Flood") {
      if (motion_activated == false) {
        lighting();
      } else if (motion_detected == true && auto_intensity == true) {
        brightness = map(ldr_value, 0, 990, 255, 0);
        analogWrite(led1, brightness);
        analogWrite(led2, brightness);
        analogWrite(led3, brightness);
      } else if (motion_detected == true && auto_intensity == false) {
        digitalWrite(led1, motion_value);
        digitalWrite(led2, motion_value);
        digitalWrite(led3, motion_value);
      } else {
        digitalWrite(led1, LOW);
        digitalWrite(led2, LOW);
        digitalWrite(led3, LOW);
      }
    } else {
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
      digitalWrite(led3, LOW);
    }
  } else {
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
  }
}
void lighting() {
  //if its night, the lights should be on else use remote to deactivate if need be
  if (night == true && led1_state == LOW && auto_intensity == false) {
    digitalWrite(led1, HIGH);
  } else if (night == true && led1_state == LOW && auto_intensity == true) {
    brightness = map(ldr_value, 0, 990, 255, 0);
    analogWrite(led1, brightness);
  } else if (night == false && led1_state == HIGH && auto_intensity == true) {//if day time, turn on lights with remote
    brightness = map(ldr_value, 0, 990, 255, 0);
    analogWrite(led1, brightness);
  } else if (night == false && led1_state == HIGH && auto_intensity == false) {
    digitalWrite(led1, HIGH);
  }
  else {
    digitalWrite(led1, LOW);
  }

  if (night == true && led2_state == LOW && auto_intensity == false) {
    digitalWrite(led2, HIGH);
  } else if (night == true && led2_state == LOW && auto_intensity == true) {
    brightness = map(ldr_value, 0, 990, 255, 0);
    analogWrite(led2, brightness);
  } else if (night == false && led2_state == HIGH && auto_intensity == true) {//if day time, turn on lights with remote
    brightness = map(ldr_value, 0, 990, 255, 0);
    analogWrite(led2, brightness);
  } else if (night == false && led2_state == HIGH && auto_intensity == false) {
    digitalWrite(led2, HIGH);
  }
  else {
    digitalWrite(led2, LOW);
  }

  if (night == true && led3_state == LOW && auto_intensity == false) {
    digitalWrite(led3, HIGH);
  } else if (night == true && led3_state == LOW && auto_intensity == true) {
    brightness = map(ldr_value, 0, 990, 255, 0);
    analogWrite(led3, brightness);
  } else if (night == false && led3_state == HIGH && auto_intensity == true) {//if day time, turn on lights with remote
    brightness = map(ldr_value, 0, 990, 255, 0);
    analogWrite(led3, brightness);
  } else if (night == false && led3_state == HIGH && auto_intensity == false) {
    digitalWrite(led3, HIGH);
  }
  else {
    digitalWrite(led3, LOW);
  }
}
void motion() {
  motion_value = digitalRead(motion_sensor);
  if (motion_value == HIGH) {
    motion_detected = true;
  } else {
    motion_detected = false;
  }
  //Serial.println(ldr_value);
}

void ldr_reading() {
  ldr_value = analogRead(ldr);
  //Serial.println(ldr_value);
}

void rainSensor() {
  sensorReading = analogRead(A1);
  rainThreshold = map(sensorReading, sensorMin, sensorMax, 0, 3);
  // range value:
  switch (rainThreshold) {
    case 0:    // Sensor getting wet
      rain_falling = "Flood";
      break;
    case 1:    // Sensor getting wet
      rain_falling = "Rain Warning";
      break;
    case 2:
      rain_falling = "Not Raining";
      break;
  }
  //delay(1);  // delay between reads
}
void myRemote() {
  if (IrReceiver.decode()) {
    //Serial.println(IrReceiver.decodedIRData.decodedRawData);
    IrReceiver.resume();
    switch (IrReceiver.decodedIRData.decodedRawData) {
      case 3125149440:
        if (system_activated == true) {
          Serial.println("button 1 pressed");
          digitalWrite(led1, !digitalRead(led1));
          led1_state = !led1_state;
        }
        break;
      case 3108437760:
        if (system_activated == true) {
          Serial.println("button 2 pressed");
          digitalWrite(led2, !digitalRead(led2));
          led2_state = !led2_state;
        }
        break;
      case 3091726080:
        if (system_activated == true) {
          Serial.println("button 3 pressed");
          digitalWrite(led3, !digitalRead(led3));
          led3_state = !led3_state;
        }
        break;
      case 3810328320:
        if (system_activated == true) {
          Serial.println("Auto light control Cofiguration");
          auto_intensity = !auto_intensity;
        }
        break;
      case 3910598400:
        if (system_activated == true) {
          Serial.println("Motion Activation");
          motion_activated = !motion_activated;
        }
        break;
      case 4061003520:
        Serial.println("Switching system on | off");
        system_activated = !system_activated;
        break;
      default:
        //Serial.println("none of the specified");
        break;
    }
  }
}
void oled() {
  dt = clock.getDateTime();

  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0, 0);            // Start at top-left corner
  //display.println(F("Welcome To My System!"));
  display.println("Time: " + String(dt.hour) + ":" + String(dt.minute) + ":" + String(dt.second));
  display.println("Date: " + String(dt.day) + "-" + String(dt.month) + ":" + String(dt.year));
  if (rain_falling != "Flood") {
    if (auto_intensity == false) {
      if (system_activated == true) {
        display.println(F("Brightness: 100% ON"));
      } else {
        display.println(F("Brightness: 100% OFF"));
      }
    } else {
      if (system_activated == true) {
        display.println(F("Brightness: Auto ON"));
      } else {
        display.println(F("Brightness: Auto OFF"));
      }
    }
    if (motion_activated == false) {
      display.println(F("Motion: de-activated!"));
    } else {
      display.println(F("Motion: activated"));
    }
  } else {
    display.println(F("Raining: Lights Off!"));
    if (auto_intensity == false && motion_activated == false) {
      display.println(F("B: 100% | M: OFF"));
    } else if (auto_intensity == true && motion_activated == true) {
      display.println(F("B: Auto | M: ON"));
    } else if (auto_intensity == false && motion_activated == true) {
      display.println(F("B: 100% | M: ON"));
    } else {
      display.println(F("B: Auto | M: OFF"));
    }
  }

  if ((int(dt.hour) >= 18 && int(dt.hour <= 23)) || (int(dt.hour) >= 0 && int(dt.hour <= 6))) {
    night = true;
  } else {
    night = false;
  }
  display.display();

}
