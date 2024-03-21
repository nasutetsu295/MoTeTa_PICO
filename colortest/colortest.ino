#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "ClosedCube_TCA9548A.h"

// Pick analog outputs, for the UNO these three work well
// use ~560  ohm resistor between Red & Blue, ~1K for green (its brighter)
#define redpin 3
#define greenpin 5
#define bluepin 6
// for a common anode LED, connect the common pin to +5V
// for common cathode, connect the common to ground

// set to false if using a common cathode LED
#define commonAnode true

#define UART_BAUD 9600
#define TCA9548A_I2C_ADDRESS  0x70

// our RGB -> eye-recognized gamma color
byte gammatable[256];

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
ClosedCube::Wired::TCA9548A tca9548a;

void init_color(){
  tca9548a.address(0x70);
    tca9548a.selectChannel(5);   
    if (tcs.begin()) {
      Serial.println("Color is good");
    } else {
      Serial.println("No TCS34725 found ... check your connections");
      while (1){
        Serial.println("COLOR IS NOT GOOD.");
        delay(100);
        }; // halt!
    }
}

void setup() {
  Serial.begin(9600);
  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin();
  init_color();

//   // use these three pins to drive an LED
// #if defined(ARDUINO_ARCH_ESP32)
//   ledcAttach(redpin, 12000, 8);
//   ledcAttach(greenpin, 12000, 8);
//   ledcAttach(bluepin, 12000, 8);
// #else
//   pinMode(redpin, OUTPUT);
//   pinMode(greenpin, OUTPUT);
//   pinMode(bluepin, OUTPUT);
// #endif

//   // thanks PhilB for this gamma table!
//   // it helps convert RGB colors to what humans see
//   for (int i=0; i<256; i++) {
//     float x = i;
//     x /= 255;
//     x = pow(x, 2.5);
//     x *= 255;

//     if (commonAnode) {
//       gammatable[i] = 255 - x;
//     } else {
//       gammatable[i] = x;
//     }
//     //Serial.println(gammatable[i]);
//   }
}

// The commented out code in loop is example of getRawData with clear value.
// Processing example colorview.pde can work with this kind of data too, but It requires manual conversion to 
// [0-255] RGB value. You can still uncomments parts of colorview.pde and play with clear value.
float red, green, blue;
void getcolor() {

  tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);
  
  tcs.setInterrupt(true);  // turn off LED

  Serial.print("R:\t"); Serial.print(int(red)); 
  Serial.print("\tG:\t"); Serial.print(int(green)); 
  Serial.print("\tB:\t"); Serial.print(int(blue));

  Serial.print("\n");
}

void loop(){
  getcolor();
}