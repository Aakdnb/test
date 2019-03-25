#include <Arduino.h>
#include "algorithm.h"
#include "max30102.h"
#include <SoftwareSerial.h>    //블루투스 시리얼 통신 라이브러리 추가
#include <math.h>



SoftwareSerial BTSerial(2, 3);


//if Adafruit Flora development board is chosen, include NeoPixel library and define an NeoPixel object
#if defined(ARDUINO_AVR_FLORA8)
#include "adafruit_neopixel.h"
#define BRIGHTNESS_DIVISOR 8  //to lower the max brightness of the neopixel LED
Adafruit_NeoPixel LED = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800);
#endif

#define MAX_BRIGHTNESS 255

#if defined(ARDUINO_AVR_NANO)
//Arduino NANO doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated.  Samples become 16-bit data.
uint16_t aun_ir_buffer[100]; //infrared LED sensor data
uint16_t aun_red_buffer[100];  //red LED sensor data
#else
uint32_t aun_ir_buffer[100]; //infrared LED sensor data
uint32_t aun_red_buffer[100];  //red LED sensor data
#endif
int32_t n_ir_buffer_length; //data length
int32_t n_spo2;  //SPO2 value
int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
int32_t n_heart_rate; //heart rate value
int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;

//Analog read pins
const int xPin = 0;
const int yPin = 1;
const int zPin = 2;

//The minimum and maximum values that came from
//the accelerometer while standing still
//You very well may need to change these
int minVal = 265;
int maxVal = 402;


//to hold the caculated values
double x;
double y;
double z;

// the setup routine runs once when you press reset:
void setup() {

  maxim_max30102_reset(); //resets the MAX30102
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  BTSerial.begin(9600);
  pinMode(10, INPUT);  //pin D10 connects to the interrupt output pin of the MAX30102
  delay(1000);
  maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);  //Reads/clears the interrupt status register
  while(BTSerial.available()==0)  //wait until user presses a key
  {
    BTSerial.write(27);       // ESC command
    BTSerial.print("[2J");    // clear screen command
#if defined(ARDUINO_AVR_LILYPAD_USB)    
    Serial.println("Lilypad");
#endif
#if defined(ARDUINO_AVR_FLORA8)
    Serial.println(F("Adafruit Flora"));
#endif
    BTSerial.println("Press any key to start conversion");
   delay(1000);
 }
  uch_dummy=Serial.read();
  maxim_max30102_init();  //initialize the MAX30102
}

// the loop routine runs over and over again forever:
void loop() {
  uint32_t un_min, un_max, un_prev_data, un_brightness;  //variables to calculate the on-board LED brightness that reflects the heartbeats
  int32_t i;
  float f_temp;
  
  un_brightness=0;
  un_min=0x3FFFF;
  un_max=0;
  
  n_ir_buffer_length=100;  //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for(i=0;i<n_ir_buffer_length;i++)
  {
    while(digitalRead(10)==1);  //wait until the interrupt pin asserts
    maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO
    
    if(un_min>aun_red_buffer[i])
      un_min=aun_red_buffer[i];  //update signal min
    if(un_max<aun_red_buffer[i])
      un_max=aun_red_buffer[i];  //update signal max
    
  }
  un_prev_data=aun_red_buffer[i];
  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while(1)
  {
    i=0;
    un_min=0x3FFFF;
    un_max=0;

    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for(i=25;i<100;i++)
    {
      aun_red_buffer[i-25]=aun_red_buffer[i];
      aun_ir_buffer[i-25]=aun_ir_buffer[i];

      //update the signal min and max
      if(un_min>aun_red_buffer[i])
        un_min=aun_red_buffer[i];
      if(un_max<aun_red_buffer[i])
        un_max=aun_red_buffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for(i=75;i<100;i++)
    {
      un_prev_data=aun_red_buffer[i-1];
      while(digitalRead(10)==1);
      digitalWrite(9, !digitalRead(9));
      maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));

      //calculate the brightness of the LED
      if(aun_red_buffer[i]>un_prev_data)
      {
        f_temp=aun_red_buffer[i]-un_prev_data;
        f_temp/=(un_max-un_min);
        f_temp*=MAX_BRIGHTNESS;
        f_temp=un_brightness-f_temp;
        if(f_temp<0)
          un_brightness=0;
        else
          un_brightness=(int)f_temp;
      }
      else
      {
        f_temp=un_prev_data-aun_red_buffer[i];
        f_temp/=(un_max-un_min);
        f_temp*=MAX_BRIGHTNESS;
        un_brightness+=(int)f_temp;
        if(un_brightness>MAX_BRIGHTNESS)
          un_brightness=MAX_BRIGHTNESS;
      }
#if defined(ARDUINO_AVR_LILYPAD_USB)  
      analogWrite(13, un_brightness);
#endif

#if defined(ARDUINO_AVR_FLORA8)
      LED.setPixelColor(0, un_brightness/BRIGHTNESS_DIVISOR, 0, 0);
      LED.show();
#endif

      //send samples and calculation result to terminal program through UART
     
      
      BTSerial.print("HR=");
      BTSerial.print(n_heart_rate, DEC);
      
           
      BTSerial.print(", SPO2=");
      BTSerial.println(n_spo2, DEC);

     

        int xRead = analogRead(xPin);
        int yRead = analogRead(yPin);
        int zRead = analogRead(zPin);

  //convert read values to degrees -90 to 90 - Needed for atan2
        int xAng = map(xRead, minVal, maxVal, -90, 90);
        int yAng = map(yRead, minVal, maxVal, -90, 90);
        int zAng = map(zRead, minVal, maxVal, -90, 90);

  //Caculate 360deg values like so: atan2(-yAng, -zAng)
  //atan2 outputs the value of -π to π (radians)
  //We are then converting the radians to degrees
        x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
        y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
        z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  //Output the caculations
        BTSerial.print("x: ");
        BTSerial.print(x);
        BTSerial.print(" | y: ");
        BTSerial.print(y);
        BTSerial.print(" | z: ");
        BTSerial.println(z);
    }
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
  }
}
 
