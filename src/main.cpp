#include <Arduino.h>
#include <FastLED.h>
#include <SPI.h>
#include "ESP32_fft.h"

#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_SCLK 18
#define VSPI_CS   5

#define FFT_N 512 //FFT SAMPLES to be used
#define SAMPLEFREQ 40000 //time taken to obtain 512 samples at 40,000Hz
#define noise_filter 

#define touch1 32
#define touch2 33
#define touch3 27

static const uint8_t max_brightness=190;
static const uint8_t min_brightness=2;

static TaskHandle_t rainbow_handle, ISR_handler_handle, spi_handle, fill_white_lights_handle;
static TimerHandle_t touch1_timer, touch2_timer, touch3_timer, hue_timer, mapping_decay_timer, peak_decay_timer;

float fft_input_ch0[FFT_N]; //input array for fft calculations
float fft_input_ch1[FFT_N];
float fft_output_ch0[FFT_N];// fft output array
float fft_output_ch1[FFT_N];

typedef struct spi_out{
  float spi_out_ch0[FFT_N];
  float spi_out_ch1[FFT_N];
}spi_out;

uint_fast16_t amplitude[2][10], old_amplitude[2][10];

ESP_fft fft_ch0(FFT_N, SAMPLEFREQ, FFT_REAL, FFT_FORWARD, fft_input_ch0, fft_output_ch0);
ESP_fft fft_ch1(FFT_N, SAMPLEFREQ, FFT_REAL, FFT_FORWARD, fft_input_ch1, fft_output_ch1);

uint16_t bands[10] = {1,3,6,13,20,26,32,40,52,64}; //156.25, 468.75, 973.5, 2031.25, 3125, 4062.5, 5000, 6250, 8125, 10000, (Frequencies to analyse)
uint16_t mapping_max =2000;

static const int spiClk =8000000; //8 MHZ SPI CLock

SPIClass *vspi = NULL;

static uint8_t received_ch0[2];
static uint8_t received_ch1[2];

//create queue to hold fft input buffer
QueueHandle_t fft_in_queue;


//define the pins that will be used to interface the components of the project
const uint8_t right_leds=16;
const uint8_t left_leds=17;
const uint8_t num_leds=60;
const uint8_t relay_pin=12;

//const uint16_t touch_threshold = 40;

bool touch1_active = false;
bool touch2_active = false;
bool touch3_active = false;
bool touch1_letgo = true;
bool touch2_letgo = true;
bool touch3_letgo = true;
bool short_press1 = true;
bool short_press2 = true;
bool short_press3 = true;
bool timer1_started = false;
bool timer2_started = false;
bool timer3_started = false;
bool long_press1 = true;
bool long_press2 = true;
bool long_press3 = true;

bool brightness_control_flag = false;
bool modePalette_control_flag = false;

static int touch1_time, touch2_time, touch3_time;

uint8_t hue = 0;
uint8_t brightness = 60;
uint8_t column_height[2][10], peak_column_height[2][10];
uint8_t palette_index = 2;

//create the structure that will be be used to address the leds
static CRGB leds[2][num_leds];

CRGBPalette16 palettes_arr[] ={
  RainbowColors_p,
  //bb palette 1
  CRGBPalette16(CRGB(255, 0, 84),
                CRGB(255, 0, 84),
                CRGB(60, 60, 255),
                CRGB(60, 60, 255),

                CRGB(75, 203, 246),
                CRGB(75, 203, 246),
                CRGB(255, 245, 50),
                CRGB(255, 245, 50),
                
                CRGB(255, 245, 50),
                CRGB(255, 245, 50),
                CRGB(75, 203, 246),
                CRGB(75, 203, 246),

                CRGB(60, 60, 255),
                CRGB(60, 60, 255),
                CRGB(255, 0, 84),
                CRGB(255, 0, 84)),
  //bb palette 2
  CRGBPalette16(CRGB(255, 0, 0),
                CRGB(255, 0, 0),
                CRGB(255, 100, 100),
                CRGB(255, 100, 100),

                CRGB(160, 20, 255),
                CRGB(160, 20, 255),
                CRGB(255, 230, 40),
                CRGB(255, 230, 40),
                
                CRGB(255, 230, 40),
                CRGB(255, 230, 40),
                CRGB(160, 20, 255),
                CRGB(160, 20, 255),

                CRGB(255, 100, 100),
                CRGB(255, 100, 100),
                CRGB(255, 0, 0),
                CRGB(255, 0, 0))
};

/* sort the leds into rows and columns for easy addressing 
   there are 60 leds per channel for left and right,
   10 columns and 6 rows for each channel

*/
static uint8_t leds_arr[10][6];


void zero(uint8_t lr, uint8_t pos){
  for(uint8_t i=0; i<5; i++){
    leds[lr][leds_arr[0+pos][i]] = CHSV(0,0,brightness);
    leds[lr][leds_arr[2+pos][i]] = CHSV(0,0,brightness);
  }
  leds[lr][leds_arr[1+pos][4]] = CHSV(0,0,brightness);
  leds[lr][leds_arr[1+pos][0]] = CHSV(0,0,brightness);
}

void one(uint8_t lr, uint8_t pos){

  for(uint8_t i=0; i<5; i++){
    leds[lr][leds_arr[1+pos][i]] = CHSV(0,0,brightness);
  }
}

void two(uint8_t lr, uint8_t pos){
  if (lr == 0)
  {
    for(uint8_t i=0; i<3; i++){
      for(uint8_t j=0; j<5; j+=2){
        leds[lr][leds_arr[i+pos][j]] = CHSV(0,0,brightness);
      }
    }
    leds[lr][leds_arr[0+pos][3]] = CHSV(0,0,brightness);
    leds[lr][leds_arr[2+pos][1]] = CHSV(0,0,brightness);
  }
  else{
    for(uint8_t i=0; i<3; i++){
      for(uint8_t j=0; j<5; j+=2){
        leds[lr][leds_arr[i+pos][j]] = CHSV(0,0,brightness);
      }
    }
    leds[lr][leds_arr[0+pos][1]] = CHSV(0,0,brightness);
    leds[lr][leds_arr[2+pos][3]] = CHSV(0,0,brightness);
  }
}

void three(uint8_t lr, uint8_t pos){
  if (lr == 0)
  {
    for(uint8_t i=0; i<3; i++){
      for(uint8_t j=0; j<5; j+=2){
        leds[lr][leds_arr[i+pos][j]] = CHSV(0,0,brightness);
      }
    }
    leds[lr][leds_arr[0+pos][3]] = CHSV(0,0,brightness);
    leds[lr][leds_arr[0+pos][1]] = CHSV(0,0,brightness);
  }
  else{
    for(uint8_t i=0; i<3; i++){
      for(uint8_t j=0; j<5; j+=2){
        leds[lr][leds_arr[i+pos][j]] = CHSV(0,0,brightness);
      }
    }
    leds[lr][leds_arr[2+pos][1]] = CHSV(0,0,brightness);
    leds[lr][leds_arr[2+pos][3]] = CHSV(0,0,brightness);
  }
}

void four(uint8_t lr, uint8_t pos){
  if(lr ==0){
    for(uint8_t i=0; i<5; i++){
      leds[lr][leds_arr[0+pos][i]]= CHSV(0,0,brightness);
    }
    for(uint8_t j=4; j>1; j--){
      leds[lr][leds_arr[2+pos][j]]= CHSV(0,0,brightness);
    }
    leds[lr][leds_arr[1+pos][2]] = CHSV(0,0,brightness);
  }
  else{
    for(uint8_t i=0; i<5; i++){
      leds[lr][leds_arr[2+pos][i]]= CHSV(0,0,brightness);
    }
    for(uint8_t j=4; j>1; j--){
      leds[lr][leds_arr[0+pos][j]]= CHSV(0,0,brightness);
    }
    leds[lr][leds_arr[1+pos][2]] = CHSV(0,0,brightness);
  }
}

void five(uint8_t lr, uint8_t pos){
  if (lr == 0)
  {
    for(uint8_t i=0; i<3; i++){
      for(uint8_t j=0; j<5; j+=2){
        leds[lr][leds_arr[i+pos][j]] = CHSV(0,0,brightness);
      }
    }
    leds[lr][leds_arr[2+pos][3]] = CHSV(0,0,brightness);
    leds[lr][leds_arr[0+pos][1]] = CHSV(0,0,brightness);
  }
  else{
    for(uint8_t i=0; i<3; i++){
      for(uint8_t j=0; j<5; j+=2){
        leds[lr][leds_arr[i+pos][j]] = CHSV(0,0,brightness);
      }
    }
    leds[lr][leds_arr[2+pos][1]] = CHSV(0,0,brightness);
    leds[lr][leds_arr[0+pos][3]] = CHSV(0,0,brightness);
  }
}

void six(uint8_t lr, uint8_t pos){
  if(lr ==0){
    for(uint8_t i=0; i<6; i+=2){
      for(uint8_t j=0; j<3; j++){
        leds[lr][leds_arr[j+pos][i]]= CHSV(0,0,brightness);
      }
    }
    leds[lr][leds_arr[0+pos][1]] = CHSV(0,0,brightness);
    leds[lr][leds_arr[2+pos][1]] = CHSV(0,0,brightness);
    leds[lr][leds_arr[2+pos][3]] = CHSV(0,0,brightness);
  }
  else{
    for(uint8_t i=0; i<6; i+=2){
      for(uint8_t j=0; j<3; j++){
        leds[lr][leds_arr[j+pos][i]]= CHSV(0,0,brightness);
      }
    }
    leds[lr][leds_arr[2+pos][1]] = CHSV(0,0,brightness);
    leds[lr][leds_arr[0+pos][1]] = CHSV(0,0,brightness);
    leds[lr][leds_arr[0+pos][3]] = CHSV(0,0,brightness);
  }
}

void seven(uint8_t lr, uint8_t pos){
  if(lr ==0){
    for(uint8_t i=0; i<3; i++){
      leds[lr][leds_arr[i+pos][4]] = CHSV(0,0,brightness);
      leds[lr][leds_arr[i+pos][3-i]] = CHSV(0,0,brightness);
    }
    leds[lr][leds_arr[2+pos][0]] = CHSV(0,0,brightness);
  }
  else{
    for(uint8_t i=0; i<3; i++){
      leds[lr][leds_arr[i+pos][4]] = CHSV(0,0,brightness);
      leds[lr][leds_arr[i+pos][1+i]] = CHSV(0,0,brightness);
    }
    leds[lr][leds_arr[0+pos][0]] = CHSV(0,0,brightness);
  }
}

void eight(uint8_t lr, uint8_t pos){
  for(uint8_t i=0; i<3; i++){
    for(uint8_t j=0; j<5; j++){
    leds[lr][leds_arr[i+pos][j]] =CHSV(0,0,brightness);
    }
  }
  leds[lr][leds_arr[1+pos][1]]= CHSV(0,0,0);
  leds[lr][leds_arr[1+pos][3]]= CHSV(0,0,0);
}

void nine(uint8_t lr, uint8_t pos){
  if(lr ==0){
    for(uint8_t i=0; i<6; i+=2){
      for(uint8_t j=0; j<3; j++){
        leds[lr][leds_arr[j+pos][i]]= CHSV(0,0,brightness);
      }
    }
    leds[lr][leds_arr[0+pos][3]] = CHSV(0,0,brightness);
    leds[lr][leds_arr[2+pos][3]] = CHSV(0,0,brightness);
    leds[lr][leds_arr[0+pos][1]] = CHSV(0,0,brightness);
  }
  else{
    for(uint8_t i=0; i<6; i+=2){
      for(uint8_t j=0; j<3; j++){
        leds[lr][leds_arr[j+pos][i]]= CHSV(0,0,brightness);
      }
    }
    leds[lr][leds_arr[2+pos][3]] = CHSV(0,0,brightness);
    leds[lr][leds_arr[0+pos][3]] = CHSV(0,0,brightness);
    leds[lr][leds_arr[2+pos][1]] = CHSV(0,0,brightness);
  }
}

void mode_palette_control(uint8_t choice){
  switch (choice)
  {
  case 0:
    for(uint8_t j=0; j<10; j++){
      for(uint8_t i=0; i<5; i++){
        if(j == (9 or 7 or 6 or 4 or 3))
        leds[0][leds_arr[j][i]] = ColorFromPalette(palettes_arr[palette_index], j*6 + hue, brightness, LINEARBLEND);
      }
      if(j == 8){
        leds[0][leds_arr[j][3]] = ColorFromPalette(palettes_arr[palette_index], j*6 + hue, brightness, LINEARBLEND);
      }
      if(j == (5 or 2)){
        leds[0][leds_arr[j][4]] = ColorFromPalette(palettes_arr[palette_index], j*6 + hue, brightness, LINEARBLEND);
        leds[0][leds_arr[j][0]] = ColorFromPalette(palettes_arr[palette_index], j*6 + hue, brightness, LINEARBLEND);
      }
      if(j == 1){
        leds[0][leds_arr[j][1]] = ColorFromPalette(palettes_arr[palette_index], j*6 + hue, brightness, LINEARBLEND);
        leds[0][leds_arr[j][2]] = ColorFromPalette(palettes_arr[palette_index], j*6 + hue, brightness, LINEARBLEND);
        leds[0][leds_arr[j][3]] = ColorFromPalette(palettes_arr[palette_index], j*6 + hue, brightness, LINEARBLEND);
      }
    }  

    for(uint8_t i=0; i<10; i++){
      if(i == (0 or 6)){
        for(uint8_t j=0; j<4; j++){
          leds[i][leds_arr[i][j]] = ColorFromPalette(palettes_arr[palette_index], i*6 + hue, brightness, LINEARBLEND);
        }
      }

    }
  break;
  
  case 1:

  break;
  default:
    break;
  }
}
//function to clear 1 array of leds
void clear_leds(uint8_t led_arr_lr){
  fill_solid(leds[led_arr_lr],num_leds,CHSV(0,0,0));
  FastLED.show();
}

void fill_white_lights(){
  fill_solid(leds[0],num_leds,CHSV(0,0,brightness));
  fill_solid(leds[1],num_leds,CHSV(0,0,brightness));
  FastLED.show();
}
//function to display numbers on wither side of the led array
void number_display(bool left_pos){
  String number_str;
  if (left_pos){
    clear_leds(0);
    number_str = (String) brightness;
    for(uint8_t i=0; i<=number_str.length(); i++){
      switch (number_str[i])
      {
        case '0':
          zero(0,6-i*3);
          break;
        case '1':
          one(0,6-i*3);
          break;
        case '2':
          two(0,6-i*3);
          break;
        case '3':
          three(0,6-i*3);
          break;
        case '4':
          four(0,6-i*3);
          break;
        case '5':
          five(0,6-i*3);
          break;
        case '6':
          six(0,6-i*3);
          break;
        case '7':
          seven(0,6-i*3);
          break;
        case '8':
          eight(0,6-i*3);
          break;
        case '9':
          nine(0,6-i*3);
          break;
      }
    }
  }
  else{
    clear_leds(1);
    number_str = (String) brightness;
    for(uint8_t i=0; i<number_str.length(); i++){
      switch (number_str[i])
      {
        case '0':
          zero(1,i*3);
          break;
        case '1':
          one(1,i*3);
          break;
        case '2':
          two(1,i*3);
          break;
        case '3':
          three(1,i*3);
          break;
        case '4':
          four(1,i*3);
          break;
        case '5':
          five(1,i*3);
          break;
        case '6':
          six(1,i*3);
          break;
        case '7':
          seven(1,i*3);
          break;
        case '8':
          eight(1,i*3);
          break;
        case '9':
          nine(1,i*3);
          break;
      }
    }
  }
  FastLED.show();
}

void brightness_control(uint8_t choice){
  uint8_t *p_brightness = &brightness;
  switch(choice){
    //show brightness controls for case 0
    case 0:
      for(uint8_t i=0; i<2; i++){
        for(uint8_t j=0; j<10; j++){
          for(uint8_t k=0; k<6; k++){
            leds[i][leds_arr[j][k]]=CRGB(0,0,0);
          }
      }
      }
      for(uint8_t l=2; l<8; l++){
        for(uint8_t m=2; m<4; m++){
          leds[0][leds_arr[l][m]] =CHSV(0,0,brightness);
          leds[1][leds_arr[l][m]] =CHSV(0,0,brightness);
        }
      }
      for(uint8_t n=4; n<6; n++){
        for(uint8_t o=0; o<6; o++){
          leds[1][leds_arr[n][o]] = CHSV(0,0,brightness);
        }
      }
      FastLED.show();
      break;
    // increase brigtness by 1 and show brightness value `on the left
    case 1:
      brightness +=2;
      if(brightness>max_brightness){
        brightness = max_brightness;
      }
      clear_leds(1);
      for(uint8_t l=2; l<8; l++){
        for(uint8_t m=2; m<4; m++){
          leds[1][leds_arr[l][m]] = CHSV(0,0,brightness);
        }
      }
      for(uint8_t n=4; n<6; n++){
        for(uint8_t o=0; o<6; o++){
          leds[1][leds_arr[n][o]] = CHSV(0,0,brightness);
        }
      }
      number_display(true);
      break;
    // decrease brightness by 2 and show brighness value on the right.
    case 2:
      brightness -=2;
      if (brightness<min_brightness){
        brightness = min_brightness;
      }
      clear_leds(0);
      for(uint8_t l=2; l<8; l++){
        for(uint8_t m=2; m<4; m++){
          leds[0][leds_arr[l][m]] =CHSV(0,0,brightness);
        }
      }
      number_display(false);
      break;
  }
}

void timer1_callback(TimerHandle_t pxTimer){
  timer1_started = true;
  brightness_control(0);
  brightness_control_flag = false;
  Serial.println("timer1 tick");
}

void timer3_callback(TimerHandle_t pxTimer){
  timer3_started = true;
  brightness_control(1);
  Serial.println("timer 3 tick");
}

void timer2_callback(TimerHandle_t pxTimer){
  timer2_started = true;
  brightness_control(2);
  Serial.println("timer 2 tick");
}

uint8_t touch_reader(){
  uint_fast8_t bin1, bin2, bin3;
  uint_fast8_t touch_out;
  if (touchRead(touch1)<=40){
    bin1 = B100;
    if(touch1_letgo){
      touch1_time = millis();
      touch1_letgo = false;
      short_press1 = true;
      long_press1 = true;
    }
  }
  else{
    bin1 = B0;
    touch1_letgo = true;
  }
  if(touchRead(touch2)<=40){
    bin2 = B010;
    if(touch2_letgo){
      touch2_time = millis();
      touch2_letgo = false;
      short_press2 = true;
      long_press2 = true;
    }
  }
  else{
    bin2 = B0;
    touch2_letgo = true;
  }
  if(touchRead(touch3)<40){
    bin3 = B001;
    if(touch3_letgo){
      touch3_time = millis();
      touch3_letgo = false;
      short_press3 = true;
      long_press3 = true;
    }
  }
  else{
    bin3 = B0;
    touch3_letgo = true;
  }
  touch_out = bin1 + bin2 + bin3;
  return touch_out;
}

void touch_func(void* parameters){
  uint8_t touch_reading;
  uint8_t first_touch_val;
  bool first_touch_flag = true;
  bool suspend_flag = false;
  bool white_lights = false;
  bool speaker_on = true;
  while(1){
    if(first_touch_flag ==true and touch_reader()>0){
      first_touch_val = touch_reader();
      first_touch_flag = false;
      Serial.print("first touch val: ");
      Serial.println(first_touch_val,BIN);
     }
    switch (first_touch_val)
    {
    case B100:
      //brightness controls
      touch_reading = touch_reader();
      vTaskSuspend(rainbow_handle);
      vTaskSuspend(spi_handle);
      suspend_flag =true;
      switch (touch_reading){
        case B100:
          // if brightness was changed before coming here, wait 500ms before showing default -+ menu
          if (brightness_control_flag == true){
            if(timer1_started == false){
              xTimerStart(touch1_timer,0);
              timer1_started = true;
              Serial.println("timer1 started");
            }
            if(timer2_started){
              xTimerStop(touch2_timer,0);
              timer2_started = false;
            }
            if(timer3_started){
              xTimerStop(touch3_timer,0);
              timer3_started = false;
            }
          }
          else{
            brightness_control(0);
          }
        break;
          
        case B101:
          if (timer1_started){
            xTimerStop(touch1_timer,0);
            timer1_started = false;
            Serial.println("timer1 stopped");
          }
          if(timer2_started){
            xTimerStop(touch2_timer,0);
            timer2_started = false;
          }
          if(millis()-touch3_time<1000){
            Serial.print("touch3 time: ");
            Serial.println((millis()-touch3_time));
            if(short_press3){
              brightness_control(1);
              short_press3 = false;
              Serial.println("short press 3");
            }
          }
          if(millis()-touch3_time>=1000){
            if(timer3_started == false){
              timer3_started = true;
              xTimerStart(touch3_timer, 0);
              Serial.println(brightness);
            }
          }
          brightness_control_flag = true;
        break;

        case B110:
          if(timer1_started){
            xTimerStop(touch1_timer,0);
            timer1_started = false;
          }
          if(timer3_started){
            xTimerStop(touch3_timer,0);
            timer3_started =false;
          }
          if(millis()-touch2_time<1000){
            Serial.print("touch2 time: ");
            Serial.println((millis()-touch2_time));
            if(short_press2){
              brightness_control(2);
              short_press2 = false;
              Serial.println("short press 2");
            }
          }
          if(millis()-touch2_time>=1000){
            if(timer2_started == false){
              timer2_started = true;
              xTimerStart(touch2_timer, 0);
              Serial.println(brightness);
            }
          }
          brightness_control_flag = true;
        break;

        default:
          Serial.println("exit first_touch B100 menu");
          if(timer1_started){
            xTimerStop(touch1_timer,0);
            timer1_started = false;
          }
          if(timer2_started){
            xTimerStop(touch2_timer,0);
            timer2_started = false;
          }
          if(timer3_started){
            xTimerStop(touch3_timer,0);
            timer3_started = false;
          }
          if(white_lights){
            fill_white_lights();
          }
          brightness_control_flag = false;
          first_touch_val = 0;
          first_touch_flag = true;
        break;
      }
    break;

    case B010:
      //mode and palette controls
      touch_reading = touch_reader();
      vTaskSuspend(rainbow_handle);
      vTaskSuspend(spi_handle);
      suspend_flag =true;
      switch (touch_reading)
      {
      case B010:
      // show mode and palette menu
      break;
      
      case B110:
        if(short_press1){
          short_press1 = false;
          if(palette_index<2){
            palette_index +=1;
          }
          else{
            palette_index = 0;
          }
        modePalette_control_flag = true;
        }
      break;

      case B011:
      break;

      default:
        Serial.println("exit first touch B010 menu");
        modePalette_control_flag = false;
        first_touch_val = 0;
        first_touch_flag = true;
      break;
      }
    break;

    case B001:
      //white lights and speaker on off controls
      touch_reading = touch_reader();
      vTaskSuspend(rainbow_handle);
      vTaskSuspend(spi_handle);
      suspend_flag =true;
      switch (touch_reading){
        case B111:
          //turn of visualizer for white light, touch and hold longer to turn of speaker amplifier but keep lights on
          if (millis()-touch1_time<3000 and millis()-touch2_time<3000){
              if(short_press1 and short_press2 and short_press3){
                short_press1 = short_press2 = short_press3 = false;
                white_lights = !white_lights;
                if(white_lights){
                  vTaskDelay(1);
                  fill_white_lights();
                  Serial.println("white light on"); 
                }
                else{
                  Serial.println("white light off");
                }
              }
          }
          if (millis()-touch1_time>3000 and millis()-touch2_time>3000){
            if(long_press1 and long_press2 and long_press3){
              long_press1 = long_press2 = long_press3 = false;
              speaker_on = !speaker_on;
              Serial.println("toggle speakers");
              if(speaker_on){
                digitalWrite(relay_pin,LOW);
                Serial.println("speaker_on");
              }
              if(speaker_on != true){
                digitalWrite(relay_pin,HIGH);
                Serial.println("speaker off");
              }
            }
          }
        break;

        case B000:
          first_touch_val = 0;
          first_touch_flag = true;
        break;

        default:
        break;
      }
    break;

    default:
      first_touch_val = 0;
      first_touch_flag = true;
      break;
    }
    
    if(suspend_flag and touch_reader() == 0){
      if(white_lights != true){
        vTaskResume(rainbow_handle);
        vTaskResume(spi_handle);
      suspend_flag = false;
      }
      
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

void hue_callback(TimerHandle_t pxTimer){
  //increment the hue every 65 milliseconds
  hue+=2;
  
}

void mapping_decay_callback(TimerHandle_t pxTimer){
  if (mapping_max >=1300)
  {
    mapping_max -= 100;
  }
}

void peak_decay_callback(TimerHandle_t pxTimer){
  //decay the peak column height every 70 milliseconds
  for(uint8_t i=0; i<10; i++){
    if(peak_column_height[0][i]!=0){
      peak_column_height[0][i] -=1;
    }
    if (peak_column_height[1][i]!=0){
      peak_column_height[1][i] -=1;
    }
  }
}
//0x70 = B11100000
//0x60 = B11000000

void spi(void* parameters){

  uint8_t *p_received_ch0, *p_received_ch1;
  p_received_ch0 = received_ch0;
  p_received_ch1 = received_ch1;

  float *p_fft_input_ch0, *p_fft_input_ch1;
  p_fft_input_ch0 = fft_input_ch0;
  p_fft_input_ch1 = fft_input_ch1;
 
  spi_out fft_buf;
  while(1){
    int spiTime = micros();
    int oldTime;
    const uint8_t send_ch0= B11100000;
    const uint8_t send_ch1= B11000000;
    for(uint_fast16_t i=0; i<FFT_N; i++){
      oldTime =micros();
      vspi ->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
      //take ADC reading for channel 0
      digitalWrite(VSPI_CS, LOW);
      p_received_ch0[0]= vspi->transfer(send_ch0);
      p_received_ch0[1]= vspi->transfer(0);
      digitalWrite(VSPI_CS, HIGH);
      // take readin for channel 1
      digitalWrite(VSPI_CS, LOW);
      p_received_ch1[0]= vspi->transfer(send_ch1);
      p_received_ch1[1]= vspi->transfer(0);
      digitalWrite(VSPI_CS, HIGH);
      vspi ->endTransaction();

      fft_buf.spi_out_ch0[i]= word(p_received_ch0[0], p_received_ch0[1]);
      fft_buf.spi_out_ch1[i]= word(p_received_ch1[0], p_received_ch1[1]);
      //take a sample every 25 microseconds to get a sample rate of 40,000Hz
      while (micros()-oldTime<25){
      
      }
  }
    if (xQueueSend(fft_in_queue, (void *)&fft_buf, 0) !=pdPASS){
      Serial.println("queue buffer full");
    }
    // Serial.print("spi free mem: ");
    // Serial.println(uxTaskGetStackHighWaterMark(spi_handle));
    // Serial.print("rainbow free mem:");
    // Serial.println(uxTaskGetStackHighWaterMark(rainbow_handle));
  }
}

void rainbow(void* parameters){
  while(1){
  int oldtime = micros();
  spi_out fft_buf_received;

  float *p_fft_input_ch0, *p_fft_input_ch1;
  p_fft_input_ch0 = fft_input_ch0;
  p_fft_input_ch1 = fft_input_ch1;

  uint_fast16_t (*p_amplitude)[10], (*p_old_amplitude)[10];
  p_amplitude = amplitude;
  p_old_amplitude = old_amplitude;

  float *p_fft_output_ch0, *p_fft_output_ch1;
  p_fft_output_ch0 = fft_output_ch0;
  p_fft_output_ch1 = fft_output_ch1;

  uint8_t (*p_column_height)[10], (*p_peak_column_height)[10];
  p_column_height = column_height;
  p_peak_column_height = peak_column_height;

  uint8_t (*p_leds_arr)[6];
  p_leds_arr = leds_arr;

  uint16_t *p_mapping_max;
  p_mapping_max = &mapping_max;

  CRGB (*p_leds)[num_leds];
  p_leds = leds;

  if(xQueueReceive(fft_in_queue,(void *)&fft_buf_received,0) ==pdTRUE){
    for (uint_fast16_t i = 0; i < FFT_N; i++){
      p_fft_input_ch0[i] = fft_buf_received.spi_out_ch0[i];
      p_fft_input_ch1[i] = fft_buf_received.spi_out_ch1[i];
    }
  

  //fft magic
  fft_ch0.removeDC();
  fft_ch0.hammingWindow();
  fft_ch0.execute();
  fft_ch0.complexToMagnitude();

  fft_ch1.removeDC();
  fft_ch1.hammingWindow();
  fft_ch1.execute();
  fft_ch1.complexToMagnitude();  

  //process fft output
  fft_output_ch0[bands[0]]-=60;
  fft_output_ch1[bands[0]]-=60;

  for(uint8_t f=0; f<2; f++){
    for(uint8_t i=0; i<10; i++){
      if(f==0){
        p_amplitude[f][i] = (uint_fast16_t) p_fft_output_ch0[bands[i]];
      }
      else{
        p_amplitude[f][i] = (uint_fast16_t) p_fft_output_ch1[bands[i]];
      }
    }
  }

  for(uint8_t f=0; f<2;f++){
    for (uint8_t i=0; i< 10; i++) {

      //slow down rising of lights
      if (p_amplitude[f][i] > p_old_amplitude[f][i]){
        p_amplitude[f][i] = (p_amplitude[f][i] + p_old_amplitude[f][i]); 
      }
    
      //filter out noise picked up by fft
      if (p_amplitude[f][i]>100 and p_amplitude[f][i]<10000){
        if(i==0){
          p_amplitude[f][i] *=0.9;
        }
        if(i==1){
          p_amplitude[f][i] *=0.9;
        }
        //amplify the last 3 bands after filetering to make it more visible
        if(i == 7){
          p_amplitude[f][i] *=1.08;
        }
        if(i == 8){
          p_amplitude[f][i] *=1.08;
        }
        if(i == 9){
          p_amplitude[f][i] *=1.08;
        }
      }
      else{
        p_amplitude[f][i]=0;
      }
      p_old_amplitude[f][i] = p_amplitude[f][i];
    }
  }

  for(uint8_t f=0; f<2; f++){
    for (uint8_t i = 0; i < 10; i++){
      if (p_amplitude[f][i] >= *p_mapping_max)
      {
        *p_mapping_max +=35;
      }
      
      p_column_height[f][i] = map(p_amplitude[f][i], 120, *p_mapping_max, 0, 5);

      if(p_peak_column_height[f][i] <= p_column_height[f][i]){
        p_peak_column_height[f][i] = p_column_height[f][i];
      }
      
      for(int j=0; j<=p_peak_column_height[f][i]; j++){
        p_leds[f][p_leds_arr[i][j]]= ColorFromPalette(palettes_arr[palette_index], i*5+ hue, brightness, LINEARBLEND );
      }
      
      for(int k=5; k>p_peak_column_height[f][i]; k--){
        p_leds[f][p_leds_arr[i][k]]=CHSV(0,0,0);
      }
    }
  }
  FastLED.show();
  }
  else{
   // Serial.println("queue empty");
  }
  }
}

void setup(){
  fft_in_queue = xQueueCreate(2, sizeof(spi_out));

  Serial.begin(115200);
  pinMode(relay_pin, OUTPUT); 
  pinMode(33, INPUT);

  vspi = new SPIClass(VSPI);
  vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_CS);
  pinMode(VSPI_CS, OUTPUT);
  // assign the arrays that can be processed to output the led colors.
  FastLED.addLeds<WS2812, left_leds, GRB>(leds[0], num_leds);
  FastLED.addLeds<WS2812, right_leds, GRB>(leds[1], num_leds);
  FastLED.setCorrection(TypicalSMD5050);

  uint8_t led_num =0;
  for(int i=0; i<10; i++){
    if(i%2 ==0 || i==0){
      for(int j=5;j>=0;j--){
        leds_arr[i][j]=led_num;
        led_num++;
      }
    }
    else{
      for(int k=0;k<=5;k++){
        leds_arr[i][k]=led_num;
        led_num++;
      }
    }   
  }
  
  xTaskCreatePinnedToCore(spi,"spi",5200,NULL,1,&spi_handle,1);
  xTaskCreatePinnedToCore(rainbow,"rainbow",5200,NULL,1,&rainbow_handle,0);
  xTaskCreate(touch_func,"touch_func", 3000, NULL, 1, &ISR_handler_handle);

  // create timers to manage time routine tasks
  touch1_timer = xTimerCreate("touch1_timer", pdMS_TO_TICKS(3000), pdFALSE, (void*)1, timer1_callback);
  touch3_timer = xTimerCreate("touch3_timer", pdMS_TO_TICKS(200), pdTRUE, (void*)3 , timer3_callback);
  touch2_timer = xTimerCreate("touch2_timer", pdMS_TO_TICKS(200), pdTRUE, (void*)2, timer2_callback);
  hue_timer = xTimerCreate("hue_timer", pdMS_TO_TICKS(80), pdTRUE, (void*)4, hue_callback);
  mapping_decay_timer = xTimerCreate("mapping_decay", pdMS_TO_TICKS(250), pdTRUE, (void*)5, mapping_decay_callback);
  peak_decay_timer = xTimerCreate("peak_decay_timer", pdMS_TO_TICKS(70), pdTRUE, (void*)6, peak_decay_callback);

  xTimerStart(hue_timer, 0);
  xTimerStart(mapping_decay_timer, 0);
  xTimerStart(peak_decay_timer, 0);
}

void loop(){
    vTaskDelete(NULL);
  }
 
 