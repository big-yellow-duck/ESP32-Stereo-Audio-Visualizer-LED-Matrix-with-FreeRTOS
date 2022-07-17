#include <Arduino.h>
#include <FastLED.h>
#include <SPI.h>
#include "ESP32_fft.h"
#include <EEPROM.h>

#define EEPROM_size 3

#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_SCLK 18
#define VSPI_CS   5

#define FFT_N 512 //FFT SAMPLES to be used
#define SAMPLEFREQ 40000 //time taken to obtain 512 samples at 40,000Hz
#define noise_filter 

//touch pins to use with the ESP32
#define touch1 32
#define touch2 33
#define touch3 27

static const uint8_t max_brightness=190;
static const uint8_t min_brightness=2;

static TaskHandle_t normal_vis_handle, ISR_handler_handle, spi_handle, fill_white_lights_handle, fft_handle, wave_vis_handle;
static TimerHandle_t touch1_timer, touch2_timer, touch3_timer, hue_timer, mapping_decay_timer, peak_decay_timer, wave_frame_timer, mode_animation_timer, mode_animation_random_timer;

float fft_input_ch0[FFT_N]; //input array for fft calculations
float fft_input_ch1[FFT_N];
float fft_output_ch0[FFT_N];// fft output array
float fft_output_ch1[FFT_N];

//struct to hold data to send to the spi output queue
typedef struct spi_out{
  float spi_out_ch0[FFT_N];
  float spi_out_ch1[FFT_N];
}spi_out;

//struct to hold data to send to another task that will use the fft output
typedef struct fft_out{
  uint_fast16_t post_process[2][10];
}fft_out;

// create fft classes
ESP_fft fft_ch0(FFT_N, SAMPLEFREQ, FFT_REAL, FFT_FORWARD, fft_input_ch0, fft_output_ch0);
ESP_fft fft_ch1(FFT_N, SAMPLEFREQ, FFT_REAL, FFT_FORWARD, fft_input_ch1, fft_output_ch1);

// choose the specific frequencies to analyze
uint16_t bands[10] = {1,3,6,13,20,26,32,40,52,64}; //78.125, 234.375, 468.75, 1015.625, 1562.5, 2031.25, 2500, 3125, 4062.5, 5000 (Frequencies to analyse)

//maximum map map value for the fft output
uint16_t mapping_max =2000;

static const int spiClk =8000000; //8 MHZ SPI CLock

SPIClass *vspi = NULL;

// create arrays to hold the received spi data
static uint8_t received_ch0[2];
static uint8_t received_ch1[2];

//create queue to hold fft input and output for intertask communication
QueueHandle_t fft_in_queue, fft_out_queue;

//define the pins that will be used to interface the components of the project
const uint8_t right_leds=16;
const uint8_t left_leds=17;
const uint8_t num_leds=60;
const uint8_t relay_pin=12;

// create global flags that will be used to control functions
static bool touch1_active = false;
static bool touch2_active = false;
static bool touch3_active = false;
static bool touch1_letgo = true;
static bool touch2_letgo = true;
static bool touch3_letgo = true;
static bool short_press1 = true;
static bool short_press2 = true;
static bool short_press3 = true;
static bool timer1_started = false;
static bool timer2_started = false;
static bool timer3_started = false;
static bool long_press1 = true;
static bool long_press2 = true;
static bool long_press3 = true;

static bool brightness_control_flag = false;
static bool mode_animation_started = false;

static volatile bool shift_frame = false;
static bool wave_frame_timer_flag = false;
static bool wave_frame_timer_period_change_flag = false;
static bool mode_animation_random_timer_flag = false;
static bool mode_animation_random_on = true;
static bool initial_clear = false;

static bool hue_timer_change_peiod_flag = false;

// variables to hold the duration of pin that was touched
static int touch1_time, touch2_time, touch3_time;

static uint8_t hue = 0;
static uint8_t brightness = 60;
static uint8_t column_height[2][10], peak_column_height[2][10];
static uint8_t palette_index = 0;
static uint8_t current_mode = 0;

static uint8_t mode1_column_index = 0;

//create the structure that will be be used to address the leds
static CRGB leds[2][num_leds];

/* To modify the color palettes to use your own palettes,
  Modifiy the palette index increment in the "touch_func" task 
  to add or decrease the palettes. Modify the palettes declaration section
  to customize your palettes.
*/
DEFINE_GRADIENT_PALETTE(heat_colors){
  0,  0,0,255,
  127,255,0,0,
  255,255,255,40
};

static CRGBPalette16 palettes_arr[] ={
  //standard rainbow palette
  RainbowColors_p,
  //custom palette 0
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
  //custom palette 1
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
                CRGB(255, 0, 0)),
  //heat colors palette
  heat_colors            
};

/* sort the leds into rows and columns for easy addressing 
   there are 60 leds per channel for left and right,
   10 columns and 6 rows for each channel

*/
static uint8_t leds_arr[10][6];
// create a class that will store the functions used to display numbers
class numbers{
  public:
    void zero(uint8_t lr, uint8_t pos);
    void one(uint8_t lr, uint8_t pos);
    void two(uint8_t lr, uint8_t pos);
    void three(uint8_t lr, uint8_t pos);
    void four(uint8_t lr, uint8_t pos);
    void five(uint8_t lr, uint8_t pos);
    void six(uint8_t lr, uint8_t pos);
    void seven(uint8_t lr, uint8_t pos);
    void eight(uint8_t lr, uint8_t pos);
    void nine(uint8_t lr, uint8_t pos);
};
//functions used to display numbers 
void numbers :: zero(uint8_t lr, uint8_t pos){
  for(uint8_t i=0; i<5; i++){
    leds[lr][leds_arr[0+pos][i]] = CHSV(0,0,brightness);
    leds[lr][leds_arr[2+pos][i]] = CHSV(0,0,brightness);
  }
  leds[lr][leds_arr[1+pos][4]] = CHSV(0,0,brightness);
  leds[lr][leds_arr[1+pos][0]] = CHSV(0,0,brightness);
}

void numbers :: one(uint8_t lr, uint8_t pos){

  for(uint8_t i=0; i<5; i++){
    leds[lr][leds_arr[1+pos][i]] = CHSV(0,0,brightness);
  }
}

void numbers :: two(uint8_t lr, uint8_t pos){
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

void numbers :: three(uint8_t lr, uint8_t pos){
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

void numbers :: four(uint8_t lr, uint8_t pos){
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

void numbers :: five(uint8_t lr, uint8_t pos){
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

void numbers :: six(uint8_t lr, uint8_t pos){
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

void numbers :: seven(uint8_t lr, uint8_t pos){
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

void numbers :: eight(uint8_t lr, uint8_t pos){
  for(uint8_t i=0; i<3; i++){
    for(uint8_t j=0; j<5; j++){
    leds[lr][leds_arr[i+pos][j]] =CHSV(0,0,brightness);
    }
  }
  leds[lr][leds_arr[1+pos][1]]= CHSV(0,0,0);
  leds[lr][leds_arr[1+pos][3]]= CHSV(0,0,0);
}

void numbers :: nine(uint8_t lr, uint8_t pos){
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
  numbers Number_obj;
  String number_str;
  if (left_pos){
    clear_leds(0);
    number_str = (String) brightness;
    for(uint8_t i=0; i<=number_str.length(); i++){
      switch (number_str[i])
      {
        case '0':
          Number_obj.zero(0,6-i*3);
          break;
        case '1':
          Number_obj.one(0,6-i*3);
          break;
        case '2':
          Number_obj.two(0,6-i*3);
          break;
        case '3':
          Number_obj.three(0,6-i*3);
          break;
        case '4':
          Number_obj.four(0,6-i*3);
          break;
        case '5':
          Number_obj.five(0,6-i*3);
          break;
        case '6':
          Number_obj.six(0,6-i*3);
          break;
        case '7':
          Number_obj.seven(0,6-i*3);
          break;
        case '8':
          Number_obj.eight(0,6-i*3);
          break;
        case '9':
          Number_obj.nine(0,6-i*3);
          break;
      }
    }
  }
  else{
    clear_leds(1);
    number_str = (String) brightness;
    for(uint8_t i=0; i<=number_str.length(); i++){
      switch (number_str[i])
      {
        case '0':
          Number_obj.zero(1,i*3);
          break;
        case '1':
          Number_obj.one(1,i*3);
          break;
        case '2':
          Number_obj.two(1,i*3);
          break;
        case '3':
          Number_obj.three(1,i*3);
          break;
        case '4':
          Number_obj.four(1,i*3);
          break;
        case '5':
          Number_obj.five(1,i*3);
          break;
        case '6':
          Number_obj.six(1,i*3);
          break;
        case '7':
          Number_obj.seven(1,i*3);
          break;
        case '8':
          Number_obj.eight(1,i*3);
          break;
        case '9':
          Number_obj.nine(1,i*3);
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
      EEPROM.write(0, brightness);
      EEPROM.commit();
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
      EEPROM.write(0, brightness);
      EEPROM.commit();
    break;
  }
}

void mode_animation(uint8_t animation_choice){
  uint8_t mode1_column_heights[10] = {1,2,3,4,5,5,4,3,2,1};
  switch (animation_choice){
  case 1:
    //start wave frame timer for mode 1 animation
    if(wave_frame_timer_flag == false){
      xTimerStart(wave_frame_timer,0);
      wave_frame_timer_flag =true;
      Serial.println("wave frame timer started");
    }
    if(shift_frame){
      shift_frame = false;
      column_height[1][0] = mode1_column_heights[mode1_column_index];
      for(uint8_t i=0; i<column_height[1][0]; i++){
        leds[1][leds_arr[0][i]] = ColorFromPalette(palettes_arr[2],hue, brightness, LINEARBLEND);
      }
      for(uint8_t i=5; i>column_height[1][0]; i--){
        leds[1][leds_arr[0][i]] = CRGB(0,0,0);
      }
      for(uint8_t i=9; i>0; i-- ){
        column_height[1][i] = column_height[1][i-1];
      }
      for(uint8_t i=1; i<10; i++){
        for(uint8_t j=0; j<column_height[1][i]; j++){
          leds[1][leds_arr[i][j]] = ColorFromPalette(palettes_arr[2], i*9 + hue, brightness, LINEARBLEND);
        }
        for(uint8_t k=5; k>column_height[1][i]; k--){
          leds[1][leds_arr[i][k]] = CRGB(0,0,0);
        }
      }
      mode1_column_index +=1;
      if(mode1_column_index >9){
        mode1_column_index = 0;
      }
    }
  break;
  
  case 0:
    // periodically generate random peak column heights based on the mode_animation_random_timer
    if(mode_animation_random_on){
      for(uint8_t i=0; i<10; i++){
        peak_column_height[1][i] = random8(2,6);
      }
      mode_animation_random_on = false;
    }
    if(mode_animation_random_timer_flag == false){
      xTimerStart(mode_animation_random_timer,0);
      mode_animation_random_timer_flag = true;
    }
    for(uint8_t i=0; i<10; i++){
      for(uint8_t j=0; j<peak_column_height[1][i]; j++){
        leds[1][leds_arr[i][j]] = ColorFromPalette(palettes_arr[2], i*9 + hue, brightness, LINEARBLEND);
      }
      for(uint8_t k=5; k>peak_column_height[1][i]; k--){
        leds[1][leds_arr[i][k]] = CRGB(0,0,0);
      }
    }
    
  break;
  default:
    break;
  }
  vTaskDelay(10/portTICK_PERIOD_MS);
  nblendPaletteTowardPalette(palettes_arr[2],palettes_arr[palette_index],70);
  FastLED.show();
}

void mode_palette_control(uint8_t choice){
  //shows the mod and pal text when in the mode and palette controls menu
  switch (choice)
  {
  case 0:
    for(uint8_t j=0; j<10; j++){
      if(j == 9 or j == 7 or j == 6 or j == 4 or j== 3){
      for(uint8_t i=0; i<5; i++){
        leds[0][leds_arr[j][i]] = ColorFromPalette(palettes_arr[2], j*9 + hue, brightness, LINEARBLEND);
        }
      }
      if(j == 8){
        leds[0][leds_arr[j][3]] = ColorFromPalette(palettes_arr[2], j*9 + hue, brightness, LINEARBLEND);
      }
      if(j == 5 or j == 2){
        leds[0][leds_arr[j][4]] = ColorFromPalette(palettes_arr[2], j*9 + hue, brightness, LINEARBLEND);
        leds[0][leds_arr[j][0]] = ColorFromPalette(palettes_arr[2], j*9 + hue, brightness, LINEARBLEND);
      }
      if(j == 1){
        leds[0][leds_arr[j][1]] = ColorFromPalette(palettes_arr[2], j*9 + hue, brightness, LINEARBLEND);
        leds[0][leds_arr[j][2]] = ColorFromPalette(palettes_arr[2], j*9 + hue, brightness, LINEARBLEND);
        leds[0][leds_arr[j][3]] = ColorFromPalette(palettes_arr[2], j*9 + hue, brightness, LINEARBLEND);
      }
    }  

    for(uint8_t i=0; i<10; i++){
      if(i == 0 or i == 6){
        for(uint8_t j=0; j<5; j++){
          leds[1][leds_arr[i][j]] = ColorFromPalette(palettes_arr[2], i*9 + hue, brightness, LINEARBLEND);
        }
      }
      if(i == 1){
        leds[1][leds_arr[i][4]] = ColorFromPalette(palettes_arr[2], i*9 + hue, brightness, LINEARBLEND);
        leds[1][leds_arr[i][2]] = ColorFromPalette(palettes_arr[2], i*9 + hue, brightness, LINEARBLEND);
      }
      if(i == 2){
        leds[1][leds_arr[i][4]] = ColorFromPalette(palettes_arr[2], i*9 + hue, brightness, LINEARBLEND);
        leds[1][leds_arr[i][3]] = ColorFromPalette(palettes_arr[2], i*9 + hue, brightness, LINEARBLEND);
        leds[1][leds_arr[i][2]] = ColorFromPalette(palettes_arr[2], i*9 + hue, brightness, LINEARBLEND);
      }
      if(i == 3 or i == 5){
        leds[1][leds_arr[i][0]] = ColorFromPalette(palettes_arr[2], i*9 + hue, brightness, LINEARBLEND);
        leds[1][leds_arr[i][1]] = ColorFromPalette(palettes_arr[2], i*9 + hue, brightness, LINEARBLEND);
        leds[1][leds_arr[i][2]] = ColorFromPalette(palettes_arr[2], i*9 + hue, brightness, LINEARBLEND);
        leds[1][leds_arr[i][3]] = ColorFromPalette(palettes_arr[2], i*9 + hue, brightness, LINEARBLEND);
      }
      if(i == 4){
        leds[1][leds_arr[i][4]] = ColorFromPalette(palettes_arr[2], i*9 + hue, brightness, LINEARBLEND);
        leds[1][leds_arr[i][2]] = ColorFromPalette(palettes_arr[2], i*9 + hue, brightness, LINEARBLEND);
      }
      if(i ==7 or i == 8){
        leds[1][leds_arr[i][0]] = ColorFromPalette(palettes_arr[2], i*9 + hue, brightness, LINEARBLEND);
      }
    }

  break;
  
  case 1:
    // display words "mod" at the left 
    for(uint8_t j=0; j<10; j++){
      if(j == 9 or j == 7 or j == 6 or j == 4 or j== 3){
      for(uint8_t i=0; i<5; i++){
        leds[0][leds_arr[j][i]] = ColorFromPalette(palettes_arr[2], j*9 + hue, brightness, LINEARBLEND);
        }
      }
      if(j == 8){
        leds[0][leds_arr[j][3]] = ColorFromPalette(palettes_arr[2], j*9 + hue, brightness, LINEARBLEND);
      }
      if(j == 5 or j == 2){
        leds[0][leds_arr[j][4]] = ColorFromPalette(palettes_arr[2], j*9 + hue, brightness, LINEARBLEND);
        leds[0][leds_arr[j][0]] = ColorFromPalette(palettes_arr[2], j*9 + hue, brightness, LINEARBLEND);
      }
      if(j == 1){
        leds[0][leds_arr[j][1]] = ColorFromPalette(palettes_arr[2], j*9 + hue, brightness, LINEARBLEND);
        leds[0][leds_arr[j][2]] = ColorFromPalette(palettes_arr[2], j*9 + hue, brightness, LINEARBLEND);
        leds[0][leds_arr[j][3]] = ColorFromPalette(palettes_arr[2], j*9 + hue, brightness, LINEARBLEND);
      }
    }  
  break;
  }
  nblendPaletteTowardPalette(palettes_arr[2],palettes_arr[palette_index],70);
  FastLED.show();
}
void timer1_callback(TimerHandle_t pxTimer){
  // timer 1 used in brightness controls to display numbers longer when brightness is not changed
  timer1_started = true;
  brightness_control(0);
  brightness_control_flag = false;
  Serial.println("timer1 tick");
}

void timer3_callback(TimerHandle_t pxTimer){
  // timer 3 used to increment brightness every 200ms as long as the button 3 is touched more than 1 second
  timer3_started = true;
  brightness_control(1);
  Serial.println("timer 3 tick");
}

void timer2_callback(TimerHandle_t pxTimer){
  // timer 2 used to increment brightness every 200ms as long as the button 3 is touched more than 1 second
  timer2_started = true;
  brightness_control(2);
  Serial.println("timer 2 tick");
}

// function to read the touch buttons, interrupts was not working so polling is done instead
uint8_t touch_reader(){
  /*  the touch buttons are merged into a single integer for easier decision making.
      The time touched for each button is recorded to perform other functions when
      touched for a longer time
  */
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
  bool suspend_flag = true;
  bool white_lights = false;
  bool speaker_on = true;

  while(1){
    // choose functions based on the first touch value
    if(first_touch_flag ==true and touch_reader()>0){
      // raise priority of first touch func when touch is detected
      vTaskPrioritySet(NULL, 2);
      Serial.println("touch func priority raised");
      first_touch_val = touch_reader();
      first_touch_flag = false;
      Serial.print("first touch val: ");
      Serial.println(first_touch_val,BIN);

      // suspend all passive tasks when touch is detected
      vTaskSuspend(normal_vis_handle);
      vTaskSuspend(wave_vis_handle);
      vTaskSuspend(spi_handle);
      vTaskSuspend(fft_handle);
      suspend_flag = true;
      Serial.println("all passive tasks suspended");
    }
    // choose which menu to enter based on which button was touched first
    switch (first_touch_val){
    case B100:
      //brightness controls
      touch_reading = touch_reader();
      switch (touch_reading){
        case B100:
          //if brightness was changed before coming here, wait 3 seconds before showing default -+ menu
          if (brightness_control_flag == true){
            if(timer1_started == false){
              xTimerStart(touch1_timer,0);
              timer1_started = true;
              Serial.println("timer1 started");
            }
            // stop timer 2 and timer 3 to stop brightness value from incrementing periodically
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
        // + brightness
          if (timer1_started){
            xTimerStop(touch1_timer,0);
            timer1_started = false;
            Serial.println("timer1 stopped");
          }
          if(timer2_started){
            xTimerStop(touch2_timer,0);
            timer2_started = false;
          }
          if(short_press3){
            brightness_control(1);
            short_press3 = false;
            Serial.println("short press 3");
          }
          // increntment brightness every 200ms after touch3 has been touched over 1 second
          if(millis()-touch3_time>=1000){
            if(timer3_started == false){
              timer3_started = true;
              xTimerStart(touch3_timer, 0);
            }
          }
          brightness_control_flag = true;
        break;

        case B110:
        // - brightness
          if(timer1_started){
            xTimerStop(touch1_timer,0);
            timer1_started = false;
            Serial.println("timer 1 stopped");
          }
          if(timer3_started){
            xTimerStop(touch3_timer,0);
            timer3_started =false;
          }
          if(short_press2){
            brightness_control(2);
            short_press2 = false;
            Serial.println("short press 2");
          }
          // decrease brightness every 200ms after touch3 has been touched over 1 second
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
          // stop all timers used in brightness controls before exiting
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
          // if white lights are on, continue displaying white lights after exiting brightness controls
          if(white_lights){
            fill_white_lights();
          }
          brightness_control_flag = false;
          //allow the first touch value to be changed to enter other menus
          first_touch_val = 0;
          first_touch_flag = true;
        break;
      }
    break;

    case B010:
      //mode and palette controls
      touch_reading = touch_reader();
      // disable mode and palette controls when white lights are on
      if(white_lights){
        Serial.println("white lights enabled mode and pal controls disabled");
        first_touch_val = 0;
        first_touch_flag = true;
        break;
      }
      // speed up palette hue increment in menu to view palettes better
      if(hue_timer_change_peiod_flag == false){
        xTimerChangePeriod(hue_timer, pdMS_TO_TICKS(20), 0);
        hue_timer_change_peiod_flag = true;
      }
      switch (touch_reading)
      {
      case B010:
        // only update the left led array when the mode animation is running
        if(mode_animation_started){
          mode_palette_control(1);
          mode_animation(current_mode);
        }
        else{
          //show mode palette controls when no animation is running
           if(initial_clear == false){
            clear_leds(0);
            clear_leds(1);
            initial_clear = true;
          }
          mode_palette_control(0);
        }
      break;
      
      case B110:
        //mode change 
        if(short_press1){
          short_press1 = false;
          current_mode += 1;
          if(current_mode >= 2){
            current_mode = 0;
          }

          EEPROM.write(1, current_mode);
          EEPROM.commit();

          Serial.print("current mode: ");
          Serial.println(current_mode);
          // start mode animation if not started yet
          if (mode_animation_started == false){
            xTimerStart(mode_animation_timer, 0);
            mode_animation_started = true;
            Serial.println("mode animation started");
          }
          //reset the mode animation timer every time a mode is changed
          xTimerReset(mode_animation_timer,0);
        }
        /*run the mode_animation function while mode animation function is true
          mode_animation_started will become false once it times out */
        if(mode_animation_started){
          mode_animation(current_mode);
          // constantly update the left led array while animating modes for a smooth effect
          mode_palette_control(1);
        }
        //show mode palette controls when animation times out
        if(mode_animation_started == false){
          if(initial_clear == false){
            clear_leds(1);
            initial_clear = true;
          }
          mode_palette_control(0);
          Serial.println("showing mode palette controls from mode change");
        }
        
      break;

      case B011:
        if(short_press3){
          short_press3 = false;
          if(palette_index<3){
            palette_index +=1;
          }
          else{
            palette_index = 0;
          }
          EEPROM.write(2, palette_index);
          EEPROM.commit();
          Serial.print("palette index: ");
          Serial.println(palette_index);
        }
        // if animation is running, only update the left led array
        if(mode_animation_started == true){
          mode_palette_control(1);
          mode_animation(current_mode);
        }
        else{
          mode_palette_control(0);
        }
      break;

      default:
        Serial.println("exit first touch B010 menu");
        first_touch_val = 0;
        first_touch_flag = true;
        initial_clear = false;
        if(mode_animation_started){
          mode_animation_started = false;
          xTimerStop(mode_animation_timer,0);
          Serial.println("mode animation stopped");
        }
        //change the hue increment timer back to original after closing mode and palette menu
        if(hue_timer_change_peiod_flag == true){
          xTimerChangePeriod(hue_timer, pdTICKS_TO_MS(80), 0);
          hue_timer_change_peiod_flag = false;
        }
      break;
      }
    break;

    case B001:
      //white lights and speaker on off controls
      touch_reading = touch_reader();
      switch (touch_reading){
        case B111:
          //turn of visualizer for white light, touch and hold longer to turn of speaker amplifier but keep lights on
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
          // toggle speaker on/off after touching all 3 buttons for more than 3 seconds
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
      // lower touch function prority after exiting 
      if(uxTaskPriorityGet(NULL) != 1){
        vTaskPrioritySet(NULL, 1);
        Serial.println("touch func priority lowered");
      }
      
    break;
    }
    // if passive tasks were suspended and there is now touch detected, resume tasks used in the relevant modes
    if(suspend_flag and touch_reader() == 0){
      if(white_lights != true){
        vTaskResume(spi_handle);
        vTaskResume(fft_handle);
        if(current_mode == 0){
          Serial.println("mode 0");
          vTaskResume(normal_vis_handle);
          vTaskSuspend(wave_vis_handle);
          // stop the wave_frame_timer that is not used in this mode
          if(wave_frame_timer_flag){
            xTimerStop(wave_frame_timer,0);
            wave_frame_timer_flag = false;
          }
        }
        if(current_mode == 1){
          Serial.println("mode 1");
          vTaskResume(wave_vis_handle);
          vTaskSuspend(normal_vis_handle);
          // clear leds in this mode because this mode wont automatically clear leds
          clear_leds(0);
          clear_leds(1);
          // start the wave frame timer if not started yet to use in this mode
          if(wave_frame_timer_flag == false){
            xTimerStart(wave_frame_timer,0);
            wave_frame_timer_flag = true;
            Serial.println("wave timer started");
          }
        }
      suspend_flag = false;
      }
    }
    // block task for 1ms to give space for the IDLE task
    vTaskDelay(1/portTICK_PERIOD_MS);
  }
}

// wave_frame_timer to update each frame for the wave mode
void wave_frame_callback(TimerHandle_t pxTimer){
  shift_frame = true;
  Serial.println("wave timer tick");
}

// Timer that will specify how long the mode animation should run
void mode_animation_callback(TimerHandle_t pxTimer){
 mode_animation_started = false;
 initial_clear = false;
 Serial.println("mode animation timeout");
}

// generate random numbers to use in mode0 animation periodically
void mode_animation_random_callback(TimerHandle_t pxTimer){
  mode_animation_random_on = true;
  mode_animation_random_timer_flag = false;
  Serial.println("animation random tick");
}

//increment the hue every periodically milliseconds
void hue_callback(TimerHandle_t pxTimer){
  hue+=2;
}

//decay the mappping max value to use auto range for the fft output
void mapping_decay_callback(TimerHandle_t pxTimer){
  if (mapping_max >=900){
    mapping_max -= 100;
  }
}

//decay the peak column height periodically
void peak_decay_callback(TimerHandle_t pxTimer){
  if(current_mode ==0){
    for(uint8_t i=0; i<10; i++){
      if(peak_column_height[0][i]!=0){
        peak_column_height[0][i] -=1;
      }
      if (peak_column_height[1][i]!=0){
        peak_column_height[1][i] -=1;
      }
    }
  }
  if(current_mode == 1){
    if(peak_column_height[0][0]!=0){
        peak_column_height[0][0] -=1;
      }
    if (peak_column_height[1][0]!=0){
      peak_column_height[1][0] -=1;
    }
  }
}

// spi task used to communicate with external ADC

/* The external ADC used is the MCP 3002 ADC.
   It is recommended that you check out the datasheet
   before using the ADC to understand the performance of the ADC
*/

/* It recommended that you use 8MHz spi clock speed on a soldered board.
   using 8Mhz on a breadboard caused issues for me.
*/

void spi(void* parameters){
  // pointers are used here for higher speed since speed is crucial to achieve 40,000Hz sampling rate
  uint8_t *p_received_ch0, *p_received_ch1;
  p_received_ch0 = received_ch0;
  p_received_ch1 = received_ch1;

  float *p_fft_input_ch0, *p_fft_input_ch1;
  p_fft_input_ch0 = fft_input_ch0;
  p_fft_input_ch1 = fft_input_ch1;
 
  spi_out fft_buf;

  int oldTime;

  //0x70 = B11100000
  //0x60 = B11000000
  const uint8_t send_ch0= B11100000;
  const uint8_t send_ch1= B11000000;

  while(1){
    for(uint_fast16_t i=0; i<FFT_N; i++){
      oldTime =micros();
      vspi ->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
      // take ADC reading for channel 0
      digitalWrite(VSPI_CS, LOW);
      p_received_ch0[0]= vspi->transfer(send_ch0);
      p_received_ch0[1]= vspi->transfer(0);
      digitalWrite(VSPI_CS, HIGH);

      // take ADC reading for channel 1
      digitalWrite(VSPI_CS, LOW);
      p_received_ch1[0]= vspi->transfer(send_ch1);
      p_received_ch1[1]= vspi->transfer(0);
      digitalWrite(VSPI_CS, HIGH);
      vspi ->endTransaction();

      // merge the 2 bytes received from the spi output to get 10bit ADC reading
      fft_buf.spi_out_ch0[i]= word(p_received_ch0[0], p_received_ch0[1]);
      fft_buf.spi_out_ch1[i]= word(p_received_ch1[0], p_received_ch1[1]);

      //take a sample every 25 microseconds to get a sample rate of 40,000Hz
      while (micros()-oldTime<25){
      }
    }
    // send the data collected to the fft input queue to be used by the fft task
    if (xQueueSend(fft_in_queue, (void *)&fft_buf, 1/portTICK_PERIOD_MS) !=pdPASS){
      Serial.println("fft input queue buffer full");
    }
    else{
      //Serial.println("sent for processing");
    }
  }
}

// fft task to process the audio signal
void fft(void* paramerters){
  // use pointers in fft task to address global variables faster
  while(1){
    spi_out fft_buf_received;
    fft_out processed_buf, old_processed_buf;

    float *p_fft_input_ch0, *p_fft_input_ch1;
    p_fft_input_ch0 = fft_input_ch0;
    p_fft_input_ch1 = fft_input_ch1;

    float *p_fft_output_ch0, *p_fft_output_ch1;
    p_fft_output_ch0 = fft_output_ch0;
    p_fft_output_ch1 = fft_output_ch1;

    //get the data from the fft input queue 
    if(xQueueReceive(fft_in_queue, (void *)&fft_buf_received, 1/portTICK_PERIOD_MS) ==pdPASS){
      for (uint_fast16_t i = 0; i < FFT_N; i++){
        p_fft_input_ch0[i] = fft_buf_received.spi_out_ch0[i];
        p_fft_input_ch1[i] = fft_buf_received.spi_out_ch1[i];
      } 
      //fft magic for both channels
      fft_ch0.removeDC();
      fft_ch0.hammingWindow();
      fft_ch0.execute();
      fft_ch0.complexToMagnitude();

      fft_ch1.removeDC();
      fft_ch1.hammingWindow();
      fft_ch1.execute();
      fft_ch1.complexToMagnitude();  

      //process fft output
      p_fft_output_ch0[bands[0]]-=60;
      p_fft_output_ch1[bands[0]]-=60;

      // cast float values to unsigned 16bit integer for higher speed
      for(uint8_t f=0; f<2; f++){
        for(uint8_t i=0; i<10; i++){
          if(f==0){
            processed_buf.post_process[f][i] = (uint_fast16_t) p_fft_output_ch0[bands[i]];
          }
          else{
            processed_buf.post_process[f][i] = (uint_fast16_t) p_fft_output_ch1[bands[i]];
          }
        }
      }

      for(uint8_t f=0; f<2;f++){
        for (uint8_t i=0; i< 10; i++) {

          // slow down falling of lights
          if (processed_buf.post_process[f][i] < old_processed_buf.post_process[f][i]){
            processed_buf.post_process[f][i] = (processed_buf.post_process[f][i] + old_processed_buf.post_process[f][i])/2; 
          }
          
          // filter out noise picked up by fft
          if (processed_buf.post_process[f][i]>100 and processed_buf.post_process[f][i]<10000){
            // reduce the first 2 bands to make it less dominant 
            if(i==0){
              processed_buf.post_process[f][i] *=0.9;
            }
            if(i==1){
              processed_buf.post_process[f][i] *=0.9;
            }
            // amplify the last 3 bands after filtering to make it more visible
            if(i == 7){
              processed_buf.post_process[f][i] *=1.08;
            }
            if(i == 8){
              processed_buf.post_process[f][i] *=1.08;
            }
            if(i == 9){
              processed_buf.post_process[f][i] *=1.08;
            }
          }
          else{
            processed_buf.post_process[f][i]=0;
          }
          
          old_processed_buf.post_process[f][i] = processed_buf.post_process[f][i];
        }
      }
      if(xQueueSend(fft_out_queue, (void*)&processed_buf, 0) == pdPASS){
        
      }
      else{
        Serial.println("processed output buf full");
      }
    }
  }
}

// 10 band audio visualizer task
void normal_vis(void* parameters){
  while(1){
    // use pointers for higher processing speed
    fft_out  processed_buf_received;
    
    uint8_t (*p_column_height)[10], (*p_peak_column_height)[10];
    p_column_height = column_height;
    p_peak_column_height = peak_column_height;

    uint8_t (*p_leds_arr)[6];
    p_leds_arr = leds_arr;

    uint16_t *p_mapping_max;
    p_mapping_max = &mapping_max;

    CRGB (*p_leds)[num_leds];
    p_leds = leds;

    // process the fft output received from the queue to display lights
    if(xQueueReceive(fft_out_queue, (void *)&processed_buf_received,0) == pdPASS){
      // standard 10 band visualizer output 
      for(uint8_t f=0; f<2; f++){
        for (uint8_t i = 0; i < 10; i++){
          if (processed_buf_received.post_process[f][i] >= *p_mapping_max)
            {
              *p_mapping_max +=35;
              processed_buf_received.post_process[f][i] = *p_mapping_max;
            }
          p_column_height[f][i] = map(processed_buf_received.post_process[f][i], 120, *p_mapping_max, 0, 5);

          if(p_peak_column_height[f][i] <= p_column_height[f][i]){
            p_peak_column_height[f][i] = p_column_height[f][i];
          }
          
          for(int j=0; j<=p_peak_column_height[f][i]; j++){
            p_leds[f][p_leds_arr[i][j]]= ColorFromPalette(palettes_arr[palette_index], i*5+ hue, brightness, LINEARBLEND);
          }
          
          for(int k=5; k>p_peak_column_height[f][i]; k--){
            p_leds[f][p_leds_arr[i][k]]=CHSV(0,0,0);
          }
        }
      }
      FastLED.show();
    }
    else{
      Serial.println("waiting for processing");
      vTaskDelay(5/portTICK_PERIOD_MS);
    }
  }
}

// wave pattern visualizer 
void wave_vis(void* parameters){
  while(1){
    //use pointers for faster processing
    fft_out  processed_buf_received;

    uint8_t (*p_column_height)[10], (*p_peak_column_height)[10];
    p_column_height = column_height;
    p_peak_column_height = peak_column_height;

    uint8_t (*p_leds_arr)[6];
    p_leds_arr = leds_arr;

    uint16_t *p_mapping_max;
    p_mapping_max = &mapping_max;

    CRGB (*p_leds)[num_leds];
    p_leds = leds;

    if(wave_frame_timer_flag == false){
      xTimerStart(wave_frame_timer,0);
      wave_frame_timer_flag = true;
    }
    if(xQueueReceive(fft_out_queue, (void *)&processed_buf_received, 0) == pdPASS){
      //Serial.println("processed output received");
      // standard 10 band visualizer output 
      for(uint8_t f=0; f<2; f++){
        for (uint8_t i = 0; i < 10; i++){
          if (processed_buf_received.post_process[f][i] >= *p_mapping_max){
            *p_mapping_max +=35;
            processed_buf_received.post_process[f][i] = *p_mapping_max;
          }
          p_column_height[f][i] = map(processed_buf_received.post_process[f][i], 120, *p_mapping_max, 0, 5);

          //get highest column to display 
          // reuse array to use peak decay timer
          if(p_peak_column_height[f][0] <= p_column_height[f][i]){
            p_peak_column_height[f][0] = p_column_height[f][i];
          }
        }
        for(int j=0; j<=p_peak_column_height[f][0]; j++){
          p_leds[f][p_leds_arr[0][j]]= ColorFromPalette(palettes_arr[palette_index], hue, brightness, LINEARBLEND);
          }
          
        for(int k=5; k>p_peak_column_height[f][0]; k--){
          p_leds[f][p_leds_arr[0][k]]=CHSV(0,0,0);
        }
      }
      // shift the frame when the wave_frame_timer times out
      if(shift_frame){
        shift_frame = false;
        for(uint8_t i=0; i<2; i++){
          for(uint8_t j=9; j>0; j--){
            p_peak_column_height[i][j]= p_peak_column_height[i][j-1];
          }

          for(uint8_t k=1; k<10; k++){
            for(uint8_t l=0; l <= p_peak_column_height[i][k]; l++){
              p_leds[i][p_leds_arr[k][l]] = ColorFromPalette(palettes_arr[palette_index], k*5+ hue, brightness, LINEARBLEND);
            }
            for(uint8_t m=5; m > p_peak_column_height[i][k]; m--){
              p_leds[i][p_leds_arr[k][m]] = CRGB(0,0,0);
            }
          }
        }
          
      }
      FastLED.show();
    }
    
    else{
      Serial.println("waiting for processing");
      vTaskDelay(5/portTICK_PERIOD_MS);
    }
  }
}

void setup(){
  // create the queues used to store fft input and output data
  fft_in_queue = xQueueCreate(2, sizeof(spi_out));
  fft_out_queue = xQueueCreate(2,sizeof(fft_out));

  Serial.begin(115200);
  pinMode(relay_pin, OUTPUT);

   // initialize EEPROM memory size of 3 bytes;
  if(!EEPROM.begin(3)){
    Serial.println("failed to initialize EEPROM"); 
    delay(999999);
  }
  else{
    Serial.println("EEPROM initialized successfully");
  }

  //read the data stored in EEPROM
  brightness = EEPROM.read(0);
  current_mode = EEPROM.read(1);
  palette_index = EEPROM.read(2);

  //allocate memory for VSPI
  vspi = new SPIClass(VSPI);
  vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_CS);
  pinMode(VSPI_CS, OUTPUT);

  // assign the arrays that can be processed to output the led colors.
  FastLED.addLeds<WS2812, left_leds, GRB>(leds[0], num_leds);
  FastLED.addLeds<WS2812, right_leds, GRB>(leds[1], num_leds);
  FastLED.setCorrection(TypicalSMD5050);

  // initialize the led array for easy addresing
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
  
  // create the tasks that will run in free RTOS across both cores
  xTaskCreatePinnedToCore(spi,"spi",6000,NULL,1,&spi_handle,1);
  xTaskCreate(fft, "fft", 6200, NULL, 1, &fft_handle);
  xTaskCreate(normal_vis,"normal_vis",2000,NULL,1,&normal_vis_handle);
  xTaskCreate(touch_func,"touch_func", 5000, NULL, 1, &ISR_handler_handle);
  xTaskCreate(wave_vis, "wave_vis", 1500, NULL, 1, &wave_vis_handle);

  // create software timers to handle routine periodic tasks
  touch1_timer = xTimerCreate("touch1_timer", pdMS_TO_TICKS(3000), pdFALSE, (void*)1, timer1_callback);
  touch3_timer = xTimerCreate("touch3_timer", pdMS_TO_TICKS(200), pdTRUE, (void*)3 , timer3_callback);
  touch2_timer = xTimerCreate("touch2_timer", pdMS_TO_TICKS(200), pdTRUE, (void*)2, timer2_callback);
  hue_timer = xTimerCreate("hue_timer", pdMS_TO_TICKS(80), pdTRUE, (void*)4, hue_callback);
  mapping_decay_timer = xTimerCreate("mapping_decay", pdMS_TO_TICKS(250), pdTRUE, (void*)5, mapping_decay_callback);
  peak_decay_timer = xTimerCreate("peak_decay_timer", pdMS_TO_TICKS(70), pdTRUE, (void*)6, peak_decay_callback);
  wave_frame_timer = xTimerCreate("wave_frame_timer", pdMS_TO_TICKS(70), pdTRUE, (void*)7, wave_frame_callback);
  mode_animation_timer = xTimerCreate("mode_animation_timer", pdTICKS_TO_MS(3000), pdFALSE, (void*)8, mode_animation_callback);
  mode_animation_random_timer = xTimerCreate("mode_animation_random_timer", pdTICKS_TO_MS(250), pdFALSE,(void*)9,mode_animation_random_callback);

  // start the timers that will be used globally 
  xTimerStart(hue_timer, 0);
  xTimerStart(mapping_decay_timer, 0);
  xTimerStart(peak_decay_timer, 0);

  //delete the setup task
  vTaskDelete(NULL);
}

void loop(){
  //delete the loop task 
  vTaskDelete(NULL);
  }
 
 