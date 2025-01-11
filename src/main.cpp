// Copyright(c) 2025 washishi
#include <M5Unified.h>
#include "fft.hpp"
#include <cinttypes>
#include "FastLED.h"
#ifdef ECHO_BASE
#include <M5EchoBase.h>
M5EchoBase echobase(I2S_NUM_0);
#endif

#define LED_GPIO 27 // ATOM Matrix RGB-LED
CRGB leds[25];
// 点灯時のLEDの色指定
CRGB led_table[5][5] = {
    {0x00FF00,0x00FF00,0x00FF00,0xFFFF00,0xFF0000},
    {0x008000,0x008000,0x008000,0xFFE42D,0xFF6347},
    {0x32CD32,0x32CD32,0x32CD32,0xFFD700,0xFF7F50},
    {0x32CD32,0x32CD32,0x32CD32,0xFFA500,0xFA8072},
    {0x90EE90,0x90EE90,0x90EE90,0xFF9D2A,0xFFB6C1}};

uint8_t angle = 0;    // 表示の向き
bool reverse = false; // 周波数レンジ方向 false:低→高 true:高→低

// LED の順番 (M5Atom Matrix)  
// LED0  LED1  LED2  LED3  LED4
// LED5  LED6  LED7  LED8  LED9
// LED10 LED11 LED12 LED13 LED14
// LED15 LED16 LED17 LED18 LED19
// LED20 LED21 LED22 LED23 LED24

// led表示用配列
// led[angle(0-3)][line(0-4)][No(0-4)]
uint8_t led[4][5][5]={
  {{20,15,10,5,0}, // level:↑ Freq:→ 
   {21,16,11,6,1},
   {22,17,12,7,2},
   {23,18,13,8,3},
   {24,19,14,9,4}},
  {{0,1,2,3,4}, // level:→ Freq:↓
   {5,6,7,8,9},
   {10,11,12,13,14},
   {15,16,17,18,19},
   {20,21,22,23,24}},
  {{4,9,14,19,24}, // level:↓ Freq:←
   {3,8,13,18,23},
   {2,7,12,17,22},
   {1,6,11,16,21},
   {0,5,10,15,20}},
  {{24,23,22,21,20}, // level:← Freq:↑
   {19,18,17,16,15},
   {14,13,12,11,10},
   {9,8,7,6,5},
   {4,3,2,1,0}}
};

void turn_off_led() {
  // Now turn the LED off, then pause
  for(int i=0;i<25;i++) leds[i] = CRGB::Black;
  FastLED.show();
}

void fill_led_buff(CRGB color) {
  // Now turn the LED off, then pause
  for(int i=0;i<25;i++) leds[i] =  color;
}

void clear_led_buff() {
  // Now turn the LED off, then pause
  for(int i=0;i<25;i++) leds[i] =  CRGB::Black;
}

void level_led(uint8_t *led_level) {
  clear_led_buff();
  for (int i=0;i<5;i++){  
    if(led_level[i] > 5) led_level[i] = 5;
    for(int j=0;j<led_level[i];j++){
      if(reverse){
        leds[led[angle][4-i][j]] = led_table[i][j];
      } else {
        leds[led[angle][i][j]] = led_table[i][j];
      }
    }
  }
  FastLED.show();
}

  #define READ_LEN    (2 * 256)
  #define LEVEL_MAX 10.0f
  static fft_t fft;
  static constexpr size_t WAVE_SIZE = 256 * 2;
  static constexpr const size_t record_samplerate = 16000;
  static int16_t *rec_data;
  float level_max = LEVEL_MAX;
  uint8_t level_shift = 8;
  uint32_t last_max_time_msec = 0;

void level_check() {
  uint64_t level;
  uint8_t led_level[5];
  uint8_t segment_size = 128 / 5;
  float ratio;
  float ratio_max = 0;
  
#ifdef ECHO_BASE
  size_t bytes_read = 0;
  if (i2s_read(I2S_NUM_0, rec_data, WAVE_SIZE, &bytes_read, echobase.getDuration(WAVE_SIZE)) == ESP_OK){
#else
  if ( M5.Mic.record(rec_data, WAVE_SIZE, record_samplerate)){
#endif
    fft.exec(rec_data);
    for (uint8_t i=0;i<5;i++){
      level = 0;
      for (uint8_t j=i*segment_size;j<(i+1)*segment_size;j++){
        int32_t f = fft.get(j);
        level += abs(f);
      }
      uint32_t temp_level = level / segment_size;
      ratio = (float)(temp_level / level_max);
      if (ratio > ratio_max) ratio_max = ratio;
      if (ratio > 5.0f) ratio = 5.0f;
      led_level[i] = (uint8_t)(ratio);
//    led_level[i] = 5; // LEDの色調整用 コメントを外すと全点灯します
    }
    level_led(led_level);
    
// 自動レベル調整
    if (ratio_max <= 5.0f) {      
      if ((lgfx::v1::millis() - last_max_time_msec) > 3000) {
        // 3秒以上一定以下の場合は上限を下げる
        if (level_max > 0){
          level_max -= 10.0f;
          last_max_time_msec = lgfx::v1::millis();
        } 
      }
    } else {
      if (ratio_max > 5.0f){
        // 上限を大きく超えている場合は上限を上げる
        level_max += 10.0f;
      }      
      last_max_time_msec = lgfx::v1::millis(); 
    }
    //M5_LOGI("%f %f %d\n",ratio_max,level_max,lgfx::v1::millis() - last_max_time_msec);
  }
}

void setup()
{
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Log.setLogLevel(m5::log_target_serial, ESP_LOG_INFO);
  M5.Log.setEnableColor(m5::log_target_serial, false);
#ifdef ECHO_BASE
  echobase.init(16000 /*Sample Rate*/, 25 /*I2C SDA*/, 21 /*I2C SCL*/, 23 /*I2S DIN*/, 19 /*I2S WS*/, 22 /*I2S DOUT*/, 33 /*I2S BCK*/, Wire);
  i2s_set_clk(I2S_NUM_0,16000,I2S_BITS_PER_CHAN_16BIT,I2S_CHANNEL_MONO);  // EchoBaseのライブラリのチャンネル形式が異なるため変更
  echobase.setMicGain(ES8311_MIC_GAIN_6DB);  // Set microphone gain to 6dB.
#else
  auto mic_cfg = M5.Mic.config();
  mic_cfg.sample_rate = 16000;
#if defined( PDM_PORTA )
  mic_cfg.pin_ws = 32;
  mic_cfg.pin_data_in = 26;
#endif
#if defined( PDM_GPIO19_22 )
  mic_cfg.pin_ws = 22;
  mic_cfg.pin_data_in = 19;
#endif
  M5.Mic.config(mic_cfg);
  M5.Mic.begin();
#endif
  rec_data = (typeof(rec_data))heap_caps_malloc(WAVE_SIZE * sizeof(int16_t), MALLOC_CAP_8BIT);
  memset(rec_data, 0 , WAVE_SIZE * sizeof(int16_t));
  FastLED.addLeds<WS2812B, LED_GPIO, GRB>(leds, 25);  // GRB ordering is typical
  FastLED.setBrightness(10);
  turn_off_led();
  FastLED.show();
  delay(500);
}

void loop()
{
  M5.update();
  if (M5.BtnA.wasReleasedAfterHold()) {
    reverse = ! reverse;
  } else {
    if (M5.BtnA.wasClicked()) {
      angle++;
      if (angle > 3) angle=0;
    }
  }
  if (M5.BtnPWR.wasClicked()) {
#ifdef ARDUINO
    esp_restart();
#endif
  } 
  level_check();
  lgfx::v1::delay(1);
}
