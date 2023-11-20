//
// Simple JT65/JT9/JT4/FT8/WSPR/FSQ beacon for Arduino, with the Etherkit
// Si5351A Breakout Board, by Jason Milldrum NT7S.
//
// Transmit an abritrary message of up to 13 valid characters
// (a Type 6 message) in JT65, JT9, JT4, a type 0.0 or type 0.5 FT8 message,
// a FSQ message, or a standard Type 1 message in WSPR.
//
// Connect a momentary push button to pin 12 to use as the
// transmit trigger. Get fancy by adding your own code to trigger
// off of the time from a GPS or your PC via virtual serial.
//
// Original code based on Feld Hell beacon for Arduino by Mark
// Vandewettering K6HX, adapted for the Si5351A by Robert
// Liesenfeld AK6L <ak6l@ak6l.org>.
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//


//为了处理超过13个字符的数据，所以使用gen_ft8,参考https://github.com/kholia/Easy-FT8-Beacon-v3/blob/master/Easy-FT8-Beacon-v3/Easy-FT8-Beacon-v3.ino
//注意，使用的时候不要用中文的引号。例如“CQ BI6LTD OL83” ,看上去没有问题。但是编码后和"CQ BI6LTD OL83" 差别很大
//下面是参考部分
/*

  $ ./gen_ft8 "VU3FOE VU3CER MK68" hello.wav
  Packed data: e3 e7 74 57 1f 36 dc 96 23 08
  FSK tones: 3140652707426453641754447112052035223140652130064260456762740044355664143140652

  $ ./gen_ft8 "VU3CER VU3FOE MK68" hello.wav
  Packed data: e3 e6 db 97 1f 3b a2 96 23 08
  FSK tones: 3140652707422225641757260612052036213140652317643256157154225632575077733140652

  $ ipython
  In [1]: n = "3140652707426453641754447112052035223140652130064260456762740044355664143140652"

  In [2]: %pprint
  Pretty printing has been turned OFF

  In [3]: [int(d) for d in str(n)]
  Out[3]: [3, 1, 4, 0, 6, 5, 2, 7, 0, 7, 4, 2, 6, 4, 5, 3, 6, 4, 1, 7, 5, 4, 4, 4, 7, 1, 1, 2, 0, 5, 2, 0, 3, 5, 2, 2, 3, 1, 4, 0, 6, 5, 2, 1, 3, 0, 0, 6, 4, 2, 6, 0, 4, 5, 6, 7, 6, 2, 7, 4, 0, 0, 4, 4, 3, 5, 5, 6, 6, 4, 1, 4, 3, 1, 4, 0, 6, 5, 2]

*/
//////////////////
//delay part from https://zhuanlan.zhihu.com/p/158883598
const int relayPin=7;//定义7脚为信号引脚，控制继电器是否闭合，从而控制射频功放



// by lingshunlab.com
//https://lingshunlab.com/book/arduino/arduino-use-ds3231-read-time-and-set-time
// 加载DS3231的库
#include "RTClib.h"

// 创建rtc实例
RTC_DS3231 rtc;
///////////////////////////////////////////

#include <si5351.h>
#include <JTEncode.h>
//#include <rs_common.h>
//#include <int.h>
// #include <string.h>

#include "Wire.h"

// Mode defines

//#define FT8_TONE_SPACING       625 //625          // ~6.25 Hz
#define FT8_TONE_SPACING       312 //625          // ~3.12 Hz     //产生二次谐波，就可以到6.24hz


#define FT8_DELAY               159          // Delay value for FT8


//#define FT8_DEFAULT_FREQ        8000000UL
#define FT8_DEFAULT_FREQ        25156500UL   //产生二次谐波，就可以到50.313Mhz

#define DEFAULT_MODE            MODE_FT8

// Hardware defines
#define BUTTON                  12
#define LED_PIN                 13

// Enumerations
enum mode {MODE_JT9, MODE_JT65, MODE_JT4, MODE_WSPR, MODE_FSQ_2, MODE_FSQ_3,
  MODE_FSQ_4_5, MODE_FSQ_6, MODE_FT8};

// Class instantiation
Si5351 si5351;
JTEncode jtencode;

// Global variables
unsigned long freq;
char message[] = "CQ BI6LTD OL83";
char call[] = "BI6LTD";
char loc[] = "OL83";
uint8_t dbm = 27;
uint8_t tx_buffer[255];
enum mode cur_mode = DEFAULT_MODE;
uint8_t symbol_count;
uint16_t tone_delay, tone_spacing;

uint8_t fixed_buffer[] = {3, 1, 4, 0, 6, 5, 2, 5, 2, 4, 0, 7, 4, 2, 5, 4, 5, 6, 6, 3, 1, 7, 3, 2, 0, 3, 1, 5, 7, 0, 2, 4, 2, 3, 4, 2, 3, 1, 4, 0, 6, 5, 2, 0, 7, 7, 5, 0, 1, 5, 4, 4, 7, 4, 1, 1, 4, 3, 1, 5, 5, 5, 1, 4, 4, 1, 0, 5, 6, 0, 4, 5, 3, 1, 4, 0, 6, 5, 2};


// Loop through the string, transmitting one character at a time.
void encode()
{


  
  uint8_t i;

  // Reset the tone to the base frequency and turn on the output
  si5351.output_enable(SI5351_CLK1, 1);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(relayPin,HIGH);//输出高电平，打开继电器

  // Now transmit the channel symbols

  for(i = 0; i < symbol_count; i++)
  {
      //si5351.set_freq((freq * 100) + (tx_buffer[i] * tone_spacing), SI5351_CLK1);
      si5351.set_freq((freq * 100) + (fixed_buffer[i] * tone_spacing), SI5351_CLK1);
      delay(tone_delay);
     // Serial.println(int(tx_buffer[i]));
  }

  // Turn off the output
  si5351.output_enable(SI5351_CLK1, 0);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(relayPin,LOW);//输出低电平，关闭继电器
  
}

void set_tx_buffer()
{
  // Clear out the transmit buffer
  memset(tx_buffer, 0, 255);

  // Set the proper frequency and timer CTC depending on mode
  switch(cur_mode)
  {
  
  case MODE_FT8:
    jtencode.ft8_encode(message, tx_buffer);
    break;
  
  }
}

void setup()
{
  ///////////////////////// 设置控制继电器的端口
  pinMode(relayPin,OUTPUT);//定义输出端口

  
  /////////////////////////////检查si5351是否连好
  bool i2c_found;

  // Start serial and initialize the Si5351
  Serial.begin(9600);
  i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  if(!i2c_found)
  {
    Serial.println("Device not found on I2C bus!");
  }
  //////////////////////////////


  
  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other
  // than 25 MHz
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);

  // Use the Arduino's on-board LED as a keying indicator.
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Use a button connected to pin 12 as a transmit trigger
  pinMode(BUTTON, INPUT_PULLUP);

  // Set the mode to use
  cur_mode = MODE_FT8;

  // Set the proper frequency, tone spacing, symbol count, and
  // tone delay depending on mode
  switch(cur_mode)
  {


  case MODE_FT8:
    freq = FT8_DEFAULT_FREQ;
    symbol_count = FT8_SYMBOL_COUNT; // From the library defines
    tone_spacing = FT8_TONE_SPACING;
    tone_delay = FT8_DELAY;
    break;
  
  }

  // Set CLK1 output
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA); // Set for max power if desired
  si5351.output_enable(SI5351_CLK1, 0); // Disable the clock initially

  // Encode the message in the transmit buffer
  // This is RAM intensive and should be done separately from other subroutines
  set_tx_buffer();
 // Serial.begin(9600);
 Serial.begin(9600);


//////////////////////////////////
  // 初始化rtc，
  if (! rtc.begin()) { // 若果初始化失败，则
    Serial.println("Couldn't find RTC");
    Serial.flush(); 
    abort();  // 程序停止运行
  }
//////////////////////////////////

/*
 * init(uint8_t xtal_load_c, uint32_t ref_osc_freq, int32_t corr)
 *
 * Setup communications to the Si5351 and set the crystal
 * load capacitance.
 *
 * xtal_load_c - Crystal load capacitance. Use the SI5351_CRYSTAL_LOAD_*PF
 * defines in the header file
 * xo_freq - Crystal/reference oscillator frequency in 1 Hz increments.
 * Defaults to 25000000 if a 0 is used here.
 * corr - Frequency correction constant in parts-per-billion
 *
 * Returns a boolean that indicates whether a device was found on the desired
 * I2C address.
 *
 */

  
}

void loop()
{
  // Debounce the button and trigger TX on push
 //去抖动按钮12并在按下时触发 TX,否则不会工作
 //去抖动按钮12并在按下时触发 TX,否则不会工作
 ///////////////////////////
  // 获取DS3231的时间
  DateTime now = rtc.now();
  Serial.println(now.second());
 ///////////////////////////
  if((now.second()== 15) or (now.second()==45))
  {


      encode();
      delay(50); //delay to avoid extra triggers


  }
}
