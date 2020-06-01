/*
 * tcm8230md info:
 * http://sigalrm.blogspot.com/2011/03/tcm8230md-breakout.html
 * http://sigalrm.blogspot.com/2012/01/nrfcam.html
 * datasheet:
 * https://www.sparkfun.com/datasheets/Sensors/Imaging/TCM8230MD.pdf
 *
 * Using 
 */

// CSI_MCLK AD_B1_05
#define ECLK_PIN 41
#define RST_PIN 15

#define SCL_PIN 19
#define SDA_PIN 18

// CSI_VSYNC AD_B1_06
#define VD_PIN 17
// CSI_HSYNC AD_B1_07
#define HD_PIN 16
// CSI_PIXCLK AD_B1_04
#define DCLK_PIN 40

// CSI_DATA0X...
// AD_B1_15
#define D0_PIN 27
// AD_B1_14
#define D1_PIN 26
// AD_B1_13
#define D2_PIN 39
// AD_B1_12
#define D3_PIN 38
// AD_B1_11
#define D4_PIN 21
// AD_B1_10
#define D5_PIN 20
// AD_B1_09
#define D6_PIN 23
// AD_B1_08
#define D7_PIN 22

int pins[] = {
  VD_PIN, HD_PIN, DCLK_PIN, ECLK_PIN, RST_PIN,
  D0_PIN, D1_PIN, D2_PIN, D3_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN};
int n_pins = sizeof(pins) / sizeof(int);

#define ADDR 0x3C
#define WADDR 0x78
#define RADDR 0x79

#include <Wire.h>

#define FRAME_WIDTH 128
#define FRAME_DWIDTH 256
#define FRAME_HEIGHT 96
//#define FRAME_BYTES 12288 // 128 * 96 * 1
#define FRAME_BYTES 24576 // 128 * 96 * 2

#ifdef CHUNK_OUTPUT
#define CHUNK_SIZE 64
#define CHUNK_DELAY_US 500
#endif

byte frame[FRAME_BYTES];

volatile bool in_frame = false;
volatile bool frame_done = false;
volatile bool in_line = false;
volatile int hi = 0;
volatile int fi = 0;
volatile int pi = 0;


int read_register(byte sub_addr) {
  Wire.beginTransmission(ADDR);
  Wire.write(sub_addr);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDR, 1);
  //uint32_t t0 = millis();
  while (!Wire.available()) {
    //if (millis() - t0 > 1000) return -1;
  }
  return Wire.read();
}


void write_register(byte sub_addr, byte value) {
  Wire.beginTransmission(ADDR);
  Wire.write(sub_addr);
  Wire.write(value);
  Wire.endTransmission();
}


void setup() {

  // not sure it these are needed as pin modes are muxed below  
  pinMode(ECLK_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  digitalWriteFast(RST_PIN, LOW);
  
  pinMode(VD_PIN, INPUT);
  pinMode(HD_PIN, INPUT);
  pinMode(DCLK_PIN, INPUT);
  
  pinMode(D0_PIN, INPUT);
  pinMode(D1_PIN, INPUT);
  pinMode(D2_PIN, INPUT);
  pinMode(D3_PIN, INPUT);
  pinMode(D4_PIN, INPUT);
  pinMode(D5_PIN, INPUT);
  pinMode(D6_PIN, INPUT);
  pinMode(D7_PIN, INPUT);

  // VSYNC
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_06 = 0b100;
  // HSYNC
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_07 = 0b100;
  // DCLK
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_04 = 0b100;
  // D7-D0
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_08 = 0b100;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_09 = 0b100;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_10 = 0b100;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_11 = 0b100;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_12 = 0b100;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_13 = 0b100;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_14 = 0b100;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_15 = 0b100;

  // set CSI_MCLK IO mux: to ALT4
  CCM_CCGR2 &= (~0b1100);  // turn off csi clock
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_05 = 0b100;
  CCM_CSCDR3 = 0b0;  // set csi clock source and divider = 24 MHz
  CCM_CCGR2 = (CCM_CCGR2 & (~0b1100)) | 0b1100;  // turn on csi clock

  // start external clock
  //analogWrite(ECLK_PIN, 2);
  // DCLK will run at max 2x ECLK so
  // if ECLK is is 15 MHz, DCLK should be 30 MHz
  // since DCLK is an input, I need to keep up with
  // the 30 MHz
  // when DCLK goes high, sample the data
  // this has to be done in 33 ns 
  //eclk_timer.begin(toggle_eclk, 1);  // 0.5 MHz
  
  // wait >=100 cycles
  delay(50);
  //delayMicroseconds(50);
  
  // bring reset high
  digitalWriteFast(RST_PIN, HIGH);

  // wait >=2000 cycles until sda/scl is ready
  //delayMicroseconds(200);
  delay(100);

  Wire.begin();
  Serial.begin(9600);

  // setup imaging mode
  // https://code.google.com/archive/p/nrfcam/source/default/source
  write_register(0x02, 0x00);  // 15 fps?
  write_register(0x03, 0x22);  // RGB, sQCIFf
  // in sQCIFf: [low power] dclk 1/2
  // sQCIF = 128 x 96
  // default HSYNCSEL is 1: blanking
  // default ESRLSW is 00: short
}


void print_all_registers() {
  Serial.println("Printing out registers");
  for (byte a=0; a<95; a++) {
    Serial.print(a, HEX);
    Serial.print(" : ");
    // put your main code here, to run repeatedly:
    //Serial.println(read_register(a), HEX);
    Serial.println(read_register(a));
    delay(1);
  };
  delay(1000);
}


void print_frame() {
  // print out frame
  Serial.write("====");
  #ifdef CHUNK_OUTPUT
  int n = 0;
  while (n + CHUNK_SIZE < FRAME_BYTES) {
    Serial.write(frame + n, CHUNK_SIZE);
    delayMicroseconds(CHUNK_DELAY_US);
    n += CHUNK_SIZE;
  }
  Serial.write(frame + n, FRAME_BYTES - n);
  #else
  Serial.write(frame, FRAME_BYTES);
  #endif
  
}


void loop() {
  //toggle_eclk();
  //delay(5);
  //print_all_registers();
}

