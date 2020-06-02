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

#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define FB_WIDTH 640
#define FB_HEIGHT 512
/*
#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240
#define FB_WIDTH 320
#define FB_HEIGHT 256
*/
//#define FRAME_BYTES 12288 // 128 * 96 * 1
//#define FRAME_BYTES 24576 // 128 * 96 * 2
uint32_t frame_bytes = FRAME_WIDTH * FRAME_HEIGHT * 2;

#define FB_COUNT 2
//static uint16_t fb[FB_COUNT][FRAME_HEIGHT][FRAME_WIDTH] DMAMEM __attribute__ ((aligned (64)));
//static uint16_t fb[FB_COUNT][FB_HEIGHT][FB_WIDTH] __attribute__ ((aligned (64)));
//static uint16_t fb[FB_COUNT][FB_HEIGHT][FB_WIDTH] __attribute__ ((aligned (64)));
static uint32_t *fb = (uint32_t *)(0x70000000U);

#ifdef CHUNK_OUTPUT
#define CHUNK_SIZE 64
#define CHUNK_DELAY_US 500
#endif


uint32_t csisr_state = 0;


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


#define PV(N) Serial.print(#N); Serial.print(": "); Serial.println(N, HEX);
void print_csi_registers() {
  Serial.println("==== CSI ====");
  PV(CSI_CSICR1)
  PV(CSI_CSICR2)
  PV(CSI_CSICR3)
  PV(CSI_CSISTATFIFO)
  PV(CSI_CSIRFIFO)
  PV(CSI_CSIRXCNT)
  PV(CSI_CSISR)
  PV(CSI_CSIDMASA_STATFIFO)
  PV(CSI_CSIDMATS_STATFIFO)
  PV(CSI_CSIDMASA_FB1)
  PV(CSI_CSIDMASA_FB2)
  PV(CSI_CSIFBUF_PARA)
  PV(CSI_CSIIMAG_PARA)
  PV(CSI_CSICR18)
  PV(CSI_CSICR19)
  PV(CSI_CSIRFIFO)
}


void init_sensor_clk() {
  // set CSI_MCLK IO mux: to ALT4
  CCM_CCGR2 &= (~0b1100);  // turn off csi clock
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_05 = 0b100;
  CCM_CSCDR3 = 0b0;  // set csi clock source and divider = 24 MHz
  CCM_CCGR2 = (CCM_CCGR2 & (~0b1100)) | 0b1100;  // turn on csi clock
}


void init_sensor() {
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

  // setup imaging mode
  // https://code.google.com/archive/p/nrfcam/source/default/source
  // 0x02: (default: 0x40)
  //   7: FPS: 0 (30 fps) 1 (15 fps)
  //   6: ACF: 0 (50 Hz) 1 (60 Hz)
  //   5-2: reserved
  //   1: DCLKP: 0 (normal) 1 (reversed)
  //   0: ACFDET: 0 (auto) 1 (manual)
  //write_register(0x02, 0x00);  // 50 Hz 15 fps
  write_register(0x02, 0xC0);  // 60 Hz 30 fps

  // 0x03: (default: 0x80)
  //   7: DOUTSW: 0 (on) 1 (off)
  //   6: DATAHZ: 0 (out) 1 (hi-z)
  //   5-2: PICSIZ
  //     0: VGA
  //     1: QVGA
  //     2: QVGA(z)
  //     3: QQVGA(f)
  //     4: QQVGA(z)
  //     5: CIF(f)
  //     6: QCIF(f)
  //     7: QCIF(z)
  //     8: subQCIF(f)
  //     9: subQCIF(z)
  //   1: PICFMT: 0 (YUV422) 1 (RGB565)
  //   0: CM: 0 (COLOR) 1 (BW)
  //write_register(0x03, 0x22);  // RGB, sQCIFf
  write_register(0x03, 0x02);  // RGB, VGA
  //write_register(0x03, 0x06);  // RGB, QVGA
  // in sQCIFf: [low power] dclk 1/2
  // sQCIF = 128 x 96
  // default HSYNCSEL is 1: blanking
  // default ESRLSW is 00: short
}


void setup() {
  Serial.begin(9600);
  Wire.begin();
  while (!Serial);
  delay(2000);
  print_csi_registers();

  // not sure it these are needed as pin modes are muxed below  
  //pinMode(ECLK_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  digitalWriteFast(RST_PIN, LOW);
  
  /*
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
  */

  // VSYNC
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_06 = 0x4U;
  IOMUXC_CSI_VSYNC_SELECT_INPUT = 0x1U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_06 = 0x0U;
  // HSYNC
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_07 = 0x4U;
  IOMUXC_CSI_HSYNC_SELECT_INPUT = 0x1U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_07 = 0x0U;
  // DCLK
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_04 = 0x4U;
  IOMUXC_CSI_PIXCLK_SELECT_INPUT = 0x0U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_04 = 0x0U;
  // D7-D0
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_08 = 0x4U;
  IOMUXC_CSI_DATA09_SELECT_INPUT = 0x0U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_08 = 0x0U;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_09 = 0x4U;
  IOMUXC_CSI_DATA08_SELECT_INPUT = 0x0U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_09 = 0x0U;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_10 = 0x4U;
  IOMUXC_CSI_DATA07_SELECT_INPUT = 0x0U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_10 = 0x0U;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_11 = 0x4U;
  IOMUXC_CSI_DATA06_SELECT_INPUT = 0x0U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_11 = 0x0U;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_12 = 0x4U;
  IOMUXC_CSI_DATA05_SELECT_INPUT = 0x0U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_12 = 0x0U;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_13 = 0x4U;
  IOMUXC_CSI_DATA04_SELECT_INPUT = 0x0U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_13 = 0x0U;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_14 = 0x4U;
  IOMUXC_CSI_DATA03_SELECT_INPUT = 0x0U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_14 = 0x0U;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_15 = 0x4U;
  IOMUXC_CSI_DATA02_SELECT_INPUT = 0x0U;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_15 = 0x0U;

  init_sensor_clk();

  // clear framebuffers
  //memset(fb, 0, sizeof(fb));
  memset(fb, 0, FB_COUNT * FB_WIDTH * FB_HEIGHT);
  // setup CSI
  // define configuration
  // clear framebuffer(s)
  //
  // CSIDMASA-FB1 : address of FB1 (aligned double word)
  // CSIDMASA-FB2 : address of FB2 (aligned double word)
  // CSIFBUF_PARA : set stride of fb
  // CSIIMAG_PARA : width & height of fb
  // CSICR3 : set RxFF_LEVEL and DMA_REQ_EN_RFF bits
  // run in gated clock mode (pixclk, hsync and vsync used)
  CSI_CSICR1 |= (
    //(1 << 19) |  // FB1_DMA_DONE_INTEN
    //(1 << 20) |  // FB2_DMA_DONE_INTEN
    (1 << 8) |  // FCC_MASK
    //(1 << 16) |  // SOF_INTEN  enable below?
    //(1 << 17) |  // SOF_POL
    //(1 << 18) |  // RXFF_INTEN  enable below?
    //(1 << 30) |  // EXT_VSYNC  does anything?
    (1 << 11) |  // HSYNC_POL
    (1 << 4));  // GCLK_MODE
  // CSI_CSIFBUF_PARA: stride
  CSI_CSIIMAG_PARA = (
    (FRAME_WIDTH << 17) |  // * 2 and shift 16
    (FRAME_HEIGHT));
  CSI_CSICR3 |= (
    (1 << 12));// |  // DMA_REQ_EN_RFF
    //1);  // CSI_CSICR3_ECC_AUTO_EN_MASK;
  CSI_CSICR2 |= (0b11 << 30);  // DMA_BURST_TYPE_RFF(3)
  CSI_CSICR3 = (CSI_CSICR3 & ~(0b111 << 4)) | (0b010 << 4); // RxFF_LEVEL
  Serial.print("Reflash fifo dma...");
  CSI_CSICR3 |= (1 << 14);  // CSI_CSICR3_DMA_REFLASH_RFF_MASK;
  while (CSI_CSICR3 & (1 << 14));
  Serial.println("done");

  init_sensor();

  // write to memory from second completed frame
  CSI_CSICR18 = (CSI_CSICR18 & ~(0b11 << 18)) | (0b10 << 18);  // CSI_CSICR18_MASK_OPTION(2)
  //CSI_CSICR18 = (CSI_CSICR18 & ~CSI_CSICR18_MASK_OPTION_MASK) | CSI_CSICR18_MASK_OPTION(2)

  // CSI_CSICR2 skip count?
  CSI_CSIDMASA_FB1 = (uint32_t)(fb);
  CSI_CSIDMASA_FB2 = (uint32_t)(fb + (FB_HEIGHT * FB_WIDTH));

  Serial.print("Reflash fifo dma...");
  CSI_CSICR3 |= (1 << 14);  // CSI_CSICR3_DMA_REFLASH_RFF_MASK;
  while (CSI_CSICR3 & (1 << 14));
  Serial.println("done");

  Serial.println("Before enable");
  print_csi_registers();

  // enable interrupts 
  csisr_state = CSI_CSISR;
  CSI_CSICR1 |= (
    (1 << 19) |  // FB1_DMA_DONE_INTEN
    (1 << 20));  // FB2_DMA_DONE_INTEN
  Serial.print("Reflash fifo dma...");
  CSI_CSICR3 |= (1 << 14);  // CSI_CSICR3_DMA_REFLASH_RFF_MASK;
  while (CSI_CSICR3 & (1 << 14));
  Serial.println("done");
  CSI_CSICR18 |= (1 << 31);  // CSI_CSICR18_CSI_ENABLE_MASK;

  /*
  CSI_CSICR18 |= (1 << 31);  // CSI_ENABLE
  CSI_CSICR3 |= ( 
    (1 << 12) |  // DMA_REQ_EN_RFF
    //(1 << 3) |  // TWO_8BIT_SENSOR 16 bit sensor
    (0b111 << 4));  // RxFF_LEVEL (96 double words)
  //CSI_CSICR18 |= (1 << 31);  // CSI_ENABLE
  // CSICR3 : set DMA_REFLASH_RFF to restart DMA controller
  CSI_CSICR3 |= (
    (1 << 14));  // DMA_REFLASH_RFF

  // watch for:
  // CSISR : DMA_TSF_DONE_FB1/2 done bits (or interrupt if CSICR1 set)
  // CSICR3 : set DMA_REFLASH_RFF to restart DMA controller
  // FRAME_COUNT = CSI_CSICR3 >> 16
  // status register: CSI_CSISR...
  */
}

/*
Enable clock
CSI_Reset (software reset?)
Set CSICR1
  work mode (GCLK_MODE)
  polarity flags
  CSI_CSICR1_FCC_MASK
Set CSI_CSIIMAG_PARA: width (in bytes), height
Set CSI_CSIBUF_PARA: 0
Enable ECC (CSICR3)
Set burst type CSICR2/3:
  CSICR2: CSI_CSICR2_DMA_BURST_TYPE_RFF(3U);
  CSICR3: (CSI->CSICR3 & ~CSI_CSICR3_RxFF_LEVEL_MASK) | ((2U << CSI_CSICR3_RxFF_LEVEL_SHIFT));
Reflash fifo dma:
  CSICR3 |= CSI_CSICR3_DMA_REFLASH_RFF_MASK
  wait till done: while (CSICR3 & CSI_CSICR3_DMA_REFLASH_RFF_MASK);

start device

start CSI:
Write to memory from second completed frame.
  base->CSICR18 = (base->CSICR18 & ~CSI_CSICR18_MASK_OPTION_MASK) | CSI_CSICR18_MASK_OPTION(2);
Load FB addresses
Reflash fifo dma:
  CSICR3 |= CSI_CSICR3_DMA_REFLASH_RFF_MASK
  wait till done: while (CSICR3 & CSI_CSICR3_DMA_REFLASH_RFF_MASK);
Enable interrupts
Start
  enable FIFO dma (again?)
  set start bit
*/


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


void print_frame(int i) {
  Serial.write("====");
  Serial.write((uint8_t*)(fb + (i * FB_WIDTH * FB_HEIGHT)), frame_bytes);
  /*
  for (int fbi=0; fbi<FB_COUNT; fbi++) {
    Serial.print("=="); Serial.print(fbi); Serial.println("==");
    for (int r=25; r<50; r++) {
      for (int c=25; c<50; c++) {
        Serial.print(fb[fbi][r][c], HEX);
      };
      Serial.println();
    };
  };
  */
  /*
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
  */
}

/*
void fbs(int i, uint16_t T) {
  for (int r=25; r<50; r++) {
    for (int c=25; c<50; c++) {
      Serial.print(fb[i][r][c], HEX);
    };
    Serial.println();
  };
};
*/


uint32_t last_fc = 0;
elapsedMillis fc_timer = 0;

elapsedMillis fd_timer = 0;

void loop() {
  //toggle_eclk();
  //print_all_registers();
  /*
  Serial.print("CSI_CSICR18: "); Serial.println(CSI_CSICR18, HEX);
  Serial.print("CSI_CSICR19: "); Serial.println(CSI_CSICR19, HEX);
  Serial.print("CSISR: "); Serial.println(CSI_CSISR, HEX);
  Serial.print("CSICR3: "); Serial.println(CSI_CSICR3, HEX);
  Serial.print("Frame count: "); Serial.println(CSI_CSICR3 >> 16, HEX);
  Serial.print("FB0: "); Serial.println(fb[0][50][50], HEX);
  Serial.print("FB1: "); Serial.println(fb[1][50][50], HEX);
  delay(10);
  */
  if (true & (fc_timer >= 1000)) {
    uint32_t fc = CSI_CSICR3 >> 16;
    Serial.print("FPS: "); Serial.println(fc - last_fc);
    last_fc = fc;
    fc_timer -= 1000;
  };
  if (false & (fd_timer > 30)) {
    //Serial.println("==");
    print_frame(0);
    //fbs(1, 128);
    fd_timer = 0;
  };
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'f') {
      print_frame(0);
    } else if (c == 'F') {
      print_frame(1);
    };
  };
  /*
  if (CSI_CSICR3 >> 16) {
    Serial.print("Frame count: "); Serial.println(CSI_CSICR3 >> 16, HEX);
    Serial.println(fb[0][50][50]);
    Serial.println(fb[1][50][50]);
    delay(2000);
  };
  */
  /*
  if (csisr_state != CSI_CSISR) {
    Serial.print("CSI_CSISR changes: old: ");
    Serial.print(csisr_state);
    Serial.print(" new: ");
    Serial.print(CSI_CSISR);
    Serial.print(" [");
    Serial.print(millis());
    Serial.println("]");
    csisr_state = CSI_CSISR;
  };
  */
  //print_csi_registers(); 
  //delay(500);
}

