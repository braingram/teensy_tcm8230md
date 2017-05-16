/*
 * DMAChannel.h: https://github.com/PaulStoffregen/cores/blob/master/teensy3/DMAChannel.h
 * tcm8230md info:
 * http://sigalrm.blogspot.com/2011/03/tcm8230md-breakout.html
 * http://sigalrm.blogspot.com/2012/01/nrfcam.html
 * datasheet:
 * https://www.sparkfun.com/datasheets/Sensors/Imaging/TCM8230MD.pdf
 * gigantic K66 family reference
 * http://www.nxp.com/assets/documents/data/en/reference-manuals/K66P144M180SF5RMV2.pdf
 * magic dma driven by gpio!
 * https://forum.pjrc.com/archive/index.php/t-31566.html
 */
#define ECLK_PIN 23
#define RST_PIN 22

#define SCL_PIN 19
#define SDA_PIN 18

#define VD_PIN 0  // B16
#define HD_PIN 1  // B17
#define DCLK_PIN 17  // B1


// switch to D0:7
#define D0_PIN 2  // D0
#define D1_PIN 14  // D1
#define D2_PIN 7  // D2
#define D3_PIN 8  // D3
#define D4_PIN 6  // D4
#define D5_PIN 20  // D5
#define D6_PIN 21  // D6
#define D7_PIN 5  // D7

#define ADDR 0x3C
#define WADDR 0x78
#define RADDR 0x79


#include <Wire.h>
#include "DMAChannel.h"

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

DMAChannel dma;

byte read_register(byte sub_addr) {
  Wire.beginTransmission(ADDR);
  Wire.write(sub_addr);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDR, 1);
  while (!Wire.available()) {
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

  analogWrite(ECLK_PIN, 0);

  // start external clock
  // 50% duty cycle 24.54 MHz not doable, try 15 MHz
  analogWriteResolution(2);
  analogWriteFrequency(ECLK_PIN, 11000000);
  //analogWrite(ECLK_PIN, 2);
  analogWrite(ECLK_PIN, 2);
  // DCLK will run at max 2x ECLK so
  // if ECLK is is 15 MHz, DCLK should be 30 MHz
  // since DCLK is an input, I need to keep up with
  // the 30 MHz
  // when DCLK goes high, sample the data
  // this has to be done in 33 ns 
  
  // wait >=100 cycles
  delay(5);
  //delayMicroseconds(50);
  
  // bring reset high
  digitalWriteFast(RST_PIN, HIGH);
  // wait >=2000 cycles until sda/scl is ready
  //delayMicroseconds(200);
  delay(4);

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

  /*
  attachInterrupt(VD_PIN, vd_rising, RISING);
  attachInterrupt(VD_PIN, vd_falling, FALLING);
  attachInterrupt(HD_PIN, hd_rising, RISING);
  attachInterrupt(HD_PIN, hd_falling, FALLING);
  attachInterrupt(DCLK_PIN, dclk_int, RISING);  
  */
  // now drop frequency to make reading easier
  //analogWriteFrequency(ECLK_PIN, 5000000);
  //analogWrite(ECLK_PIN, 2);

  dma.destinationBuffer(frame, FRAME_DWIDTH);
  dma.source(GPIOD_PDIR);
  dma.transferSize(1);
  dma.transferCount(FRAME_DWIDTH);
  PORTB_PCR1 |= PORT_PCR_IRQC(2); // set dma mux to 17 (b1)
  dma.triggerAtHardwareEvent(DMAMUX_SOURCE_PORTB);  // PORTB
  dma.interruptAtCompletion();
  dma.attachInterrupt(dma_done);
}

void print_all_registers() {
  for (byte a=0; a<95; a++) {
    Serial.print(a, HEX);
    Serial.print(" : ");
    // put your main code here, to run repeatedly:
    Serial.println(read_register(a), HEX);
    delay(1);
  };
  delay(1000);
}

void vd_int() {
  hi = 0;
  attachInterrupt(HD_PIN, hd_int, RISING);
}

void hd_int() {
  dma.enable();
  /*
  //noInterrupts();
  attachInterrupt(DCLK_PIN, dclk_int, RISING);
  hi += 1;
  fi = 0;
  //in_line = true;

  //interrupts();
  */
}

void dma_done() {
  hi += 1;
  dma.disable();
  dma.clearComplete();
  if (hi == FRAME_HEIGHT) {
    detachInterrupt(HD_PIN);
    detachInterrupt(VD_PIN);
    // last row
    frame_done = true;
  } else {
    dma.destinationBuffer(frame + hi * FRAME_DWIDTH, FRAME_DWIDTH);
  };
  dma.clearInterrupt();
}

void dclk_int() {
  /*
  dma.triggerManual();
  if (dma.complete()) detachInterrupt(DCLK_PIN);
  */
  /*
  //noInterrupts();
  frame[pi] = GPIOD_PDIR & 0x1F;
  pi += 1;
  fi += 1;
  if (fi == FRAME_DWIDTH) {
    detachInterrupt(DCLK_PIN);
  }
  */
  /*frame[fi] = GPIOD_PDIR & 0xFF;
  fi += 1;
  if (fi == FRAME_BYTES) {
    detachInterrupt(DCLK_PIN);
    detachInterrupt(HD_PIN);
  }
  if (!(GPIOB_PDIR & 0x02)) {
    in_line = false;
    detachInterrupt(DCLK_PIN);
  }
  */
  //interrupts();
}

void read_frame() {
  // wait for frame to finish
  while (digitalReadFast(VD_PIN) == HIGH) {};
  // wait for VD to go high, new frame
  while (digitalReadFast(VD_PIN) == LOW) {};
  int c=0;
  for (int i=0; i<FRAME_HEIGHT; i++) {
    // wait for HD to go high, new line
    while (digitalReadFast(HD_PIN) == LOW) {};
    noInterrupts();
    for (int j=0; j<FRAME_WIDTH; j++) {
      // wait for DCLK to go high
      while (digitalReadFast(DCLK_PIN) == LOW) {};
      
      //frame[o + j] = digitalReadFast(D2_PIN);
      frame[c] = GPIOD_PDIR & 0xFF;
      //frame[c] = GPIOD_PDIR & 0x1F;  // just grab 1 color
      c += 1;
      while (digitalReadFast(DCLK_PIN) == HIGH) {};
      while (digitalReadFast(DCLK_PIN) == LOW) {};
      frame[c] = GPIOD_PDIR & 0xFF;
      c += 1;
      while (digitalReadFast(DCLK_PIN) == HIGH) {};
    };
    interrupts();
    // wait for HD to go low, end of line
    while (digitalReadFast(HD_PIN) == HIGH) {};
  }
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

void start_capture() {
  frame_done = false;
  fi = 0;
  hi = 0;
  pi = 0;
  // wait for between frame period
  while (digitalReadFast(VD_PIN) == HIGH) {};
  attachInterrupt(VD_PIN, vd_int, RISING);
}

void stop_capture() {
  //detachInterrupt(VD_PIN);
  //detachInterrupt(HD_PIN);
  //detachInterrupt(DCLK_PIN);
}

/*
bool frame_done() {
  //return ((fi == FRAME_DWIDTH) & (hi == FRAME_HEIGHT));
  return ((hi == FRAME_HEIGHT) & (dma.complete()));
}*/

void capture_frame() {
  start_capture();
  while (!frame_done) {
    /*
    Serial.print("hi :"); Serial.println(hi);
    Serial.print("dma complete :"); Serial.println(dma.complete());
    Serial.print("dma error :"); Serial.println(dma.error());
    */
    delay(1);
  }
  //Serial.println("Frame done!");
  stop_capture();
}

void print_interrupt_counts() {
  start_capture();
  //Serial.println("Waiting for frame to start");
  while (!in_frame) {};  // wait for frame
  //Serial.println("in frame");
  int max_fi = 0;
  int max_hi = 0;
  while (in_frame) {
    max_fi = max(fi, max_fi);
    max_hi = max(hi, max_hi);
  }
  stop_capture();
  Serial.print("Max hi ");
  Serial.println(max_hi);
  Serial.print("Max fi ");
  Serial.println(max_fi);
  Serial.print("hi ");
  Serial.println(hi);
  Serial.print("fi ");
  Serial.println(fi);
}


void test_dma() {
  delay(1000);
  dma.clearError(); dma.clearComplete();
  dma.destinationBuffer(frame, FRAME_DWIDTH);
  Serial.print("set destination error: "); Serial.println(dma.error());
  // read from first byte of port d
  dma.source(GPIOD_PDIR);
  dma.transferSize(1);
  dma.transferCount(FRAME_DWIDTH);
  //dma.TCD->ATTR_SRC = 0;
  //dma.TCD->NBYTES = FRAME_DWIDTH;
  //dma.sourceBuffer(frame + FRAME_DWIDTH, FRAME_DWIDTH);
  Serial.print("set transfer error: "); Serial.println(dma.error());

  
  
  // set 17(b1) to rising edge
  // page 221, rising edge is PORT_PCR_IRQC(2) (also PORT_PCR_MUX(1) sets to IRQ)
  //CORE_PIN17_CONFIG = PORT_PCR_IRQC(2) | PORT_PCR_MUX(1);  // set dma mux to 17 (b1)
  PORTB_PCR1 |= PORT_PCR_IRQC(2); // | PORT_PCR_MUX(1);  // set dma mux to 17 (b1)
  
  //SIM_SCGC7 |= SIM_SCGC7_DMA;  // ??
  //SIM_SCGC6 |= SIM_SCBC6_DMAMUX_MASK;  // ??
  // use pit? periodic interrupt timer?
  dma.triggerAtHardwareEvent(DMAMUX_SOURCE_PORTB);  // PORTB
  
  Serial.print("set trigger error: "); Serial.println(dma.error());
  dma.enable();
  Serial.print("enable error: "); Serial.println(dma.error());
  /*
  int i=0;
  for (i=0; i<FRAME_DWIDTH; i++) {
    if (dma.error()) break;
    dma.triggerManual();
  }
  Serial.print("i:"); Serial.println(i);
  */
  while (!dma.complete()) {
    delay(1);
    if (dma.error()) break;
  }
  dma.disable();
  
  Serial.print("run error: "); Serial.println(dma.error());
  Serial.print("complete: "); Serial.println(dma.complete());
  Serial.println("=====");
}

void loop() {
  //print_interrupt_counts();
  //read_frame();
  capture_frame();
  print_frame();
  //test_dma();
  //delay(1000);
}

