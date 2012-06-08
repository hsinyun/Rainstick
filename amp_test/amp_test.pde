int ssPin = 10;
int LEDPIN1 = 17; // LED 1
int LEDPIN2 = 18; // LED 2
int LEDPIN3 = 19; // LED 3
int LEDPIN4 = 20; // LED 4
int ldacPin = 12; // DAC Sync
int ChnEnA = 3; // Amplifier Channel A Enable
int ChnEnB = 6; // Amplifier Channel B Enable
int RegEn = 4; // Amplifier Internal Register Enable
#define WAVELET_LEN 100
#define WAVEMAX 40
#define UPDATE_RATE 256 // number of update per sec

// triangular wave variables
uint16 value = 0x7FFF;
int direction = 1;
float distance = 0;
// Use SPI port number 1
HardwareSPI spi(1);
HardwareTimer timer(2);

static int wavelet[WAVELET_LEN] = {
  0,1,3,4,5,6,7,9,10,11,12,14,15,16,17,18,19,20,21,22,24,25,25,26,27,28,29,30,31,32,32,33,34,34,35,36,36,37,37,38,38,38,39,39,39,40,40,40,40,40,40,40,40,40,40,40,39,39,39,38,38,38,37,37,36,36,35,34,34,33,32,32,31,30,29,28,27,26,25,25,24,22,21,20,19,18,17,16,15,14,12,11,10,9,7,6,5,4,3,1,
//  0,-1,-3,-4,-5,-6,-7,-9,-10,-11,-12,-14,-15,-16,-17,-18,-19,-20,-21,-22,-24,-25,-25,-26,-27,-28,-29,-30,-31,-32,-32,-33,-34,-34,-35,-36,-36,-37,-37,-38,-38,-38,-39,-39,-39,-40,-40,-40,-40,-40,-40,-40,-40,-40,-40,-40,-39,-39,-39,-38,-38,-38,-37,-37,-36,-36,-35,-34,-34,-33,-32,-32,-31,-30,-29,-28,-27,-26,-25,-25,-24,-22,-21,-20,-19,-18,-17,-16,-15,-14,-12,-11,-10,-9,-7,-6,-5,-4,-3,-1,
};

void setup() {
  pinMode(LEDPIN1, OUTPUT);  
  pinMode(LEDPIN2, OUTPUT);  
  pinMode(LEDPIN3, OUTPUT);  
  pinMode(LEDPIN4, OUTPUT);      
  digitalWrite(LEDPIN1, LOW);  
  digitalWrite(LEDPIN2, HIGH);  
  digitalWrite(LEDPIN3, HIGH);  
  digitalWrite(LEDPIN4, LOW);  
      
  // Turn on the SPI port

  dac_init();
  
  // Set up Amplifier
  pinMode(ChnEnA, OUTPUT);  
  pinMode(ChnEnB, OUTPUT);  
  pinMode(RegEn, OUTPUT);  
  digitalWrite(ChnEnA, HIGH);  
  digitalWrite(ChnEnB, HIGH);  
  digitalWrite(RegEn, HIGH);  
  
  timer_init();
  
}

void timer_init() {
  // Set up Timer Interrupts
  // Pause the timer while we're configuring it
  timer.pause();
  // Set up period
  timer.setPeriod(1000000/UPDATE_RATE); // in microseconds
  // Set up an interrupt on channel 1
  timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  timer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
  timer.attachCompare1Interrupt(handler);
  // Refresh the timer's count, prescale, and overflow
  timer.refresh();
  // Start the timer counting
  timer.resume();
}

void handler(void) {
    // Triangular wave
//  if(value >= 0x0FFF)
//    direction = -1;
//  else if(value <= 0)
//    direction = 1;
//  if(direction == 1)
//    value += 0x0100;
//  else
//    value -= 0x0100;

  distance = (distance + 10);
  value = wavelet[(int)distance%WAVELET_LEN]*(0x0fff/WAVEMAX);         

  dac_write(value);
}

void loop() {
  
  delayMicroseconds(1000*1000);
  
  int led1=  (value&0x0100)>>8;
  int led2 = (value&0x0200)>>9;
  int led3 = (value&0x0400)>>10;
  int led4 = (value&0x0800)>>11;
  
  SerialUSB.print(distance, HEX);  SerialUSB.print(" ");
  SerialUSB.print((int)distance%WAVELET_LEN, HEX);     SerialUSB.print(" ");
  SerialUSB.print(wavelet[(int)distance%WAVELET_LEN], HEX);     SerialUSB.print(" ");
  SerialUSB.print(value, HEX);     SerialUSB.print(" ");

  SerialUSB.print(led1);  SerialUSB.print(" ");    
  SerialUSB.print(led2);  SerialUSB.print(" ");    
  SerialUSB.print(led3);  SerialUSB.print(" ");
  SerialUSB.print(led4);  SerialUSB.print(" ");
  SerialUSB.print("\n");
      
  digitalWrite(LEDPIN4, led1);  
  digitalWrite(LEDPIN3, led2);  
  digitalWrite(LEDPIN2, led3);  
  digitalWrite(LEDPIN1, led4);  
  
}

void dac_init()
{
   spi.begin(SPI_2_25MHZ, MSBFIRST, 2);
   pinMode(ssPin, OUTPUT);  
   pinMode(ldacPin, OUTPUT);  
   digitalWrite(ssPin, HIGH);  
   digitalWrite(ldacPin, HIGH);  
  
   //Software reset
   dac_cmd(0x29, 0x00, 0x01);
   
   // Set LDAC to Asynchronous mode
   dac_cmd(0x30, 0x00, 0x00);
    
   // Enable Internal Reference  
   dac_cmd(0x38, 0x00, 0x01);
  
}

void dac_cmd(uint8 val1, uint8 val2, uint8 val3)
{
  uint8 data[3];

  data[0] = val1;
  data[1] = val2;
  data[2] = val3;
  digitalWrite(ssPin, LOW);    
  spi.write(data, 3);
  delayMicroseconds(6);
  digitalWrite(ssPin, HIGH);
}

void dac_write(uint16 val)
{
  uint8 data[3];
  val = val << 4;
  data[0] = 0x07;
  data[1] = val >> 8;
  data[2] = val;
  digitalWrite(ssPin, LOW);  

  spi.write(data, 3);
  delayMicroseconds(6);
  digitalWrite(ssPin, HIGH);
  digitalWrite(ldacPin, LOW);  
  delayMicroseconds(1);
  digitalWrite(ldacPin, HIGH);  
}
