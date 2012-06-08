#include <Wire.h>
#include <i2c.h>
#include "wavelet.h"
#include "rainstick.h"



#define UPDATE_RATE 256 // number of update per sec
#define MULT_FACTOR 5000
#define ABS(a) (((a) < 0) ? -(a) : (a))
#define SGN(a) (((a) < 0) ? -(1) : (1))
#define SGN_F(a) (((a) < 0) ? -(1.0) : (1.0))
#define WAVEMAX 40



//--------------------------------------------------
// MAIN  
//--------------------------------------------------
int length = 300*(MULT_FACTOR/1000);
int bounce = 3;   // use 1/10 of this value
int mass = 32;//16; // the stronger the mass, the stronger the impact
int rolling_volume = 10; // rolling noise multiplication factor
int samplenum = 0;
//--------------------------------------------------


//--------------------------------------------------
// PHYSICS  
//--------------------------------------------------
static int acc, acc_old;
static int speed = 0, speed_old;
static float distance = 0.0; 

static unsigned int normspeed = 0;
static unsigned int end_reached = 0;
static float sin_theta;
static int wavelet_ind = 0;
static int output = 0;
static int DAC_output=0;
int acc_offset = 0;
int calibrate_samplenum;
boolean calibrate_flag = 0;
static int toggle=0;
static int switch1 = 1;
static int switch2 = 1;

// test variables
boolean test_enable = false;
float distance_test = 0.0; 
uint16 value = 0;
int display = 0;
//--------------------------------------------------
 
 
//--------------------------------------------------
// SENSOR DATA  
//--------------------------------------------------
int16 x_a, y_a, z_a, x_g, y_g, z_g;
float xf_g, yf_g, zf_g;
//-------------------------------------------------- 

//--------------------------------------------------
// I2C/Sensor
//--------------------------------------------------
boolean i2c_succeed = false;

//--------------------------------------------------
// SPI/DAC
//--------------------------------------------------
uint16 spiData = 0x7FFF;
HardwareSPI spi(1); // Use SPI port number 1
//-------------------------------------------------- 


//--------------------------------------------------
// TIMER INTERRUPTS
//--------------------------------------------------
HardwareTimer timer(2);
//--------------------------------------------------


//--------------------------------------------------
// MAIN
//--------------------------------------------------
   
void setup() { 
  

  pin_assignment(); 


  SerialUSB.println("Rainstick Firmware v0.1 \n");
    SerialUSB.println("Rainstick Firmware v0.1 \n");
    
  if (ls331_init() == 0){
    i2c_succeed = true;
    SerialUSB.println("i2c succeed\n");
  }

  dac_init();
  timer_init();
  
  // Enable Amplifier
  digitalWrite(CHN_ENB_A, HIGH);  
  digitalWrite(CHN_ENB_B, HIGH);  
  digitalWrite(REGEN, HIGH);
      
  // Write to LEDs
  digitalWrite(LEDPIN1, HIGH);  
  digitalWrite(LEDPIN2, LOW);  
  digitalWrite(LEDPIN3, HIGH);  
  digitalWrite(LEDPIN4, LOW);  
}

void loop() {

    delayMicroseconds(1000*1000);
    toggleLED();
  
    // terminal output
    update_terminal_input();
    switch1 = digitalRead(SWITCH1);
    switch2 = digitalRead(SWITCH2);
    
    if (switch1 == 0){
      if (test_enable == true) 
        test_enable = false; 
      else 
        test_enable = true;  
      SerialUSB.print("\ntest mode=");
      SerialUSB.print(test_enable);   
    }

    
    // TEST OUTPUT
    if (test_enable == true) {
      print_vals();
    }
}

// (END MAIN)
//--------------------------------------------------


void pin_assignment() {
    
  // Set up LEDs and SPI pins as outputs
  pinMode(BOARD_LED_PIN, OUTPUT);
  pinMode(LEDPIN1, OUTPUT);  
  pinMode(LEDPIN2, OUTPUT);  
  pinMode(LEDPIN3, OUTPUT);  
  pinMode(LEDPIN4, OUTPUT);  
  
  pinMode(CHN_ENB_A, OUTPUT);  
  pinMode(CHN_ENB_B, OUTPUT);  
  pinMode(REGEN, OUTPUT);
  pinMode(SWITCH1, INPUT_PULLUP);
  pinMode(SWITCH2, INPUT_PULLUP);

  
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

//--------------------------------------------------
// INTERRUPT HANDLING
//--------------------------------------------------

void handler(void) {
    
  // ACC READINGS
  x_a = ls331_read(0x28, LIS331_ADDR_ACC);
  y_a = ls331_read(0x2A, LIS331_ADDR_ACC);
  z_a = ls331_read(0x2C, LIS331_ADDR_ACC);
    
  // GYRO READINGS
  x_g = ls331_read(0x28, LIS331_ADDR_GYRO);
  y_g = ls331_read(0x2A, LIS331_ADDR_GYRO);
  z_g = ls331_read(0x2C, LIS331_ADDR_GYRO);
    
  xf_g = x_g*8.75/1000.0;
  yf_g = y_g*8.75/1000.0;
  zf_g = z_g*8.75/1000.0;
   
  update_physics();
  
  if (test_enable == true)  {  
    distance_test = (distance_test + 20);
    value = wavelet[(int)distance_test%WAVELET_LEN]*(0x03ff/WAVEMAX);    
    dac_write(value);
  } else{
    dac_write(DAC_output);
  }
    
  display = distance*15/length;
  int led1= (display&0x0001);
  int led2 = (display&0x0002)>>1;
  int led3 = (display&0x0004)>>2;
  int led4 = (display&0x0008)>>3;
  digitalWrite(LEDPIN1, led1);  
  digitalWrite(LEDPIN2, led2);  
  digitalWrite(LEDPIN3, led3);  
  digitalWrite(LEDPIN4, led4);  

}

// (END INTERRUPT HANDLING)
//--------------------------------------------------


//--------------------------------------------------
// MENUS
//--------------------------------------------------

void update_terminal_input() 
{
  
  if (SerialUSB.available() != 0) {
    int new_rcvd_char;
    
    new_rcvd_char = SerialUSB.read();
    SerialUSB.println(new_rcvd_char);
    
    switch (new_rcvd_char) {

      case 'c':
        calibrate();
        break;
     default:
        print_vals();
        break;
    }
  }   
}

// (END MENUS)
//--------------------------------------------------


//--------------------------------------------------
// PHYSICS
//--------------------------------------------------

void calibrate()
{
  int x =0;
  SerialUSB.print("calibrating:");
  //calibrate_flag = 1;
  //calibrate_samplenum = 0;
  for (int i = 0; i < 10; i++) {
    x = x + x_a;
    delayMicroseconds(1000*300);
  }
   
  acc_offset = x/10;
  SerialUSB.print("calibrated horozontal value:");
  SerialUSB.println(acc_offset);  
}

void update_physics()
{
  acc_old = acc;
  speed_old = speed;

  // update acceleration based on x axis acceleration
  sin_theta = (float)(x_a-acc_offset)/17000;
  acc = sin_theta*7*MULT_FACTOR;
  
  // calibration
//  if(calibrate_flag)
//  {
//    acc_offset = (int)((acc + calibrate_samplenum * acc_offset) / calibrate_samplenum);
//    calibrate_samplenum++;
//    if(calibrate_samplenum == UPDATE_RATE)
//      calibrate_flag = 0;
//  }
  
  // update speed. 1/UPDATE_RATE is time
  speed = speed + acc/UPDATE_RATE;

  // update distance. 1/UPDATE_RATE is time
  distance = distance + (float)speed/UPDATE_RATE;

  if ( (distance > length || distance < 0) ||
    ((distance == length || distance == 0) && end_reached == 0))  
  {
      	    
    // end reached

    // reset distance
    if (distance >= length) 
      distance = length;	        
    if (distance <= 0)
      distance = 0;
      
    // send impact
    if (end_reached == 1) {
      // end already reached, impact already sent, so return DAC to zero
      normspeed = 0;
    } else {
      // first time end reached
      normspeed = ABS(speed)/80;
      output = mass*normspeed;
    }
    speed = speed*bounce*(-0.1);
    
    end_reached = 1;
	        	        
  } 
  else if ( distance < length && distance > 0)
  {
    
    // rolling noise here
    
    end_reached = 0;
    normspeed = 0;
    output = wavelet[wavelet_ind]*rolling_volume;         
	        
  }
  else
  {
    output = 0; // prevent tick when starts to roll?
  }
       
  wavelet_ind = ((int)distance)%WAVELET_LEN;   
  DAC_output = output;
  
 
}
// (END PHYSICS)
//--------------------------------------------------



//--------------------------------------------------
// UTILITIES
//--------------------------------------------------
void print_vals(void)
{
  if (samplenum %10 == 0) {
    SerialUSB.println("To enter test mode, press switch 1 (S1) for 1 second.");
    SerialUSB.println("To exit test mode, press switch 1 (S1) for 1 second again");
    SerialUSB.println("in test mode, the program will print sensed and simulated values and output a vibration of ~80Hz.");
  }
  
  samplenum++;
  SerialUSB.print(samplenum);
  SerialUSB.print(": sin_theta = ");
  SerialUSB.print(sin_theta);      
  SerialUSB.print(",    \tacc = ");
  SerialUSB.print(acc);
  SerialUSB.print(", \tspeed = ");
  SerialUSB.print(speed);   
  SerialUSB.print(", \tdist = ");
  SerialUSB.print(distance);      
  SerialUSB.print(" ");
  
  
  SerialUSB.print("Acc=[");
  SerialUSB.print(x_a,DEC);
  SerialUSB.print(",");
  SerialUSB.print(y_a,DEC);
  SerialUSB.print(",");
  SerialUSB.print(z_a,DEC);
  SerialUSB.print("],\t");
  SerialUSB.print("Gyro=[");
  SerialUSB.print(xf_g,1);
  SerialUSB.print(",");
  SerialUSB.print(yf_g,1);
  SerialUSB.print(",");
  SerialUSB.print(zf_g,1);
  SerialUSB.print("]\n"); 
}
//--------------------------------------------------
