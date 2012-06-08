

//--------------------------------------------------
// SPI
//--------------------------------------------------
void dac_init()
{  
  
  // Set up SPI
  // NOTE: The Slave Select delay in spi_writeToDac is hard-coded based on the clock frequency.  Changing the clock frequency
  // from 2.25MHz will require adjusting the delay as indicated in spi_writeToDac.
  spi.begin(SPI_2_25MHZ, MSBFIRST, 2);
  
  pinMode(SSPIN, OUTPUT);  
  pinMode(LDACPIN, OUTPUT);  
  digitalWrite(SSPIN, HIGH);  
  digitalWrite(LDACPIN, HIGH);  
  
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
  digitalWrite(SSPIN, LOW);    
  spi.write(data, 3);
  delayMicroseconds(6);
  digitalWrite(SSPIN, HIGH);
}

// Ignores the 4 MSBs
void dac_write(uint16 val)
{
  uint8 data[3];
  val = val << 4;
  data[0] = 0x07;
  data[1] = val >> 8;
  data[2] = val;
  digitalWrite(SSPIN, LOW);  

  spi.write(data, 3);
  // 6 microsecond delay to ensure that the Slave Select pin stays low until the 24th bit is transmitted
  // (only works with 2.25MHz clock frequency)
  delayMicroseconds(6);
  digitalWrite(SSPIN, HIGH);
  digitalWrite(LDACPIN, LOW);  
  delayMicroseconds(1);
  digitalWrite(LDACPIN, HIGH);  
}

// (END SPI)
//--------------------------------------------------
