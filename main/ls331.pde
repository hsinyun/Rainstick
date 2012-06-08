
//--------------------------------------------------
// I2C
//--------------------------------------------------

int32 ls331_init()
{
  // Set up I2C
  i2c_master_enable(I2C1,I2C_BUS_RESET | I2C_FAST_MODE); 
  
  uint8 buf[2] = {0x0f, 0x0f};  
  uint8 dev_id;
  i2c_msg msg[1];
  
  msg[0].addr = LIS331_ADDR_ACC;
  msg[0].flags =0;
  msg[0].length =1;
  msg[0].data= buf;
  int ret = i2c_master_xfer(I2C1, msg, 1, 1000);
  if (ret != 0) {
    SerialUSB.println("I2C Init Error: Abort");
    return -1;
  }
  
  msg[0].addr = LIS331_ADDR_ACC;  
  msg[0].flags = I2C_MSG_READ;
  msg[0].length =1;
  msg[0].data= buf;
  ret = i2c_master_xfer(I2C1, msg, 1, 1000);
  if (ret != 0) {
    SerialUSB.println("I2C Init Error: Abort");
    return -1;
  }

  dev_id = buf[0];  
  
  if (dev_id != LIS331_DEVID) {
    SerialUSB.println("Error, incorrect LIS331 devid!");
    SerialUSB.println("Halting program, hit reset...");
    return -2;
  }
  i2c_succeed = true;
  
  //ls331_write_acc(0x20, 0x2f);
  ls331_write(0x20, 0x2f, LIS331_ADDR_ACC);
  
  /* Take out of power down mode,  200 Hz sampling */
  ls331_write(0x20, 0x6f, LIS331_ADDR_GYRO);
  /* HPF Normal mode, HPCF3@0.05Hz*/
  ls331_write(0x21, 0x28, LIS331_ADDR_GYRO);
  /* HPF Enable */
  ls331_write(0x24, 0x11, LIS331_ADDR_GYRO);
  
  return 0;
}

void ls331_write(uint8 reg, uint8 data, int16 addr)
{
  if (i2c_succeed == true)
  {
    i2c_msg msgs[1]; // we dont do any bursting here, so we only need one i2c_msg object
    uint8 msg_data[2];
   
    msg_data = {reg,data};
    msgs[0].addr = addr;
    msgs[0].flags = 0; // write
    msgs[0].length = 2;
    msgs[0].data = msg_data;
    i2c_master_xfer(I2C1, msgs, 1,0);     
  }
}

uint16 ls331_read(uint8 reg, int16 addr)
{
  if (i2c_succeed == true)
  {
    uint8 buf[2] = {reg|0x80,0x80|reg+1};  
    i2c_msg msg[3];
  
    msg[0].addr = addr;
    msg[0].flags =0;
    msg[0].length = 1;
    msg[0].data= buf;
  
    msg[1].addr = addr;  
    msg[1].flags = I2C_MSG_READ;
    msg[1].length = 2;
    msg[1].data= buf;
  
    i2c_master_xfer(I2C1, msg, 2, 0);

    return buf[1]<<8|buf[0];  
  }
  else
    return 0;
}

// (END I2C)
//--------------------------------------------------
