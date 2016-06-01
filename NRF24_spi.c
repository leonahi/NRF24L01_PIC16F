#include "NRF24_api.h"

void nrf24_read_reg(unsigned short int cmd, unsigned short int *val,unsigned short int num_bytes)
{
  unsigned short int output, i=0;
  CSN = 0;
  output = SPI1_Read(cmd);
  for(i=0; i<num_bytes; ++i)
    val[i] = SPI1_Read(cmd);
  CSN = 1;
}

void nrf24_write_reg(unsigned short int cmd, unsigned short int *val, unsigned short int num_bytes)
{
  unsigned short int output, i=0;
  CSN = 0;
  output = SPI1_Read(cmd);
  for(i=0; i<num_bytes; ++i)
    output = SPI1_Read(val[i]);
  CSN = 1;
}