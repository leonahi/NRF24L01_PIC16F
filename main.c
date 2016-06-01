#include "NRF24_reg_def.h"
#include "NRF24_api.h"

extern void nrf24_write_reg(unsigned short int cmd, unsigned short int* val, unsigned short int num_bytes);
extern void nrf24_read_reg(unsigned short int cmd, unsigned short int* val,unsigned short int num_bytes);

//#define TX_EN
#define RX_EN

#ifdef RX_EN
  unsigned short int rx_data[32];
  unsigned short int rx_data_size;
  unsigned short int pipe_number=0;
  unsigned short int tx_ack_pyld[3];
  unsigned short int ack_pyld_size=3;
#endif

#ifdef TX_EN
  unsigned short int tx_data[4];
  unsigned short int tx_data_size=4;
  unsigned short int rx_ack_pyld[32];
  unsigned short int ack_pyld_size;
#endif

void interrupt()
{
  unsigned short int status=0;
  PORTD = 0x11;
  delay_ms(2000);

  /*
  SSPCON.SSPEN = 1;
  CSN = 0;
  SSPBUF = NOP;
  while(!(SSPSTAT.BF=1))
  CSN = 1;

  PORTD = SSPBUF;
  delay_ms(1000);
  PORTD = 0xAA;
  delay_ms(1000);
  */
  INTCON &= 0x6F;
}

int main()
{
  short out, buffer, i=0;
  char txt[4];
  unsigned int short val[5], output[5];
  unsigned short int *val_ack_pyld;
  
  /*
  unsigned int val_analog=0;;
  ADCON1 = 0x84;
  TRISA.F0 = 1;
  TRISA.F1 = 1;
  TRISB = 0x00;
  TRISD = 0x00;
  PORTB = 0;
  PORTD = 0;
  
  while(1)
  {
    val_analog = ADC_Read(0);
    PORTB = val_analog;
    PORTD = val_analog >> 8;
    delay_ms(100);
  }
  */

  // ack - payload
  TRISD = 0x00;
  PORTD = 0xFF;
  SPI1_Init();
  nrf24_setup_radio();
  nrf24_enable_dpl();
  nrf24_enable_dpl_pipe(0);
  //nrf24_enable_irq();

#ifdef RX_EN
  nrf24_enter_rx_mode();
#endif

#ifdef TX_EN
  nrf24_enter_tx_mode();
#endif
  
  nrf24_power_up();
  
#ifdef RX_EN
  PORTD = 0xA0;
  i=0;
  tx_ack_pyld[0] = 0x01;
  tx_ack_pyld[1] = 0x02;
  tx_ack_pyld[2] = 0x03;
  nrf24_r_data_t_pyld(rx_data, &rx_data_size, pipe_number, tx_ack_pyld, ack_pyld_size);
  for(i=0; i<(rx_data_size); ++i)
  {
    PORTD = rx_data[i];
    delay_ms(1000);
  }
  PORTD = 0xAA;
#endif


#ifdef TX_EN
  PORTD = 0x0F;
  tx_data[0] = 0x01;
  tx_data[1] = 0x02;
  tx_data[2] = 0x03;
  tx_data[3] = 0x04;
  nrf24_t_data_r_pyld(tx_data, tx_data_size, rx_ack_pyld, &ack_pyld_size);
  i=0;
  for(i=0; i<(ack_pyld_size); ++i)
  {
    PORTD = rx_ack_pyld[i];
    delay_ms(1000);
  }
  PORTD = 0xAA;
#endif

  return 0;

}