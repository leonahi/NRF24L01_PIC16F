#include "NRF24_api.h"

extern void nrf24_write_reg(unsigned short int cmd, unsigned short int* val, unsigned short int num_bytes);
extern void nrf24_read_reg(unsigned short int cmd, unsigned short int* val,unsigned short int num_bytes);


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

void nrf24_setup_radio()
{
  unsigned short int cmd=0, val[5];
  
  // Enable CE and CSN
  EN_CSN = 0;
  EN_CE  = 0;
  CSN    = 1;
  CE     = 0;
  
  // Enable auto ack
  cmd    = (EN_AA | W_REGISTER);
  val[0] = (1 << ENAA_P0);
  nrf24_write_reg(cmd, val, 1);
  
  // Setup ACK RETR
  cmd    = (SETUP_RETR | W_REGISTER);
  val[0] = (ACK_RETRY);
  nrf24_write_reg(cmd, val, 1);
  
  // Enable RX pipe
  cmd    = (EN_RXADDR | W_REGISTER);
  val[0] = (1 << ERX_P0);
  nrf24_write_reg(cmd, val, 1);
  
  // Setup RX/TX address width
  cmd    = (SETUP_AW | W_REGISTER);
  val[0] = (ADR_WIDTH);
  nrf24_write_reg(cmd, val, 1);

  // Setup radio req
  cmd    = (RF_CH | W_REGISTER);
  val[0] = (CH_FREQ);
  nrf24_write_reg(cmd, val, 1);

  // Setup data rate and power
  cmd    = (RF_SETUP | W_REGISTER);
  val[0] = (RF_DR | RF_PWR);
  nrf24_write_reg(cmd, val, 1);
  
  // Setup RX address pipe 0
  cmd    = (RX_ADDR_P0 | W_REGISTER);
  val[0] = (RX_ADR); val[1] = (RX_ADR); val[2] = (RX_ADR);
  val[3] = (RX_ADR); val[4] = (RX_ADR);
  nrf24_write_reg(cmd, val, 5);
  
  // Setup TX address
  cmd    = (TX_ADDR | W_REGISTER);
  val[0] = (TX_ADR); val[1] = (TX_ADR); val[2] = (TX_ADR);
  val[3] = (TX_ADR); val[4] = (TX_ADR);
  nrf24_write_reg(cmd, val, 5);
  
  // Setup RX payload pipe 0
  cmd    = (RX_PW_P0 | W_REGISTER);
  val[0] = (PAYLOAD_SIZE);
  nrf24_write_reg(cmd, val, 1);

  // Setup CONFIG register
  cmd    = (CONFIG | W_REGISTER);
  val[0] = (DS_IRQ | 0x0C);
  nrf24_write_reg(cmd, val, 1);
}

void nrf24_flush_tx_fifo()
{
  nrf24_write_reg(FLUSH_TX, 0, 0);
}

void nrf24_flush_rx_fifo()
{
  nrf24_write_reg(FLUSH_RX, 0, 0);
}

void nrf24_fill_tx_fifo(unsigned short int* tx_payload, unsigned short int payload_size)
{
  nrf24_write_reg(W_TX_PAYLOAD, tx_payload, payload_size);
}

void nrf24_read_rx_fifo(unsigned short int *rx_data, unsigned short int data_size)
{
  nrf24_read_reg(R_RX_PAYLOAD, rx_data, data_size);
}

unsigned short int nrf24_read_status()
{
  unsigned short int status=0;
  CSN = 0;
  status = SPI1_Read((NOP));
  CSN = 1;
  return status;
}

void nrf24_reset_status()
{
  unsigned short int val[1];
  val[0] = RST_STAT;
  CSN = 0;
  nrf24_write_reg((STATUS | W_REGISTER), val, 1);
  CSN = 1;
}

void nrf24_enable_irq()
{
  unsigned short int val[1];
  #define IRQ
  INTCON |= 0x90;
  OPTION_REG = 0x40;
  //TRISB.F4 = 1;
  //INTCON |= 0x88;
  nrf24_read_reg((CONFIG | R_REGISTER), val, 1);
  val[0] &= 0x0F;
  nrf24_write_reg((CONFIG | W_REGISTER), val, 1);
}

void nrf24_disable_irq()
{
  unsigned short int val[1];
  #undef IRQ
  INTCON &= 0x7F;
  nrf24_read_reg((CONFIG | R_REGISTER), val, 1);
  val[0] &= 0x7F;
  nrf24_write_reg((CONFIG | W_REGISTER), val, 1);
}

void nrf24_enter_transceiver_mode()
{
  CE = 1;
}

void nrf24_exit_transceiver_mode()
{
  CE = 0;
}

void nrf24_receive_data(unsigned short int* rx_data, unsigned short int data_size)
{
  unsigned short int status=0;
  nrf24_reset_status();
  nrf24_flush_rx_fifo();
  nrf24_enter_transceiver_mode();
  while(1)
  {
    status = nrf24_read_status();
    if ((status & 0x40) == 0x40)
    {
      nrf24_exit_transceiver_mode();
      nrf24_reset_status();
      break;
    }
    delay_ms(5);
  }
  nrf24_read_rx_fifo(rx_data, data_size);
}


void nrf24_transmit_data(unsigned short int* tx_data, unsigned short int data_size)
{
  unsigned short int status=0;
  unsigned int i=0;
  
  nrf24_reset_status();
  nrf24_flush_tx_fifo();
  
  nrf24_fill_tx_fifo(tx_data, data_size);
  
  nrf24_enter_transceiver_mode();
  for(i=0; i<1000; ++i);
  nrf24_exit_transceiver_mode();
}

void nrf24_r_data_t_pyld(unsigned short int *rx_data, unsigned short int *rx_data_size,
                         unsigned short int pipe_number, unsigned short int *tx_ack_pyld,
                         unsigned short int ack_pyld_size)
{
  unsigned short int status=0;
  nrf24_reset_status();
  nrf24_flush_rx_fifo();
  nrf24_fill_ack_payload(tx_ack_pyld, pipe_number, ack_pyld_size);
  nrf24_enter_transceiver_mode();
  while(1)
  {
    status = nrf24_read_status();
    if ((status & 0x40) == 0x40)
    {
      nrf24_exit_transceiver_mode();
      nrf24_reset_status();
      break;
    }
    delay_ms(5);
  }
  *rx_data_size = nrf24_get_payload_size();
  //rx_data = (unsigned short int *)malloc((sizeof(unsigned short int)*(*rx_data_size)));
  nrf24_read_rx_fifo(rx_data, *rx_data_size);
}

void nrf24_t_data_r_pyld(unsigned short int *tx_data, unsigned short int tx_data_size,
                         unsigned short int *rx_ack_pyld, unsigned short int* ack_pyld_size)
{
  unsigned short int status=0, i=0;

  nrf24_reset_status();
  nrf24_flush_tx_fifo();

  nrf24_fill_tx_fifo(tx_data, tx_data_size);

  nrf24_enter_transceiver_mode();
  for(i=0; i<100; ++i);
  nrf24_exit_transceiver_mode();
  while(1)
  {
    status = nrf24_read_status();
    if ((status & 0x40) == 0x40)
      break;
    delay_ms(5);
  }
  nrf24_reset_status();
  *ack_pyld_size = nrf24_get_payload_size();
  //rx_ack_pyld = (unsigned short int *)malloc(sizeof(unsigned short int)*(*ack_pyld_size));
  nrf24_read_rx_fifo(rx_ack_pyld, *ack_pyld_size);

}

void nrf24_power_down()
{
  unsigned short int val[1];
  
  nrf24_read_reg((CONFIG | R_REGISTER), val, 1);
  val[0] &= 0xFD;
  nrf24_write_reg((CONFIG | W_REGISTER), val, 1);
}

void nrf24_power_up()
{
  unsigned short int val[1];
  
  nrf24_read_reg((CONFIG | R_REGISTER), val, 1);
  val[0] |= 0x02;
  nrf24_write_reg((CONFIG | W_REGISTER), val, 1);
}

void nrf24_enter_tx_mode()
{
  unsigned short int val[1];

  nrf24_read_reg((CONFIG | R_REGISTER), val, 1);
  val[0] &= 0xFE;
  nrf24_write_reg((CONFIG | W_REGISTER), val, 1);
}

void nrf24_enter_rx_mode()
{
  unsigned short int val[1];

  nrf24_read_reg((CONFIG | R_REGISTER), val, 1);
  val[0] |= 0x01;
  nrf24_write_reg((CONFIG | W_REGISTER), val, 1);
}

void nrf24_enable_dpl()
{
  unsigned short int val[1]={0x06};
  nrf24_write_reg((FEATURE | W_REGISTER), val, 1);
}

void nrf24_disable_dpl()
{
  unsigned short int val[1]={0};
  nrf24_write_reg((FEATURE | W_REGISTER), val, 1);
}


void nrf24_enable_dpl_pipe(unsigned short int pipe_number)
{
  unsigned short int val[1];
  val[0] = (1 << pipe_number);
  nrf24_write_reg((DYNPD | W_REGISTER), val, 1);
}

void nrf24_fill_ack_payload(unsigned short int* ack_payload, unsigned short int pipe_number, unsigned short int ack_payload_size)
{
  nrf24_write_reg((W_ACK_PAYLOAD | pipe_number), ack_payload, ack_payload_size);
}

unsigned short int nrf24_get_payload_size()
{
  unsigned short int size[1];
  nrf24_read_reg(R_RX_PL_WID, size, 1);
  return size[0];
}