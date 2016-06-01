#include "NRF24_reg_def.h"

#define RX_PIPE         0
#define PAYLOAD_SIZE    1

#define CSN     PORTC.F1    // Chip Select Not - Always high when not doing spi operation, pull low during spi operation
#define EN_CSN  TRISC.F1

#define CE      PORTC.F0    // TX mode - Pull low only when tramitting, RX mode - Pull high when data is received
#define EN_CE   TRISC.F0


#define ACK_RETRY       0x2F          // 15 retries, 750us pause b/w retry
#define ADR_WIDTH       0x03          // 5 bytes address width
#define CH_FREQ         0x01          // set channel freq to 2.401 MHz
#define RX_ADR          0x12          // set rx address
#define TX_ADR          0x12          // set tx address

#define DS_IRQ          0x70          // Disable IRQ - RX_DR, TX_DS, MAX_RT
#define EN_IRQ          0x00          // Enable IRQ - RX_DR, TX_DS, MAX_RT

#define RF_PWR          0x06          // 0 dBm
#define RF_DR           0x20          // set data rate to 250 kbps - 0x00 for 1 Mbps, 0x01 for 2 Mbps

#define RST_STAT        0x70

void nrf24_setup_radio();                                                                 // Setup nrf24l01 radio
void nrf24_change_adr_rx(unsigned short int* val, unsigned short int rx_port);            // Change RX address of rx_port
void nrf24_change_adr_tx(unsigned short int* val);                                        // Change TX address
void nrf24_change_radio_mode(unsigned char mode);                                         // Change radio mode - RX or TX
void nrf24_flush_tx_fifo();                                                               // Flush TX FIFO
void nrf24_flush_rx_fifo();                                                               // Flush RX FIFO

void nrf24_fill_tx_fifo(unsigned short int* tx_payload, unsigned short int payload_size); // Fill TX FIFO with data to transmit
void nrf24_read_rx_fifo(unsigned short int* rx_data, unsigned short int data_size);       // Read RX FIFO

void nrf24_reset_status();
unsigned short int nrf24_read_status();

void nrf24_enable_irq();
void nrf24_disable_irq();
void nrf24_rx_irq_handler(unsigned short int *rx_data, unsigned short int *rx_data_size);

void nrf24_receive_data(unsigned short int* rx_data, unsigned short int data_size);
void nrf24_transmit_data(unsigned short int* tx_data, unsigned short int data_size);

void nrf24_r_data_t_pyld(unsigned short int *rx_data, unsigned short int *rx_data_size,
                         unsigned short int pipe_number, unsigned short int *tx_ack_pyld,
                         unsigned short int ack_pyld_size);
void nrf24_t_data_r_pyld(unsigned short int *tx_data, unsigned short int tx_data_size,
                         unsigned short int *rx_ack_pyld, unsigned short int* pyld_size);
                         

void nrf24_power_up();
void nrf24_power_down();

void nrf24_enter_tx_mode();
void nrf24_enter_rx_mode();

void nrf24_enable_dpl();
void nrf24_disable_dpl();
void nrf24_enable_dpl_pipe(unsigned short int pipe_number);

void nrf24_fill_ack_payload(unsigned short int* ack_payload, unsigned short int pipe_number, unsigned short int ack_payload_size);
unsigned short int nrf24_get_payload_size();