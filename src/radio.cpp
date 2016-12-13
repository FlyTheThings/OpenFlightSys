#include "radio.h"
#include "data_struct.h"

nRF24L01P radio(SPI_MOSI, SPI_MISO, SPI_SCK, SPI_CS, D8);

void init_radio()
{
  radio.powerUp();
  radio.setRfFrequency(NRF24L01P_MIN_RF_FREQUENCY);
  radio.setAirDataRate(NRF24L01P_DATARATE_1_MBPS);
  radio.setRxAddress(((unsigned long long) 0xE7E7E7E7F7 ));
  radio.setTxAddress(((unsigned long long) 0xE7E7E7E7E7 ));
  radio.setTransferSize(25);
  radio.setTransmitMode();
  radio.enable();

  #ifdef _DEBUG_
    pc.printf( "nRF24L01+ Frequency    : %d MHz\r\n", radio.getRfFrequency() );
    pc.printf( "nRF24L01+ Output power : %d dBm\r\n",  radio.getRfOutputPower() );
    pc.printf( "nRF24L01+ Data Rate    : %d kbps\r\n", radio.getAirDataRate() );
    pc.printf( "nRF24L01+ TX Address   : 0x%010llX\r\n", radio.getTxAddress() );
    pc.printf( "nRF24L01+ RX Address   : 0x%010llX\r\n", radio.getRxAddress() );
  #endif
}

void transmit_radio()
{
  radio.write( NRF24L01P_PIPE_P0, nav_data.ch, RADIO_PCK_SIZE );
  wait_ms(1);
  radio.write( NRF24L01P_PIPE_P0, &nav_data.ch[RADIO_PCK_SIZE], RADIO_PCK_SIZE );
  wait_ms(1);
  radio.write( NRF24L01P_PIPE_P0, &nav_data.ch[2*RADIO_PCK_SIZE], RADIO_PCK_SIZE );
}
