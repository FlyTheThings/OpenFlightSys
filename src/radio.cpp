/************************************************************************
* FILENAME :        radio.cpp             DESIGN REF:
*
* DESCRIPTION :
*       Radio source file for Drone Control system
*
* NOTES :
*       This file is a part of the OpenFlightSys Github project
*       Visit https://github.com/mcprakash/OpenFlightSys for more details
*
*       Copyright <Chandra Mangipudi> (2016)  All rights reserved.
*
* AUTHOR :    Chandra Mangipudi        START DATE :    01/01/2017
*
* LICENSE:
*       This program is free software: you can redistribute it and/or modify
*       it under the terms of the GNU General Public License as published by
*       the Free Software Foundation, either version 3 of the License, or
*       (at your option) any later version.
*
*       This program is distributed in the hope that it will be useful,
*       but WITHOUT ANY WARRANTY; without even the implied warranty of
*        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*        GNU General Public License for more details.
*
*       You should have received a copy of the GNU General Public License
*       along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
**/

#include "radio.h"
#include "data_struct.h"

nRF24L01P radio(SPI_MOSI, SPI_MISO, SPI_SCK, SPI_CS, D8);

void init_radio()
{
  radio.powerDown();
  radio.disable();
  wait_ms(100);
  radio.powerUp();
  radio.setRfFrequency(DEFAULT_NRF24L01P_RF_FREQUENCY);
  radio.setAirDataRate(NRF24L01P_DATARATE_1_MBPS);
  radio.setRxAddress(((unsigned long long) 0xE7E7E7E7F7 ));
  radio.setTxAddress(((unsigned long long) 0xE7E7E7E7E7 ));
  radio.setTransferSize(RADIO_PCK_SIZE);
  radio.setTransmitMode();
  radio.enable();

  #ifdef _DEBUG_
    pc.printf("<------------------------------------>\r\n");
    pc.printf("            Initializing Radio          \r\n");
    pc.printf( "nRF24L01+ Frequency    : %d MHz\r\n", radio.getRfFrequency() );
    pc.printf( "nRF24L01+ Output power : %d dBm\r\n",  radio.getRfOutputPower() );
    pc.printf( "nRF24L01+ Data Rate    : %d kbps\r\n", radio.getAirDataRate() );
    pc.printf( "nRF24L01+ TX Address   : 0x%010llX\r\n", radio.getTxAddress() );
    pc.printf( "nRF24L01+ RX Address   : 0x%010llX\r\n", radio.getRxAddress() );
    pc.printf("            Initializied Radio          \r\n");
    pc.printf("<------------------------------------>\r\n");
  #endif
}

void transmit_radio1()
{
  radio.write( NRF24L01P_PIPE_P0, nav_data.ch, RADIO_PCK_SIZE );
}

void transmit_radio2()
{
  radio.write( NRF24L01P_PIPE_P0, &nav_data.ch[RADIO_PCK_SIZE], RADIO_PCK_SIZE );
}

void transmit_radio3()
{
  radio.write( NRF24L01P_PIPE_P0, &nav_data.ch[2*RADIO_PCK_SIZE], RADIO_PCK_SIZE );
}
