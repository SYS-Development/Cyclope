/******************************************************************************
                         Cyclope-EDU sample code
                            by Samuel Bonnard

  This project is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Cyclope sample code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Cyclope-edu code. If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

#define CyclopeRX_BIND_COUNT       1000
#define CyclopeRX_PACKET_PERIOD    1000
#define CyclopeRX_PACKET_SIZE      15
#define CyclopeRX_RF_NUM_CHANNELS  4
#define CyclopeRX_RF_BIND_CHANNEL  0
#define CyclopeRX_ADDRESS_LENGTH   5

static uint8_t CyclopeRX_rf_chan;
static uint8_t CyclopeRX_rf_channels[CyclopeRX_RF_NUM_CHANNELS] = {0,};
static uint8_t CyclopeRX_rx_tx_addr[CyclopeRX_ADDRESS_LENGTH];
static uint16_t CyclopeRX_telemetry_count = 0;
static uint16_t CyclopeRX_last_telemetry_count = 0;
static uint16_t CyclopeRX_loopcount = 0;
static uint16_t CyclopeRX_count = 0;
int B_and_C = 0;
int A_and_D = 0;


uint32_t process_CyclopeRX()
{
  uint32_t timeout = micros() + CyclopeRX_PACKET_PERIOD;
  if (CyclopeRX_count == 0)
    CyclopeRX_send_packet(0);
  CyclopeRX_count++;
  CyclopeRX_count %= 2;

  return timeout;
}

void CyclopeRX_init()
{
  uint8_t i;
  const u8 bind_address[] = {0, 0, 0, 0, 0};
  for (i = 0; i < CyclopeRX_ADDRESS_LENGTH; i++) {
    CyclopeRX_rx_tx_addr[i] = random() & 0xff;
  }
  CyclopeRX_rf_channels[0] = 0x00;
  for (i = 1; i < CyclopeRX_RF_NUM_CHANNELS; i++) {
    CyclopeRX_rf_channels[i] = random() % 0x42;
  }
  NRF24L01_Initialize();
  NRF24L01_SetTxRxMode(TX_EN);
  XN297_SetTXAddr(bind_address, CyclopeRX_ADDRESS_LENGTH);
  NRF24L01_FlushTx();
  NRF24L01_FlushRx();
  NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      
  NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);
  NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);
  NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, CyclopeRX_PACKET_SIZE);
  NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); 
  NRF24L01_SetBitrate(NRF24L01_BR_1M);             
  NRF24L01_SetPower(RF_POWER);
  NRF24L01_Activate(0x73);                         
  NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);      
  NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x01);
  NRF24L01_Activate(0x73);
  delay(150);
}

void CyclopeRX_bind()
{
  uint16_t counter = CyclopeRX_BIND_COUNT;
  while (counter) {
    if (CyclopeRX_count == 0)
      CyclopeRX_send_packet(1);
    CyclopeRX_count++;
    CyclopeRX_count %= 4;
    delayMicroseconds(CyclopeRX_PACKET_PERIOD);
    digitalWrite(ledPinB, counter-- & 0x10);
    digitalWrite(ledPinR, counter-- & 0x10);
  }
  XN297_SetTXAddr(CyclopeRX_rx_tx_addr, CyclopeRX_ADDRESS_LENGTH);
  XN297_SetRXAddr(CyclopeRX_rx_tx_addr, CyclopeRX_ADDRESS_LENGTH);
}

#define DYNTRIM(chval) ((u8)((chval >> 2) & 0xfc))

void CyclopeRX_send_packet(u8 bind)
{

  if (bind) {

    packet[0] = 0xa4;
    memcpy(&packet[1], CyclopeRX_rx_tx_addr, 5);
    memcpy(&packet[6], CyclopeRX_rf_channels, 4);
    packet[10] = transmitterID[0];
    packet[11] = transmitterID[1];
  } else {

    if (Throttle <= 1150 || POWER == 1) {
      digitalWrite(ledPinB, HIGH);
    }
    else {
      digitalWrite(ledPinR, HIGH);
      digitalWrite(ledPinB, LOW);

    }

    ppm[THROTTLE] = Throttle;

    packet[0] = 0xa5;
    packet[1] = 0xfa;

    int IAILERON = map(ppm[AILERON], PPM_MIN, PPM_MAX, 0, 255);   // aileron
    packet[5] = IAILERON;
    int IELEVATOR = map(ppm[ELEVATOR], PPM_MIN, PPM_MAX, 0, 255);   // elevator
    packet[6] = IELEVATOR;
    int ITHROTTLE = map(ppm[THROTTLE], PPM_MIN, PPM_MAX, 0, 255);   // throttle
    packet[7] = ITHROTTLE;
    int IRUDDER = map(ppm[RUDDER], PPM_MIN, PPM_MAX, 0, 255);   // rudder
    packet[8] = IRUDDER;

    packet[3] = map(analogRead(LOW_Pin), 0, 1023, 0, 1);
    packet[4] = B_and_C;
    packet[12] = A_and_D;
    packet[13] = map(analogRead(HIGH_Pin), 0, 1023, 0, 1);

  }
  packet[14] = CyclopeRX_checksum();

  NRF24L01_SetTxRxMode(TX_EN);
  XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
  NRF24L01_WriteReg(NRF24L01_05_RF_CH, bind ? CyclopeRX_RF_BIND_CHANNEL : CyclopeRX_rf_channels[CyclopeRX_rf_chan++]);
  CyclopeRX_rf_chan %= sizeof(CyclopeRX_rf_channels);
  NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
  NRF24L01_FlushTx();
  XN297_WritePayload(packet, CyclopeRX_PACKET_SIZE);
}

static uint8_t CyclopeRX_checksum()
{
  uint8_t sum = packet[0];
  for (uint8_t i = 1; i < CyclopeRX_PACKET_SIZE - 1; i++)
    sum += packet[i];
  return sum;
}

static uint8_t CyclopeRX_check_rx()
{

  if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR)) {
    // data received from aircraft
    XN297_ReadPayload(packet, CyclopeRX_PACKET_SIZE);

    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0xff);

    NRF24L01_FlushRx();
    if (packet[0] == 0x85 && packet[14] == CyclopeRX_checksum()) {
      CyclopeRX_telemetry_count++;
      return 1;
    }                       
  }
  return 0;
}
