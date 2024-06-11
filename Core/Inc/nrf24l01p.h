#ifndef _NRF24L01P_
#define _NRF24L01P_

#include "stm32f4xx_hal.h"

// Transceiver modes
#define MODE_TX       ((uint8_t) 0)
#define MODE_RX       ((uint8_t) 1)

// States
#define STATE_OFF 0
#define STATE_ON 1

// Commands
#define CMD_R_REGISTER          0x00 // Read command/status regs
#define CMD_W_REGISTER          0x20 // Write command/status regs
#define CMD_R_RX_PAYLOAD        0x61 // Read rx payload
#define CMD_W_TX_PAYLOAD        0xA0 // Write tx payload
#define CMD_FLUSH_TX            0xE1 // Flush tx fifo
#define CMD_FLUSH_RX            0xE2 // Flush rx fifo
#define CMD_REUSE_TX_PL         0xE3 // Reuse last transmitted payload (PTX device)
#define CMD_R_RX_PL_WID         0x60 // Read rx payload width
#define CMD_W_ACK_PAYLOAD       0xA8 // Write payload, ack to pipe (rx mode)
#define CMD_W_TX_PAYLOAD_NO_ACK 0xB0 // Disable autoack for specific packet (tx mode)
#define CMD_NOP                 0xFF // NOP

// Registers
#define CONFIG_REG      0x00 // Configuration
#define EN_AA_REG       0x01 // Enable auto ack
#define EN_RXADDR_REG   0x02 // Enabled rx addrs
#define SETUP_AW_REG    0x03 // Addr width config for pipes
#define SETUP_RETR_REG  0x04 // Retransmit config
#define RF_CH_REG       0x05 // RF channel
#define RF_SETUP_REG    0x06 // RF registers
#define STATUS_REG      0x07 // Status register cfg
#define OBSERVE_TX_REG  0x08 // Transmit observe cfg
#define RPD_REG         0x09 // RPD status
#define RX_ADDR_P0_REG  0x0A // Rx pipe 0
#define RX_ADDR_P1_REG  0x0B // Rx pipe 1
#define RX_ADDR_P2_REG  0x0C // Rx pipe 2
#define RX_ADDR_P3_REG  0x0D // Rx pipe 3
#define RX_ADDR_P4_REG  0x0E // Rx pipe 4
#define RX_ADDR_P5_REG  0x0F // Rx pipe 5
#define TX_ADDR_REG     0x10 // Tx addr
#define RX_PW_P0_REG    0x11 
#define RX_PW_P1_REG    0x12
#define RX_PW_P2_REG    0x13
#define RX_PW_P3_REG    0x14
#define RX_PW_P4_REG    0x15
#define RX_PW_P5_REG    0x16
#define FIFO_STATUS_REG 0x17 // FIFO status
#define DYNPD_REG       0x1C // Enable dynamic payload length
#define FEATURE_REG     0x1D // Feature register

// CONFIG_REG bits
#define MASK_RX_DR_B    6
#define MASK_TX_DS_B    5
#define MASK_MAX_RT_B   4
#define EN_CRC_B        3
#define CRCO_B          2
#define PWR_UP_B        1
#define PRIM_RX_B       0

// EN_AA_REG bits
#define ENAA_P5_B 5
#define ENAA_P4_B 4
#define ENAA_P3_B 3
#define ENAA_P2_B 2
#define ENAA_P1_B 1
#define ENAA_P0_B 0

// EN_RXADDR_REG bits
#define ERX_P5_B 5
#define ERX_P4_B 4
#define ERX_P3_B 3
#define ERX_P2_B 2
#define ERX_P1_B 1
#define ERX_P0_B 0

// SETUP_RETR_REG bits
#define ARD_B 4
#define ARC_B 0

// RF_SETUP_REG bits
#define CONT_WAVE_B     7
#define RF_DR_LOW_B     5
#define PLL_LOCK_B      4
#define RF_DR_HIGH_B    3
#define RF_PWR_HIGH_B   2
#define RF_PWR_LOW_B    1

//  STATUS_REG bits
#define RX_DR_B  6
#define TX_DS_B  5
#define MAX_RT_B  4
#define RX_P_NO_B 1
#define TX_FULL_B 0

// OBSERVE_TX_REG bits
#define PLOS_CNT_B 4
#define ARC_CNT_B  0

// RPD_REG bits
#define RPD_B 0

// FIFO_STATUS_REG bits
#define TX_FIFO_REUSE_B 6
#define TX_FIFO_FULL_B  5
#define TX_FIFO_EMPTY_B 4
#define RX_FIFO_FULL_B  1
#define RX_FIFO_EMPTY_B 0

// DYNPD_REG bits
#define DPL_P5_B 5
#define DPL_P4_B 4
#define DPL_P3_B 3
#define DPL_P2_B 2
#define DPL_P1_B 1
#define DPL_P0_B 0

// FEATURE_REG bitis
#define EN_DPL_B     2
#define EN_ACK_PAY_B 1
#define EN_DYN_ACK_B 0

// Reg sizes
#define SIZE_TX_ADDR 5
#define SIZE_RX_P0_ADDR 5
#define SIZE_RX_P1_ADDR 5

// Masks
#define MASK_CONFIG        0x7F
#define MASK_AUTOACK       0x3F
#define MASK_ENABLERX_ADDR 0x3F
#define MASK_ADDR_WIDTH    0x03
#define MASK_RF_CH         0x7F
#define MASK_RF_SETUP      0xBE
#define MASK_STATUS        0x7F
#define MASK_RX_PW_PIPE    0x3F
#define MASK_FIFO_STATUS   0xF3
#define MASK_DYNPD         0x3F
#define MASK_FEATURE       0x03
#define MASK_CRC           0x0C
#define MASK_IRQ           0x70

// Default reg values
#define DEF_CONFIG 0x08
#define DEF_AUTOACK 0x3F
#define DEF_ENABLED_RX_ADDRS 0x03
#define DEF_ADDR_WIDTH 0x03
#define DEF_AUTO_RETR 0x03
#define DEF_RF_CHANNEL 0x02
#define DEF_RF_SETUP 0x0E
#define DEF_STATUS 0x00
#define DEF_RX_P0_ADDR 0xE7E7E7E7E7
#define DEF_RX_P1_ADDR 0XC2C2C2C2C2
#define DEF_RX_P2_ADDR 0xC3
#define DEF_RX_P3_ADDR 0xC4
#define DEF_RX_P4_ADDR 0xC5
#define DEF_RX_P5_ADDR 0xC6
#define DEF_TX_ADDR 0xE7E7E7E7E7
#define DEF_PW_P0 0x00
#define DEF_PW_P1 0x00
#define DEF_PW_P2 0x00
#define DEF_PW_P3 0x00
#define DEF_PW_P4 0x00
#define DEF_PW_P5 0x00
#define DEF_PD 0x00
#define DEF_FEATURE 0x00

// Test values
#define TEST_TX_ADDR "NRF24"

typedef struct nrf24 {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *csn;
    GPIO_TypeDef *ce;
    uint16_t csnPin;
    uint16_t cePin;
} nrf24_t;


typedef enum addrWidth {
    aw3 = 0x01, // 3-bit address
    aw4 = 0x02, // 4-bit address
    aw5 = 0x03  // 5-bit address
} addrWidth_t;

// Retry time in us
typedef enum retrTime {
    t250 =  0x00,
    t500 =  0x01,
    t750 =  0x02,
    t1000 = 0x03,
    t1250 = 0x04,
    t1500 = 0x05,
    t1750 = 0x06,
    t2000 = 0x07,
    t2250 = 0x08,
    t2500 = 0x09,
    t2750 = 0x0A,
    t3000 = 0x0B,
    t3250 = 0x0C,
    t3500 = 0x0D,
    t3750 = 0x0E,
    t4000 = 0x0F
} retrTime_t;

typedef enum rfPwr {
    rf18dbm = 0x00, // -18dBm
    rf12dbm = 0x01, // -12dBm
    rf6dbm =  0x02, // -6dBm
    rf0dbm =  0x03  // 0dBm
} rfPwr_t;

typedef enum rfDataRate {
    mbps1 =   0x00, // 1Mbps
    mbps2 =   0x02, // 2Mbps
    kbps250 = 0x01  // 250Kbps
} rfDataRate_t;

typedef enum pipe {
    p0 = 0,
    p1 = 1,
    p2 = 2,
    p3 = 3,
    p4 = 4,
    p5 = 5,
    pAll = 6
} pipe_t;

typedef enum crc {
    crc_off = 0,
    crc_1byte = 0x08,
    crc_2byte = 0x0C
} crc_t;

uint8_t nRF24_Init(nrf24_t *nrf, SPI_HandleTypeDef *hspi, GPIO_TypeDef *csnPort, uint16_t csnPin, GPIO_TypeDef *cePort, uint16_t cePin);
uint8_t nRF24_Test(nrf24_t *nrf);
void nRF24_WriteReg(nrf24_t *nrf, uint8_t reg, uint8_t value);
uint8_t nRF24_ReadReg(nrf24_t *nrf, uint8_t reg);

// Commands
void nRF24_FlushTx(nrf24_t *nrf);
void nRF24_FlushRx(nrf24_t *nrf);

// Reg getters
uint8_t nRF24_GetConfig(nrf24_t *nrf);
uint8_t nRF24_GetAutoAck(nrf24_t *nrf);
uint8_t nRF24_GetEnabledRxAddrs(nrf24_t *nrf);
uint8_t nRF24_GetAddrWidth(nrf24_t *nrf);
uint8_t nRF24_GetAutoRetransmit(nrf24_t *nrf);
uint8_t nRF24_GetRFChannel(nrf24_t *nrf);
uint8_t nRF24_GetRFConfig(nrf24_t *nrf);
uint8_t nRF24_GetStatus(nrf24_t *nrf);
uint8_t nRF24_GetTxObserve(nrf24_t *nrf);
uint8_t nRF24_GetRPD(nrf24_t *nrf);
void nRF24_GetRXPipeAddr(nrf24_t *nrf, pipe_t pipe, uint8_t *buf, size_t size);
void nRF24_GetTXAddr(nrf24_t *nrf, uint8_t *buf, size_t size);
uint8_t nRF24_GetRXPipeWidth(nrf24_t *nrf, pipe_t pipe);
uint8_t nRF24_GetFifoStatus(nrf24_t *nrf);
uint8_t nRF24_GetDynamicPayloadConfig(nrf24_t *nrf);
uint8_t nRF24_GetFeatureConfig(nrf24_t *nrf);

// Reg setters
void nRF24_SetConfig(nrf24_t *nrf, uint8_t value);
void nRF24_SetAutoAck(nrf24_t *nrf, uint8_t value);
void nRF24_SetEnabledRxAddrs(nrf24_t *nrf, uint8_t value);
void nRF24_SetAddrWidth(nrf24_t *nrf, uint8_t value);
void nRF24_SetAutoRetransmit(nrf24_t *nrf, uint8_t value);
void nRF24_SetRFChannel(nrf24_t *nrf, uint8_t value);
void nRF24_SetRFConfig(nrf24_t *nrf, uint8_t value);
void nRF24_SetCRC(nrf24_t *nrf, crc_t value);
void nRF24_SetRFDataRate(nrf24_t *nrf, rfDataRate_t rate);
void nRF24_SetStatus(nrf24_t *nrf, uint8_t value);
void nRF24_SetRXPipeAddr(nrf24_t *nrf, pipe_t pipe, uint8_t *buf, size_t size);
void nRF24_SetTXAddr(nrf24_t *nrf, uint8_t *buf, size_t size);
void nRF24_SetRXPipeWidth(nrf24_t *nrf, pipe_t pipe, uint8_t value);
void nRF24_SetDynamicPayloadConfig(nrf24_t *nrf, uint8_t value);
void nRF24_SetFeatureConfig(nrf24_t *nrf, uint8_t value);
void nRF24_ClearFifoStatus(nrf24_t *nrf);
void nRF24_SetPower(nrf24_t *nrf, rfPwr_t value);
void nRF24_SetPowerState(nrf24_t *nrf, uint8_t state);
void nRF24_ClearIRQ(nrf24_t *nrf);

// General
void nRF24_SetOperationalMode(nrf24_t *nrf, uint8_t mode);
void nRF24_SetPipeAA(nrf24_t *nrf, pipe_t pipe, uint8_t state);

#endif