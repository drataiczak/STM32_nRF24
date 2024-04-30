#include "nrf24l01p.h"

void _CSN_L(nrf24_t *nrf) {
    HAL_GPIO_WritePin(nrf->csn, nrf->csnPin, GPIO_PIN_RESET);
}

void _CSN_H(nrf24_t *nrf) {
    HAL_GPIO_WritePin(nrf->csn, nrf->csnPin, GPIO_PIN_SET);
}

void _CE_L(nrf24_t *nrf) {
    HAL_GPIO_WritePin(nrf->ce, nrf->cePin, GPIO_PIN_RESET);
}

void _CE_H(nrf24_t *nrf) {
    HAL_GPIO_WritePin(nrf->ce, nrf->cePin, GPIO_PIN_SET);
}

uint8_t _RW(nrf24_t *nrf, uint8_t reg) {
    uint8_t value;
    
    HAL_SPI_TransmitReceive(nrf->hspi, &reg, &value, 1, HAL_MAX_DELAY);

    return value;
}

uint8_t nRF24_Init(nrf24_t *nrf, SPI_HandleTypeDef *hspi, GPIO_TypeDef *csnPort, uint16_t csnPin, GPIO_TypeDef *cePort, uint16_t cePin) {
    uint8_t status = 0;

    nrf->hspi = hspi;
    nrf->csn = csnPort;
    nrf->ce = cePort;
    nrf->csnPin = csnPin;
    nrf->cePin = cePin;

    status += nRF24_Test(nrf);
    if(status > 0) {
        // Bail, trying to configure the transceiver doesnt make sense if we couldnt RW registers
        return status;
    }



    return status;
}

void nRF24_WriteReg(nrf24_t *nrf, uint8_t reg, uint8_t value) {
    uint8_t address = reg | CMD_W_REGISTER;

    _CSN_L(nrf);
    _RW(nrf, address);
    _RW(nrf, value);
    _CSN_H(nrf);
}

uint8_t nRF24_ReadReg(nrf24_t *nrf, uint8_t reg) {
    uint8_t addr = reg | CMD_R_REGISTER;
    uint8_t value;

    _CSN_L(nrf);
    value = _RW(nrf, addr);
    _CSN_H(nrf);

    return value;
}

void nRF24_WriteMBReg(nrf24_t *nrf, uint8_t reg, uint8_t *buf, size_t size) {
    uint8_t addr = reg | CMD_W_REGISTER;

    _CSN_L(nrf);

    _RW(nrf, addr);
    for(uint8_t i = 0; i < size; i++) {
        _RW(nrf, buf[i]);
    }

    _CSN_H(nrf);
}

void nRF24_ReadMBReg(nrf24_t *nrf, uint8_t reg, uint8_t *buf, size_t size) {
    uint8_t addr = reg | CMD_R_REGISTER;
    uint8_t status;

    _CSN_L(nrf);

    status = _RW(nrf, addr);
	for(uint8_t i = 0; i < size; i++) {
		buf[i] = _RW(nrf, CMD_NOP);
	}

    _CSN_H(nrf);
}

uint8_t nRF24_Test(nrf24_t *nrf) {
    uint8_t status = 0;
    uint8_t buf[SIZE_TX_ADDR] = {0};

    // Validate that the transceiver exists and we're able to correctly access registers for RW
    nRF24_WriteMBReg(nrf, TX_ADDR_REG, (uint8_t *) TEST_TX_ADDR, SIZE_TX_ADDR);
    nRF24_ReadMBReg(nrf, TX_ADDR_REG, buf, SIZE_TX_ADDR);

    for(uint8_t i = 0; i < SIZE_TX_ADDR; i++) {
        if(buf[i] != TEST_TX_ADDR[i]) {
            // Found mismatch, bail with error
            status++;
            break; 
        }
    }

    return status;
}

void nRF24_FlushTx(nrf24_t *nrf) {
    uint8_t status = 0;
    
    nRF24_WriteReg(nrf, CMD_FLUSH_TX, status);
}

void nRF24_FlushRx(nrf24_t *nrf) {
    uint8_t status = 0;

    nRF24_WriteReg(nrf, CMD_FLUSH_RX, status);
}

uint8_t nRF24_GetConfig(nrf24_t *nrf) {
    return nRF24_ReadReg(nrf, CONFIG_REG);
}

uint8_t nRF24_GetAutoAck(nrf24_t *nrf) {
    return nRF24_ReadReg(nrf, EN_AA_REG);
}

uint8_t nRF24_GetEnabledRxAddrs(nrf24_t *nrf) {
    return nRF24_ReadReg(nrf, EN_RXADDR_REG);
}

uint8_t nRF24_GetAddrWidth(nrf24_t *nrf) {
    return nRF24_ReadReg(nrf, SETUP_AW_REG);
}

uint8_t nRF24_GetAutoRetransmit(nrf24_t *nrf) {
    return nRF24_ReadReg(nrf, SETUP_RETR_REG); 
}

uint8_t nRF24_GetRFChannel(nrf24_t *nrf) {
    return nRF24_ReadReg(nrf, RF_CH_REG);
}

uint8_t nRF24_GetRFConfig(nrf24_t *nrf) {
    return nRF24_ReadReg(nrf, RF_SETUP_REG);
}

uint8_t nRF24_GetStatus(nrf24_t *nrf) {
    return nRF24_ReadReg(nrf, STATUS_REG);
}

uint8_t nRF24_GetTxObserve(nrf24_t *nrf) {
    return nRF24_ReadReg(nrf, OBSERVE_TX_REG);
}

uint8_t nRF24_GetRPD(nrf24_t *nrf) {
    return nRF24_ReadReg(nrf, RPD_REG);
}

void nRF24_GetRXPipeAddr(nrf24_t *nrf, pipe_t pipe, uint8_t *buf, size_t size) {
    uint8_t addr;
    
    switch(pipe) {
        case p0:
            addr = RX_ADDR_P0_REG;
            break;
        case p1:
            addr = RX_ADDR_P1_REG;
            break;
        case p2:
            addr = RX_ADDR_P2_REG;
            break;
        case p3:
            addr = RX_ADDR_P3_REG;
            break;
        case p4:
            addr = RX_ADDR_P4_REG;
            break;
        case p5:
            addr = RX_ADDR_P5_REG;
            break;
        default:
            return;
    }
    
    nRF24_ReadMBReg(nrf, addr, buf, size);
}

void nRF24_GetTXAddr(nrf24_t *nrf, uint8_t *buf, size_t size) {
    nRF24_ReadMBReg(nrf, TX_ADDR_REG, buf, size);
}

uint8_t nRF24_GetRXPipeWidth(nrf24_t *nrf, pipe_t pipe) {
    uint8_t addr;

    switch(pipe) {
        case p0:
            addr = RX_PW_P0_REG;
            break;
        case p1:
            addr = RX_PW_P1_REG;
            break;
        case p2:
            addr = RX_PW_P2_REG;
            break;
        case p3:
            addr = RX_PW_P3_REG;
            break;
        case p4:
            addr = RX_PW_P4_REG;
            break;
        case p5:
            addr = RX_PW_P5_REG;
            break;
        default:
            return;
    }

    return nRF24_ReadReg(nrf, addr);
}

uint8_t nRF24_GetFifoStatus(nrf24_t *nrf) {
    return nRF24_ReadReg(nrf, FIFO_STATUS_REG);
}

uint8_t nRF24_GetDynamicPayloadConfig(nrf24_t *nrf) {
    return nRF24_ReadReg(nrf, DYNPD_REG);
}

uint8_t nRF24_GetFeatureConfig(nrf24_t *nrf) {
    return nRF24_ReadReg(nrf, FEATURE_REG);
}

void nRF24_SetConfig(nrf24_t *nrf, uint8_t value) {
    value &= MASK_CONFIG;

    nRF24_WriteReg(nrf, CONFIG_REG, value);
}

void nRF24_SetAutoAck(nrf24_t *nrf, uint8_t value) {
    value &= MASK_AUTOACK;

    nRF24_WriteReg(nrf, EN_AA_REG, value);
}

void nRF24_SetEnabledRxAddrs(nrf24_t *nrf, uint8_t value) {
    value &= MASK_ENABLERX_ADDR;

    nRF24_WriteReg(nrf, EN_RXADDR_REG, value);
}

void nRF24_SetAddrWidth(nrf24_t *nrf, uint8_t value) {
    value &= MASK_ADDR_WIDTH;

    nRF24_WriteReg(nrf, SETUP_AW_REG, value);
}

void nRF24_SetAutoRetransmit(nrf24_t *nrf, uint8_t value) {
    nRF24_WriteReg(nrf, SETUP_RETR_REG, value);
}

void nRF24_SetRFChannel(nrf24_t *nrf, uint8_t value) {
    value &= MASK_RF_CH;

    nRF24_WriteReg(nrf, RF_CH_REG, value);
}

void nRF24_SetRFConfig(nrf24_t *nrf, uint8_t value) {
    value &= MASK_RF_CONFIG;

    nRF24_WriteReg(nrf, RF_SETUP_REG, value);
}

void nRF24_SetStatus(nrf24_t *nrf, uint8_t value) {
    value &= MASK_STATUS;

    nRF24_WriteReg(nrf, STATUS_REG, value);
}

void nRF24_SetRXPipeAddr(nrf24_t *nrf, pipe_t pipe, uint8_t *buf, size_t size){
    uint8_t addr;

    switch(pipe) {
        case p0:
            addr = RX_ADDR_P0_REG;
            break;
        case p1:
            addr = RX_ADDR_P1_REG;
            break;
        case p2:
            addr = RX_ADDR_P2_REG;
            break;
        case p3:
            addr = RX_ADDR_P3_REG;
            break;
        case p4:
            addr = RX_ADDR_P4_REG;
            break;
        case p5:
            addr = RX_ADDR_P5_REG;
            break;
        default:
            return;
    }

    nRF24_WriteMBReg(nrf, addr, buf, size);
}

void nRF24_SetTXAddr(nrf24_t *nrf, uint8_t *buf, size_t size){
    nRF24_WriteMBReg(nrf, TX_ADDR_REG, buf, size);
}

void nRF24_SetRXPipeWidth(nrf24_t *nrf, pipe_t pipe, uint8_t value){
    uint8_t addr;

    value &= MASK_RX_PW_PIPE;

    switch(pipe) {
        case p0:
            addr = RX_PW_P0_REG;
            break;
        case p1:
            addr = RX_PW_P1_REG;
            break;
        case p2:
            addr = RX_PW_P2_REG;
            break;
        case p3:
            addr = RX_PW_P3_REG;
            break;
        case p4:
            addr = RX_PW_P4_REG;
            break;
        case p5:
            addr = RX_PW_P5_REG;
            break;
        default:
            return;
    }

    nRF24_WriteReg(nrf, addr, value);
}

void nRF24_SetDynamicPayloadConfig(nrf24_t *nrf, uint8_t value){
    value &= MASK_DYNPD;

    nRF24_WriteReg(nrf, DYNPD_REG, value);
}

void nRF24_SetFeatureConfig(nrf24_t *nrf, uint8_t value){
    value &= MASK_FEATURE;

    nRF24_WriteReg(nrf, FEATURE_REG, value);
}