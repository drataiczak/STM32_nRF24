#include "nrf24l01p.h"
#include "main.h"

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
    HAL_StatusTypeDef halStatus = HAL_OK;

    nrf->hspi = hspi;
    nrf->csn = csnPort;
    nrf->ce = cePort;
    nrf->csnPin = csnPin;
    nrf->cePin = cePin;

    nRF24_Test(nrf);

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
    printf("Called w\r\n");
    for(uint8_t i = 0; i < size; i++) {
        printf("Called w\r\n");
        _RW(nrf, buf[i]);
    }

    _CSN_H(nrf);
}

void nRF24_ReadMBReg(nrf24_t *nrf, uint8_t reg, uint8_t *buf, size_t size) {
    uint8_t addr = reg | CMD_R_REGISTER;
    uint8_t status;

    _CSN_L(nrf);

    status = _RW(nrf, addr);
    printf("Called r\r\n");
	for(uint8_t i = 0; i < size; i++) {
        printf("Called r\r\n");
		buf[i] = _RW(nrf, CMD_NOP);
	}

    _CSN_H(nrf);
}

uint8_t nRF24_Test(nrf24_t *nrf) {
    uint8_t status = 0;
    uint8_t buf[SIZE_TX_ADDR] = {0};

    nRF24_WriteMBReg(nrf, TX_ADDR_REG, (uint8_t *) "nrf24", SIZE_TX_ADDR);
    nRF24_ReadMBReg(nrf, TX_ADDR_REG, buf, SIZE_TX_ADDR);

    return status;
}