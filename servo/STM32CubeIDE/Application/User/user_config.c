/*
 * user_config.c
 *
 *  Created on: Jun 12, 2021
 *      Author: sunbl
 */

#include "user_config.h"
#include "stm32g4xx_hal.h"

#define CONFIG_FLASH_ADDRESS (0x0801FC00)

ConfigPointers_t ConfigPointers;
ConfigFlashStorage_t *ConfigFlashStorage = (ConfigFlashStorage_t *)CONFIG_FLASH_ADDRESS;
extern ConfigPointers_t *_ptr_list_addr;

uint32_t crc32_for_byte(uint32_t r) {
  for(int j = 0; j < 8; ++j)
    r = (r & 1? 0: (uint32_t)0xEDB88320L) ^ r >> 1;
  return r ^ (uint32_t)0xFF000000L;
}

void crc32(const void *data, size_t n_bytes, uint32_t* crc) {
  static uint32_t table[0x100];
  if(!*table)
    for(size_t i = 0; i < 0x100; ++i)
      table[i] = crc32_for_byte(i);
  for(size_t i = 0; i < n_bytes; ++i)
    *crc = table[(uint8_t)*crc ^ ((uint8_t*)data)[i]] ^ *crc >> 8;
}

void CONFIG_Init() {
    // Magic byte sequence exists to check if we read the correct struct
    ConfigPointers.magic[0] = 0x54;
    ConfigPointers.magic[1] = 0xA4;
    ConfigPointers.magic[2] = 0x2F;
    ConfigPointers.magic[3] = 0x6F;
    ConfigPointers.magic[4] = 0x07;
    ConfigPointers.magic[5] = 0x8A;
    ConfigPointers.magic[6] = 0x48;

    ConfigPointers.servo_config = &ServoHandle_M1.Config;
    ConfigPointers.servo_state = &ServoHandle_M1.state;
    ConfigPointers.oscilloscope = &OscilloscopeHandle_M1;
    ConfigPointers.oscilloscope_data = &OscilloscopeHandle_M1.Data[0];
    ConfigPointers.swd_command_buffer = &SWDBufferInfo;

    // write the address in a location we can always find.
    _ptr_list_addr = &ConfigPointers;

    ConfigPointers.ready = 1;

    uint32_t crc = 0;
    crc32((void*)ConfigFlashStorage, sizeof(ConfigFlashStorage_t) - 4, &crc);

    if (crc != ConfigFlashStorage->crc) {
        // invalid stored parameters, write the default ones
        CONFIG_Save();
    }

    CONFIG_Load();
}

void CONFIG_Load() {

    uint32_t crc = 0;
    crc32((void*)ConfigFlashStorage, sizeof(ConfigFlashStorage_t) - 4, &crc);

    if (crc == ConfigFlashStorage->crc) {
        *(ConfigPointers.servo_config) = ConfigFlashStorage->servo_config;
    }
}

void CONFIG_Save() {
    // collect the data to be saved

    ConfigFlashStorage_t SaveFlashData = {
        .servo_config = *ConfigPointers.servo_config
    };
    // calculate the crc
    crc32((void*)&SaveFlashData, sizeof(ConfigFlashStorage_t) - 4, &SaveFlashData.crc);

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef FLASH_EraseInitStruct = {0};

    FLASH_EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    FLASH_EraseInitStruct.Page = 127;
    FLASH_EraseInitStruct.Banks = FLASH_BANK_BOTH;
    FLASH_EraseInitStruct.NbPages = 1;

    uint32_t error = 0;
    HAL_FLASHEx_Erase(&FLASH_EraseInitStruct, &error);


    for (uint32_t i = 0; i < sizeof(ConfigFlashStorage_t); i += 8) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CONFIG_FLASH_ADDRESS + i, *(uint64_t*)(((void *) &SaveFlashData) + i));
    }

    HAL_FLASH_Lock();
}