#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define TOKEN_TO_ARRAY(t)   (const uint8_t *)(t)
#define TOKEN_TO_UINT32(t) (*(const uint32_t *)(t))
#define TOKEN_TO_UINT8(t)  (*(const uint8_t *)(t))

bool getTokenArray(uint32_t tokenAddr, const uint8_t* array, size_t size);
bool getTokenU32(uint32_t tokenAddr, uint32_t* value);
bool getTokenU8(uint32_t tokenAddr, uint8_t* value);
bool getTokenFloat(uint32_t tokenAddr, float *value, int divisor, float defaultValue);
