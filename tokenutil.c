#include "tokenutil.h"


bool getTokenArray(uint32_t tokenAddr, const uint8_t* array, size_t size)
{
  bool isSet = false;
  array = TOKEN_TO_ARRAY(tokenAddr);
  for (size_t i = 0; i < size; i++){
    if (array[i] != 0xFF){
      isSet = true;
      break;
    }
  }

  return isSet;
}

bool getTokenU32(uint32_t tokenAddr, uint32_t* value)
{
  *value = TOKEN_TO_UINT32(tokenAddr);
  return *value != 0xFFFFFFFF;
}

bool getTokenU8(uint32_t tokenAddr, uint8_t* value)
{
  *value = TOKEN_TO_UINT8(tokenAddr);
  return *value != 0xFF;
}

bool getTokenFloat(uint32_t tokenAddr, float *value, int divisor, float defaultValue)
{
  bool ret;
  if (getTokenU32(tokenAddr, (uint32_t*)value)){
    *value /= divisor;
    ret = true;
  }
  else {
    *value = defaultValue;
    ret = false;
  }

  return ret;
}
