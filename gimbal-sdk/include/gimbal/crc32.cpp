#include "include/crc32.h"

unsigned int CRC32Software(const unsigned char *pData, unsigned short Length) {
  unsigned int nReg;
  unsigned int nTemp = 0;
  unsigned short i, n;

  nReg = 0xFFFFFFFF;
  for (n = 0; n < Length; n++) {
    nReg ^= (unsigned int)pData[n];

    for (i = 0; i < 4; i++) {
      nTemp = Crc32Table[(unsigned char)((nReg >> 24) & 0xff)];
      nReg <<= 8;
      nReg ^= nTemp;
    }
  }
  return nReg;
}