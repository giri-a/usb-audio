#include <math.h>

int32_t q31_multiply(int32_t a, int32_t b){
    // Q31 format numbers are assumed to be in 1.31 format (ranges from -1 to 0.99999)
    // Multiplication is usual operation. However the result is in 2.62 format and requires 
    // 64 bits to contain. To convert this result to 1.31 format, a left shift is performed
    // and upper 32 bits are returned. 

    int64_t t = (int64_t)a * (int64_t)b;
    t = t<<1;
    // It is a little endian machine; &t points to the LSB and (&t)+3 to the MSB.
    // We are casting &t to the pointer of a int32_t data type so that *p gives
    // lower 32 bits and *(p+1) gives us upper 32 bits.
    int32_t *p = (int32_t*) &t;
    return *++p;
}

int32_t mul_8p24x8p24(int32_t a, int32_t b){
    // Q7.24 format numbers have 8bits left to the binary point and 24 bits to the right.
    // Multiplication result has 16bits to the left of binary point and 48bits to the right
    // and requires 64 bits to be contained. To convert this back to Q7.24, we need to 
    // shift right by 24 places and pick out lower 32 bits OR shift left by 8 places and 
    // pick out the upper 32 bits. A check of the overflow is required for satuarting the
    // result.

    int64_t t = (int64_t)a * (int64_t)b;
    // Overflow check: we'll get upper 16bits and check if it is < -256 or > 255
    // It is a little endian machine; &t points to the LSB and (&t)+3 to the MSB.
    // We are casting &t to the pointer of a int16_t data type so that *p gives
    // lowest 16 bits and *(p+3) gives us highest 16 bits.
    int16_t *p = (int16_t*) &t;
    p += 3;
    if(*p < -128) return 0x80000000;
    if(*p >  127) return 0x7fffffff;

    t = t<<8;
    int32_t *q = (int32_t*) &t;
    return *++q;
}

int16_t mul_1p31x8p24(int32_t sig, int32_t gain)
{
    int64_t t = (int64_t)sig * (int64_t)gain;

    // sig is assumed to be in 1.31 and gain in 8.24 formats.
    // Therefore t must be in 9.55 fotmat. We need to return a signed 16bit
    // data in 1.15 format. So we need shift the result left by 8 positions. 
    // Before we do that we need to make sure that the result is
    // within range of a signed 16bit representation.

    int16_t *p = (int16_t*)&t;
    int16_t m = (*(p+3)) >> 7;  
    // m has the most significant 9 bits of t. We want to make sure that these
    // are all zero's or all 1's. Otherwise there is overflow or underflow.

    if(m < -1) return 0xffff; // most negative in 16bits 
    if(m >  0) return 0x7fff; // most positive in 16bits

    t = t<<8;
    return *(p+3);
}