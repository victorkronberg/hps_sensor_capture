#ifndef __FILTER_H__
#define __FILTER_H__

#include <stdint.h.>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define X_IDX   0
#define Y_IDX   1
#define Z_IDX   2
#define ENTRIES 0
#define NEXT_ENTRY  1
#define SUM     2
#define AVG     3
// Number filtered must be power of 2 for bitmasking
#ifdef  AVERAGING_FILTER
    #define NUM_FILTERED     8
#else
    #define NUM_FILTERED    9
#endif

uint16_t averaging_filter(int16_t filter_data[4][3], int16_t szHistoricalData[NUM_FILTERED][3], int16_t szData16[3]);
uint16_t low_pass_filter(int16_t filter_data[4][3], int16_t szHistoricalData[NUM_FILTERED][3], int16_t szData16[3]);

#endif /* filter.h */
