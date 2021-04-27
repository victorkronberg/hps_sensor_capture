#include "filter.h"

uint16_t averaging_filter(int16_t filter_data[4][3], int16_t szHistoricalData[NUM_FILTERED][3], int16_t szData16[3])
{
    uint16_t i,j;
    for(i=0; i<3; i++)
    {
        // Check if filter is full
        if(filter_data[ENTRIES][i] >= (NUM_FILTERED))
        {
            // Subtract oldest from sum
            filter_data[SUM][i] -= szHistoricalData[filter_data[NEXT_ENTRY][i]][i];
        }
        else
        {
            // Increment number of entries
            filter_data[ENTRIES][i] += 1;
        }

        // Add current value to sum and store in historical array
        filter_data[SUM][i] += szData16[i];
        szHistoricalData[filter_data[NEXT_ENTRY][i]][i] = szData16[i];

        // Increment pointer with bitmasking
        filter_data[NEXT_ENTRY][i] = (filter_data[NEXT_ENTRY][i]+1)&(NUM_FILTERED-1);

        // Take average
        filter_data[AVG][i] = filter_data[SUM][i]/filter_data[ENTRIES][i];

    }

    // Exercise the processor
    for(i=0; i<128; i++)
    {
        j = (filter_data[AVG][2] * i)/5;
    }

    return j;

}
