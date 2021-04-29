#include "filter.h"

uint16_t averaging_filter(int16_t filter_data[4][3], int16_t szHistoricalData[NUM_FILTERED][3], int16_t szData16[3])
{
    uint32_t i,j;
    j = 0;
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

    return j;

}

uint16_t low_pass_filter(int16_t filter_data[4][3], int16_t szHistoricalData[NUM_FILTERED][3], int16_t szData16[3])
{
    uint32_t i,j,k;
    double b[9];
    double result;
    b[0] = 0.01618297;
    b[1] = 0.04623686;
    b[2] = 0.1214632;
    b[3] = 0.19971808;
    b[4] = 0.23279779;
    b[5] = 0.19971808;
    b[6] = 0.1214632;
    b[7] = 0.04623686;
    b[8] = 0.01618297;

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

        // Store latest in FIFO
        szHistoricalData[filter_data[NEXT_ENTRY][i]][i] = szData16[i];

        result = 0.0;

        for(j = 0; j<filter_data[ENTRIES][i]; j++)
        {
            k = filter_data[NEXT_ENTRY][i]+j;

            // Handled wrapping of index
            if(k > (NUM_FILTERED - 1))
            {
                k = k -(NUM_FILTERED - 1);
            }

            result += (b[j] * szHistoricalData[k][i]);
        }

        // Increment pointer with bitmasking
        filter_data[NEXT_ENTRY][i]++;

        if(filter_data[NEXT_ENTRY][i] > NUM_FILTERED - 1)
        {
            filter_data[NEXT_ENTRY][i] = 0;
        }
        // Take average
        filter_data[AVG][i] = (int16_t) round(result);
        //printf("Axis: %d, Result: %f, Rounded: %d\n",i,result,filter_data[AVG][i]);

    }

    return j;

}
