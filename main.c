#define _GNU_SOURCE

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include "hps_0.h"
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <sched.h>
#include <syslog.h>
#include <sys/sysinfo.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "ADXL345.h"
#include "logging.h"
#include "filter.h"

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )
#define FPGA_REQ_MASK (0x00010000)

// Set filtering mode by defining one of the following macros
//	CPU_FILTERING: filter using ARM processor
//	FPGA_FILTERING: filter using FPGA fabric
#define CPU_FILTERING 1

#define LOOP_COUNT	60

#define FPGA_START_TIME	0
#define FPGA_REQ_TIME	1
#define FPGA_ACK_TIME	2
#define FPGA_REQ_ACK	3
#define FPGA_NREQ		4
#define FPGA_END_TIME	5

#define CPU_START_TIME		0
#define CPU_I2C_READ		1
#define CPU_END_TIME		2

#define NUM_THREADS 1
#define THREAD_1 1
#define THREAD_2 2

void *CPU_filter(void *threadp);

// FPGA Timestamps
//	[0] - Start time
//	[1] - Req sent
//	[2] - Ack received
//	[3] - Req low sent
//	[4] - Data read (end time)
double fpga_timestamps[6][75];

// CPU Timestamps
// [0] - Start time
// [1] - I2C read
// [2] - Filter done
double cpu_timestamps[3][75];

typedef struct
{
    int threadIdx;
} threadParams_t;


void *CPU_filter(void *threadp)
{
	// I2C variables
	int file;
	const char *filename = "/dev/i2c-0";
	uint8_t id;
	bool bSuccess;
	const int mg_per_digi = 4;
	uint16_t szXYZ[3];

	//Current data
	int16_t szXYZ_data[3];

	//Historical filter data
	int16_t szHistoricalData[NUM_FILTERED][3];

	//3x3 array which includes the following for X, Y, and Z:
	//[0][x]: Number of entries
	//[1][x]: Pointer to next entry
	//[2][x]: Current sum
	//[3][x]: Current average
	int16_t szXYZ_filter[4][3];
	int cnt=0;
	void *virtual_base;
	int fd, i;
	int loop_count;
	int16_t sw_value;
	uint32_t sw_mask;
	void *h2p_lw_filter_out_addr;
	void *h2p_lw_filter_in_addr;
	//void *h2p_lw_hexL_addr;
	printf("Filtering thread entered\n");
	/////////////////////////////////////////gsensor init and configuration//////////////////////////////////
		printf("===== gsensor test =====\r\n");

		// open bus
		if ((file = open(filename, O_RDWR)) < 0) {
	  	  /* ERROR HANDLING: you can check errno to see what went wrong */
		    perror("Failed to open the i2c bus of gsensor");
	  	 	pthread_exit(NULL);
		}


		// init
		// gsensor i2c address: 101_0011
		int addr = 0b01010011;
		if (ioctl(file, I2C_SLAVE, addr) < 0) {
	  	  printf("Failed to acquire bus access and/or talk to slave.\n");
		    /* ERROR HANDLING; you can check errno to see what went wrong */
	  	  pthread_exit(NULL);
		}

	#ifdef CPU_FILTERING
		// Initialize averaging filter
		for(i=0; i<3; i++)
		{
			szXYZ_filter[ENTRIES][i] = 0;
			szXYZ_filter[NEXT_ENTRY][i] = 0;
			szXYZ_filter[SUM][i] = 0;
			szXYZ_filter[AVG][i] = 0;
		}
	#endif



	///////////////IO configuration and memory mapping////////////////////////////////////////////////////////////////
		// map the address space for the LED registers into user space so we can interact with them.
		// we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span

		if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
			printf( "ERROR: could not open \"/dev/mem\"...\n" );
			pthread_exit(NULL);
		}

		virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );

		if( virtual_base == MAP_FAILED ) {
			printf( "ERROR: mmap() failed...\n" );
			close( fd );
			pthread_exit(NULL);
		}

		h2p_lw_filter_out_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_OUTPUT_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
		h2p_lw_filter_in_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_INPUT_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
		//h2p_lw_hexL_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_HEX_X_BASE ) & ( unsigned long)( HW_REGS_MASK ) );


		// toggle the LEDs a bit
		sw_value = 3;

		loop_count = 0;

		// configure accelerometer as +-2g and start measure////////////////////////////////////////
	    bSuccess = ADXL345_Init(file);
	    if (bSuccess){
	        // dump chip id
	        bSuccess = ADXL345_IdRead(file, &id);
	        if (bSuccess)
	            printf("id=%02Xh\r\n", id);
	    }

		while( bSuccess && (loop_count < LOOP_COUNT) ) {
			if (ADXL345_IsDataReady(file)){
				bSuccess = ADXL345_XYZ_Read(file, szXYZ);
				if (bSuccess){
					  cnt++;
					// control led
					fpga_timestamps[FPGA_START_TIME][loop_count] = getTimeMsec();
					sw_mask = (uint32_t)((szXYZ[0]*mg_per_digi) | FPGA_REQ_MASK);
					//printf("Sending Req %#08x\n Initial value: %d\n",sw_mask, (int16_t)(szXYZ[0]*mg_per_digi));
					*(uint32_t *)h2p_lw_filter_out_addr = sw_mask;
					fpga_timestamps[FPGA_REQ_TIME][loop_count] = getTimeMsec();

					usleep( 100 );

					fpga_timestamps[FPGA_ACK_TIME][loop_count] = getTimeMsec();
					sw_mask = *(uint32_t *)h2p_lw_filter_in_addr;
					//printf("Ack response %#08x\n",sw_mask);
					fpga_timestamps[FPGA_REQ_ACK][loop_count] = getTimeMsec();
					sw_mask = (uint32_t)(szXYZ[0]*mg_per_digi);
					*(uint32_t *)h2p_lw_filter_out_addr = sw_mask;
					fpga_timestamps[FPGA_NREQ][loop_count] = getTimeMsec();
					sw_mask = *(uint32_t *)h2p_lw_filter_in_addr;
					sw_value = (int16_t)sw_mask;
					fpga_timestamps[FPGA_END_TIME][loop_count] = getTimeMsec();
					//printf("Final response %#08x\n",sw_mask);
					//printf("Final response convert: %d\n", sw_value);
					printf("Filter time elapsed: %f\r\n",(fpga_timestamps[FPGA_END_TIME][loop_count]-fpga_timestamps[FPGA_ACK_TIME][loop_count]) );
					printf("Filter response X=%d mg\n",sw_value);

					// wait 250ms
					usleep( 250*1000 );

					loop_count++;
				}
			}


		} // while

		loop_count = 0;

		while(bSuccess && (loop_count < LOOP_COUNT)){
			cpu_timestamps[CPU_START_TIME][loop_count] = getTimeMsec();
	        if (ADXL345_IsDataReady(file)){
	            bSuccess = ADXL345_XYZ_Read(file, szXYZ);
	            if (bSuccess){
		              cnt++;
	                //printf("[%d]X=%d mg, Y=%d mg, Z=%d mg\r\n", cnt,(int16_t)szXYZ[0]*mg_per_digi, (int16_t)szXYZ[1]*mg_per_digi, (int16_t)szXYZ[2]*mg_per_digi);
	                // show raw data,
	                //printf("X=%04x, Y=%04x, Z=%04x\r\n", (alt_u16)szXYZ[0], (alt_u16)szXYZ[1],(alt_u16)szXYZ[2]);
					cpu_timestamps[CPU_I2C_READ][loop_count] = getTimeMsec();
					// Convert to proper units
					szXYZ_data[0] = (int16_t)szXYZ[0]*mg_per_digi;
					szXYZ_data[1] = (int16_t)szXYZ[1]*mg_per_digi;
					szXYZ_data[2] = (int16_t)szXYZ[2]*mg_per_digi;

					i = averaging_filter(szXYZ_filter,szHistoricalData, szXYZ_data);

					cpu_timestamps[CPU_END_TIME][loop_count] = getTimeMsec();

					// Sleep before next acquisition
	                usleep(250*1000);

					printf("Filter time elapsed: %f\r\n",(cpu_timestamps[CPU_END_TIME][loop_count] - cpu_timestamps[CPU_START_TIME][loop_count]) );
					printf("Filter results: [%d]X=%d mg, Y=%d mg, Z=%d mg\n", szXYZ_filter[ENTRIES][X_IDX],szXYZ_filter[AVG][X_IDX],szXYZ_filter[AVG][Y_IDX],szXYZ_filter[AVG][Z_IDX]);
					loop_count++;
	            }
	        }
	    }

	    if (!bSuccess)
	        printf("Failed to access accelerometer\r\n");

			if (file)
				close(file);

			printf("gsensor, bye!\r\n");


		// clean up our memory mapping and exit

		if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
			printf( "ERROR: munmap() failed...\n" );
			close( fd );
			pthread_exit(NULL);
		}

		close( fd );

	pthread_exit(NULL);
}

// api for register access, defined in main.c
bool ADXL345_REG_WRITE(int file, uint8_t address, uint8_t value){
	bool bSuccess = false;
	uint8_t szValue[2];

	// write to define register
	szValue[0] = address;
	szValue[1] = value;
	if (write(file, &szValue, sizeof(szValue)) == sizeof(szValue)){
			bSuccess = true;
	}


	return bSuccess;
}

bool ADXL345_REG_READ(int file, uint8_t address,uint8_t *value){
	bool bSuccess = false;
	uint8_t Value;

	// write to define register
	if (write(file, &address, sizeof(address)) == sizeof(address)){

		// read back value
		if (read(file, &Value, sizeof(Value)) == sizeof(Value)){
			*value = Value;
			bSuccess = true;
		}
	}


	return bSuccess;
}

bool ADXL345_REG_MULTI_READ(int file, uint8_t readaddr,uint8_t readdata[], uint8_t len){
	bool bSuccess = false;

	// write to define register
	if (write(file, &readaddr, sizeof(readaddr)) == sizeof(readaddr)){
		// read back value
		if (read(file, readdata, len) == len){
			bSuccess = true;
		}
	}


	return bSuccess;
}

int main(int argc, char *argv[]) {
	double total_time;
	// Real-time processor initialization
	pthread_attr_t rt_sched_attr;
	pthread_t threads[NUM_THREADS];
	threadParams_t threadParams[NUM_THREADS];
	int rt_max_prio;
	struct sched_param rt_param;
	int rc, i=0;
	cpu_set_t threadcpu;
	int coreid;
	CPU_ZERO(&threadcpu);
	coreid=0;
	printf("Setting thread %d to core %d\n", i, coreid);
	CPU_SET(coreid, &threadcpu);
	printf("\nLaunching thread %d\n", i);

	// configure Real-Time scheduler attribles
	rc = pthread_attr_init(&rt_sched_attr);
	if (rc) {printf("ERROR; pthread_attr_init() rc is %d\n", rc);}
	rc = pthread_attr_setinheritsched(&rt_sched_attr, PTHREAD_EXPLICIT_SCHED);
	if (rc) {printf("ERROR; pthread_attr_setinheritsched() rc is %d\n", rc);}
	rc = pthread_attr_setschedpolicy(&rt_sched_attr, SCHED_FIFO);
	if (rc) {printf("ERROR; pthread_attr_setschedpolicy() rc is %d\n", rc);}
	//pthread_attr_setaffinity_np(&rt_sched_attr, sizeof(cpu_set_t), &threadcpu);
	rt_max_prio = sched_get_priority_max(SCHED_FIFO);
	printf("rt_max_prio=%d\n", rt_max_prio);
	rt_param.sched_priority=rt_max_prio;
	rc = pthread_attr_setschedparam(&rt_sched_attr, &rt_param);
	if (rc) {printf("ERROR; pthread_attr_setschedparam() rc is %d\n", rc);}
	// Create Thread 1
	// Thread 1 is the modifying thread - it will attemp to modify and timestamp the
	// navigations state of the system
	printf("Creating thread %d\n", THREAD_1);
	threadParams[THREAD_1].threadIdx=THREAD_1;
	rc = pthread_create(&threads[0], &rt_sched_attr, CPU_filter, (void *)&threadParams[THREAD_1]);
	if (rc) {printf("ERROR; pthread_create() rc is %d\n", rc); perror(NULL); exit(-1);}
	printf("Thread 1 spawned\n");

	printf("will try to join threads when access is complete\n");


	if(pthread_join(threads[0], NULL) == 0)
		printf("Thread 1: %x done\n", (unsigned int)threads[0]);
	else
	   perror("Thread 1");

	syslog(LOG_CRIT,"FPGA_timing,Total_Time,Req_Write,Ack_Receive,nReq_Write,Final_Read,Units\n");

	for(i = 0; i < LOOP_COUNT; i++)
	{
		total_time = (fpga_timestamps[FPGA_END_TIME][i] - fpga_timestamps[FPGA_ACK_TIME][i]);
		total_time += (fpga_timestamps[FPGA_REQ_TIME][i] - fpga_timestamps[FPGA_START_TIME][i]);
		syslog(LOG_CRIT,"FPGA_timing,%f,%f,%f,%f,%f,msec\n",
	    	total_time,fpga_timestamps[FPGA_REQ_TIME][i]-fpga_timestamps[FPGA_START_TIME][i],
			fpga_timestamps[FPGA_REQ_ACK][i]-fpga_timestamps[FPGA_ACK_TIME][i],
			fpga_timestamps[FPGA_NREQ][i]-fpga_timestamps[FPGA_REQ_ACK][i],
			fpga_timestamps[FPGA_END_TIME][i]-fpga_timestamps[FPGA_NREQ][i]);
	}

	syslog(LOG_CRIT,"CPU_timing,Total_Time,I2C_Read_Time,Filter_Time,Units\n");

	for(i = 0; i < LOOP_COUNT; i++)
	{
		total_time = (cpu_timestamps[CPU_END_TIME][i] - cpu_timestamps[CPU_START_TIME][i]);

		syslog(LOG_CRIT,"CPU_timing,%f,%f,%f,msec\n",
			total_time,cpu_timestamps[CPU_I2C_READ][i]-cpu_timestamps[CPU_START_TIME][i],
			cpu_timestamps[CPU_END_TIME][i]-cpu_timestamps[CPU_I2C_READ][i]);
	}

 	return 0;

}
