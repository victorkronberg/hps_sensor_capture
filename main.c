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
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "ADXL345.h"

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )



int main(int argc, char *argv[]) {

	// I2C variables
	int file;
	const char *filename = "/dev/i2c-0";
	uint8_t id;
	bool bSuccess;
	const int mg_per_digi = 4;
	uint16_t szXYZ[3];
	int cnt=0, max_cnt=0;

	// LED variables
	void *virtual_base;
	int fd;
	int loop_count;
	int sw_mask;
	void *h2p_lw_led_addr;
	void *h2p_lw_sw_addr;

/////////////////////////////////////////gsensor init and configuration//////////////////////////////////
	printf("===== gsensor test =====\r\n");

	if (argc == 2){
		max_cnt = atoi(argv[1]);
	}

	// open bus
	if ((file = open(filename, O_RDWR)) < 0) {
  	  /* ERROR HANDLING: you can check errno to see what went wrong */
	    perror("Failed to open the i2c bus of gsensor");
  	  exit(1);
	}


	// init
	// gsensor i2c address: 101_0011
	int addr = 0b01010011;
	if (ioctl(file, I2C_SLAVE, addr) < 0) {
  	  printf("Failed to acquire bus access and/or talk to slave.\n");
	    /* ERROR HANDLING; you can check errno to see what went wrong */
  	  exit(1);
	}


///////////////IO configuration and memory mapping////////////////////////////////////////////////////////////////
	// map the address space for the LED registers into user space so we can interact with them.
	// we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span

	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}

	virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );

	if( virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	h2p_lw_led_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_OUTPUT_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_sw_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PIO_INPUT_BASE ) & ( unsigned long)( HW_REGS_MASK ) );


	// toggle the LEDs a bit

	loop_count = 0;
	while( loop_count < 60 ) {

		// control led
		*(uint32_t *)h2p_lw_led_addr = sw_mask;
		sw_mask = *(uint32_t *)h2p_lw_sw_addr;

		// wait 250ms
		usleep( 250*1000 );

		loop_count++;


	} // while

	// configure accelerometer as +-2g and start measure////////////////////////////////////////
    bSuccess = ADXL345_Init(file);
    if (bSuccess){
        // dump chip id
        bSuccess = ADXL345_IdRead(file, &id);
        if (bSuccess)
            printf("id=%02Xh\r\n", id);
    }

	while(bSuccess && (max_cnt == 0 || cnt < max_cnt)){
        if (ADXL345_IsDataReady(file)){
            bSuccess = ADXL345_XYZ_Read(file, szXYZ);
            if (bSuccess){
	              cnt++;
                printf("[%d]X=%d mg, Y=%d mg, Z=%d mg\r\n", cnt,(int16_t)szXYZ[0]*mg_per_digi, (int16_t)szXYZ[1]*mg_per_digi, (int16_t)szXYZ[2]*mg_per_digi);
                // show raw data,
                //printf("X=%04x, Y=%04x, Z=%04x\r\n", (alt_u16)szXYZ[0], (alt_u16)szXYZ[1],(alt_u16)szXYZ[2]);
                usleep(1000*1000);
            }
        }
    }

    if (!bSuccess)
        printf("Failed to access accelerometer\r\n");

		if (file)
			close(file);

		printf("gsensor, bye!\r\n");


	return 0;
	// clean up our memory mapping and exit

	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	close( fd );

	return( 0 );
}
