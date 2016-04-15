/**
 * Project Untitled
 */

#include <stdlib.h>
#include "AcqLidar.h"

#define DEVICE_PORT "/dev/ttyUSB0"
#define PERSISTANCE 2
/**
 *
 */
AcqLidar::AcqLidar() {
	for (int i = 0; i < NB_PTS; ++i) {
		buffData[i].azimut = 0;
		buffData[i].distance = 0;
		buffData[i].quality = 0;
		buffData[i].updTour = 0;
		buffData[i].valid = 0;
		buffData[i].warning = 0;
	}
	nbRevo = 0;
}

/**
 * Read LIDAR data from serial, and call #updateBuff for each measures.
 * @return
 */
int AcqLidar::updateData() {
	static int init_level = 0;
	static unsigned char index = 0;
		if(init_level == 0){
			unsigned char b=0;
			serial.ReadChar(&b);
			if(b==0xFA){
				init_level = 1;
			}
			else{
				init_level = 0;
			}
		}
		else if(init_level == 1){
			unsigned char b=0;
			serial.ReadChar(&b);
			if(b>=0xA0 && b<= 0xF9){
				index = b-0XA0;
				init_level = 2;
			}
			else if( b != 0xFA){
				init_level = 0;
			}
		}
		else if(init_level == 2){
			unsigned char b_speed[2];
			unsigned char b_data0[4];
			unsigned char b_data1[4];
			unsigned char b_data2[4];
			unsigned char b_data3[4];
			unsigned char b_checksum[2];

			serial.Read(b_speed,2);
			serial.Read(b_data0,4);
			serial.Read(b_data1,4);
			serial.Read(b_data2,4);
			serial.Read(b_data3,4);
			serial.Read(b_checksum,2);
			int checksum = b_checksum[0] | b_checksum[1] << 8;

			unsigned char all_data[20];
			all_data[0] = 0xFA;
			all_data[1] = index+0xA0;
			for(int i = 0;i<2;i++){ all_data[i+2]=b_speed[i]; }
			for(int i = 0;i<4;i++){ all_data[i+4]=b_data0[i]; }
			for(int i = 0;i<4;i++){ all_data[i+8]=b_data1[i]; }
			for(int i = 0;i<4;i++){ all_data[i+12]=b_data2[i]; }
			for(int i = 0;i<4;i++){ all_data[i+16]=b_data3[i]; }

			if(calc_checksum(all_data) == checksum){
				updateBuff(index * 4 + 0, b_data0);
				updateBuff(index * 4 + 1, b_data1);
				updateBuff(index * 4 + 2, b_data2);
				updateBuff(index * 4 + 3, b_data3);
			}
			else{
				printf("ERROR\n");
				unsigned char error_tab[] = {0, 0x80, 0, 0};
				for(int i = 0;i<4;i++){ updateBuff(index * 4 + i, error_tab); }
			}
			init_level = 0;
		}
		else{
			printf("WTF ? keskicepass ?");
			init_level = 0;
		}

	return 0;
}

/**
 * @return vector<PtLidar>
 */
vector<PtLidar> AcqLidar::getData() {
	vector<PtLidar> vdata(buffData, buffData + NB_PTS);
    return vdata;
}

/**
 * @return int
 */
int AcqLidar::init() {
	int ret = 0;
		if((ret=serial.Open(DEVICE_PORT,115200)) != 1) {
			printf ("Error while opening port. Permission problem ?\n");
			return ret;
		}
    return 0;
}

/**
 * @return int
 */
void AcqLidar::close() {
	serial.Close();
}

/**
 *
 * @param angle
 * @param data
 */
void AcqLidar::updateBuff(int angle, unsigned char * data) {
	unsigned char x = data[0];
	unsigned char x1 = data[1];
	unsigned char x2 = data[2];
	unsigned char x3 = data[3];

	int dist_mm = x | ((x1 & 0x3f) << 8);
	int quality = x2 | (x3 << 8);

	if (data[1] & 0x80 ) {	//bad data
		if(nbRevo - buffData[angle].updTour > PERSISTANCE) {	//data to old
			buffData[angle].valid = false;
		}
	}
	else {
		buffData[angle].azimut = angle;
		buffData[angle].distance = dist_mm;
		buffData[angle].quality = quality;
		buffData[angle].updTour = nbRevo;
		buffData[angle].valid = true;

		if (x1 & 0x40) {				//WARNING flag
			buffData[angle].warning = true;
		} else {
			buffData[angle].warning = false;
		}
	}
}

/**
 * @param data
 *
 * Compute and return the checksum as an int.
 * data is an array of 20 bytes, in the order they arrived in.
 *
 */
int AcqLidar::calc_checksum(unsigned char * data) {
	int data_tab[10];

	for(int i=0;i<10;i++){
		data_tab[i] = data[2*i] | data[2*i+1]<<8;
	}

	int chk32 = 0;
	for(int i=0;i<10;i++){
		chk32 = (chk32 << 1) + data_tab[i];
	}

	int checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 );
	checksum = checksum & 0x7FFF;
	return checksum;
    return 0;
}
