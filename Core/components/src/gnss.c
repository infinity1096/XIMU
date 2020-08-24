/*
 * GNSS.c
 *
 *  Created on: Jun 14, 2020
 *      Author: yuchen
 */

#include "gnss.h"

void GNSS_set_huart(UART_HandleTypeDef* huart){
	huartx = huart;
}

/**
 * parse GNSS.GNGGA sentence into fields as strings.(char[] s)
 * @param msg GNSS.GNGGA message
 * @param fields GNSS.GNGGA message parser output
 */
void parse_GNGGA(unsigned char* msg, char fields[][MAX_FIELD_SIZE]){
    int current_field = 0, current_pos = 0;
    unsigned char* ch_ptr = msg;
    while (*ch_ptr != '\0'){
        current_pos = 0;
        while (*ch_ptr != ',' && *ch_ptr != '\0'){
	        fields[current_field][current_pos] = *ch_ptr;
	        current_pos++;
	        ch_ptr++;
	    }
        fields[current_field][current_pos]='\0';
	    current_field++;
	    if (*ch_ptr != '\0'){ // skip ',' when not at end of string
	        ch_ptr++;
	    }
    }
}

/**
 * convert raw latitude and longitude, which have format dddmm.mmmm
 * into decimal.
 * @param raw raw latitude and longitude input
 * @return decimal latitude and longitude
 */
double convert_raw_lat_lon(double raw){
	return ((int)raw / 100) + (fmod(raw,100)/60);
}

/**
 * initializes the GNSS interrupt.
 *
 * call this function before main loop.
 */
void GNSS_Init(){
	HAL_UART_Receive_DMA(huartx, GNSS.dma_buffer, DMA_BUFFER_SIZE);
	//https://www.devcoons.com/stm32-uart-receive-unknown-size-data-using-dma-and-freertos/
}

int GNSS_RX_Update(){

	__disable_irq();//I don't understand this
	int i = DMA_BUFFER_SIZE - huartx->hdmarx->Instance->CNDTR; //last byte received
	__enable_irq();//I don't understand this

	int new_information = 0;

	while (GNSS.start != i){

		if (GNSS.dma_buffer[GNSS.start] != '$'){
			//find new start
			for(;GNSS.start != i;GNSS.start = (GNSS.start + 1) % DMA_BUFFER_SIZE){
				if (GNSS.dma_buffer[GNSS.start] == '$'){
					break;
				}
			}
		}

		if (GNSS.dma_buffer[GNSS.start] != '$'){break;}//if start not found, return.

		int end = GNSS.start;
		for(end = GNSS.start; end != i; end = (end + 1) % DMA_BUFFER_SIZE){
			if (GNSS.dma_buffer[end] == '\n'){
				break;
			}
		}

		if (GNSS.dma_buffer[end] != '\n'){break;}//if end not found, return.

		//parse message from start to end

		//copy message to parse
		int counter = 0;
		for(int j = GNSS.start; j != end; j = (j+1)%DMA_BUFFER_SIZE){
			GNSS.message_buffer[counter] = GNSS.dma_buffer[j];
			counter++;
		}
		GNSS.message_buffer[counter] = '\n';
		GNSS.message_buffer[counter+1] = '\0';
		GNSS.start = end;

		//parse message if it is GNGGA
		char* str = strstr((char*)(GNSS.message_buffer),"$GNGGA,");

		if (str != NULL){
			char fields[NUM_FIELDS][MAX_FIELD_SIZE];
			parse_GNGGA(GNSS.message_buffer,fields);
			//field 1: UTC time
			sscanf(fields[1],"%2d%2d%2d.%3d",&GNSS.GNGGA.UTC_Hour,&GNSS.GNGGA.UTC_Min,&GNSS.GNGGA.UTC_Second,&GNSS.GNGGA.UTC_Millis);
			//field 2: lat
			GNSS.GNGGA.lat_raw = atof(fields[2]);
			//field 3: NorthSouth
			GNSS.GNGGA.North_South = fields[3][0];
			//field 4: lat
			GNSS.GNGGA.lon_raw = atof(fields[4]);
			//field 5: EastWest
			GNSS.GNGGA.East_West = fields[5][0];
			//field 6: status
			GNSS.GNGGA.status = atoi(fields[6]);
			//field 7: # of satellites
			GNSS.GNGGA.num_sat = atoi(fields[7]);
			//field 8: HDOP
			GNSS.GNGGA.HDOP = atof(fields[8]);
			//field 9: Mean surface level altitude
			GNSS.GNGGA.MSL_alt = atof(fields[9]);
			//field 10: unit of MSL altitude
			GNSS.GNGGA.MSL_Unit = fields[10][0];
			//field 11: Geoid separation
			GNSS.GNGGA.Geoid_Separation = atof(fields[11]);
			//field 12: unit of Geoid separation
			GNSS.GNGGA.Geoid_Unit = fields[12][0];
			//field 13: empty
			//field 14: checksum
			GNSS.GNGGA.checksum[0] = fields[14][1];//fields[14][0] is always '*'
			GNSS.GNGGA.checksum[1] = fields[14][2];

			//convert raw latitude and longitude to decimal format
			GNSS.GNGGA.lat = convert_raw_lat_lon(GNSS.GNGGA.lat_raw);
			GNSS.GNGGA.lon = convert_raw_lat_lon(GNSS.GNGGA.lon_raw);

			//add sign based on N/S , E/W
			GNSS.GNGGA.lat = GNSS.GNGGA.North_South == 'N' ? GNSS.GNGGA.lat : -GNSS.GNGGA.lat;
			GNSS.GNGGA.lon = GNSS.GNGGA.East_West == 'E' ? GNSS.GNGGA.lon : -GNSS.GNGGA.lon;

			new_information = 1;
		}
	}

	return new_information;
}




