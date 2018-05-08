#include <stdio.h>
#include <string.h>
#include "../include/SonarMessages.h"

int main(){

	FILE *fid;
	int i;
	SonarMessageHeaderType hdr; /* Basic 16-byte message header */
	fid = fopen("../data_test/MMT_354_INE_FR_SSS_SS_ATR_3.016.jsf", "rb");
	if (fid == NULL) return 0;
	while(!feof(fid)){
		if (fread(&hdr, sizeof(hdr), 1, fid) != 1)
		break;
		if (hdr.startOfMessage != SONAR_MESSAGE_HEADER_START){
			printf("Invalid file format\n");
			break;
		}
		printf("Size of the message: %lu \n", hdr.byteCount);
		for(i = 0; i < hdr.byteCount; i++){
			if (getc(fid) == EOF){
				printf("Invalid file format\n");
				break;
			}
		}
		printf("Message Type %d\n", hdr.sonarMessage);
	}
	fclose(fid);

	return 0;
}
