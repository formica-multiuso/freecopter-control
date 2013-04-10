#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>

#define PACKET_LENGHT 87	// Packet Lenght in CHAR


typedef struct data
{
	int gyroX;
	int gyroY;
	int gyroZ;
	int accelX;
	int accelY;
	int accelZ;
	int magnX;
	int magnY;
	int magnZ;

} inertial_data;				// Actually not used!


uint32_t imu_data[12] = {0,0,0,0,0,0,0,0,0,0,0,0};


int tokenizer(char *s)
{
	
	char dels[] = ":";
	int i=0;
        int j=0;
	char *p;

	for(p=strtok(s,dels); p!=NULL; p=strtok(NULL,dels))
	{
		//      printf("%s\n",p);
		if(j>0 && j<14)
			imu_data[i++] = atoi(p);
		j++;
	}

	return 1;
}

int main(int argc, char *argv[])
{
	
	char s;
	int serialfd,n;

	uint32_t tRoll,tPitch,tYaw;
	float Roll,Pitch,Yaw;
	
	char dels[] = ":";			// List of delimitators
	char packet[PACKET_LENGHT];
	
	int packet_flag = 0;
	
//	int imu_data[9] = {0,0,0,0,0,0,0,0,0};
	int i = 0;
	int j = 0;
	int k = 0;

	if(argc<2)
	{
		printf("Insert device parameter: /dev/ttyXXX\n");
		return 0;
	}
	
	serialfd = open(argv[1], O_RDONLY | O_NOCTTY | O_SYNC);
	if (serialfd == -1)
	{
		perror("open_port: Unable to open the file descriptor");
	}

	while(read(serialfd, &s, 1))
	{	
		//printf("%c",s);
		
		if((packet_flag == 0) && (s == 'S'))
		{	
			packet_flag = 1;
		}
		else if((packet_flag == 1) && (s == 'E'))
		{	
			packet_flag = 0;
			i = 0;
			packet[86] = '\0';
			tokenizer(packet);
		//	printf("%s\n",packet);

			tRoll = (uint32_t)((imu_data[0] << 31) | (imu_data[1] << 23) | (imu_data[2] << 11) | imu_data[3]);
			tPitch = (uint32_t)((imu_data[4] << 31) | (imu_data[5] << 23) | (imu_data[6] << 11) | imu_data[7]);
			tYaw = (uint32_t)((imu_data[8] << 31) | (imu_data[9] << 23) | (imu_data[10] << 11) | imu_data[11]);

			Roll = (*(float*)&tRoll);
			Pitch = (*(float*)&tPitch);
			Yaw = (*(float*)&tYaw);
			printf("Roll:%10f,Pitch:%10f,Yaw:%10f\n",Roll,Pitch,Yaw);

//			printf(":%6d:%6d:%6d:%6d:%6d:%6d:%6d:%6d:%6d:%6d:%6d:%6d\n",imu_data[0],imu_data[1],imu_data[2],imu_data[3],imu_data[4],imu_data[5],imu_data[6],imu_data[7],imu_data[8],imu_data[9],imu_data[10],imu_data[11]);
		}
		
		if(packet_flag == 1)
			packet[i++] = s;
		
	}
	return 0;


}
