/*
Ahmed Abdulkareem
ECE 373, HW6
06/03/2016
ECE LED Driver
*/


#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

int main(int argc, char *argv[])
{
	int fd;
	ssize_t num_read;
	int head, tail;
	char  my_read[10];
	
	// Open the kernel
	fd = open("/dev/HW6", O_RDWR);
 	 if (fd==-1) 
	{
		perror("cannot open file! \n");
		return -1;
	}

	// Read The value syscall_val from the kernel
	num_read = read(fd, &my_read, sizeof(int));
  	
	if (num_read==-1)
	{
		perror("Info from kernel cannot be read! \n");
		return -1;
	}
	head = (*my_read >> 16);
	tail = *my_read & 0xFFFF;

	printf("Head: 0x%x\n",head);
	printf("Tail: 0x%x\n",tail);
	
	close(fd);

	return 0;
}
