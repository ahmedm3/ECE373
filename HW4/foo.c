// Ahmed Abdulkareem
// 05/07/2016
// HW 4
// ECE 373

// this program will read the current value 
// of LED register, write the new value to turn LED on for
// 2 seconds, display the new value, write a value to turn led off

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int main()
{
	int fd;
	ssize_t num_read, num_written;
	unsigned int  my_read_str;
	const char my_write_str[10] = "78";
	const char LED_REG_OFF[10] = "15";

	fd = open("/dev/part4", O_RDWR); //open char device

	num_read = read(fd, &my_read_str, 0); //read 
	printf("current value of the LED reg is 0x%06x, (%zd bytes)\n",
	      my_read_str, num_read); //output value

	num_written = write(fd, my_write_str, sizeof(my_write_str)); // write new value to turn off LED
	printf("successfully wrote %s (%zd bytes) to the LED reg to turn LED on\n",my_write_str, num_written);

	num_read = read(fd, &my_read_str, 0);
	printf("Value of the LED reg after being written is 0x%04x, (%zd bytes)\n", my_read_str, num_read);

	sleep(2);

	num_written = write(fd, LED_REG_OFF, sizeof(LED_REG_OFF));
	printf("successfully wrote %s (%zd bytes) to the LED reg to turn LED off\n",LED_REG_OFF, num_written);	
	
	close(fd); 

	return 0;
}
