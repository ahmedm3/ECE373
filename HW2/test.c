// Ahmed Abdulkareem
// 04/10/2016
// Tests read and write functions implemented in part2.c

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>

int main()
{
	int fd;
	ssize_t num_read, num_read1, num_written;
	char my_read_str[10], my_read_str1[10];
	const char my_write_str[27] = "5";

	fd = open("/dev/part2", O_RDWR);

	num_read = read(fd, &my_read_str, 10);

	printf("returned %d from the system call, num bytes read: %zu\n",
	       *my_read_str, num_read);

	num_written = write(fd, my_write_str, sizeof(my_write_str));
	printf("successfully wrote %zu bytes to the kernel\n", num_written);

	num_read1 = read(fd, &my_read_str1, sizeof(int) - 1);
	printf("returned %d from the system call, num bytes read: %zu\n", *my_read_str1, num_read1);

	close(fd);

	return 0;
}
