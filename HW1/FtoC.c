// Ahmed Abdulkareem
// ECE 373 
// 04/05/2016
// This program takes in a farenheit temperature value and outputs a 
// Celcius value

#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    if (argc < 2) {
        printf("Please provide a farenheit value!\n");
        return 1;
    }

    double celc = 5/9.0 * (atoi(argv[1]) - 32);
    printf("Celcius value = %.2f\n", celc);
    return 0;
}
