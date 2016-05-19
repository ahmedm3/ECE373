/*
Ahmed Abdulkareem
ECE 373
05/17/2016
HW 5
*/

/* This program follows what HW5 specifies to do:
1. Find one of ethernet devices
2. Save current LEDCTL value and print it for the user to read
3. Turn both green LEDs on for 2 seconds
4. Turn all LEDs off for 2 seconds
5. Loop 5 times and turn each LED (amber, green on right, green on left) on for 1 second
6. Restore LEDCTL to initial value
7. Read and print the contents of the Good Packets Received statistics register
8. Clean up and exit
*/

/* 
ANSWER to question 2:
We don't have to use it because we're not messing with any kernel stuff
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <getopt.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <pci/pci.h>
#include <stdint.h>
#include <linux/types.h>
#include <signal.h>

#define DOUBLE_GREEN 0x404000;
#define ALL_TURNED_OFF 0x0;

#define MEM_WINDOW_SZ  0x1F400

#define LEDCTL 0xE00
#define PACKETS 0x4074

static struct pci_filter filter;    /* Device filter */

static struct option opts[] = {
        {"address", 1, NULL, 'a' },
        {"debug", 0, NULL, 'D' },
        {"device", 1, NULL, 'd' },
        {"slot", 1, NULL, 's' },
        { 0, 0, NULL, '0' }
};

static void usage(char *progname, char *idfile)
{
        printf( "Usage: %s [options] [device]   (%s)\n\n"
                "Options:\n"
                "-D\t\tPCI debugging\n"
                "-o\t\tRegister offset\n"
                "Device:\n"
                "-d [<vendor>]:[<device>]\t\t\tShow selected devices\n"
                "-s [[[[<domain>]:]<bus>]:][<slot>][.[<func>]]"
                        "\tShow devices in selected slots\n\n",
                progname, idfile);
}

static int print_register(struct pci_dev *dev, u32 offset)
{
        volatile void *mem;
        int dev_mem_fd, i;
        dev_mem_fd = open("/dev/mem", O_RDWR);
        if (dev_mem_fd < 0) {
                perror("open");
                return -1;
        }

        mem = mmap(NULL, MEM_WINDOW_SZ, PROT_READ | PROT_WRITE, MAP_SHARED, dev__
mem_fd,(dev->base_addr[0] & PCI_ADDR_MEM_MASK));
        if (mem == MAP_FAILED) {
                perror("mmap/readable - try rebooting with iomem=relaxed");
                close(dev_mem_fd);
                return -1;
        }

        u32 OldLEDCTLAddress, NewLEDCTLAddress, packets;
        printf("current LEDCTL: 0x%x\n", *((u32 *)(mem + LEDCTL)));

        OldLEDCTLAddress = *((u32 *)(mem + LEDCTL));
        printf("Turn dobule greens!\n");
        NewLEDCTLAddress = DOUBLE_GREEN;
        *((u32 *)(mem + LEDCTL)) = NewLEDCTLAddress;

        printf("Sleeping for 2 secs!\n");
        sleep(2);

        printf("All off!\n");
        NewLEDCTLAddress = ALL_TURNED_OFF;
        *((u32 *)(mem + LEDCTL)) = NewLEDCTLAddress;

                printf("Sleeping for 2 seconds!\n");
        sleep(2);

        printf("Turn each LED on for just a sec!\n");
        NewLEDCTLAddress = 0x40;
        int i = 5;
        for( i = 0; i < 5; ++i ) {
                *((u32 *)(mem + LEDCTL)) = NewLEDCTLAddress;
                sleep(1);
                NewLEDCTLAddress = NewLEDCTLAddress << 8;

                *((u32 *)(mem + LEDCTL)) = NewLEDCTLAddress;
                sleep(1);
                NewLEDCTLAddress = NewLEDCTLAddress << 8;

                *((u32 *)(mem + LEDCTL)) = NewLEDCTLAddress;
                sleep(1);
                NewLEDCTLAddress = 0x40;
        }

        *((u32 *)(mem + LEDCTL)) = OldLEDCTLAddress;

        printf("LEDCTL initial value: 0x%x\n", *((u32 *)(mem + LEDCTL)));
        printf("DONE!!!!!!!\n");
        while(!(packets = *((u32 *)(mem + PACKETS))))
                sleep(1);

        printf("Packets Received register (good ones): 00
x%x\n", packets);


        close(dev_mem_fd);
        munmap((void *)mem, MEM_WINDOW_SZ);

        return 0;
}


int main(int argc, char **argv)
{
        int ch;
        struct pci_access *pacc;
        struct pci_dev *dev;
        char *errmsg;
        char buf[128];
        u32 offset = 0;
        u32 value = 0;
        u64 lvalue = 0;
        int device_specified = 0;
        int got_offset = 0;
        int ret;

        if (getuid() != 0) {
                printf("%s: must be run as root\n", argv[0]);
                exit(1);
        }

        pacc = pci_alloc();             /* Get the pci_access structure */
        if (pacc == NULL) {
                perror("pci_alloc");
                exit(1);
        }
        pci_filter_init(pacc, &filter);

        while ((ch = getopt_long(argc, argv, "o:d:s:v:", opts, NULL)) != -1) {
                switch (ch) {
                case 'v':
                        value = strtol(optarg, NULL, 0);
                        break;
                case 'D':
                        pacc->debugging++;
                        break;
                case 'o':
                        offset = strtol(optarg, NULL, 0);
                        got_offset++;
                        break;
                case 'd':
                        /* Show only selected devices */
                        if ((errmsg = pci_filter_parse_id(&filter, optarg))) {
                                printf("%s\n", errmsg);
                                exit(1);
                        }
                        device_specified++;
                        break;
                case 's':
                        /* Show only devices in selected slots */
                        if ((errmsg = pci_filter_parse_slot(&filter, optarg))) {
                                printf("%s\n", errmsg);
                                exit(1);
                        }
                        device_specified++;
                        break;
                case '?':
                default:
                        usage(argv[0], pacc->id_file_name);
                        exit(1);
                        break;
                }
        }

        if (!device_specified) {
                printf("No device given\n");
                usage(argv[0], pacc->id_file_name);
                exit(1);
        }

        if (!got_offset) {
                printf("No offset given\n");
                usage(argv[0], pacc->id_file_name);
                exit(1);
        }

        pci_init(pacc);                 /* Initialize the PCI library */
        pci_scan_bus(pacc);             /* Get the list of devices */

        if (pacc->debugging)
                printf( "filter: "
                        "bus=0x%x slot=0x%x func=0x%x\n"
                        "\tvendor=0x%x device=0x%x\n\n",
                        filter.bus, filter.slot, filter.func,
                        filter.vendor, filter.device);

        /* Iterate over all devices to find the single one we want */
        for (dev = pacc->devices; dev; dev = dev->next) {
                if (pci_filter_match(&filter, dev))
                        break;
        }

        if (!dev) {
                printf("no device found\n");
                ret = -1;
                goto out;
        }

        /* Fill in header info we need */
        pci_fill_info(dev, PCI_FILL_IDENT | PCI_FILL_BASES | PCI_FILL_SIZES);

        printf( "%02x:%02x.%d (%04x:%04x)\n%s\n",
                        dev->bus, dev->dev, dev->func,
                        dev->vendor_id, dev->device_id,
                        pci_lookup_name(pacc, buf, sizeof(buf),
                                PCI_LOOKUP_VENDOR|PCI_LOOKUP_DEVICE,
                                dev->vendor_id, dev->device_id, 0, 0));

        ret = print_register(dev, offset);

out:
        pci_cleanup(pacc);              /* Close everything */
        return ret;
}
