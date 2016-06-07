/*
Ahmed Abdulkareem
ECE 373, HW6
06/03/2016
ECE LED Driver
*/


#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

#define DEVCNT 1
#define DEVNAME "HW6"
#define PE_LED_MASK 0xC
#define PE_REG_LEDS 0xe00
#define PE_REG_CTRL 0x0004
#define RDBAL 0x2800
#define RDBAH 0x2804

// Define for interrupt
#define ICR   0x000C0
#define IMS   0x000D0
#define IMC   0x000D8
#define ICS   0x000C8
#define IRQ_REG_TX  0x0001
#define IRQ_REG_LSC 0x0005

// Define Receive Register
#define RCTL 0x00100 
#define IRQ_REG_RXQ 0x100000
#define RDLEN 0x2808
#define HEAD 0x2810
#define TAIL 0x2818

static char *pe_driver_name = "Realtek_poker";
static struct cookie
{
	struct pci_dev *pdev;
}irq;

// Struct buffer infor
struct buf_info
{
	dma_addr_t phy;
	void *mem;
}buf_info[16];

// Receive Descriptor
struct e1000e_rx_desc
{
	__le64 buffer_addr; // Address of the descriptor's data buffer 
	union
	{
		__le32 data;
		struct
		{
			__le16 length; // Data buffer length 
			u16 cso; // Checksum offset
		} flags;
	} lower;
	union 
	{
		__le32 data;
		struct
		{
			u8 status; // Descriptor status
			u8 error; // Checksum start 
			__le16  Vlan;
		} field;
	} upper;
};

static DEFINE_PCI_DEVICE_TABLE(pe_pci_tbl) = {
	{ PCI_DEVICE(0x8086, 0x150c) },
	{ }, /* must have an empty entry at the end! */
};


// Create Parameter delay with default = 2
static int blink_rate = 2;
module_param(blink_rate,int,S_IRUSR | S_IWUSR);

static struct timer_list my_time;

// Struct Pes
struct pes {
	struct pci_dev *pdev;
	void *hw_addr;
	struct cdev cdev;
	struct work_struct task;
};

// Struct mydev
 static struct mydev_dev {
	struct cdev cdev;
	void *hw_addr;    
	u32 syscall_val;  
	dma_addr_t addr;
	struct e1000e_rx_desc  *cpu_addr;
	struct work_struct service_task;	
	int tail; 
	int head_tail;
} mydev; 


static void irq_work(struct work_struct *work)
{
	printk(KERN_INFO "IN WORK QUEUE !!!!!\n");
	printk(KERN_INFO "Start and sleep 0.5 seconds \n"); 
	msleep(500);
	printk(KERN_INFO "Turn off both green LED");
	writel(0x0F0F0F, mydev.hw_addr + PE_REG_LEDS);
	printk(KERN_INFO "Task is doing ! \n");
}


// REQUEST IRQ
static irqreturn_t pe_irq(int irq,void *data)
{
	int i;
	u32 status;

// Turn on both Green LED when go to interrupt handler
	printk(KERN_INFO "Got an interrupt! \n");
	writel(0x4E4E0F,mydev.hw_addr + PE_REG_LEDS);
	writel(0x100000,mydev.hw_addr + IMS);
	status = readl(mydev.hw_addr+ICR);
	status = status & 0x00FFFFFF; 
	printk(KERN_INFO "Status value: 0x%x \n",status);
	for(i=0;i<16;i++)
	printk(KERN_INFO "Data: 0x%x    DD bit: 0x%x\n",
	mydev.cpu_addr[i].upper.data,mydev.cpu_addr[i].upper.field.status &0x1);
	
	if(mydev.tail==15)
		mydev.tail=0;
	else
	{
		mydev.cpu_addr[mydev.tail].upper.field.status &= 0xFE;
		writel(mydev.tail,mydev.hw_addr + TAIL);
		mydev.tail++;
	}

        // interrupt signal
	switch(status)
	{
	 
		case IRQ_REG_TX:
		case IRQ_REG_LSC:
		case IRQ_REG_RXQ: 
			writel(status,mydev.hw_addr + IMC);
			schedule_work(&mydev.service_task);
			break;
		default:
			printk(KERN_INFO "Unknow IRQ \n");
			writel(0x4,mydev.hw_addr + IMS);
			return IRQ_NONE;
		
	}

        // Clear interrupt bit
	writel(0x4,mydev.hw_addr+ IMS);
	return IRQ_HANDLED;
}  

static int flag;

static void timer_cb(unsigned long flag)
{
	if(blink_rate <=0)
	{
	 printk(KERN_ERR "Error! \n");
	 return;
	}

	if(mydev.syscall_val==0x7844E)
	{	
		flag=0;
		mydev.syscall_val = 0x7840E;
		printk(KERN_ERR "LED is off: %x, flag : %lu\n",mydev.syscall_val,flag);
		writeb(mydev.syscall_val,(mydev.hw_addr + PE_REG_LEDS));
		mod_timer(&my_time,HZ/blink_rate + jiffies);
	}
	else
	{	
		flag=1;
		mydev.syscall_val = 0x7844E;
		printk(KERN_ERR "LED is on : %x, flag : %lu\n",mydev.syscall_val,flag);
		writeb(mydev.syscall_val,(mydev.hw_addr+ PE_REG_LEDS));
		mod_timer(&my_time,HZ/blink_rate + jiffies);
	}
}

static int pe_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct pes *pe;
	u32 ioremap_len;
		
	int err;
	int i;
	unsigned int reg;
	int temp;
	err = pci_enable_device_mem(pdev);
	if (err)
		return err;

	 // set up for high or low DMA
	err = dma_set_mask(&pdev->dev, DMA_BIT_MASK(64));
	if (err) {
		dev_err(&pdev->dev, "DMA configuration failed: 0x%x\n", err);
		goto err_dma;
	}

	// set up pci connections 
	err = pci_request_selected_regions(pdev, pci_select_bars(pdev,
					   IORESOURCE_MEM), pe_driver_name);
	if (err) {
		dev_info(&pdev->dev, "pci_request_selected_regions failed %d\n", err);
		goto err_pci_reg;
	}

	pci_set_master(pdev);

	pe = kzalloc(sizeof(*pe), GFP_KERNEL);
	if (!pe) {
		err = -ENOMEM;
		goto err_pe_alloc;
	}
	pe->pdev = pdev;
	pci_set_drvdata(pdev, pe);
	
	// map device memory 
	ioremap_len = min_t(int, pci_resource_len(pdev, 0),32768 );
	
	// Taking address of LED for mydev
	mydev.hw_addr = ioremap(pci_resource_start(pdev, 0), ioremap_len);
	if (!mydev.hw_addr) {
		err = -EIO;
		dev_info(&pdev->dev, "ioremap(0x%04x, 0x%04x) failed: 0x%x\n",
			 (unsigned int)pci_resource_start(pdev, 0),
			 (unsigned int)pci_resource_len(pdev, 0), err);
		goto err_ioremap;
	}
	pe->hw_addr = ioremap(pci_resource_start(pdev,0),ioremap_len);
        
	//Read current value LED from address LED
	mydev.syscall_val = readl(mydev.hw_addr + PE_REG_LEDS);
	
	// Display it in Kernel
	dev_info(&pdev->dev, "led_reg = 0x%02x\n",mydev.syscall_val);

	mydev.cpu_addr= dma_alloc_coherent(&pdev->dev,256,&mydev.addr,GFP_KERNEL);
	reg = (mydev.addr >> 32) & 0xFFFFFFFF;
	printk(KERN_INFO "Higher: 0x%x \n", reg);
	writel(reg, mydev.hw_addr + RDBAH);

	reg = mydev.addr &0xFFFFFFFF;
	printk(KERN_INFO "Lower: 0x%x \n", reg);
	writel(reg,mydev.hw_addr+ RDBAL);
	
        // 16 descriptors
	for(i=0; i<16; ++i)
	{
		buf_info[i].mem= kzalloc(2048,GFP_KERNEL);
		if(!buf_info[i].mem)
		{
			err = -ENOMEM;
			goto err_descriptor;
		}
		buf_info[i].phy = dma_map_single(&pdev->dev,buf_info[i].mem,2048,DMA_TO_DEVICE);
		if(!buf_info[i].phy)
		{
			err = -ENOMEM;
			goto err_descriptor;
		} 
	}

// Set the Length
	temp = readl(mydev.hw_addr + RDLEN);
        temp = temp & 0x0;
	temp = temp | 0x2000;
        writel(temp,mydev.hw_addr + RDLEN);

	printk(KERN_INFO "Ring Descriptor successfully created!  \n");
	printk(KERN_INFO "INFORMATION ABOUT PHY ADDRESS! \n");
	for(i=0;i<16;i++)
		printk(KERN_INFO " The address is: 0x%x \n",(int)buf_info[i].phy);
	

	printk(KERN_INFO "Filling descriptor buffer:");
	for(i=0;i<16;i++)
	{
		mydev.cpu_addr[i].buffer_addr= buf_info[i].phy;
		printk(KERN_INFO "Address of buffer: 0x%x \n",(int)mydev.cpu_addr[i].buffer_addr);
	
	}
	
	// Enable receiving
	temp = readl(mydev.hw_addr + PE_REG_CTRL); // Read
	temp = temp | 0x40; // Modify
	printk(KERN_INFO "Receive Enable: 0x%x",temp);
	writel(temp,mydev.hw_addr + PE_REG_CTRL); // Write
	temp = readl(mydev.hw_addr + RCTL); // Read
	temp = temp | 0x10 ; // Modify
	writel(temp,mydev.hw_addr + RCTL); // Write

	// Set tail descriptor
	writel(15,mydev.hw_addr + TAIL);
	temp = readl(mydev.hw_addr + RCTL); // Read
        temp = temp | 0x2 ; // Modify
        writel(temp,mydev.hw_addr + RCTL); // Write

        // Set Enable Receive enable RCTL
	writel(0x100000,mydev.hw_addr + ICS);
	writel(0x100000,mydev.hw_addr + IMS);

        // Request IRQ
	if (request_irq(pdev->irq,pe_irq,IRQF_SHARED,"Ahmed Abdulkareem",(void *)&irq));
		dev_info(&pdev->dev,"IRQ requested successfully \n");
	printk(KERN_INFO "value: 0x%x",readl(mydev.hw_addr+IMS));

        // Work queue
	INIT_WORK(&mydev.service_task,irq_work);
	return 0;
 
err_descriptor:

err_ioremap:
	kfree(pe);

err_pe_alloc:
	pci_release_selected_regions(pdev, pci_select_bars(pdev, IORESOURCE_MEM));

err_pci_reg:

err_dma:
	pci_disable_device(pdev);
	return err;
}

static void pe_remove(struct pci_dev *pdev)
{
	int i;
	struct pes *pe = pci_get_drvdata(pdev);
	cancel_work_sync(&mydev.service_task);

        // Free ring descriptor
	for(i=0;i<16;i++)
		dma_unmap_single(&pdev->dev,buf_info[i].phy,2048,DMA_TO_DEVICE);
	dma_free_coherent(&pdev->dev,256,mydev.cpu_addr,mydev.addr);
	free_irq(pdev->irq,(void *)&irq);	
	iounmap(pe->hw_addr);
	kfree(pe);
	pci_release_selected_regions(pdev, pci_select_bars(pdev, IORESOURCE_MEM));
	pci_disable_device(pdev);	
}

static struct pci_driver pe_driver = {
	.name     = "pci_HW6",
	.id_table = pe_pci_tbl,
	.probe    = pe_probe,
	.remove   = pe_remove,
};

// Device Driver
static dev_t mydev_node;

/* this shows up under /sys/modules/HW6/parameters */
static int exam = 15;
module_param(exam, int, S_IRUSR | S_IWUSR);
/* this doesn't appear in /sys/modules */
static int exam_nosysfs = 15;
module_param(exam_nosysfs, int, 0);

static int HW6_open(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "successfully opened! \n");
	return 0;
}

static ssize_t HW6_read(struct file *file, char __user *buf,
                             size_t len, loff_t *offset)
{
	// Get a local kernel buffer set aside
	int ret;
	int temp;
	if (*offset >= sizeof(int))
		return 0;

	// Make sure our user wasn't bad
	if (!buf) {
		ret = -EINVAL;
		goto out;
	}
	// Copy blink_rate from Kernel to User space
	if (copy_to_user(buf, &mydev.head_tail, sizeof(int))) {
		ret = -EFAULT;
		goto out;
	}
	ret = sizeof(int);
	*offset += len;
	mydev.head_tail = readl(mydev.hw_addr + HEAD); // Read HEAD's value
	mydev.head_tail = mydev.head_tail << 16; // 16 bit left shift
	temp = readl(mydev.hw_addr + TAIL); // Read TAIL's value
	mydev.head_tail = mydev.head_tail | temp; // 16 bit right shift	

out:
	return ret;
}

static ssize_t HW6_write(struct file *file, const char __user *buf,
                              size_t len, loff_t *offset)
{
	// Have local kernel memory ready
	char *kern_buf;
	int ret;
	
	// Make sure our user isn't bad
	if (!buf) {
		ret = -EINVAL;
		goto out;
	}

	// Get memory to copy into
	kern_buf = kmalloc(len, GFP_KERNEL);

	// Make sure the memory is good to go
	if (!kern_buf) {
		ret = -ENOMEM;
		goto out;
	}

	// Copy from the user buffer
	if (copy_from_user(kern_buf, buf, len)) {
		ret = -EFAULT;
		goto mem_out;
		goto out;
	}

	// Update syscall_val value
	if (kstrtoint(kern_buf,10,&blink_rate))
        {
		ret = -ERANGE;
		goto mem_out;
	}
	ret = len;

	// Print syscall_val's new value
	printk(KERN_INFO "The new value for blink_rate is: \"%d \n", blink_rate);
	
mem_out:
	kfree(kern_buf);
out:
	return ret;
}

// File operations for this device
static struct file_operations mydev_fops = {
	.owner = THIS_MODULE,
	.open = HW6_open,
	.read = HW6_read,
	.write = HW6_write,
};

static int __init HW6_init(void)
{
	int ret;
	printk(KERN_INFO "%s Module is loaded\n", pe_driver.name);
	
	/* Dynamically allocalte a char driver*/
	if (alloc_chrdev_region(&mydev_node, 0, DEVCNT, DEVNAME)) {
		printk(KERN_ERR "alloc_chrdev_region() failed!\n");
		return -1;
	}
	printk(KERN_INFO "Allocated %d devices at major: %d\n", DEVCNT,
	       MAJOR(mydev_node));

	/* Initialize the character device and add it to the kernel */
	cdev_init(&mydev.cdev, &mydev_fops);
	mydev.cdev.owner = THIS_MODULE;

	if (cdev_add(&mydev.cdev, mydev_node, DEVCNT)) {
		printk(KERN_ERR "cdev_add() failed!\n");
		/* clean up chrdev allocation */
		unregister_chrdev_region(mydev_node, DEVCNT);
		return -1;
	}
	ret = pci_register_driver(&pe_driver);
	
	// Setup the timer
	setup_timer(&my_time, timer_cb,(unsigned long)&flag);
	
	// Initialize the tail value
	mydev.tail = 0;
	return ret;
}

static void __exit HW6_exit(void)
{
	pci_unregister_driver(&pe_driver);

	//destroy the cdev
	cdev_del(&mydev.cdev);

	//clean up the devices
	unregister_chrdev_region(mydev_node, DEVCNT);

	//Unload Timmer
	del_timer_sync(&my_time);
	printk(KERN_INFO "%s Module is unloaded\n", pe_driver.name);
}
MODULE_AUTHOR("Ahmed Abdulkareem");
MODULE_LICENSE("GPL");
module_init(HW6_init);
module_exit(HW6_exit);

