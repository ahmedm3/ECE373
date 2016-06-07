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
#define DEVCNT 5
#define DEVNAME "example5"
#define PE_LED_MASK 0xC
#define PE_REG_LEDS 0xe00
#define PE_REG_CTRL 0x0004
#define RDBAL 0x2800
#define RDBAH 0x2804
/* Define for interrupt*/
#define ICR   0x000C0
#define IMS   0x000D0
#define IMC   0x000D8
#define ICS   0x000C8
#define IRQ_REG_TX  0x0001
#define IRQ_REG_LSC 0x0005
/* Define Receive Register -> set promiscuous*/
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
	__le64 buffer_addr;
	union
	{
		__le32 data;
		struct
		{
			__le16 length; // Data buffer length
			u16 cso;       // Checksum
		} flags;
	} lower;
	union 
	{
		__le32 data;
		struct
		{
			u8 status;
			u8 error;
			__le16  Vlan;
		} field;
	} upper;
};

static DEFINE_PCI_DEVICE_TABLE(pe_pci_tbl) = {
	{ PCI_DEVICE(0x8086, 0x150c) },
	{ }, /* must have an empty entry at the end! */
};
/* Create Parameter delay with default = 2*/
static int blink_rate = 2;
module_param(blink_rate,int,S_IRUSR | S_IWUSR);

/* Struct my time*/
static struct timer_list my_time;
/* Struct Pes*/
struct pes {
	struct pci_dev *pdev;
	void *hw_addr;
	struct cdev cdev;
	struct work_struct task;
};
/* Struct mydev */
 static struct mydev_dev {
	struct cdev cdev;
	void *hw_addr;    /* Taking address of LED */
	u32 syscall_val;  /* Using it for update LED on or off */
	dma_addr_t addr;
	struct e1000e_rx_desc  *cpu_addr; // Pointer controls ring
	struct work_struct service_task;	// work queue
	int tail; 
	int head_tail;  // Use to transmit to user space
} mydev; 

 // INTERRUPT PART
 // WORK IRQ
static void irq_work(struct work_struct *work)
{

	struct mydev_dev *id = container_of(work,struct mydev_dev,service_task);
	printk(KERN_INFO "IN WORK QUEUE !!!!!\n");
	printk(KERN_INFO "Start and sleep 0.5 seconds \n"); 
	msleep(500);
	printk(KERN_INFO "Turn off both green LED");
	writel(0x0F0F0F, mydev.hw_addr + PE_REG_LEDS);

	printk(KERN_INFO "Task is doing ! \n");
}
// REQUEST IRQ ( Function pe_irq will be called from Pe_Probe
static irqreturn_t pe_irq(int irq,void *data)
{
	int i;
	/* Turn on both Green LED when go to interrupt handler*/
	printk(KERN_INFO "INSIDE INTERRUPT !!!!! \n");
	writel(0x4E4E0F,mydev.hw_addr + PE_REG_LEDS);
	u32 int_status;
	writel(0x100000,mydev.hw_addr + IMS);
	int_status = readl(mydev.hw_addr+ICR);
	int_status = int_status & 0x00FFFFFF; 
	printk(KERN_INFO "Value of int_status: 0x%x \n",int_status);
	for(i=0;i<16;i++)
	printk(KERN_INFO "Data: 0x%x    DD bit: 0x%x \n",mydev.cpu_addr[i].upper.data,mydev.cpu_addr[i].upper.field.status &0x1);
	
	// If the last tail
	if(mydev.tail==15)
		mydev.tail=0;
	else
	{
		
		printk(KERN_INFO "Test \n");
		mydev.cpu_addr[mydev.tail].upper.field.status &= 0xFE;
		writel(mydev.tail,mydev.hw_addr + TAIL);
		mydev.tail++;
	}
	// Case for interrupt signal
	switch(int_status)
	{
	 
		case IRQ_REG_TX:
		case IRQ_REG_LSC:
		case IRQ_REG_RXQ: 
			writel(int_status,mydev.hw_addr + IMC);
			schedule_work(&mydev.service_task);
			break;
		default:
			printk(KERN_INFO "Unknow IRQ \n");
			writel(0x4,mydev.hw_addr + IMS);
			return IRQ_NONE;
		
	}
	/* Write '1' to clear the interrupt bit */ 
	writel(0x4,mydev.hw_addr+ IMS);
	return IRQ_HANDLED;
	
}  
static int flag;
/* Function timer with delay*/
static void timer_cb(unsigned long flag)
{

	/* If blink_rate is negative --> Error*/
	if(blink_rate <=0)
	{
	 printk(KERN_ERR "Blink_rate is negative ! Error");
	 return;
	/* Check the current state of LED
	   If bit bit 6 = 1 ( Current state LED is on --> change off)
	*/
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

	 /* set up for high or low dma */
	err = dma_set_mask(&pdev->dev, DMA_BIT_MASK(64));
	if (err) {
		dev_err(&pdev->dev, "DMA configuration failed: 0x%x\n", err);
		goto err_dma;
	}

	/* set up pci connections */
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
	
	/* map device memory 
	The base LED control is 0 and size = 4096*/
	ioremap_len = min_t(int, pci_resource_len(pdev, 0),32768 );
	/* Taking address of LED for mydev*/
	mydev.hw_addr = ioremap(pci_resource_start(pdev, 0), ioremap_len);
	if (!mydev.hw_addr) {
		err = -EIO;
		dev_info(&pdev->dev, "ioremap(0x%04x, 0x%04x) failed: 0x%x\n",
			 (unsigned int)pci_resource_start(pdev, 0),
			 (unsigned int)pci_resource_len(pdev, 0), err);
		goto err_ioremap;
	}
	pe->hw_addr = ioremap(pci_resource_start(pdev,0),ioremap_len);
        /* Read current value LED from address LED*/
	mydev.syscall_val = readl(mydev.hw_addr + PE_REG_LEDS);
	/* Display it in Kernel*/
	dev_info(&pdev->dev, "led_reg = 0x%02x\n",mydev.syscall_val);
	/* Check if parameter new_leds =1 -> update to turn on LED */
	
	// Create RING /////////////////////////////////////////////////

	mydev.cpu_addr= dma_alloc_coherent(&pdev->dev,256,&mydev.addr,GFP_KERNEL);
	reg = (mydev.addr >> 32) & 0xFFFFFFFF;
	printk(KERN_INFO "Higher: 0x%x \n", reg);
	writel(reg, mydev.hw_addr + RDBAH);

	reg = mydev.addr &0xFFFFFFFF;
	printk(KERN_INFO "Lower: 0x%x \n", reg);
	writel(reg,mydev.hw_addr+ RDBAL);
	// Create 16 descriptor
	for(i=0;i<16;i++)
	{
		buf_info[i].mem= kzalloc(2048,GFP_KERNEL);
		if(!buf_info[i].mem)
		{
			err = -ENOMEM;
			goto err_descriptor;
		}

		// Pin
		buf_info[i].phy = dma_map_single(&pdev->dev,buf_info[i].mem,2048,DMA_TO_DEVICE);
		if(!buf_info[i].phy)
		{
			err = -ENOMEM;
			goto err_descriptor;
		} 
	}
	/* Set Length*/
	temp = readl(mydev.hw_addr + RDLEN); /* Read   */
        temp = temp & 0x0;                /* Modify */
	temp = temp | 0x2000;
        writel(temp,mydev.hw_addr + RDLEN);  /* Write  */

	printk(KERN_INFO "Successful creating Ring Descriptor \n");
	printk(KERN_INFO "INFORMATION OF PHY ADDRESS \n");
	for(i=0;i<16;i++)
		printk(KERN_INFO " Address: 0x%x \n",(int)buf_info[i].phy);
	

	printk(KERN_INFO "Filling descriptor buffer");
	for(i=0;i<16;i++)
	{
		mydev.cpu_addr[i].buffer_addr= buf_info[i].phy;
		printk(KERN_INFO "ADRRESS of buffer: 0x%x \n",(int)mydev.cpu_addr[i].buffer_addr);
	
	}
	
	// Enable receive
	temp = readl(mydev.hw_addr + PE_REG_CTRL);
	temp = temp | 0x40;
	printk(KERN_INFO "Receive Enable: 0x%x",temp);
	writel(temp,mydev.hw_addr + PE_REG_CTRL);
	/* Setting Promiscuous */
	temp = readl(mydev.hw_addr + RCTL); /* Read   */
	temp = temp | 0x10 ; 		    /* Modify */
	writel(temp,mydev.hw_addr + RCTL);  /* Write  */

	// SET TAIL DESCRIPTOR
	writel(15,mydev.hw_addr + TAIL);

	temp = readl(mydev.hw_addr + RCTL); /* Read   */
        temp = temp | 0x2 ;                /* Modify */
        writel(temp,mydev.hw_addr + RCTL);  /* Write  */

	// Set Enable Receive enable RCTL
	
	/*  INTERRUPT HANDLE */	
	// Triger IRQ
	writel(0x100000,mydev.hw_addr + ICS);
	writel(0x100000,mydev.hw_addr + IMS);
	// Reques IRQ
	if (request_irq(pdev->irq,pe_irq,IRQF_SHARED,"Ahmed Abdulkareem",(void *)&irq));
		dev_info(&pdev->dev,"request IRQ successful \n");


	printk(KERN_INFO "value: 0x%x",readl(mydev.hw_addr+IMS));

	// WORK QUEUE
	INIT_WORK(&mydev.service_task,irq_work);
	return 0;
 
err_descriptor:
err_ioremap:
	kfree(pe);
err_pe_alloc:
	pci_release_selected_regions(pdev,
				     pci_select_bars(pdev, IORESOURCE_MEM));
err_pci_reg:
err_dma:
	pci_disable_device(pdev);
	return err;
}


/* modprobe r8169 */

static void pe_remove(struct pci_dev *pdev)
{
	int i;
	struct pes *pe = pci_get_drvdata(pdev);
	
	cancel_work_sync(&mydev.service_task);
	// Free Ring descriptor
	for(i=0;i<16;i++)
		dma_unmap_single(&pdev->dev,buf_info[i].phy,2048,DMA_TO_DEVICE);
	// Free coherent
	dma_free_coherent(&pdev->dev,256,mydev.cpu_addr,mydev.addr);
	
	free_irq(pdev->irq,(void *)&irq);	
	iounmap(pe->hw_addr);
	kfree(pe);
	pci_release_selected_regions(pdev,
				     pci_select_bars(pdev, IORESOURCE_MEM));
	pci_disable_device(pdev);
	
}

static struct pci_driver pe_driver = {
	.name     = "pci_example",
	.id_table = pe_pci_tbl,
	.probe    = pe_probe,
	.remove   = pe_remove,
};
/* Character Device Driver */
static dev_t mydev_node;

/* this shows up under /sys/modules/example5/parameters */
static int exam = 15;
module_param(exam, int, S_IRUSR | S_IWUSR);
/* this doesn't appear in /sys/modules */
static int exam_nosysfs = 15;
module_param(exam_nosysfs, int, 0);

static int example5_open(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "successfully opened! \n");
	return 0;
}

static ssize_t example5_read(struct file *file, char __user *buf,
                             size_t len, loff_t *offset)
{
	/* Get a local kernel buffer set aside */
	int ret;
	int temp;
	if (*offset >= sizeof(int))
		return 0;

	/* Make sure our user wasn't bad... */
	if (!buf) {
		ret = -EINVAL;
		goto out;
	}
	/* Coppy blink_rate from Kernel to User space*/
	if (copy_to_user(buf, &mydev.head_tail, sizeof(int))) {
		ret = -EFAULT;
		goto out;
	}
	ret = sizeof(int);
	*offset += len;
	mydev.head_tail = readl(mydev.hw_addr+HEAD); // Read value Head
	mydev.head_tail = mydev.head_tail <<16;	       // Shift left 16 bit
	temp 	  = readl(mydev.hw_addr+TAIL); // Get value Tail
	mydev.head_tail = mydev.head_tail | temp;	       // shift right 16 bit	

out:
	return ret;
}

static ssize_t example5_write(struct file *file, const char __user *buf,
                              size_t len, loff_t *offset)
{
	/* Have local kernel memory ready */
	char *kern_buf;
	int ret;
	/* Make sure our user isn't bad... */
	if (!buf) {
		ret = -EINVAL;
		goto out;
	}

	/* Get some memory to copy into... */
	kern_buf = kmalloc(len, GFP_KERNEL);

	/* ...and make sure it's good to go */
	if (!kern_buf) {
		ret = -ENOMEM;
		goto out;
	}

	/* Copy from the user-provided buffer */
	if (copy_from_user(kern_buf, buf, len)) {
		ret = -EFAULT;
		goto mem_out;
		goto out;
	}

	/* Update value syscall_val */
	if (kstrtoint(kern_buf,10,&blink_rate))
        {
		ret = -ERANGE;
		goto mem_out;
	}
	ret = len;

	/* print what userspace gave back us the new value syscall_val */
	printk(KERN_INFO "Great! Userspace wrote back the new value for blink_rate \"%d\" to us\n", blink_rate);
	
mem_out:
	kfree(kern_buf);
out:
	return ret;
}

/* File operations for our device */
static struct file_operations mydev_fops = {
	.owner = THIS_MODULE,
	.open = example5_open,
	.read = example5_read,
	.write = example5_write,

};

static int __init hello_init(void)
{
	int ret;
	printk(KERN_INFO "%s loaded\n", pe_driver.name);
	
	
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
	
	/* Setup Timer*/
	setup_timer(&my_time, timer_cb,(unsigned long)&flag);
	
	// Initial tail
	mydev.tail = 0;
	return ret;
}

static void __exit hello_exit(void)
{
	// Cancel Work Queue
	pci_unregister_driver(&pe_driver);
	/* destroy the cdev */
	cdev_del(&mydev.cdev);

	/* clean up the devices */
	unregister_chrdev_region(mydev_node, DEVCNT);
	/* Unload Timmer*/
	del_timer_sync(&my_time);
	printk(KERN_INFO "%s unloaded\n", pe_driver.name);
}
MODULE_AUTHOR("Ahmed Abdulkareem");
MODULE_LICENSE("GPL");
module_init(hello_init);
module_exit(hello_exit);

