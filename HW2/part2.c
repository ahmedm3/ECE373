// Ahmed Abdulkareem
// ECE 373
// 04/10/2016
// Register character device

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/usb.h>
#include <linux/slab.h>
//#include <linux/kernel.h>


#define DEVCNT 1
#define DEVNAME "cdev2"

// function prototypes
static ssize_t cdev2_read(struct file *, char __user *, size_t, loff_t *);
static ssize_t cdev2_write(struct file *, const char __user *, size_t, loff_t *);
static int cdev2_open(struct inode *inode, struct file *file);

static struct mydev_dev {
	struct cdev cdev;
	dev_t cdev2_node;
	int syscall_val;
} mydev;

// file ops
static struct file_operations mydev_fops = {
	.owner = THIS_MODULE,
	.read = cdev2_read,
	.write = cdev2_write,
	.open = cdev2_open,
};


static int cdev2_open(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "opened!\n");
	mydev.syscall_val = 25;

	return 0;
}


static ssize_t cdev2_read(struct file *file, char __user *buf, size_t len, loff_t *offset)
{

	// local kernel buffer
	int ret;

	printk(KERN_INFO "Read function, current int is: %d\n", mydev.syscall_val);
	
	if (*offset >= sizeof(int))
		return 0;

	// if user is bad
	if (!buf) {
		ret = -EINVAL;
		goto out;
	}
	
	if (copy_to_user(buf, &mydev.syscall_val, sizeof(int))) {
		ret = -EFAULT;
		goto out;
	}

	printk(KERN_INFO "buf: %d\n", *buf);
	ret = sizeof(int);
	*offset += len;

	printk(KERN_INFO "User got from us: %d\n", mydev.syscall_val);

out:
	return ret;
}

static ssize_t cdev2_write(struct file *file, const char __user *buf, size_t len, loff_t *offset)
{

	// Have local kernel memory 
	char *kern_buf;
	int ret;
	long int_val;

	printk(KERN_INFO "Inside write function!\n");
	
	// make sure user isn't bad
	if (!buf) {
		ret = -EINVAL;
		goto out;
	}

	// get some mem
	kern_buf = kmalloc(len, GFP_KERNEL);

	// make sure mem is good to go
	if (!kern_buf) {
		ret = -ENOMEM;
		goto out;
	}

	// copy from user 
	if (copy_from_user(kern_buf, buf, len)) {
		ret = -EFAULT;
		goto mem_out;
	}

	if(kstrtol(kern_buf, 10, &int_val)) {
		goto out;
	}
	printk(KERN_INFO "int_Val: %d!\n", int_val);
	ret = len;
	mydev.syscall_val = int_val;
	printk(KERN_INFO "sysintl: %d!\n", mydev.syscall_val);
	// print whatever userspace gives us
	printk(KERN_INFO "Userspace wrote \"%s\" to us\n", kern_buf);

mem_out:
	kfree(kern_buf);
out:
	return ret;

}


int __init cdev2_init(void)
{
	printk(KERN_INFO "cdev2 module loading...\n");

	if (alloc_chrdev_region(&mydev.cdev2_node, 0, DEVCNT, DEVNAME)) {
		printk(KERN_ERR "alloc_chrdev_region() failed!\n");
		return -1;
	}

	printk(KERN_INFO "Allocated %d device(s) at (major, minor): (%d, %d)\n", DEVCNT,
	       MAJOR(mydev.cdev2_node), MINOR(mydev.cdev2_node));

	/* Initialize the character device and add it to the kernel */
	cdev_init(&mydev.cdev, &mydev_fops);
	mydev.cdev.owner = THIS_MODULE;

	// initialize the int value
	mydev.syscall_val = 25;

	if (cdev_add(&mydev.cdev, mydev.cdev2_node, DEVCNT)) {
		printk(KERN_ERR "cdev_add() failed!\n");
		// clean up chrdev 
		unregister_chrdev_region(mydev.cdev2_node, DEVCNT);

		return -1;
	}

	return 0;
}

void __exit cdev2_exit(void)
{
	/* destroy the cdev */
	cdev_del(&mydev.cdev);

	/* clean up the devices */
	unregister_chrdev_region(mydev.cdev2_node, DEVCNT);

	printk(KERN_INFO "cdev2 module unloaded!\n");
}

MODULE_AUTHOR("Ahmed Abdulkareem");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
module_init(cdev2_init);
module_exit(cdev2_exit);
