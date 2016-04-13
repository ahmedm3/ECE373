// Ahmed Abdulkareem
// ECE 373
// 04/10/2016
// Register character device

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>

#define DEVCNT 1
#define DEVNAME "cdev2"

// function prototypes
static ssize_t cdev2_read(struct file *file, char __user *user, size_t size, loff_t *loff);

static struct mydev_dev {
	struct cdev cdev;
	dev_t cdev2_node;
	int syscall_val;
} mydev;

// file ops
static struct file_operations mydev_fops = {
	.owner = THIS_MODULE,
	.read = cdev2_read,
};


static ssize_t cdev2_read(struct file *file, char __user *user, size_t size, loff_t *loff)
{
	printk(KERN_INFO "Read function\n");
	return 0;
}

int __init cdev2_init(void)
{
	printk(KERN_INFO "cdev2 module loading...\n");

	if (alloc_chrdev_region(&mydev.cdev2_node, 0, DEVCNT, DEVNAME)) {
		printk(KERN_ERR "alloc_chrdev_region() failed!\n");
		return -1;
	}

	printk(KERN_INFO "Allocated %d device(s) at major: %d\n", DEVCNT,
	       MAJOR(mydev.cdev2_node));

	/* Initialize the character device and add it to the kernel */
	cdev_init(&mydev.cdev, &mydev_fops);
	mydev.cdev.owner = THIS_MODULE;

	if (cdev_add(&mydev.cdev, mydev.cdev2_node, DEVCNT)) {
		printk(KERN_ERR "cdev_add() failed!\n");
		/* clean up chrdev allocation */
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
