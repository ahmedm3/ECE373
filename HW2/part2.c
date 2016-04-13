// Ahmed Abdulkareem
// ECE 373
// 04/10/2016
// Hooking up our char driver

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>

#define DEVCNT 1
static dev_t cdev_node;

int __init cdev1_init(void)
{
	printk(KERN_INFO "cdev1 module loading...\n");

	if (alloc_chrdev_region(&cdev_node, 0, DEVCNT, "cdev1")) {
		printk(KERN_ERR "alloc_chrdev_region() failed!\n");
		return -1;
	}

	printk(KERN_INFO "Allocated %d device(s) at major: %d\n", DEVCNT,
	       MAJOR(cdev_node));

	return 0;
}

void __exit cdev1_exit(void)
{
	// clean up
	unregister_chrdev_region(cdev_node, DEVCNT);

	printk(KERN_INFO "cdev1 module unloaded!\n");
}

MODULE_AUTHOR("Ahmed Abdulkareem");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
module_init(cdev1_init);
module_exit(cdev1_exit);
