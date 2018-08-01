#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>                 // Required for the GPIO functions
#include <linux/interrupt.h>            // Required for the IRQ code
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <asm/uaccess.h>    //put_user
#include <linux/device.h>
#include <linux/fs.h>
#define DEVICE_NAME "RPM"
#define CLASS_NAME "rpm"


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Weihan Wang");
MODULE_VERSION("0.1");

static unsigned int gpioButton = 161;
static unsigned int irqNumber;     
static unsigned int numberPresses = 0;
static unsigned int Velocity = 0;

static int majorNumber;
static struct class* RPMClass = NULL;
static struct device* RPMDevice =NULL;

static int dev_open(struct inode *, struct file *);
static int dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);

static struct file_operations fops =
{
	.open = dev_open,
	.read = dev_read,
	.write = dev_write,
	.release = dev_release,
}; 

static irq_handler_t  ebbgpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);

void timerFun(unsigned long arg);
static struct timer_list myTimer;
int i;

void timerFun (unsigned long arg) {
    int tmp;
    i++;
    tmp = i;
	//char *ps = "0xbebf8410";
	//put_user(numberPresses, ps);
    // printk (KERN_INFO "Called timer %d times\n", numberPresses); 
    myTimer.expires = jiffies + HZ;
    add_timer (&myTimer); /* setup the timer again */
	Velocity = numberPresses;
	//Velocity = 100;
    numberPresses = 0;
}

static int __init ebbgpio_init(void){
    unsigned long currentTime = jiffies; 
    unsigned long expiryTime = currentTime + HZ; /* HZ gives number of ticks per second */
    int result = 0;

    gpio_request(gpioButton, "sysfs");       // Set up the gpioButton
    gpio_direction_input(gpioButton);        // Set the button GPIO to be an input
    gpio_set_debounce(gpioButton, 20);      // Debounce the button with a delay of 200ms
    gpio_export(gpioButton, false);          // Causes gpio115 to appear in /sys/class/gpio
    
    irqNumber = gpio_to_irq(gpioButton);
    result = request_irq(irqNumber,             // The interrupt number requested
                        (irq_handler_t) ebbgpio_irq_handler, // The pointer to the handler function below
                        IRQF_TRIGGER_RISING,   // Interrupt on rising edge (button press, not release)
                        "ebb_gpio_handler",    // Used in /proc/interrupts to identify the owner
                        NULL);


    init_timer (&myTimer);
    myTimer.function = timerFun;
    myTimer.expires = expiryTime;
    myTimer.data = 0;
    add_timer (&myTimer);
    printk (KERN_INFO "timer added \n");

	printk(KERN_INFO "RPM: Initializing the RPM\n");
	
	majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
	if (majorNumber < 0){
		printk(KERN_ALERT "RPM failed to register a major number\n");
		return majorNumber;
	}
	printk(KERN_INFO "RPM: registered correctly with majot number\n");

	RPMClass = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(RPMClass)){
		unregister_chrdev(majorNumber, DEVICE_NAME);
		printk(KERN_ALERT "Failed to register device class\n");
		return PTR_ERR(RPMClass);
	}
	printk(KERN_INFO "RPM: device class registered correctly\n");

	RPMDevice = device_create(RPMClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
	if (IS_ERR(RPMDevice)){
		class_destroy(RPMClass);
		unregister_chrdev(majorNumber, DEVICE_NAME);
		printk(KERN_ALERT "Failed to create the device\n");
	}
	printk(KERN_INFO "RPM: device class created correctly\n");
    return result;
}

static void __exit ebbgpio_exit(void){
    if (!del_timer (&myTimer)) {
	printk (KERN_INFO "Couldn't remove timer!!\n");
    }
    else {
	printk (KERN_INFO "timer removed \n");
    }
    free_irq(irqNumber, NULL);               // Free the IRQ number, no *dev_id required in this case
    gpio_unexport(gpioButton);
    gpio_free(gpioButton);
	
	device_destroy(RPMClass, MKDEV(majorNumber, 0));
	class_unregister(RPMClass);
	class_destroy(RPMClass);
	unregister_chrdev(majorNumber, DEVICE_NAME);
	printk(KERN_INFO "RPM: Goodbye from the LKM!\n");
}

static irq_handler_t ebbgpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs){
   numberPresses++;                      
   return (irq_handler_t) IRQ_HANDLED;  
}

static int dev_open(struct inode *inodep, struct file *filep){
	printk(KERN_INFO "RPM: Device has been opened \n");
	return 0;
}

static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset){
	int error_count = 0;
	error_count = copy_to_user(buffer, &Velocity, sizeof(Velocity));
	//error_count = copy_to_user(buffer, "wwwhhhh", strlen("wwwhhhh"));
	
	if (error_count==0){
		printk(KERN_INFO "RPM: Sent %d characters to the user\n", strlen("Shabi"));
		return 0;
	}
	else {
		printk(KERN_INFO "RPM: Failed to send %d characters to the user\n", error_count);
		return -EFAULT;
	}
}

static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset){
	printk(KERN_INFO "RPM: I cannot receive any information from user space\n");
	return 0;
}

static int dev_release(struct inode *inodep, struct file *filep){
	printk(KERN_INFO "RPM: Device successfully closed\n");
	return 0;
}
module_init(ebbgpio_init);
module_exit(ebbgpio_exit);
