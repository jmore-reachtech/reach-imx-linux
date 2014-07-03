#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <mach/pinctrl.h>

#define DEV_NAME    		"nutouch-i2c"

#define NUM_FINGERS_SUPPORTED                 1

#define X_MIN                                 0x00
#define Y_MIN                                 0x00
#define X_MAX                                 0xFFF
#define Y_MAX                                 0xFFF
#define NUTOUCH_MAX_REPORTED_PRESSURE         10
#define NUTOUCH_MAX_TOUCH_SIZE                100
#define NUTOUCH_DRIVER_VERSION                1

#define NUTOUCH_ReadResponseMsg_Noaddress       1
#define NUTOUCH_ReadResponseMsg                 2
#define NUTOUCH_ReadSendCMDgetCRCMsg            3
#define NUTOUCH_HandleTouchMsg                  4

#define OFFSET_CID		0
#define OFFSET_STATUS	0
#define OFFSET_X_L		1
#define OFFSET_Y_L		2
#define OFFSET_X_H		3
#define OFFSET_Y_H		3
#define OFFSET_PRESSURE	4
#define OFFSET_AREA		5

#define MULTITOUCH_INT_GPIO		MXS_PIN_ENCODE(0x2, 19) /* bank2 starts at 64 and this is pin 19*/

struct point_data {
        short Status;
        short X;
        short Y;
        short Pressure;
        short Area;
};

struct nutouch_data
{
	struct i2c_client    *client;
	struct input_dev     *input;
	struct semaphore     sema;
	struct workqueue_struct *wq;
	struct work_struct work;
	int irq;
	short irq_type;
	struct point_data PointBuf[NUM_FINGERS_SUPPORTED];
	#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
	#endif
	int dev_major;
	int dev_minor;
	struct cdev *dev_cdevp;
	struct class *dev_classp;
	int update_mode;
	int x_range;
	int y_range;
	int orient;
	int backup_rc;
	int en_water_proof;
	int fw_ver[16];
	int delta;
};

static int LastUpdateID = 0;
static struct nutouch_data *nutouch_gpts = NULL;

int      nutouch_release(struct inode *, struct file *);
int      nutouch_open(struct inode *, struct file *);
ssize_t  nutouch_write(struct file *file, const char *buf, size_t count, loff_t *ppos);
ssize_t  nutouch_read(struct file *file, char *buf, size_t count, loff_t *ppos);
long	 nutouch_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static struct cdev nutouch_cdev;
static struct class *nutouch_class;
static int nutouch_major = 0;

int nutouch_open(struct inode *inode, struct file *filp)
{
	return 0;
}
EXPORT_SYMBOL(nutouch_open);

int  nutouch_release(struct inode *inode, struct file *filp)
{
	return 0;
}
EXPORT_SYMBOL(nutouch_release);

ssize_t nutouch_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	int ret;
	char *tmp;

	if (count > 8192)
		count = 8192;

	tmp = (char *)kmalloc(count,GFP_KERNEL);
	if (tmp==NULL)
		return -ENOMEM;
	if (copy_from_user(tmp,buf,count)) {
		kfree(tmp);
		return -EFAULT;
	}
	printk("writing %zu bytes.\n", count);

	ret = i2c_master_send(nutouch_gpts->client, tmp, count);
	kfree(tmp);
	return ret;
}
EXPORT_SYMBOL(nutouch_write);

ssize_t nutouch_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	char *tmp;
	int ret;

	if (count > 8192)
		count = 8192;

	tmp = (char *)kmalloc(count,GFP_KERNEL);
	if (tmp==NULL)
		return -ENOMEM;

	printk("reading %zu bytes.\n", count);

	ret = i2c_master_recv(nutouch_gpts->client, tmp, count);
	if (ret >= 0)
		ret = copy_to_user(buf,tmp,count)?-EFAULT:ret;
	kfree(tmp);
	return ret;
}
EXPORT_SYMBOL(nutouch_read);

long nutouch_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return 0;
}
EXPORT_SYMBOL(nutouch_ioctl);

static inline int nutouch_parser_packet(u8 *buff, struct nutouch_data *nutouch)
{
	int i, FID;
	short ContactID = 0;
	
	for (i=0;i<10; i++) {
		nutouch->PointBuf[ContactID].Status = -1;
	}
	
	for (i=0; i<10; i++) {
		FID = i*6;

		if (buff[FID] == 0xFF) {
			LastUpdateID = i;
			//printk("\nFID = %d, 0x%02x\n", FID, buff[FID]);
			if(FID == 0) {
				for (i=0; i<60; i++) {
					if(i==0 || i==10 || i==20 || i==30 || i==40 || i==50)
						printk("\nbuff[%02d] : ", i);
					printk("0x%02x ", buff[i]);
				}
				goto ERR_PACKET;
			}
			break;
		}
		
		ContactID = ( (buff[FID + OFFSET_CID]&0xF8) >> 3) - 1;		
		if (ContactID < 0 || ContactID > 9)	goto ERR_PACKET;
		
		nutouch->PointBuf[ContactID].Status = (buff[FID + OFFSET_STATUS] & 0x01);
		nutouch->PointBuf[ContactID].X = ( ((buff[FID + OFFSET_X_H] & 0x0F) << 8) | (buff[FID + OFFSET_X_L] ) );
		nutouch->PointBuf[ContactID].Y = ( ((buff[FID + OFFSET_Y_H] & 0xF0) << 4) | (buff[FID + OFFSET_Y_L] ) );
		nutouch->PointBuf[ContactID].Pressure = buff[FID + OFFSET_PRESSURE];
		nutouch->PointBuf[ContactID].Area = buff[FID + OFFSET_AREA];
		
		#if 0
		printk("\n[%d] : (%d, %d) (%d) (%d, %d)\n",
			ContactID,
			nutouch->PointBuf[ContactID].X,
			nutouch->PointBuf[ContactID].Y,
			nutouch->PointBuf[ContactID].Status,
			nutouch->PointBuf[ContactID].Pressure,
			nutouch->PointBuf[ContactID].Area);
		#endif
        
        /* tslib only support a single touch */
        if (ContactID == 0) {
            input_report_abs(nutouch->input, ABS_X, nutouch->PointBuf[ContactID].X);
		    input_report_abs(nutouch->input, ABS_Y, nutouch->PointBuf[ContactID].Y);
		    input_report_abs(nutouch->input, ABS_PRESSURE, nutouch->PointBuf[ContactID].Status);
		    input_sync(nutouch->input);
        }

	}
	
	return 0;

ERR_PACKET:
	printk("ERR PACKET\n");
	return -1;
}

static inline int nutouch_read_block_onetime(struct i2c_client *client, u16 length, u8 *value)
{
	struct nutouch_data *nutouch;
	int rc = 0;

	nutouch = i2c_get_clientdata(client);

	rc = i2c_master_recv(nutouch->client, value, length);

	if (rc == length) {
		return length;
	} else {
		mdelay(10);
		printk("NuTouch: i2c read failed\n");
		return -1;
	}	
}

static void nutouch_worker(struct work_struct *work)
{
    struct nutouch_data *nutouch = container_of(work, struct nutouch_data, work);
    u8 buffer[60] = {0}; 
    //int i;

    if(nutouch_read_block_onetime(nutouch->client, 60, &buffer[0]) < 0) {
       printk("NuTouch: nutouch_read_block failed, try again\n"); 
    } 

    #if 0
        for (i=0; i<60; i++) {
			if(i==0 || i==6 || i==12 || i==18 || i==24 || i==30 || i==36 || i==42 || i==48 || i==54 || i==60)
				printk("\nfinger[%d] : ", (i/6)+1);
			printk("0x%02x ", buffer[i]);
    }
    #endif

    nutouch_parser_packet(buffer,nutouch);

    enable_irq(gpio_to_irq(nutouch->client->irq));
}

static irqreturn_t nutouch_irq_handler(int irq, void *dev_id)
{
	struct nutouch_data *ts = dev_id;

    //printk("%s: \n", __func__);

	disable_irq_nosync(gpio_to_irq(ts->client->irq));
    schedule_work(&ts->work);

	return IRQ_HANDLED;
}


static int nutouch_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct nutouch_data *nutouch;
    int error = 0;
    int ret = 0; 
    uint16_t max_x = 0, max_y = 0;

    printk("%s: \n", __func__);
    
    nutouch = kzalloc(sizeof(struct nutouch_data), GFP_KERNEL);
    if (nutouch == NULL) {
		printk("NuTouch: insufficient memory\n");
		error = -ENOMEM;
		goto err_nutouch_alloc;
	}
    
    INIT_WORK(&nutouch->work, nutouch_worker);
    nutouch->client = client;
    i2c_set_clientdata(client, nutouch);  

   	ret = request_irq(gpio_to_irq(client->irq), nutouch_irq_handler, IRQF_TRIGGER_FALLING | IRQF_DISABLED, client->name, nutouch);
	if (ret == 0)
        printk("%s: using IRQ \n", __func__);
	else
	    dev_err(&client->dev, "request_irq failed\n");

    nutouch_gpts = kzalloc(sizeof(*nutouch), GFP_KERNEL);
     
    nutouch->input = input_allocate_device();
	if (nutouch->input == NULL){
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

    nutouch->input->name = "nutouch-ts";
    set_bit(EV_KEY, nutouch->input->evbit);
	set_bit(BTN_TOUCH, nutouch->input->keybit);
	set_bit(EV_ABS, nutouch->input->evbit);

   	max_x = nutouch->x_range;
	max_y = nutouch->y_range;
 
    __set_bit(ABS_X, nutouch->input->absbit);
	__set_bit(ABS_Y, nutouch->input->absbit);

    input_set_abs_params(nutouch->input, ABS_X, 0, max_x, 0, 0);
	input_set_abs_params(nutouch->input, ABS_Y, 0, max_y, 0, 0);
    input_set_abs_params(nutouch->input, ABS_PRESSURE, 0, 10, 0, 0);

    if(input_register_device(nutouch->input)){
		printk("Can not register input device.");
		goto err_input_register_device_failed;
	}
  
 
    return 0;
err_input_register_device_failed:
	input_free_device(nutouch->input);
err_nutouch_alloc:
	return error;

err_input_dev_alloc_failed:
    kfree(nutouch);

    return ret;
}

static int nutouch_remove(struct i2c_client *client)
{
	struct nutouch_data *nutouch;

    printk("%s: \n", __func__);

	nutouch = i2c_get_clientdata(client);

	if (nutouch != NULL) {
		if (nutouch->irq)
    	    free_irq(gpio_to_irq(client->irq), nutouch);

		input_unregister_device(nutouch->input);
	}
	kfree(nutouch);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id nutouch_id[] =
{
	{ "NuTouch", 0 },
	{},
};

static struct i2c_driver nutouch_driver =
{
	.probe      = nutouch_probe,
	.remove     = nutouch_remove,
	.id_table   = nutouch_id,
	.driver     = {
		.name   = "NuTouch",
	},
};

static struct file_operations nc_fops = {
	.owner =        THIS_MODULE,
	.write		= nutouch_write,
	.read		= nutouch_read,
	.open		= nutouch_open,
	.unlocked_ioctl = nutouch_ioctl,
	.release	= nutouch_release,
};

static int __devinit nutouch_init(void)
{
   	int result;
	int err = 0;
	dev_t devno = MKDEV(nutouch_major, 0);

    printk("%s: \n", __func__);
	result  = alloc_chrdev_region(&devno, 0, 1, DEV_NAME);
	if(result < 0){
		printk("fail to allocate chrdev (%d) \n", result);
		return 0;
	}
	nutouch_major = MAJOR(devno);
        cdev_init(&nutouch_cdev, &nc_fops);
	nutouch_cdev.owner = THIS_MODULE;
	nutouch_cdev.ops = &nc_fops;
        err =  cdev_add(&nutouch_cdev, devno, 1);
	if(err){
		printk("fail to add cdev (%d) \n", err);
		return 0;
	}

	nutouch_class = class_create(THIS_MODULE, DEV_NAME);
	if (IS_ERR(nutouch_class)) {
		result = PTR_ERR(nutouch_class);
		unregister_chrdev(nutouch_major, DEV_NAME);
		printk("fail to create class (%d) \n", result);
		return result;
	}
	device_create(nutouch_class, NULL, MKDEV(nutouch_major, 0), NULL, DEV_NAME);
 

	return i2c_add_driver(&nutouch_driver);
}

static void __exit nutouch_exit(void)
{
    dev_t dev_id = MKDEV(nutouch_major, 0);

    printk("%s: \n", __func__);

	i2c_del_driver(&nutouch_driver);

    cdev_del(&nutouch_cdev);

	device_destroy(nutouch_class, dev_id); //delete device node under /dev
	class_destroy(nutouch_class); //delete class created by us
	unregister_chrdev_region(dev_id, 1);

}

module_init(nutouch_init);
module_exit(nutouch_exit);

MODULE_DESCRIPTION("Driver for NuTouch Touchscreen Controller");
MODULE_LICENSE("GPL");
