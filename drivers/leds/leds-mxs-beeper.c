/* 
 * Beeper driver
 * 
 * 
 * 
 * 
 * */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/device.h>
#include <mach/regs-pwm.h>

/*
 * PWM enables are the lowest bits of HW_PWM_CTRL register
 */
#define BM_PWM_CTRL_PWM_ENABLE	((1<<(CONFIG_MXS_PWM_CHANNELS)) - 1)
#define BF_PWM_CTRL_PWM_ENABLE(n) ((1<<(n)) & BM_PWM_CTRL_PWM_ENABLE)

#define CLKPERIOD 167
#define PWMFREQ 20000
#define PWM4 4
#define PWM7 7
#define PWM_VOL_ACTIVE (REGS_PWM_BASE + HW_PWM_ACTIVEn(PWM4))
#define PWM_FREQ_ACTIVE (REGS_PWM_BASE + HW_PWM_ACTIVEn(PWM7))
#define PWM_VOL_PERIOD (REGS_PWM_BASE + HW_PWM_PERIODn(PWM4))
#define PWM_FREQ_PERIOD (REGS_PWM_BASE + HW_PWM_PERIODn(PWM7))

#define PERIOD(P) (BF_PWM_PERIODn_CDIV(2) /* divide by 24Mhz clock by 4  */ \
	| BF_PWM_PERIODn_INACTIVE_STATE(2) /* low output for inactive     */ \
	| BF_PWM_PERIODn_ACTIVE_STATE(3) /* high output for active      */ \
	| BF_PWM_PERIODn_PERIOD(P - 1)) /* period field = period - 1   */

#define MS_TO_NS(x)	(x * 1E6L)

typedef struct mxs_beeper_t {
	int vol_delay_on;
	int vol_delay_off;
	int freq_delay_on;
	int freq_delay_off;
};

static struct mxs_beeper_t mxs_beeper;
static struct hrtimer hr_timer;

/* for sysfs interface */
static int beep = 0;
static int vol = 50;
static int freq = 3000;
static unsigned long duration = 100;

static void beeper_set_freq(void);
static void beeper_set_vol(void);

static ssize_t beeper_vol_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,"%d\n",vol);
}

static ssize_t beeper_vol_store(struct kobject *kobj, struct kobj_attribute *attr, char *buf, size_t count)
{
	sscanf(buf,"%du",&vol);
	
	beeper_set_vol();
	
	return count;
}

static void beeper_set_vol()
{
	mxs_beeper.vol_delay_on =  (1000000000/(2*PWMFREQ)/CLKPERIOD*vol/100);
	mxs_beeper.vol_delay_off =  (1000000000/(2*PWMFREQ)/CLKPERIOD*(100-vol)/100);
	
	/* set up volume */
	__raw_writel(BF_PWM_ACTIVEn_INACTIVE(mxs_beeper.vol_delay_on) |
		BF_PWM_ACTIVEn_ACTIVE(0),
		REGS_PWM_BASE + HW_PWM_ACTIVEn(PWM4));
		
	__raw_writel(PERIOD(mxs_beeper.vol_delay_on + mxs_beeper.vol_delay_off),
		REGS_PWM_BASE + HW_PWM_PERIODn(PWM4));
}

static struct kobj_attribute beeper_vol_attribute = 
	__ATTR(vol, 0666, beeper_vol_show, beeper_vol_store);
	
static ssize_t beeper_freq_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,"%d\n",freq);
}

static ssize_t beeper_freq_store(struct kobject *kobj, struct kobj_attribute *attr, char *buf, size_t count)
{
	sscanf(buf,"%du",&freq);
	
	beeper_set_freq();
	
	return count;
}

static void beeper_set_freq()
{
	mxs_beeper.freq_delay_on =  (1000000000/(2*freq)/CLKPERIOD);
	mxs_beeper.freq_delay_off =  (1000000000/(2*freq)/CLKPERIOD);
	
	/* set up frequency */
	__raw_writel(BF_PWM_ACTIVEn_INACTIVE(mxs_beeper.freq_delay_on) |
		BF_PWM_ACTIVEn_ACTIVE(0),
		REGS_PWM_BASE + HW_PWM_ACTIVEn(PWM7));
		
	__raw_writel(PERIOD(mxs_beeper.freq_delay_on + mxs_beeper.freq_delay_off),
		REGS_PWM_BASE + HW_PWM_PERIODn(PWM7));
}

static struct kobj_attribute beeper_freq_attribute = 
	__ATTR(freq, 0666, beeper_freq_show, beeper_freq_store);	

static ssize_t beeper_beep_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,"%d\n",0);
}

static ssize_t beeper_beep_store(struct kobject *kobj, struct kobj_attribute *attr, char *buf, size_t count)
{
	ktime_t ktime;
	
	pr_info("TURN ON BEEPER \n");
	__raw_writel(BF_PWM_CTRL_PWM_ENABLE(PWM4),
			 REGS_PWM_BASE + HW_PWM_CTRL_SET);
	__raw_writel(BF_PWM_CTRL_PWM_ENABLE(PWM7),
			 REGS_PWM_BASE + HW_PWM_CTRL_SET);
	
	pr_info( "Starting timer to fire in %lu ms  (%ld)\n", duration, jiffies );
	ktime = ktime_set( 0, duration*((unsigned long)1E6L) );
	hrtimer_start( &hr_timer, ktime, HRTIMER_MODE_REL );
		
	return count;
}

static struct kobj_attribute beeper_beep_attribute = 
	__ATTR(beep, 0666, beeper_beep_show, beeper_beep_store);
	
static ssize_t beeper_duration_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,"%lu\n",duration);
}

static ssize_t beeper_duration_store(struct kobject *kobj, struct kobj_attribute *attr, char *buf, size_t count)
{
	sscanf(buf,"%lu",&duration);
		
	return count;
}

static struct kobj_attribute beeper_duration_attribute = 
	__ATTR(duration, 0666, beeper_duration_show, beeper_duration_store);

static struct attribute *attrs[] = {
	&beeper_vol_attribute.attr,
	&beeper_freq_attribute.attr,
	&beeper_beep_attribute.attr,
	&beeper_duration_attribute.attr,
	NULL,					/* need to NULL terminate the list */
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *beeper_kobj;

enum hrtimer_restart beeper_hrtimer_callback( struct hrtimer *timer )
{
	printk( "beeper_hrtimer_callback called (%ld).\n", jiffies );
	
	__raw_writel(BF_PWM_CTRL_PWM_ENABLE(PWM4),
		REGS_PWM_BASE + HW_PWM_CTRL_CLR);
	__raw_writel(BF_PWM_CTRL_PWM_ENABLE(PWM7),
		REGS_PWM_BASE + HW_PWM_CTRL_CLR);

  return HRTIMER_NORESTART;
}

static void beeper_status(void)
{
	int reg;
	
	pr_info("*******************\n");
	/* pwm ctrl */
	reg =  __raw_readl(REGS_PWM_BASE);
	pr_info("%s: [0x%08X] REGS_PWM_BASE = 0x%08X \n",__FUNCTION__,(unsigned int)REGS_PWM_BASE,reg);

	/* pwm4 */
	reg =  __raw_readl(PWM_VOL_ACTIVE);
	pr_info("%s: [0x%08X] PWM_VOL_ACTIVE = 0x%08X \n",__FUNCTION__,(unsigned int)PWM_VOL_ACTIVE,reg);
	reg =  __raw_readl(PWM_FREQ_ACTIVE);
	pr_info("%s: [0x%08X] PWM_FREQ_ACTIVE = 0x%08X \n",__FUNCTION__,(unsigned int)PWM_FREQ_ACTIVE,reg);

	/* pwm7 */
	reg =  __raw_readl(PWM_VOL_PERIOD);
	pr_info("%s: [0x%08X] PWM_VOL_PERIOD = 0x%08X \n",__FUNCTION__,(unsigned int)PWM_VOL_PERIOD,reg);
	reg =  __raw_readl(PWM_FREQ_PERIOD);
	pr_info("%s: [0x%08X] PWM_FREQ_PERIOD = 0x%08X \n",__FUNCTION__,(unsigned int)PWM_FREQ_PERIOD,reg);
	pr_info("*******************\n");
}

static int __init beeper_init(void)
{
	int ret;
	
	pr_info("\nSucceeded in registering device\n");
	
	beeper_status();
	
	/* turn off */
	__raw_writel(BF_PWM_CTRL_PWM_ENABLE(PWM4),
		REGS_PWM_BASE + HW_PWM_CTRL_CLR);
	__raw_writel(BF_PWM_CTRL_PWM_ENABLE(PWM7),
		REGS_PWM_BASE + HW_PWM_CTRL_CLR);
	
	
	beeper_set_vol();
	beeper_set_freq();
	
	beeper_status();
	
	hrtimer_init( &hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	hr_timer.function = &beeper_hrtimer_callback;
	
	/* setup sysfs */
	beeper_kobj = kobject_create_and_add("beeper",kernel_kobj);
	
	if(!beeper_kobj) {
		return -ENOMEM;
	}
	
	ret = sysfs_create_group(beeper_kobj, &attr_group);
	
	return 0;
}

static void __exit beeper_exit(void)
{
	int ret;
	
	pr_info("\ndevice unregistered\n");

	ret = hrtimer_cancel( &hr_timer );
	pr_info("HR Timer module uninstalling\n");
	
	/* turn off */
	__raw_writel(BF_PWM_CTRL_PWM_ENABLE(PWM4),
		REGS_PWM_BASE + HW_PWM_CTRL_CLR);
	__raw_writel(BF_PWM_CTRL_PWM_ENABLE(PWM7),
		REGS_PWM_BASE + HW_PWM_CTRL_CLR);
		
	kobject_put(beeper_kobj);
}

module_init(beeper_init);
module_exit(beeper_exit);

MODULE_AUTHOR("Jeff Horn");
MODULE_LICENSE("GPL v2");
