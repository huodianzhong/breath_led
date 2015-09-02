/*
 * AMLOGIC breath driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the named License,
 * or any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/of_gpio.h>
#include <linux/amlogic/cpu_version.h>
#include <linux/amlogic/iomap.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/amlogic/pm.h>


#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
static struct early_suspend early_suspend;
#endif

//#define aml_breathled_DBG
#define BREATHLED_MODULE_NAME   "aml-breathled"

//reserved ,now not used
struct aml_breathled_platform_data {
	const char *name;
	struct resource res;
	unsigned char __iomem *reg_base;
	int size;
	unsigned int  pwm_prescale; //reserved 
	unsigned int  pwm_entire_cycle; //reserved 
	unsigned int  pwm_channel; //reserved
};

struct aml_breathled {
	struct aml_breathled_platform_data	*pdata;
	struct platform_device			*pdev;	
};

typedef enum aml_breathled_mode_e{
    ALWAYS_ON= 0,
    ALWAYS_OFF,
    LED_BLINK,
    LED_BREATH,
}aml_breathled_mode_t;

struct aml_breathled_platform_data *Mypdata = NULL;
/* pwm reg define*/
#define PWM_PWM_B 0
#define PWM_MISC_REG_AB 4
#define PWM_B_EN    (1<<1)
#define PWM_B_CLK_EN (1<<23)
#define PWM_B_CLK_SEL(index) ((index&0x03) << 6)
#define PWM_B_CLK_DRV(prescale)  (prescale<<16)

/*here for pwm config*/
static unsigned int pwm_prescale = 0x03; //AO_PWM_MISC_REG_AB(PWM_B_CLK_DIV) [22:16] 0-0x7f
static unsigned int pwm_entire_cycle = 0xFFFF; //AO_PWM_PWM_B(PWM_B_DUTY_CYCLE) [31:15]H,[15-0]L   //915KHz
/* Timer period, which determines the breath rynthm of Led */
static unsigned int timer_n_ms = 10;
/* Current index for locating duty_array */
static unsigned int cur_index = 0;
/* Timer */
static struct timer_list g_timer;
/* led blink flg,we need it because need to change prescale */
static unsigned int blink_status = 0;
/* led mode */
unsigned int led_mode_s = 0;
/* led breath timeout 20ms*/
static unsigned int led_breath_timeout = 0;

#define BREATH_TIMEOUT_COUNT 1500 /* 15s */
#define DUTY_ARRAY_COUNT  160
//#define MODULE_LED_DEBUG
#define AUTOSUSPEND_TEST

static int led_control(unsigned int led_mode );
/* Determine brightness of led */
static __u32 duty_array[DUTY_ARRAY_COUNT] = {
         0,  1,  1,  1,  2,  3,  4,  5,  6,  7,
         8,  9,  10, 11, 12, 13, 14, 15, 16, 17,
         18, 19, 20, 21, 22, 23, 24, 25, 26, 27,
         28, 29, 30, 31, 32, 33, 34, 35, 36, 37,
         38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
         48, 49, 50, 51, 52, 53, 54, 55, 56, 57,
         58, 59, 60, 61, 62, 63, 64, 65, 66, 67,
         68, 69, 70, 71, 72, 73, 74, 75, 76, 77,
         78, 79, 80, 81, 82, 83, 84, 85, 86, 87,
         88, 89, 90, 91, 92, 93, 94, 95, 96, 97,
         98, 99, 100,96, 94, 90, 88, 86, 84, 82,
         80, 78, 74, 72, 70, 68, 66, 64, 62, 60,
         59, 58, 56, 54, 53, 51, 49, 46, 44, 42,
         41, 40, 39, 38, 36, 34, 32, 30, 28, 26,
         24, 22, 20, 18, 16, 14, 12, 10,  9,  8,
          8, 8,  7,  6,  5,  4,  3,   2,  1,  0
};

static void leds_pwm_setting(unsigned int entire_cycle, unsigned int duty)
{
    unsigned int tmp = 0;
    unsigned int active_cycle_h,active_cycle_l;
    struct aml_breathled_platform_data *pdata=Mypdata;

    active_cycle_h = entire_cycle * duty / 100;
    active_cycle_l = entire_cycle * (100-duty) / 100;
    tmp |= (active_cycle_h << 16);
    tmp |= active_cycle_l;

    writel(tmp,pdata->reg_base+PWM_PWM_B);
}

static void g_timer_handle(unsigned long arg)
{
    led_breath_timeout++;
    if(led_breath_timeout > BREATH_TIMEOUT_COUNT){
        led_breath_timeout = 0;
        led_control(ALWAYS_ON);
        printk(KERN_INFO"led breath 15s timeout.\n");
        return;
    }

    mod_timer(&g_timer, jiffies + msecs_to_jiffies(timer_n_ms));
    if (cur_index == DUTY_ARRAY_COUNT - 1) {
        cur_index = 0;
    } else {
        cur_index++;
    }

    leds_pwm_setting(pwm_entire_cycle, duty_array[cur_index]);
#ifdef MODULE_LED_DEBUG 
    printk(KERN_INFO"cur_index=%d \n",cur_index);
#endif
}
static void g_timer_init(unsigned int timer_expires_ms)
{
    init_timer(&g_timer);
    g_timer.function = &g_timer_handle;
    g_timer.expires = jiffies + msecs_to_jiffies(timer_expires_ms);//HZ / 1000 * timer_expires_ms;
}
static void g_timer_start(unsigned int timer_expires_ms)
{
    mod_timer(&g_timer, jiffies + msecs_to_jiffies(timer_expires_ms));//HZ / 1000 * timer_expires_ms);
}
static void g_timer_del(void)
{
    int ret;

    ret=del_timer(&g_timer);
    if (ret)
        printk(KERN_INFO"The timer is still in use...\n");
    else
        printk(KERN_INFO"The timer init but not use...\n");
}
static void leds_pwm_init( unsigned int prescale, unsigned int cycle)
{
    struct aml_breathled_platform_data *pdata=Mypdata;
    u32 val;

    val=readl(pdata->reg_base+PWM_MISC_REG_AB);
    val = PWM_B_CLK_DRV(prescale) | PWM_B_EN | PWM_B_CLK_EN | PWM_B_CLK_SEL(0);
    printk(KERN_INFO"%s writel:0x%x\n",__func__,val);
    writel(val,pdata->reg_base+PWM_MISC_REG_AB);

    writel(cycle,pdata->reg_base+PWM_PWM_B);  //0x2154 [31:15]H,[15-0]L 
}
/*led control*/
static int led_control(unsigned int led_mode )
{
    if(blink_status == 1 )
    {
        blink_status = 0;
        leds_pwm_init(pwm_prescale,pwm_entire_cycle); 
    }

    switch(led_mode)
    {
        case ALWAYS_ON:
            g_timer_del();
            mdelay(10);
            leds_pwm_setting(pwm_entire_cycle, 100);
            led_mode_s = ALWAYS_ON;
            break;
        case LED_BREATH:
            g_timer_start(timer_n_ms);
            led_mode_s = LED_BREATH;
            break;
        case LED_BLINK:
            blink_status = 1;
            g_timer_del();
            mdelay(10);
            leds_pwm_init(0x7f,0xffffffff); //clk 1.35Hz for recovery facotry reset
            led_mode_s = LED_BLINK;
            break;
        case ALWAYS_OFF:
            g_timer_del();
            mdelay(10);
            leds_pwm_setting(pwm_entire_cycle, 0);
            led_mode_s = ALWAYS_OFF;
            break;
    }
    return 0;
}
/*breathled class*/
static ssize_t class_set_led_mode(struct class *cla, struct class_attribute *attr, const char *buf, size_t count)
{
	unsigned int led_mode=0;

	if (!strcmp(attr->attr.name, "led_mode")) {        
		printk(KERN_INFO"----%s", buf);
		sscanf(buf, "%d", &led_mode);       
	}
	led_control(led_mode);
	return count;
}
static ssize_t class_get_led_mode(struct class *cla, struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", led_mode_s);
}
static struct class_attribute breathled_class_attrs[] = {
	__ATTR(led_mode, 0666, class_get_led_mode, class_set_led_mode),
	__ATTR_NULL
};
static struct class breathled_class = {
	.name = "aml_breathled",
	.class_attrs = breathled_class_attrs,
};
/*driver match*/
static const struct of_device_id aml_breathled_match[] =
{
	{
		.compatible = "amlogic, aml_breathled",
	},
	{},
};
/*early suspend&resume*/
#ifdef CONFIG_HAS_EARLYSUSPEND
static void aml_breathled_early_suspend(struct early_suspend *dev)
{
	printk(KERN_INFO "enter aml_breathled_early_suspend\n");
	led_control(ALWAYS_OFF);
}
static void aml_breathled_late_resume(struct early_suspend *dev)
{
	printk(KERN_INFO "enter aml_breathled_late_resume\n");
	printk(KERN_INFO"dgt, aml_breathled_late_resume breath led\n");
	led_control(ALWAYS_ON);
}
#endif
/*suspend&resume*/
static int aml_breathled_suspend(struct platform_device *pdev,pm_message_t state)
{
	printk(KERN_INFO "enter aml_breathled_suspend\n");
	return 0;
}

static int aml_breathled_resume(struct platform_device *pdev)
{
    printk(KERN_INFO "enter aml_breathled_resume\n");
    if (get_resume_method() == REMOTE_WAKEUP
#ifdef AUTOSUSPEND_TEST
|| get_resume_method() == AUTO_WAKEUP
#endif
)
    {
        printk(KERN_INFO "breathled_resume:led on\n");
        led_control(ALWAYS_ON);
    }
    return 0;
}

static int aml_breathled_probe(struct platform_device *pdev)
{
    struct aml_breathled_platform_data *pdata;
    struct aml_breathled *amlbreathled;
    int retval=-1,ret;
    int ret_dts = -1;
    struct device_node *np = pdev->dev.of_node;
    int value = -1;
    struct pinctrl *pin_ctl;
    struct resource *res;
    int size=0;

    printk(KERN_INFO "enter aml_breathled_probe\n");
    amlbreathled = kzalloc(sizeof(struct aml_breathled), GFP_KERNEL);
    if (!amlbreathled)
    {   
        printk(KERN_ERR "kzalloc error\n");
        return -ENOMEM;
    }
    pdata=kzalloc(sizeof(struct aml_breathled_platform_data),GFP_KERNEL);
    if(!pdata){
        goto err;
    }
    memset((void* )pdata,0,sizeof(*pdata));
    amlbreathled->pdev = pdev;

    //dts init
    if(np == NULL ){
        printk(KERN_ERR "np == null \n");
        goto err;
    }
    printk(KERN_INFO"start read breathled dts \n");

    /* map reg mem addr*/
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(&pdev->dev, "cannot obtain I/O memory region\n");
        return -ENODEV;
    }
    pdata->res = *res;
    size = resource_size(res);
    pdata->size = size;
    if (!devm_request_mem_region(&pdev->dev, res->start, size,
                pdev->name)) {
        printk(KERN_ERR"Memory region busy\n");
        return -EBUSY;
    }
    pdata->reg_base = devm_ioremap_nocache(&pdev->dev, res->start, size);
    if (pdata->reg_base == NULL){
        dev_err(&pdev->dev, "devm_ioremap_nocache failed!\n");
        goto err1;
    }
    printk(KERN_INFO"pwmb reg addr = %p,size=%d \n",pdata->reg_base,pdata->size);
    /*set pin ctl */
    pin_ctl = devm_pinctrl_get_select(&pdev->dev, "aml_breathled_pin");
    if (IS_ERR(pin_ctl))
        printk(KERN_ERR"aml breathled pinmux set error!\n");

    //read dev_name
    ret_dts=of_property_read_string(pdev->dev.of_node,"dev_name",&pdata->name);
    if (ret_dts){
        dev_err(&pdev->dev, "read %s  error\n","dev_name");
        goto err;
    }
    printk(KERN_INFO"breathled pdata->name:%s\n",pdata->name);


    //read pwm_prescale
    value=-1;
    ret_dts=of_property_read_u32(pdev->dev.of_node,"pwm_prescale",&value);
    if (ret_dts){
        dev_err(&pdev->dev, "read %s  error\n","pwm_prescale");
        goto err;
    }
    pdata->pwm_prescale = value;
    if(pwm_prescale != pdata->pwm_prescale)
        pwm_prescale = pdata->pwm_prescale;
    printk(KERN_INFO"breathled pdata->pwm_prescale:%d\n",pwm_prescale);

    //read pwm_entire_cycle
    value=-1;
    ret_dts=of_property_read_u32(pdev->dev.of_node,"pwm_entire_cycle",&value);
    if (ret_dts){
        dev_err(&pdev->dev, "read %s  error\n","pwm_entire_cycle");
        goto err;
    }
    pdata->pwm_entire_cycle=value;
    if(pwm_entire_cycle != pdata->pwm_entire_cycle)
        pwm_entire_cycle = pdata->pwm_entire_cycle;
    printk(KERN_INFO"breathled pdata->pwm_entire_cycle:0x%x\n",pwm_entire_cycle);

    //end dts init

    if (!pdata) {
        printk(KERN_ERR "missing platform data\n");
        retval = -ENODEV;
        goto err;
    }	
    amlbreathled->pdata = pdata;
    Mypdata=pdata;
    platform_set_drvdata(pdev, amlbreathled);

#ifdef CONFIG_HAS_EARLYSUSPEND
    early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 9;
    early_suspend.suspend = aml_breathled_early_suspend;
    early_suspend.resume = aml_breathled_late_resume;
    register_early_suspend(&early_suspend);
#endif

    /*class register */
    ret = class_register(&breathled_class);
    if (ret){
        printk(KERN_ERR "class_register breathled_class failed\n");
        goto err;
    }
    /*PWM init */
    g_timer_init(timer_n_ms);
    leds_pwm_init(pwm_prescale,pwm_entire_cycle);
    leds_pwm_setting(pwm_entire_cycle, 100);
    return 0;

err:
    kfree(amlbreathled);
    kfree(pdata);
    return retval;
err1:
    devm_release_mem_region(&pdev->dev, res->start, size);
    printk(KERN_ERR"Canvas driver probe failed\n");
    return retval;
}

static int __exit aml_breathled_remove(struct platform_device *pdev)
{
	struct aml_breathled *amlbreathled = platform_get_drvdata(pdev);
	struct aml_breathled_platform_data *pdata=Mypdata;

	printk(KERN_INFO "enter aml_breathled_remove\n");
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&early_suspend);
#endif
	devm_iounmap(&pdev->dev, pdata->reg_base);
	devm_release_mem_region(&pdev->dev,
		pdata->res.start,
		pdata->size); 
	class_unregister(&breathled_class);
	g_timer_del();
	platform_set_drvdata(pdev, NULL);
	kfree(amlbreathled);
	led_mode_s = ALWAYS_ON;

	return 0;
}

static struct platform_driver aml_breathled_driver = {
	.driver = {
		.name = "aml_breathled",
	.owner = THIS_MODULE,
	.of_match_table = of_match_ptr(aml_breathled_match),
	},
	.probe = aml_breathled_probe,
	.remove = __exit_p(aml_breathled_remove),
	.suspend = aml_breathled_suspend,
	.resume  = aml_breathled_resume,
};

static int __init aml_breathled_init(void)
{
	int ret = -1;
  
	printk(KERN_INFO "enter aml_breathled_init\n");
	ret = platform_driver_register(&aml_breathled_driver);
  
	if (ret != 0) {
		printk(KERN_ERR "failed to register breathled driver, error %d\n", ret);
		return -ENODEV;
	}
	return ret;    
}
module_init(aml_breathled_init);

static void __exit aml_breathled_exit(void)
{
	platform_driver_unregister(&aml_breathled_driver);
}
module_exit(aml_breathled_exit);

MODULE_DESCRIPTION("Amlogic breathled driver");
MODULE_AUTHOR("dianzhong.huo <dianzhong.huo@amlogic.com>");
MODULE_LICENSE("GPL");
