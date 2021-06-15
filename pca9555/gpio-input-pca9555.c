/*
 *  PCA953x 4/8/16/24/40 bit I/O ports
 *
 *  Copyright (C) 2005 Ben Gardner <bgardner@wabtec.com>
 *  Copyright (C) 2007 Marvell International Ltd.
 *
 *  Derived from drivers/i2c/chips/pca9539.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <linux/proc_fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/regmap.h>


#include <linux/platform_data/pca953x.h>
#include <linux/slab.h>
#ifdef CONFIG_OF_GPIO
#include <linux/of_platform.h>
#endif
#include <linux/acpi.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/timer.h>




#define PCA_INT			0x0100
#define PCA953X_TYPE		0x1000
#define PCA957X_TYPE		0x2000
#define PCA_TYPE_MASK		0xF000

#define PAC9535_NAME  "pca9535"

enum pca9535kbd_cmd  
{  
    PCA9535_INPUT_0     = 0,  
    PCA9535_INPUT_1     = 1,  
    PCA9535_OUTPUT_0    = 2,  
    PCA9535_OUTPUT_1    = 3,  
    PCA9535_INVERT_0    = 4,  
    PCA9535_INVERT_1    = 5,  
    PCA9535_DIRECTION_0 = 6,  
    PCA9535_DIRECTION_1 = 7,  
};  


struct gpio_key_button {  
    u16         gpio_mask;  
    int         keycode;  
}; 
struct gpio_key_button key_buttons[] = {  
    { .gpio_mask = 0x0001, .keycode = KEY_1 },  
    { .gpio_mask = 0x0002, .keycode = KEY_2 },  
    { .gpio_mask = 0x0004, .keycode = KEY_3 },  
    { .gpio_mask = 0x0008, .keycode = KEY_4 },  
    { .gpio_mask = 0x0010, .keycode = KEY_5 }, 
    { .gpio_mask = 0x0020, .keycode = KEY_6 },  
    { .gpio_mask = 0x0040, .keycode = KEY_7 }, 
    { .gpio_mask = 0x0080, .keycode = KEY_8 },
    { .gpio_mask = 0x0100, .keycode = KEY_9 },  
    { .gpio_mask = 0x0200, .keycode = KEY_BACKSPACE },
	{ .gpio_mask = 0x0400, .keycode = KEY_0 },
	{ .gpio_mask = 0x0800, .keycode = KEY_ENTER },
	{ .gpio_mask = 0x1000, .keycode = KEY_UP },
	{ .gpio_mask = 0x2000, .keycode = KEY_DOWN },
	{ .gpio_mask = 0x4000, .keycode = KEY_LEFT },
	{ .gpio_mask = 0x8000, .keycode = KEY_RIGHT },
		
}; 

static const struct i2c_device_id pca953x_id[] = {
	{ "pca9535", 16 | PCA953X_TYPE | PCA_INT, },
	
	{ }
};
MODULE_DEVICE_TABLE(i2c, pca953x_id);

static const struct acpi_device_id pca953x_acpi_ids[] = {
	{ "INT3491", 16 | PCA953X_TYPE | PCA_INT, },
	{ }
};
MODULE_DEVICE_TABLE(acpi, pca953x_acpi_ids);




struct pca953x_priv{
	struct i2c_client *client;
	struct input_dev    *input;  
	struct work_struct  work;
	int irq_gpio_number ;
	int	rst_gpio_number ;
	int irq_det_invert;
	u16 port_state;
	
};
static int device_pca953x_init(struct pca953x_priv *pdata){

	 /* Initialize the PCA9535 to known state */  
	int ret;
    ret = i2c_smbus_write_byte_data(pdata->client, PCA9535_DIRECTION_0, 0xFF);
	if(ret < 0){
		dev_err(&(pdata->client->dev), "===============fail to i2c_smbus_write_byte_data PCA9535_DIRECTION_0   ret=%d\n",ret);
		goto exit;
	}
    ret = i2c_smbus_write_byte_data(pdata->client, PCA9535_DIRECTION_1, 0xFF);
	if(ret < 0){
		dev_err(&(pdata->client->dev), "==========fail to i2c_smbus_write_byte_data PCA9535_DIRECTION_1\n");
		goto exit;
	}
    ret = i2c_smbus_write_byte_data(pdata->client, PCA9535_INVERT_0, 0xFF); 
	if(ret < 0){
		dev_err(&(pdata->client->dev), "============fail to i2c_smbus_write_byte_data PCA9535_INVERT_0\n");
		goto exit;
	}
    ret = i2c_smbus_write_byte_data(pdata->client, PCA9535_INVERT_1, 0xFF); 
	if(ret < 0){
		dev_err(&(pdata->client->dev), "=============fail to i2c_smbus_write_byte_data PCA9535_INVERT_1\n");
		goto exit;
	}
	exit:
		return ret;
}
static void pca953x_do_work(struct work_struct *work_data) {
	int i;  
    u16 new_state, pressed, released;  
    struct pca953x_priv *data = container_of(work_data,  struct pca953x_priv, work);  
    new_state = i2c_smbus_read_byte_data(data->client, PCA9535_INPUT_0)   
                | (i2c_smbus_read_byte_data(data->client, PCA9535_INPUT_1) << 8);  
     
	dev_err(&(data->client->dev), "================%s:0x%x" ,__func__, new_state);
    if(new_state == data->port_state)  
        return;    
    /* detect buttons which be newly pressed */  
    released = data->port_state & ~new_state;  
    /* detect buttons which just has been released */  
    pressed = new_state & ~data->port_state;  
      
    for(i = 0; i < ARRAY_SIZE(key_buttons); i++) {  
        if(pressed & key_buttons[i].gpio_mask) {  
            input_event(data->input, EV_KEY, key_buttons[i].keycode, 1);  
            input_sync(data->input);  
        }  
        if(released & key_buttons[i].gpio_mask) {  
            input_event(data->input, EV_KEY, key_buttons[i].keycode, 0);  
            input_sync(data->input);  
        }  
    }  
    data->port_state = new_state;  
}
static irqreturn_t pca9535_irq_handler(int irq, void *data)
{
	struct pca953x_priv *pdata = data;

	schedule_work(&pdata->work);

	return IRQ_HANDLED;
}

static int pca953x_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{

	struct	pca953x_priv *pdata;
	struct device_node *np = client->dev.of_node;
	int ret ;
	int i;
	int pca9535_irq;
	enum of_gpio_flags flags;
	pdata = devm_kzalloc(&client->dev,sizeof(struct pca953x_priv), GFP_KERNEL);
	if (pdata == NULL)
		return -ENOMEM;
	memset(pdata,0,sizeof(struct pca953x_priv));
	pdata->client= client;
	
	dev_err(&client->dev, "=============pca953x_probe\n");
	/*input dev init*/
	pdata->input = input_allocate_device();
	if(!pdata->input)
		goto fail0;
	pdata->input->name = PAC9535_NAME;
	pdata->input->id.bustype = BUS_I2C;	
	pdata->input->id.vendor = 0x0001;  
	pdata->input->id.product = 0x0001;  
	pdata->input->id.version = 0x0100;
	__set_bit(EV_KEY, pdata->input->evbit);
	__set_bit(INPUT_PROP_DIRECT, pdata->input->propbit);
    for(i = 0; i < ARRAY_SIZE(key_buttons); i++) {  
        input_set_capability(pdata->input, EV_KEY, key_buttons[i].keycode);  
    }  
	
	if((ret = input_register_device(pdata->input))) {
		dev_err(&client->dev, "%s:=================== failed to register input device: %s\n", 
			__func__, dev_name(&client->dev));
		goto fail0;
	}
	
	pdata->irq_gpio_number = of_get_named_gpio_flags(np,
						      "irq_gpio_number",
						      0,
						      &flags);
	if (pdata->irq_gpio_number < 0) {
		dev_err(&client->dev, "=============Can not read property hp_det_gpio\n");
		pdata->irq_gpio_number = -1;
	} else {
		INIT_WORK(&pdata->work, pca953x_do_work);
		pdata->irq_det_invert = !!(flags & OF_GPIO_ACTIVE_LOW);
		ret = devm_gpio_request_one(&client->dev, pdata->irq_gpio_number,
					    GPIOF_IN, "pca9535_irq");
		if (ret < 0)
			return ret;
		pca9535_irq = gpio_to_irq(pdata->irq_gpio_number);
		ret = devm_request_threaded_irq(&client->dev, pca9535_irq, NULL,
						pca9535_irq_handler,
						IRQF_TRIGGER_FALLING |
						IRQF_TRIGGER_RISING |
						IRQF_ONESHOT,
						"pca9535_interrupt", pdata);
		if (ret < 0) {
			dev_err(&client->dev, "=========request_irq failed: %d\n", ret);
			return ret;
		}
	
	}
	/*pdata->rst_gpio_number = of_get_named_gpio_flags(np,
						      "rst_gpio_number",
						      0,
						      &flags);
	ret = devm_gpio_request_one(&client->dev, pdata->rst_gpio_number,
					    GPIOF_IN, "pca9535_rst");
	if (ret < 0)
		return ret;
	*/
	i2c_set_clientdata(client, pdata);
	ret = device_pca953x_init(pdata);

	return 0;

	fail0:
		return -1;
}
static int pca953x_remove(struct i2c_client *client)
{
	
	struct pca953x_priv *pdata = i2c_get_clientdata(client);
	if(pdata == NULL)
		return 0;
	dev_err(&(client->dev), "================%s:%d" ,__func__, __LINE__);
	if(pdata->input){
		input_unregister_device(pdata->input);
		input_free_device(pdata->input);
	}
	dev_err(&(client->dev), "===============%s:%d" ,__func__, __LINE__);
	i2c_set_clientdata(client, NULL);
	kfree(pdata);
	dev_err(&(client->dev), "=============%s:%d" ,__func__, __LINE__);
	return 0;
}

static const struct of_device_id pca953x_dt_ids[] = {
	{ .compatible = "nxp,pca9535_keypad", },
	
	{ }
};

MODULE_DEVICE_TABLE(of, pca953x_dt_ids);

static struct i2c_driver pca953x_driver = {
	.driver = {
		.name	= PAC9535_NAME,
		.of_match_table = pca953x_dt_ids,
		.acpi_match_table = ACPI_PTR(pca953x_acpi_ids),
	},
	.probe		= pca953x_probe,
	.remove		= pca953x_remove,
	.id_table	= pca953x_id,
};

static int __init pca953x_init(void)
{
	return i2c_add_driver(&pca953x_driver);
}
/* register after i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
module_init(pca953x_init);

static void __exit pca953x_exit(void)
{
	i2c_del_driver(&pca953x_driver);
}
module_exit(pca953x_exit);

MODULE_AUTHOR("jerry deng <394419178@qq.com>");
MODULE_DESCRIPTION("keypad driver for PCA953x");
MODULE_LICENSE("GPL");

