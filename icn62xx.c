/*
 * drivers/input/touchscreen/gslX680.c
 *
 * Copyright (c) 2012 Shanghai Basewin
 *	Guan Yuwei<guanyuwei@basewin.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/input/mt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>


#include "icn62xx.h"


#define ICN62XX_I2C_NAME 	"ICN62XX_LCD"
#define ICN62XX_I2C_ADDR 	0x58
#define IRQ_PORT		PB_PIO_IRQ(CFG_IO_TOUCH_PENDOWN_DETECT)//IRQ_EINT(8)

#define TS_POLL_PERIOD 200000000
#define TS_POLL_DELAY 1000000

#define ICN_DEBUG 1

struct icn62xx_info {
	struct i2c_client *client;
	struct delayed_work work;
	struct workqueue_struct *wq;
	//struct gsl_ts_data *dd;
	//u8 *touch_data;
	u8 device_id;
	struct hrtimer	timer;
	int irq;
	int irq_pin;
	int rst_pin;
	int rst_val;
	spinlock_t		lock;
};


static struct icn62xx_info *icn_info;
//static struct i2c_client *icn_client = NULL;


#ifdef ICN_DEBUG
#define print_info(fmt, args...)   \
        do{                              \
                printk(fmt, ##args);     \
        }while(0)
#else
#define print_info(fmt, args...)
#endif


#if 0
static u32 gsl_read_interface(struct i2c_client *client, u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[2];

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = &reg;

	xfer_msg[1].addr = client->addr;
	xfer_msg[1].len = num;
	xfer_msg[1].flags |= I2C_M_RD;
	xfer_msg[1].buf = buf;

	if (reg < 0x80) {
		i2c_transfer(client->adapter, xfer_msg, ARRAY_SIZE(xfer_msg));
		msleep(5);
	}

	return i2c_transfer(client->adapter, xfer_msg, ARRAY_SIZE(xfer_msg)) == ARRAY_SIZE(xfer_msg) ? 0 : -EFAULT;
}
#endif

u32  icn62xx_write_interface(struct i2c_client *client, const u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[1];

	buf[0] = reg;

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = buf;

	return i2c_transfer(client->adapter, xfer_msg, 1) == 1 ? 0 : -EFAULT;
}

int icn62xx_write(struct i2c_client *client, u8 addr, u8 *pdata, int datalen)
{
	int ret = 0;
	u8 tmp_buf[128];
	unsigned int bytelen = 0;
	if (datalen > 125)
	{
		printk(KERN_ERR"%s too big datalen = %d!\n", __func__, datalen);
		return -1;
	}

	tmp_buf[0] = addr;
	bytelen++;

	if (datalen != 0 && pdata != NULL)
	{
		memcpy(&tmp_buf[bytelen], pdata, datalen);
		bytelen += datalen;
	}

	ret = i2c_master_send(client, tmp_buf, bytelen);
	return ret;
}

int icn62xx_readbyte(struct i2c_client *client, u8 addr, u8 *pdata)
{
	int ret = 0;
	ret = icn62xx_write(client, addr, NULL, 0);
	if (ret < 0)
	{
		printk(KERN_ERR"%s set data address fail!\n", __func__);
		return ret;
	}

	return i2c_master_recv(client, pdata, 1);
}

 int icn62xx_read(struct i2c_client *client, u8 addr, u8 *pdata, unsigned int datalen)
{
	int ret = 0;
	int i = 0;

	if (datalen > 126)
	{
		printk("%s too big datalen = %d!\n", __func__, datalen);
		return -1;
	}
	for(i=0; i<datalen; i++){
		ret = icn62xx_readbyte(client, addr+i, pdata+i);
		if(ret < 0)
			return ret;
	}
	return ret;
}

 int test_i2c_write( u8 addr, u8 *pdata, int datalen)
 {
 	if(icn_info != NULL)
 	{
		 return icn62xx_write(icn_info->client,addr,pdata,datalen);
 	}
	else
	{
		printk(KERN_ERR"########%s:icn62xx i2c driver not inititioned#######\n", __func__);
		return -1;
	}		
 }

 
 int icn62xx_i2c_write( u8 addr, u8 *pdata, int datalen)
 {
 	if(icn_info != NULL)
 	{
		 return icn62xx_write(icn_info->client,addr,pdata,datalen);
 	}
	else
	{
		printk(KERN_ERR"########%s:icn62xx i2c driver not inititioned#######\n", __func__);
		return -1;
	}		
 }

 int icn62xx_i2c_read(u8 addr, u8 *pdata, unsigned int datalen)
{
 	if(icn_info != NULL)
 	{
		 return icn62xx_read(icn_info->client,addr,pdata,datalen);
 	}
	else
	{
		printk(KERN_ERR"########%s:icn62xx i2c driver not inititioned#######\n", __func__);
		return -1;
	}	

}
#if 0
static int test_i2c(struct i2c_client *client)
{
	u8 addr;
	u8 buf;

	addr = 0x56;
	buf = 0x93;
	

	pr_err("===============%s============\n", __func__);
	if(icn62xx_write(client, addr, &buf, 1) < 0)
		{
		printk(KERN_ERR"#########%s:write 0x56 fail########\n", __func__);

		}
	else
		{
		printk(KERN_ERR"#########%s:write 0x56 success########\n", __func__);
		}

	buf = 0x00;
	if(icn62xx_read(client, addr, &buf, 1) < 0)
	{
		printk(KERN_ERR"#########%s:read 0x56 fail########\n", __func__);
		return -1;
	}
	else
		printk(KERN_ERR"#########%s:read 0x56 value=%d########\n", __func__, buf);

	return 0;
}


static void reset_chip(struct i2c_client *client)
{
	//u8 tmp = 0x88;
	//u8 buf[4] = {0x00};

	/*gsl_ts_write(client, 0xe0, &tmp, sizeof(tmp));
	msleep(20);
	tmp = 0x04;
	gsl_ts_write(client, 0xe4, &tmp, sizeof(tmp));
	msleep(10);
	gsl_ts_write(client, 0xbc, buf, sizeof(buf));
	msleep(10);*/
}

static void clr_reg(struct i2c_client *client)
{
	//u8 write_buf[4]	= {0};

	/*write_buf[0] = 0x88;
	gsl_ts_write(client, 0xe0, &write_buf[0], 1);
	//msleep(20);
	write_buf[0] = 0x03;
	gsl_ts_write(client, 0x80, &write_buf[0], 1);
	//msleep(5);
	write_buf[0] = 0x04;
	gsl_ts_write(client, 0xe4, &write_buf[0], 1);
	//msleep(5);
	write_buf[0] = 0x00;
	gsl_ts_write(client, 0xe0, &write_buf[0], 1);
	//msleep(20);*/
}
#endif
#if 0
typedef struct
{
	const char *name;   /* name of the fdt property defining this */
	uint gpio;      /* GPIO number, or FDT_GPIO_NONE if none */
	u8 flags;       /* FDT_GPIO_... flags */
}gpio_config;

__maybe_unused static gpio_config		board_model_dc_gpio[3];


__maybe_unused void boardModelDetectGpioInit(void)
{
	board_model_dc_gpio[0].name = "board_model_dc1";
	board_model_dc_gpio[0].flags = 0;
	board_model_dc_gpio[0].gpio = (GPIO_BANK1 | GPIO_A0);
	gpio_free(board_model_dc_gpio[0].gpio);
	gpio_request(board_model_dc_gpio[0].gpio,board_model_dc_gpio[0].name);
	gpio_direction_input(board_model_dc_gpio[0].gpio);

	board_model_dc_gpio[1].name = "board_model_dc2";
	board_model_dc_gpio[1].flags = 0;
	board_model_dc_gpio[1].gpio = (GPIO_BANK1 | GPIO_A1);
	
	gpio_free(board_model_dc_gpio[1].gpio);
	gpio_request(board_model_dc_gpio[1].gpio,board_model_dc_gpio[1].name);
	gpio_direction_input(board_model_dc_gpio[0].gpio);

	board_model_dc_gpio[2].name = "board_model_dc3";
	board_model_dc_gpio[2].flags = 0;
	board_model_dc_gpio[2].gpio = (GPIO_BANK0 | GPIO_B0);
	
	gpio_free(board_model_dc_gpio[2].gpio);
	gpio_request(board_model_dc_gpio[2].gpio,board_model_dc_gpio[2].name);
	gpio_direction_input(board_model_dc_gpio[0].gpio);
	

}

unsigned int getBoardModel(void)
{
	int model = 0;
	
	boardModelDetectGpioInit();
	
	model  |= (gpio_get_value(board_model_dc_gpio[0].gpio)) << 2;
	model  |= (gpio_get_value(board_model_dc_gpio[1].gpio)) << 1;
	model  |= (gpio_get_value(board_model_dc_gpio[2].gpio));

	return model;
}

#endif

static int init_chip(struct i2c_client *client)
{
	int rc = 0;
	u8 addr;
	u8 buf;
	u8 model=1;


	addr = 0x20;
	buf = 0x20;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}
	addr = 0x21;
	buf = 0x58;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}

	addr = 0x22;
	buf = 0x23;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}

	addr = 0x23;
	buf = 0xF0;//0x78;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}

	addr = 0x24;
	buf =0x14;// 0x3C;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}

	addr = 0x25;
	buf = 0x1A;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}

	addr = 0x26;
	buf = 0x00;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}

	addr = 0x27;
	buf =0x14;// 0x06;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}

	addr = 0x28;
	buf = 0x05;//0x06;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}

	addr = 0x29;
	buf = 0x12;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
		
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}

	addr = 0x34;
	buf = 0x80;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}

	addr = 0x36;
	buf =0xF0;// 0x78;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}

	
	addr = 0xB5;
	buf = 0xA0;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}

	addr = 0x5C;
	buf = 0xFF;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}

	
	addr = 0x2A;
	buf = 0x01;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}
	
	addr = 0x56;
	buf = 0x90;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}

	addr = 0x6B;
	buf = 0x71;//0x71;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}

	addr = 0x69;
	buf = 0x32;//0x32;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}

	addr = 0xB6;
	buf = 0x20;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}

	addr = 0x51;
	buf = 0x20;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}



	addr = 0x90;
	buf = 0x0f;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}

	
	addr = 0x09;
	buf = 0x10;
	if(icn62xx_write(client, addr, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, addr, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, addr, buf);
	}



	return rc;
}



 int read_chip_reg(void)
{
	int rc = 0;
	u8 reg;
	u8 buf = 0;

	print_info(KERN_ERR"=====icn62xx i2c read chip reg value=====\n");

#ifdef ICN_DEBUG
	reg = 0x88;
	buf = 0x80;
	if(icn62xx_write(icn_info->client, reg, &buf, 1) < 0)
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x fail########\n", __func__, reg, buf);
		//return -1;
	}
	else
	{
		print_info(KERN_ERR"#########%s:write 0x%2x:0x%2x success########\n", __func__, reg, buf);
	}

	reg = 0x80;
	buf = 0;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);
	reg = 0x81;
	buf = 0;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);


	reg = 0x20;
	buf = 0;
	icn62xx_readbyte(icn_info->client, reg, &buf );
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);

	reg = 0x21;
	buf = 0;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);




	reg = 0x22;
	buf = 0;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);




	reg = 0x23;
	buf = 0;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);




	reg = 0x24;
	buf = 0;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);



	reg = 0x25;
	buf = 0;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);




	reg = 0x26;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);




	reg = 0x27;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);




	reg = 0x28;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);




	reg = 0x29;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);




	reg = 0x34;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);




	reg = 0x36;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);




	/*reg = 0x86;
	buf = 0x29;
	i2c_reg_write(addr, reg, buf) ;*/
	
	reg = 0xB5;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);




	reg = 0x5C;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);




	
	reg = 0x2A;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);



	
	reg = 0x56;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);



	reg = 0x6B;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);




	reg = 0x69;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);




	reg = 0x10;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);




	reg = 0x11;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);




	reg = 0xB6;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);




	reg = 0x51;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);




	/*reg = 0x31;
	buf = 0xFF;
	i2c_reg_write(addr, reg, buf) ;*/


	reg = 0x14;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);


	reg = 0x2A;
	icn62xx_i2c_read(reg,&buf,1);
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);


	/*reg = 0x2B;
	buf = 0xFF;//0x49;
	i2c_reg_write(addr, reg, buf) ;

	reg = 0x2C;
	buf = 0x00;//0x49;
	i2c_reg_write(addr, reg, buf) ;

	reg = 0x2D;
	buf = 0x00;//0x49;
	i2c_reg_write(addr, reg, buf) ;*/
	
	reg = 0x09;
	//icn62xx_i2c_read(reg,&buf,1);
	icn62xx_readbyte(icn_info->client, reg, &buf );
	print_info(KERN_ERR"reg(0x%2x):value=0x%2x\n", reg, buf);

#endif

	print_info(KERN_ERR"=====icn62xx i2c read chip reg end!=====\n");
	return rc;
}

void do_work(struct work_struct *w)
{
	read_chip_reg();
	queue_delayed_work(icn_info->wq, &icn_info->work,3000);
}

static enum hrtimer_restart icn62xx_timer(struct hrtimer *handle)
{
	struct icn62xx_info	*ts = container_of(handle, struct icn62xx_info, timer);
	spin_lock(&ts->lock);

	printk(KERN_ERR"#############%s############\n", __func__);
	//init_chip(icn_info->client);
	read_chip_reg();

	
	hrtimer_start(&ts->timer, ktime_set(0, TS_POLL_PERIOD),  HRTIMER_MODE_REL);
	spin_unlock(&ts->lock);
	return HRTIMER_NORESTART;
}



static int  icn62xx_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct icn62xx_info *icn;
	int rc;


	//printk("GSLX680 Enter %s\n", __func__);
	printk(KERN_ERR"#############%s############\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C functionality not supported\n");
		return -ENODEV;
	}

	icn = kzalloc(sizeof(struct icn62xx_info), GFP_KERNEL);
	if (!icn)
		return -ENOMEM;
	printk(KERN_ERR"==kzalloc success=\n");

	icn->client = client;
	i2c_set_clientdata(client, icn);
	icn->device_id = 0;

#if 0
	struct device_node *np = client->dev.of_node;
	int power_trig_gpio;
	unsigned long gpio_flags;
	int ret = 0;

	power_trig_gpio = of_get_named_gpio_flags(np, "power-trig", 0, (enum of_gpio_flags *)&gpio_flags);
	//printk(KERN_ERR "===============%s:%d:power_trig_gpio=%d==============\n", __func__, __LINE__, power_trig_gpio);
	if (gpio_is_valid(power_trig_gpio)) {
		ret = devm_gpio_request_one(&client->dev, icn->rst_pin, (gpio_flags & OF_GPIO_ACTIVE_LOW) ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW, "power_trig_gpio  pin");
		if (ret != 0) {
			dev_err(&client->dev, "icn62xx gpio_request error\n");
			//return -EIO;
		}
		gpio_direction_output(power_trig_gpio, 0);
		//gpio_set_value(power_trig_gpio,1);
	} else {
		dev_err(&client->dev, "power_trig_gpio pin invalid\n");
	}


		icn->irq_pin = of_get_named_gpio_flags(np, "touch-gpio", 0, (enum of_gpio_flags *)&irq_flags);
		icn->rst_pin = of_get_named_gpio_flags(np, "reset-gpio", 0, &rst_flags);
		if (gpio_is_valid(ts->rst_pin)) {
			icn->rst_val = (rst_flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
			ret = devm_gpio_request_one(&client->dev, icn->rst_pin, (rst_flags & OF_GPIO_ACTIVE_LOW) ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW, "icn62xx reset pin");
			if (ret != 0) {
				dev_err(&client->dev, "goodix gpio_request error\n");
				return -EIO;
			}
			gpio_direction_output(icn->rst_pin, 0);
			gpio_set_value(tsicnrst_pin, 1);
			msleep(20);
		} else {
			dev_info(&client->dev, "reset pin invalid\n");
		}
#endif
	
		icn_info= icn;
		spin_lock_init(&icn->lock);

		//udelay(50);
		
		init_chip(client);
#ifdef ICN_DEBUG
		//read_chip_reg();
#endif
#if 0	
		rc = gslX680_ts_init(client, ts);
		if (rc < 0) {
			dev_err(&client->dev, "GSLX680 init failed\n");
			goto error_mutex_destroy;
		}
		icn_client = client;
	
		if(init_chip(icn->client) < 0)
			return -1;
	
		icn->irq=gpio_to_irq(icn->irq_pin);
		if (icn->irq)
		{
			rc=  request_irq(icn->irq, gsl_ts_irq, IRQF_TRIGGER_RISING, client->name, icn);
			if (rc != 0) {
				printk(KERN_ALERT "Cannot allocate ts INT!ERRNO:%d\n", ret);
				goto error_req_irq_fail;
			}
		}
#endif

		hrtimer_init(&icn->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		icn->timer.function = icn62xx_timer;
		//hrtimer_start(&icn->timer, ktime_set(0, TS_POLL_DELAY), HRTIMER_MODE_REL);
		INIT_DELAYED_WORK(&icn->work, do_work);
		icn->wq = create_singlethread_workqueue("icn62xx work");
		//queue_delayed_work(icn->wq, &icn->work,100);

		printk("[ICN6X22] End %s\n", __func__);
	
		return 0;
	
	//exit_set_irq_mode:
	//error_req_irq_fail:
	//	free_irq(icn->irq, icn);

		kfree(icn);

	return rc;
}

static int  icn62xx_remove(struct i2c_client *client)
{
	//struct gsl_ts *ts = i2c_get_clientdata(client);
	pr_err("==icn62xx_remove=\n");
	kfree(icn_info);


	/*cancel_delayed_work_sync(&gsl_monitor_work);
	destroy_workqueue(gsl_monitor_workqueue);


	device_init_wakeup(&client->dev, 0);
	cancel_work_sync(&ts->work);
	free_irq(ts->irq, ts);
	destroy_workqueue(ts->wq);
	input_unregister_device(ts->input);
	//device_remove_file(&ts->input->dev, &dev_attr_debug_enable);

	kfree(ts->touch_data);
	kfree(ts);

	gpio_free(gts->rst_pin);*/

	return 0;
}

static const struct i2c_device_id icn62xx_id[] = {
	{ICN62XX_I2C_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, icn62xx_id);

static struct of_device_id icn62xx_dt_ids[] = {
	{ .compatible = "9tripod,icn62xx" },
	{ }
};

static struct i2c_driver icn62xx_driver = {
	.driver = {
		.name = ICN62XX_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(icn62xx_dt_ids),
	},
	.probe		= icn62xx_probe,
	.remove		= icn62xx_remove,
	.id_table	= icn62xx_id,
};

static int __init icn62xx_init(void)
{
	int ret;
	// pr_err("===============%s:i2c_add_driver(&icn62xx_driver)============\n", __func__);
	printk(KERN_ERR"===============%s:i2c_add_driver(&icn62xx_driver)============\n", __func__);

	ret = i2c_add_driver(&icn62xx_driver);
	return ret;
}
static void __exit icn62xx_exit(void)
{
	pr_err("===============%s:i2c_del_driver(&icn62xx_driver)============\n", __func__);
	i2c_del_driver(&icn62xx_driver);
/*	
	gpio_free(board_model_dc_gpio[0].gpio);
	
	gpio_free(board_model_dc_gpio[1].gpio);
	
	gpio_free(board_model_dc_gpio[2].gpio);
	*/
	return;
}

module_init(icn62xx_init);
module_exit(icn62xx_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("icn62xx display controller driver");
MODULE_AUTHOR("Tecom,yanghaishan@tecom-cn.com");
MODULE_ALIAS("platform:icn62xx");
