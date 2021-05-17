/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
//#include <linux/leds-aw2013.h>

/* register address */
#define AW_REG_RESET			0x00
#define AW_REG_GLOBAL_CONTROL		0x01
#define AW_REG_LED_STATUS		0x02
#define AW_REG_LED_ENABLE		0x30
#define AW_REG_LED_CONFIG_BASE		0x31
#define AW_REG_LED_BRIGHTNESS_BASE	0x34
#define AW_REG_TIMESET0_BASE		0x37
#define AW_REG_TIMESET1_BASE		0x38

/* register bits */
#define AW2013_CHIPID			0x33
#define AW_LED_MOUDLE_ENABLE_MASK	0x01
#define AW_LED_FADE_OFF_MASK		0x40
#define AW_LED_FADE_ON_MASK		0x20
#define AW_LED_BREATHE_MODE_MASK	0x10
#define AW_LED_RESET_MASK		0x55

#define AW_LED_RESET_DELAY		8
#define AW_LED_POWER_ON_DELAY	1
#define AW_LED_POWER_OFF_DELAY	10
#define AW2013_VDD_MIN_UV		2600000
#define AW2013_VDD_MAX_UV		3300000
#define AW2013_VI2C_MIN_UV		1800000
#define AW2013_VI2C_MAX_UV		1800000

#define MAX_RISE_TIME_MS		7
#define MAX_HOLD_TIME_MS		5
#define MAX_FALL_TIME_MS		7
#define MAX_OFF_TIME_MS			5



struct aw2013_platform_data {
	int max_current;
	int rise_time_ms;
	int hold_time_ms;
	int fall_time_ms;
	int off_time_ms;
	struct aw2013_led *led;
};


struct aw2013_led {
	struct i2c_client *client;//i2c设备信息结构体
	struct led_classdev cdev;
	struct aw2013_platform_data *pdata;
	struct work_struct brightness_work;
	struct mutex lock;
	struct regulator *vdd;
	struct regulator *vcc;
	int num_leds;
	int id;
	bool poweron;
};


/******************************************************************************
* 函数名：aw2013_write
* 功  能：通过i2c向reg写入val
* 输  入：led：led设备结构体	reg：寄存器		val：写入寄存器的值
* 返  回：无
*/
static int aw2013_write(struct aw2013_led *led, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(led->client, reg, val);
}


/******************************************************************************
* 函数名：aw2013_read
* 功  能：通过i2c向reg读出保存至val
* 输  入：led：led设备结构体	reg：寄存器		val：读出的寄存器的值
* 返  回：无
*/
static int aw2013_read(struct aw2013_led *led, u8 reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(led->client, reg);
	if (ret < 0)
		return ret;

	*val = ret;
	return 0;
}


/******************************************************************************
* 函数名：aw2013_power_on
* 功  能：使能电压
* 输  入：led：led设备结构体	开关on
* 返  回：无
*/
static int aw2013_power_on(struct aw2013_led *led, bool on)
{
	int rc;

	if (on) {
		rc = regulator_enable(led->vdd);
		if (rc) {
			dev_err(&led->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_enable(led->vcc);
		if (rc) {
			dev_err(&led->client->dev,
				"Regulator vcc enable failed rc=%d\n", rc);
			goto fail_enable_reg;
		}
		led->poweron = true;
		msleep(AW_LED_POWER_ON_DELAY);
	} else {
		rc = regulator_disable(led->vdd);
		if (rc) {
			dev_err(&led->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_disable(led->vcc);
		if (rc) {
			dev_err(&led->client->dev,
				"Regulator vcc disable failed rc=%d\n", rc);
			goto fail_disable_reg;
		}
		led->poweron = false;
		msleep(AW_LED_POWER_OFF_DELAY);
	}
	return rc;

fail_enable_reg:
	rc = regulator_disable(led->vdd);
	if (rc)
		dev_err(&led->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);

	return rc;

fail_disable_reg:
	rc = regulator_enable(led->vdd);
	if (rc)
		dev_err(&led->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);

	return rc;
}

/******************************************************************************
* 函数名：aw2013_power_init
* 功  能：电压初始化，设置电压范围
* 输  入：开关on
* 返  回：无
*/
static int aw2013_power_init(struct aw2013_led *led, bool on)
{
	int rc;

	if (on) {
		led->vdd = regulator_get(&led->client->dev, "vdd");//获取DTS关于电压的
		if (IS_ERR(led->vdd)) {
			rc = PTR_ERR(led->vdd);
			dev_err(&led->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(led->vdd) > 0) {
			rc = regulator_set_voltage(led->vdd, AW2013_VDD_MIN_UV,
						   AW2013_VDD_MAX_UV);
			if (rc) {
				dev_err(&led->client->dev,
					"Regulator set_vtg failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		led->vcc = regulator_get(&led->client->dev, "vcc");
		if (IS_ERR(led->vcc)) {
			rc = PTR_ERR(led->vcc);
			dev_err(&led->client->dev,
				"Regulator get failed vcc rc=%d\n", rc);
			goto reg_vdd_set_vtg;
		}

		if (regulator_count_voltages(led->vcc) > 0) {
			rc = regulator_set_voltage(led->vcc, AW2013_VI2C_MIN_UV,
						   AW2013_VI2C_MAX_UV);
			if (rc) {
				dev_err(&led->client->dev,
				"Regulator set_vtg failed vcc rc=%d\n", rc);
				goto reg_vcc_put;
			}
		}
	} else {
		if (regulator_count_voltages(led->vdd) > 0)
			regulator_set_voltage(led->vdd, 0, AW2013_VDD_MAX_UV);

		regulator_put(led->vdd);

		if (regulator_count_voltages(led->vcc) > 0)
			regulator_set_voltage(led->vcc, 0, AW2013_VI2C_MAX_UV);

		regulator_put(led->vcc);
	}
	return 0;

reg_vcc_put:
	regulator_put(led->vcc);
reg_vdd_set_vtg:
	if (regulator_count_voltages(led->vdd) > 0)
		regulator_set_voltage(led->vdd, 0, AW2013_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(led->vdd);
	return rc;
}


/******************************************************************************
* 函数名：aw2013_brightness_work
* 功  能：工作初始化
* 输  入： 
* 返  回：无
*/
static void aw2013_brightness_work(struct work_struct *work)
{
	struct aw2013_led *led = container_of(work, struct aw2013_led,
					brightness_work);
	u8 val;

	mutex_lock(&led->pdata->led->lock);

	/* enable regulators if they are disabled */
	if (!led->pdata->led->poweron) {
		if (aw2013_power_on(led->pdata->led, true)) {
			dev_err(&led->pdata->led->client->dev, "power on failed");
			mutex_unlock(&led->pdata->led->lock);
			return;
		}
	}

	if (led->cdev.brightness > 0) {
		if (led->cdev.brightness > led->cdev.max_brightness)
			led->cdev.brightness = led->cdev.max_brightness;
		aw2013_write(led, AW_REG_GLOBAL_CONTROL,//使能LED
			AW_LED_MOUDLE_ENABLE_MASK);
		aw2013_write(led, AW_REG_LED_CONFIG_BASE + led->id,//设置电流强度
			led->pdata->max_current);
		aw2013_write(led, AW_REG_LED_BRIGHTNESS_BASE + led->id,//设置PWM亮度 0~255，此时设置为255最亮
			led->cdev.brightness);
		aw2013_read(led, AW_REG_LED_ENABLE, &val);//读LED通道使能状态
		aw2013_write(led, AW_REG_LED_ENABLE, val | (1 << led->id));
	} else {
		aw2013_read(led, AW_REG_LED_ENABLE, &val);
		aw2013_write(led, AW_REG_LED_ENABLE, val & (~(1 << led->id)));//关闭该id的LED
	}

	aw2013_read(led, AW_REG_LED_ENABLE, &val);
/*
	 * 如果AW_REG_LED_ENABLE中的值为0，则表示RGB led灯全部
	 * 关掉。所以我们需要关掉电源。
	 */
	if (val == 0) {
		if (aw2013_power_on(led->pdata->led, false)) {
			dev_err(&led->pdata->led->client->dev,
				"power off failed");
			mutex_unlock(&led->pdata->led->lock);
			return;
		}
	}

	mutex_unlock(&led->pdata->led->lock);
}

/******************************************************************************
* 函数名：aw2013_led_blink_set
* 功  能：闪烁设置
* 输  入：dev : led class设备；	blinking：0-->关闭闪烁  1--->闪烁
* 返  回：无
*/
static void aw2013_led_blink_set(struct aw2013_led *led, unsigned long blinking)//闪烁
{
	u8 val;

	/* enable regulators if they are disabled */
	if (!led->pdata->led->poweron) {
		if (aw2013_power_on(led->pdata->led, true)) {
			dev_err(&led->pdata->led->client->dev, "power on failed");
			return;
		}
	}

	//
	led->cdev.brightness = blinking ? led->cdev.max_brightness : 0;//blinking等于0则=0，为真时=led->cdev.max_brightness 
	if (blinking > 0) {
		aw2013_write(led, AW_REG_GLOBAL_CONTROL,
			AW_LED_MOUDLE_ENABLE_MASK);//使能LED function
		aw2013_write(led, AW_REG_LED_CONFIG_BASE + led->id,
			AW_LED_FADE_OFF_MASK | AW_LED_FADE_ON_MASK |
			AW_LED_BREATHE_MODE_MASK | led->pdata->max_current);//选择编程模式
		aw2013_write(led, AW_REG_LED_BRIGHTNESS_BASE + led->id,//PWM强度设置
			led->cdev.brightness);
		aw2013_write(led, AW_REG_TIMESET0_BASE + led->id * 3,//设置时间
			led->pdata->rise_time_ms << 4 |
			led->pdata->hold_time_ms);
		aw2013_write(led, AW_REG_TIMESET1_BASE + led->id * 3,//设置时间
			led->pdata->fall_time_ms << 4 |
			led->pdata->off_time_ms);
		aw2013_read(led, AW_REG_LED_ENABLE, &val);
		aw2013_write(led, AW_REG_LED_ENABLE, val | (1 << led->id));
	} else {
		aw2013_read(led, AW_REG_LED_ENABLE, &val);
		aw2013_write(led, AW_REG_LED_ENABLE, val & (~(1 << led->id)));
	}

	aw2013_read(led, AW_REG_LED_ENABLE, &val);
	/*
	 * If value in AW_REG_LED_ENABLE is 0, it means the RGB leds are
	 * all off. So we need to power it off.
	 */
	if (val == 0) {
		if (aw2013_power_on(led->pdata->led, false)) {
			dev_err(&led->pdata->led->client->dev,
				"power off failed");
			return;
		}
	}
}

/******************************************************************************
* 函数名：aw2013_set_brightness
* 功  能：设置亮度
* 输  入：dev : led class设备；	brightness：亮度等级（0~255）
* 返  回：无
*/
static void aw2013_set_brightness(struct led_classdev *cdev,
			     enum led_brightness brightness)
{
	struct aw2013_led *led = container_of(cdev, struct aw2013_led, cdev);//使用container_of函数，已知成员led_classdev，求出原始结构体的原始地址ptr。

	led->cdev.brightness = brightness;//赋值

	schedule_work(&led->brightness_work);//调度工作
}

/******************************************************************************
* 函数名：aw2013_store_blink
* 功  能：用户层可通过echo 访问该接口	
* 输  入：dev : led class设备；	buf : 写入的内容blinking  0为关闭 >1为打开闪烁  len : 内容长度
* 返  回：长度
*/
static ssize_t aw2013_store_blink(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	unsigned long blinking;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);//获取led的设备
	struct aw2013_led *led =
			container_of(led_cdev, struct aw2013_led, cdev);//使用container_of函数，已知成员led_cdev，求出原始结构体的原始地址ptr。
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &blinking);//将参数buf字符串根据参数base来转换成无符号的长整型数保存至blinking。
	if (ret)
		return ret;
	mutex_lock(&led->pdata->led->lock);
	aw2013_led_blink_set(led, blinking);
	mutex_unlock(&led->pdata->led->lock);

	return len;
}


/******************************************************************************
* 函数名：aw2013_led_time_show
* 功  能：用户层可通过cat 访问该接口	
* 输  入：dev : led class设备； buf : 读入的内容msg 
* 返  回：长度
*/
static ssize_t aw2013_led_time_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);//获取led的设备
	struct aw2013_led *led =
			container_of(led_cdev, struct aw2013_led, cdev);//使用container_of函数，已知成员led_cdev，求出原始结构体的原始地址ptr。

	return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n",
			led->pdata->rise_time_ms, led->pdata->hold_time_ms,
			led->pdata->fall_time_ms, led->pdata->off_time_ms);
}


/******************************************************************************
* 函数名：aw2013_led_time_store
* 功  能：用户层可通过echo+buf 访问该接口	echo 5 4 5 4
* 输  入：dev : led class设备； buf : 写入的内容msg  len : 内容长度
* 输  出：len
* 返  回：int
*/
static ssize_t aw2013_led_time_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);//获取led的设备
	struct aw2013_led *led =
			container_of(led_cdev, struct aw2013_led, cdev);//使用container_of函数，已知成员led_cdev，求出原始结构体的原始地址ptr。
	int rc, rise_time_ms, hold_time_ms, fall_time_ms, off_time_ms;//上升 保持 下降 关闭的时间  毫秒

	rc = sscanf(buf, "%d %d %d %d",
			&rise_time_ms, &hold_time_ms,
			&fall_time_ms, &off_time_ms);//将buf字符串按规定格式输出  分别保存至4个参数  及4组时间

	mutex_lock(&led->pdata->led->lock);
	led->pdata->rise_time_ms = (rise_time_ms > MAX_RISE_TIME_MS) ?//MAX_RISE_TIME_MS=7
				MAX_RISE_TIME_MS : rise_time_ms;
	led->pdata->hold_time_ms = (hold_time_ms > MAX_HOLD_TIME_MS) ?//MAX_HOLD_TIME_MS=5
				MAX_HOLD_TIME_MS : hold_time_ms;
	led->pdata->fall_time_ms = (fall_time_ms > MAX_FALL_TIME_MS) ?//MAX_FALL_TIME_MS=7
				MAX_FALL_TIME_MS : fall_time_ms;
	led->pdata->off_time_ms = (off_time_ms > MAX_OFF_TIME_MS) ?//MAX_OFF_TIME_MS=5
				MAX_OFF_TIME_MS : off_time_ms;
	aw2013_led_blink_set(led, 1);
	mutex_unlock(&led->pdata->led->lock);
	return len;
}

//权限以及echo--> store  cat-->show
static DEVICE_ATTR(blink, 0664, NULL, aw2013_store_blink);
static DEVICE_ATTR(led_time, 0664, aw2013_led_time_show, aw2013_led_time_store);

//创个目录名为:(1)blink		(2)led_time
static struct attribute *aw2013_led_attributes[] = {
	&dev_attr_blink.attr,
	&dev_attr_led_time.attr,
	NULL,
};
//sysfs group
static struct attribute_group aw2013_led_attr_group = {
	.attrs = aw2013_led_attributes
};


/******************************************************************************
* 函数名：aw2013_check_chipid
* 功  能：向00h寄存器发送55h,IC重置。读00h返回33h
*/
static int aw2013_check_chipid(struct aw2013_led *led)
{
	u8 val;

	aw2013_write(led, AW_REG_RESET, AW_LED_RESET_MASK);//00h --> 55h   重置所有电路
	udelay(AW_LED_RESET_DELAY);
	aw2013_read(led, AW_REG_RESET, &val);
	if (val == AW2013_CHIPID)//返回33h则重置成功
		return 0;
	else
		return -EINVAL;
}

/******************************************************************************
* 函数名：aw2013_led_err_handle
* 功  能：移除sysfs接口
*/
static int aw2013_led_err_handle(struct aw2013_led *led_array,
				int parsed_leds)
{
	int i;
	/*
	 * If probe fails, cannot free resource of all LEDs, only free
	 * resources of LEDs which have allocated these resource really.
	 */
	for (i = 0; i < parsed_leds; i++) {
		sysfs_remove_group(&led_array[i].cdev.dev->kobj,
				&aw2013_led_attr_group);
		led_classdev_unregister(&led_array[i].cdev);
		cancel_work_sync(&led_array[i].brightness_work);
		devm_kfree(&led_array->client->dev, led_array[i].pdata);
		led_array[i].pdata = NULL;
	}
	return i;
}
/******************************************************************************
* 函数名：aw2013_led_parse_child_node
* 功  能：解析dts节点
* 输  入：led_array : led结构体； node : dts节点
* 输  出：无
* 返  回：int
*/
static int aw2013_led_parse_child_node(struct aw2013_led *led_array,
				struct device_node *node)
{
	struct aw2013_led *led;
	struct device_node *temp;
	struct aw2013_platform_data *pdata;//存放led数据
	int rc = 0, parsed_leds = 0;

	for_each_child_of_node(node, temp) {//遍历节点
		led = &led_array[parsed_leds];//从0下标开始存led信息
		led->client = led_array->client;

		pdata = devm_kzalloc(&led->client->dev,
				sizeof(struct aw2013_platform_data),
				GFP_KERNEL);//申请内核内存
		if (!pdata) {
			dev_err(&led->client->dev,
				"Failed to allocate memory\n");
			goto free_err;
		}
		pdata->led = led_array;
		led->pdata = pdata;

		rc = of_property_read_string(temp, "aw2013,name",//读aw2013,name节点
			&led->cdev.name);//存到led->cdev.name
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading led name, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw2013,id",//读aw2013,id节点
			&led->id);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading id, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw2013,max-brightness",//读aw2013,max-brightness节点
			&led->cdev.max_brightness);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading max-brightness, rc = %d\n",
				rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw2013,max-current",
			&led->pdata->max_current);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading max-current, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw2013,rise-time-ms",
			&led->pdata->rise_time_ms);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading rise-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw2013,hold-time-ms",
			&led->pdata->hold_time_ms);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading hold-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw2013,fall-time-ms",
			&led->pdata->fall_time_ms);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading fall-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw2013,off-time-ms",
			&led->pdata->off_time_ms);
		if (rc < 0) {
			dev_err(&led->client->dev,
				"Failure reading off-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		INIT_WORK(&led->brightness_work, aw2013_brightness_work);//工作队列 将led->brightness_work参数传入aw2013_brightness_work。

		led->cdev.brightness_set = aw2013_set_brightness;

		rc = led_classdev_register(&led->client->dev, &led->cdev);//注册led设备在/sys/bus/led下
		if (rc) {
			dev_err(&led->client->dev,
				"unable to register led %d,rc=%d\n",
				led->id, rc);
			goto free_pdata;
		}
		//在调试驱动，可能需要对驱动里的某些变量进行读写，或函数调用。
		//可通过sysfs接口创建驱动对应的属性，
		//使得可以在用户空间通过sysfs接口的show和store函数与硬件交互
		rc = sysfs_create_group(&led->cdev.dev->kobj,
				&aw2013_led_attr_group);
		if (rc) {
			dev_err(&led->client->dev, "led sysfs rc: %d\n", rc);
			goto free_class;
		}
		parsed_leds++;
	}

	return 0;

free_class:
	aw2013_led_err_handle(led_array, parsed_leds);
	led_classdev_unregister(&led_array[parsed_leds].cdev);
	cancel_work_sync(&led_array[parsed_leds].brightness_work);
	devm_kfree(&led->client->dev, led_array[parsed_leds].pdata);
	led_array[parsed_leds].pdata = NULL;
	return rc;

free_pdata:
	aw2013_led_err_handle(led_array, parsed_leds);
	devm_kfree(&led->client->dev, led_array[parsed_leds].pdata);
	return rc;

free_err:
	aw2013_led_err_handle(led_array, parsed_leds);
	return rc;
}

/******************************************************************************
* 函数名：aw2013_led_probe
* 功  能：解析dts节点
* 输  入：led_array : led结构体； node : dts节点
* 输  出：无
* 返  回：无
*/
static int aw2013_led_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct aw2013_led *led_array;//led结构体
	struct device_node *node;//设备树DTS转换成设备的载体
	int ret, num_leds = 0;
	node = client->dev.of_node;//获取dts中的信息节点
	if (node == NULL)
		return -EINVAL;

	num_leds = of_get_child_count(node);//获取aw2013dts中的节点数

	if (!num_leds)
		return -EINVAL;

	//申请内核内存
	led_array = devm_kzalloc(&client->dev,
			(sizeof(struct aw2013_led) * num_leds), GFP_KERNEL);//获取num_leds数量的数组结构体
	if (!led_array)
		return -ENOMEM;

	led_array->client = client;//保存i2c设备
	led_array->num_leds = num_leds;//保存节点数（红、绿、蓝）

	mutex_init(&led_array->lock);//初始化锁

	ret = aw2013_led_parse_child_node(led_array, node);//解析DTS 创建sysfs
	if (ret) {
		dev_err(&client->dev, "parsed node error\n");
		goto free_led_arry;
	}
	//自定义的设备结构dev赋给设备驱动client的私有指针
	i2c_set_clientdata(client, led_array);
	
	//处理DTS 电压参数
    ret = aw2013_power_init(led_array, true);
	if (ret) {
		dev_err(&client->dev, "power init failed");
		goto fail_parsed_node;
	}
 	if(!led_array->poweron)
    {
            ret = aw2013_power_on(led_array->pdata->led, true);
            if(ret) {
                    dev_err(&client->dev, "AW2013 Probe power on fail\n");
		            goto fail_parsed_node;
            }
    }
	//所有电路将重置
    ret = aw2013_check_chipid(led_array);
	if (ret) {
		dev_err(&client->dev, "Check chip id error\n");
		goto fail_parsed_node;
	}

	return 0;

fail_parsed_node:
	aw2013_led_err_handle(led_array, num_leds);
free_led_arry:
	mutex_destroy(&led_array->lock);
	devm_kfree(&client->dev, led_array);
	led_array = NULL;
	return ret;
}

/******************************************************************************
* 函数名：aw2013_led_remove
* 功  能：设备移除接口
*/
static int aw2013_led_remove(struct i2c_client *client)
{
	struct aw2013_led *led_array = i2c_get_clientdata(client);
	int i, parsed_leds = led_array->num_leds;

	for (i = 0; i < parsed_leds; i++) {
		sysfs_remove_group(&led_array[i].cdev.dev->kobj,
				&aw2013_led_attr_group);
		led_classdev_unregister(&led_array[i].cdev);
		cancel_work_sync(&led_array[i].brightness_work);
		devm_kfree(&client->dev, led_array[i].pdata);
		led_array[i].pdata = NULL;
	}
	mutex_destroy(&led_array->lock);
	devm_kfree(&client->dev, led_array);
	led_array = NULL;
	return 0;
}

static const struct i2c_device_id aw2013_led_id[] = {
	{"aw2013_led", 0}, //匹配i2c client名为aw2013_led的设备
	{},
};

MODULE_DEVICE_TABLE(i2c, aw2013_led_id);

static struct of_device_id aw2013_match_table[] = {
	{ .compatible = "awinic,aw2013",},// //匹配i2c client名为aw2013的设备
	{ },
};

static struct i2c_driver aw2013_led_driver = {//i2c驱动
	.probe = aw2013_led_probe,//组装设备匹配时候的匹配动作
	.remove = aw2013_led_remove,//组装设备移除接口
	.driver = {
		.name = "aw2013_led",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw2013_match_table),//与dts匹配的设备信息，优先级1
	},
	.id_table = aw2013_led_id,//要匹配的从设备列表	优先级2
};

static int __init aw2013_led_init(void)
{
	return i2c_add_driver(&aw2013_led_driver);//因为是i2c驱动  调用通用的i2c_add_driver注册i2c驱动
}
module_init(aw2013_led_init);//首先会进入驱动init函数

static void __exit aw2013_led_exit(void)
{
	i2c_del_driver(&aw2013_led_driver);
}
module_exit(aw2013_led_exit);

MODULE_DESCRIPTION("AWINIC aw2013 LED driver");
MODULE_LICENSE("GPL v2");
