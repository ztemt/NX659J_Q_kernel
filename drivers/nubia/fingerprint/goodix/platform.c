/*
 * platform indepent driver interface
 * Copyright (C) 2016 Goodix
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

int gf_pinctrl_init(struct gf_dev* gf_dev)
{
	int ret = 0;
	struct device *dev = &gf_dev->spi->dev;

	gf_dev->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(gf_dev->pinctrl)) {
		FP_LOG_ERROR("Target does not use pinctrl\n");
		ret = PTR_ERR(gf_dev->pinctrl);
		goto err;
	}

	gf_dev->gpio_state_active = pinctrl_lookup_state(gf_dev->pinctrl, "gf_fp_active");
	if (IS_ERR_OR_NULL(gf_dev->gpio_state_active)) {
		FP_LOG_ERROR("Cannot get active pinstate\n");
		ret = PTR_ERR(gf_dev->gpio_state_active);
		goto err;
	}

	gf_dev->gpio_state_suspend = pinctrl_lookup_state(gf_dev->pinctrl, "gf_fp_suspend");
	if (IS_ERR_OR_NULL(gf_dev->gpio_state_suspend)) {
		FP_LOG_ERROR("Cannot get sleep pinstate\n");
		ret = PTR_ERR(gf_dev->gpio_state_suspend);
		goto err;
	}
	FP_LOG_INFO("success\n");
	return 0;
err:
	gf_dev->pinctrl = NULL;
	gf_dev->gpio_state_active = NULL;
	gf_dev->gpio_state_suspend = NULL;
	return ret;
}

int gf_pinctrl_select(struct gf_dev* gf_dev, bool on)
{
	int ret = 0;
	struct pinctrl_state *pins_state;

	pins_state = on ? gf_dev->gpio_state_active : gf_dev->gpio_state_suspend;
	if (IS_ERR_OR_NULL(pins_state)) {
		FP_LOG_ERROR("not a valid '%s' pinstate\n",
			on ? "gf_fp_active" : "gf_fp_suspend");
		return -1;
	}

	ret = pinctrl_select_state(gf_dev->pinctrl, pins_state);
	if (ret) {
		FP_LOG_ERROR("can not set %s pins\n",
			on ? "gf_fp_active" : "gf_fp_suspend");
	}
	FP_LOG_INFO("success\n");
	return ret;
}

int gf_parse_dts(struct gf_dev *gf_dev)
{
	int rc = 0;
	struct device *dev = &gf_dev->spi->dev;
	struct device_node *np = dev->of_node;

	/************avdd*************/
	gf_dev->pwr_avdd_gpio = of_get_named_gpio(np, "goodix,goodix_pwr_avdd", 0);
	if (gf_dev->pwr_avdd_gpio < 0) {
		FP_LOG_ERROR("falied to get goodix_pwr_avdd gpio!\n");
		return gf_dev->pwr_avdd_gpio;
	}
	FP_LOG_INFO("goodix_pwr_avdd gpio:%d\n", gf_dev->pwr_avdd_gpio);
	gpio_free(gf_dev->pwr_avdd_gpio);
	rc = devm_gpio_request(dev, gf_dev->pwr_avdd_gpio, "goodix_pwr_avdd");
	if (rc) {
		FP_LOG_ERROR("failed to request goodix_pwr_avdd gpio, rc = %d\n", rc);
		goto err_avdd;
	}
	gpio_direction_output(gf_dev->pwr_avdd_gpio, 1);

	/************vddio*************/
	gf_dev->pwr_vddio_gpio = of_get_named_gpio(np, "goodix,goodix_pwr_vddio", 0);
	if (gf_dev->pwr_vddio_gpio < 0) {
		FP_LOG_ERROR("falied to get goodix_pwr_vddio gpio!\n");
		return gf_dev->pwr_vddio_gpio;
	}
	FP_LOG_INFO("goodix_pwr_vddio gpio:%d\n", gf_dev->pwr_vddio_gpio);
	gpio_free(gf_dev->pwr_vddio_gpio);
	rc = devm_gpio_request(dev, gf_dev->pwr_vddio_gpio, "goodix_pwr_vddio");
	if (rc) {
		FP_LOG_ERROR("failed to request goodix_pwr_vddio gpio, rc = %d\n", rc);
		goto err_vddio;
	}
	gpio_direction_output(gf_dev->pwr_vddio_gpio, 1);

	/************reset*************/
	gf_dev->reset_gpio = of_get_named_gpio(np, "goodix,goodix_reset", 0);
	if (gf_dev->reset_gpio < 0) {
		FP_LOG_ERROR("falied to get reset gpio!\n");
		return gf_dev->reset_gpio;
	}
	FP_LOG_INFO("reset gpio:%d\n", gf_dev->reset_gpio);
	gpio_free(gf_dev->reset_gpio);
	rc = devm_gpio_request(dev, gf_dev->reset_gpio, "goodix_reset");
	if (rc) {
		FP_LOG_ERROR("failed to request reset gpio, rc = %d\n", rc);
		goto err_reset;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);

	/************init set*************/
	if (gpio_is_valid(gf_dev->pwr_avdd_gpio)) {
		gpio_set_value(gf_dev->pwr_avdd_gpio, 1);
	}
	if (gpio_is_valid(gf_dev->pwr_vddio_gpio)) {
		gpio_set_value(gf_dev->pwr_vddio_gpio, 1);
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_set_value(gf_dev->reset_gpio, 1);
	}
	FP_LOG_INFO("avdd power %s\n", gpio_get_value(gf_dev->pwr_avdd_gpio)? "on" : "off");
	FP_LOG_INFO("vddio power %s\n", gpio_get_value(gf_dev->pwr_vddio_gpio)? "on" : "off");

	/************irq*************/
	gf_dev->irq_gpio = of_get_named_gpio(np, "goodix,goodix_irq", 0);
	if (gf_dev->irq_gpio < 0) {
		FP_LOG_ERROR("falied to get irq gpio!\n");
		return gf_dev->irq_gpio;
	}
	FP_LOG_INFO("irq_gpio:%d\n", gf_dev->irq_gpio);
	gpio_free(gf_dev->irq_gpio);
	rc = devm_gpio_request(dev, gf_dev->irq_gpio, "goodix_irq");
	if (rc) {
		FP_LOG_ERROR("failed to request irq gpio, rc = %d\n", rc);
		goto err_irq;
	}
	gpio_direction_input(gf_dev->irq_gpio);

	/************id*************/
	gf_dev->id_gpio = of_get_named_gpio(np, "goodix,goodix_id", 0);
	if (gf_dev->id_gpio < 0) {
		FP_LOG_ERROR("falied to get id gpio!\n");
		return gf_dev->id_gpio;
	}
	FP_LOG_INFO("id_gpio:%d\n", gf_dev->id_gpio);
	gpio_free(gf_dev->id_gpio);
	rc = devm_gpio_request(dev, gf_dev->id_gpio, "goodix_id");
	if (rc) {
		FP_LOG_ERROR("failed to request id gpio, rc = %d\n", rc);
		goto err_id;
	}
	gpio_direction_input(gf_dev->id_gpio);

	FP_LOG_INFO("parse success\n");

err_id:
	devm_gpio_free(dev, gf_dev->irq_gpio);
err_irq:
	devm_gpio_free(dev, gf_dev->reset_gpio);
err_reset:
	devm_gpio_free(dev, gf_dev->pwr_vddio_gpio);
err_vddio:
	devm_gpio_free(dev, gf_dev->pwr_avdd_gpio);
err_avdd:
	return rc;
}

void gf_cleanup(struct gf_dev *gf_dev)
{
	FP_LOG_INFO("[info] %s\n", __func__);

	if (gpio_is_valid(gf_dev->pwr_avdd_gpio))
	{
		gpio_free(gf_dev->pwr_avdd_gpio);
		FP_LOG_INFO("remove pwr_avdd_gpio success\n");
	}

	if (gpio_is_valid(gf_dev->pwr_vddio_gpio))
	{
		gpio_free(gf_dev->pwr_vddio_gpio);
		FP_LOG_INFO("remove pwr_vddio_gpio success\n");
	}

	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		FP_LOG_INFO("remove irq_gpio success\n");
	}

	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		FP_LOG_INFO("remove reset_gpio success\n");
	}
}

int gf_power_on(struct gf_dev *gf_dev)
{
	int rc = 0;

	if(!gf_dev) {
		FP_LOG_ERROR("gf_dev null\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_set_value(gf_dev->reset_gpio, 1);
	}
	if (gpio_is_valid(gf_dev->pwr_avdd_gpio)) {
		gpio_set_value(gf_dev->pwr_avdd_gpio, 1);
	}
	if (gpio_is_valid(gf_dev->pwr_vddio_gpio)) {
		gpio_set_value(gf_dev->pwr_vddio_gpio, 1);
	}
	msleep(10);
	FP_LOG_INFO("power on\n");

	return rc;
}

int gf_power_off(struct gf_dev *gf_dev)
{
	int rc = 0;

	if(!gf_dev) {
		FP_LOG_ERROR("gf_dev null\n");
	}

	if (gpio_is_valid(gf_dev->pwr_avdd_gpio)) {
		gpio_set_value(gf_dev->pwr_avdd_gpio, 0);
	}
	if (gpio_is_valid(gf_dev->pwr_vddio_gpio)) {
		gpio_set_value(gf_dev->pwr_vddio_gpio, 0);
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_set_value(gf_dev->reset_gpio, 0);
	}
	gf_cleanup(gf_dev); //free gpio resource
	FP_LOG_INFO("power off\n");

	return rc;
}

int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if (!gf_dev) {
		FP_LOG_ERROR("Input buff gf_dev is NULL.\n");
		return -ENODEV;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	mdelay(5);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms*10);
	return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
	if (!gf_dev) {
		FP_LOG_ERROR("Input buff gf_dev is NULL.\n");
		return -ENODEV;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}

