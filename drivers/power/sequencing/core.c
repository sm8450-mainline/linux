// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024 Linaro Ltd.
 */

#include <linux/bug.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/idr.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/pwrseq/consumer.h>
#include <linux/pwrseq/provider.h>
#include <linux/rwsem.h>

static DEFINE_IDA(pwrseq_ida);
/*
 * Protects the device list on the bus from concurrent modifications but allows
 * simultaneous read-only access.
 */
static DECLARE_RWSEM(pwrseq_sem);

/**
 * struct pwrseq_device - Private power sequencing data.
 * @dev: Device struct associated with this sequencer.
 * @id: Device ID.
 * @owner: Prevents removal of active power sequencing providers.
 * @pwrup_count: Keeps track of power state change requests.
 * @sem: Protects the device against being unregistered while in use.
 * @drvdata: Provider driver private data.
 * @match: Power sequencer matching callback.
 * @power_on: Power-on callback.
 * @power_off: Power-off callback.
 */
struct pwrseq_device {
	struct device dev;
	int id;
	struct module *owner;
	unsigned int pwrup_count;
	struct rw_semaphore dev_sem;
	struct mutex state_lock;
	void *drvdata;
	pwrseq_match_func match;
	pwrseq_power_state_func power_on;
	pwrseq_power_state_func power_off;
};

/**
 * struct pwrseq_desc - Wraps access to the pwrseq_device and ensures that one
 *                      user cannot break the reference counting for others.
 * @pwrseq: Reference to the power sequencing device.
 * @powered_up: Power state set by the holder of the descriptor (not necessarily
 * corresponding to the actual power state of the device).
 */
struct pwrseq_desc {
	struct pwrseq_device *pwrseq;
	bool powered_up;
};

static struct pwrseq_device *to_pwrseq_device(struct device *dev)
{
	return container_of(dev, struct pwrseq_device, dev);
}

static struct pwrseq_device *pwrseq_device_get(struct pwrseq_device *pwrseq)
{
	get_device(&pwrseq->dev);

	return pwrseq;
}

static void pwrseq_device_put(struct pwrseq_device *pwrseq)
{
	put_device(&pwrseq->dev);
}

static struct bus_type pwrseq_bus = {
	.name = "pwrseq",
};

static void pwrseq_release(struct device *dev)
{
	struct pwrseq_device *pwrseq = to_pwrseq_device(dev);

	mutex_destroy(&pwrseq->state_lock);
	ida_free(&pwrseq_ida, pwrseq->id);
	kfree(pwrseq);
}

static const struct device_type pwrseq_device_type = {
	.name = "power_sequencer",
	.release = pwrseq_release,
};

/**
 * pwrseq_device_register() - Register a new power sequencer.
 * @config: Configuration of the new power sequencing device.
 *
 * The config structure is only used during the call and can be freed after
 * the function returns. The config structure *must* have the parent device
 * as well as the match(), power_on() and power_off() callbacks registered.
 *
 * Returns:
 * Returns the address of the new pwrseq device or ERR_PTR() on failure.
 */
struct pwrseq_device *pwrseq_device_register(struct pwrseq_config *config)
{
	struct pwrseq_device *pwrseq;
	int ret;

	/*
	 * Power sequencer must have a parent device and at least the power-on,
	 * power-off and match callbacks.
	 */
	if (!config->parent || !config->match || !config->power_on ||
	    !config->power_off)
		return ERR_PTR(-EINVAL);

	pwrseq = kzalloc(sizeof(*pwrseq), GFP_KERNEL);
	if (!pwrseq)
		return ERR_PTR(-ENOMEM);

	pwrseq->dev.type = &pwrseq_device_type;
	pwrseq->dev.bus = &pwrseq_bus;
	pwrseq->dev.parent = config->parent;
	device_set_node(&pwrseq->dev, dev_fwnode(config->parent));

	pwrseq->id = ida_alloc(&pwrseq_ida, GFP_KERNEL);
	if (pwrseq->id < 0) {
		kfree(pwrseq);
		return ERR_PTR(pwrseq->id);
	}

	/*
	 * From this point onwards the device's release() callback is
	 * responsible for freeing resources.
	 */
	device_initialize(&pwrseq->dev);

	ret = dev_set_name(&pwrseq->dev, "pwrseq.%d", pwrseq->id);
	if (ret)
		goto err_put_pwrseq;

	pwrseq->owner = config->owner ?: THIS_MODULE;
	pwrseq->drvdata = config->drvdata;
	pwrseq->match = config->match;
	pwrseq->power_on = config->power_on;
	pwrseq->power_off = config->power_off;

	init_rwsem(&pwrseq->dev_sem);
	mutex_init(&pwrseq->state_lock);

	scoped_guard(rwsem_write, &pwrseq_sem) {
		ret = device_add(&pwrseq->dev);
		if (ret)
			goto err_put_pwrseq;
	}

	return pwrseq;

err_put_pwrseq:
	pwrseq_device_put(pwrseq);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(pwrseq_device_register);

/**
 * pwrseq_device_unregister() - Unregister the power sequencer.
 * @pwrseq: Power sequencer to unregister.
 */
void pwrseq_device_unregister(struct pwrseq_device *pwrseq)
{
	struct device *dev = &pwrseq->dev;

	scoped_guard(mutex, &pwrseq->state_lock) {
		WARN_ONCE(pwrseq->pwrup_count > 0,
			  "%s: UNREGISTERING POWER SEQUENCER WITH ACTIVE USERS\n",
			  dev_name(&pwrseq->dev));

		scoped_guard(rwsem_write, &pwrseq_sem) {
			scoped_guard(rwsem_write, &pwrseq->dev_sem)
				device_del(dev);
		}
	}

	pwrseq_device_put(pwrseq);
}
EXPORT_SYMBOL_GPL(pwrseq_device_unregister);

static void devm_pwrseq_device_unregister(void *data)
{
	struct pwrseq_device *pwrseq = data;

	pwrseq_device_unregister(pwrseq);
}

/**
 * devm_pwrseq_device_register() - Managed variant of pwrseq_device_register().
 * @dev: Managing device.
 * @config: Configuration of the new power sequencing device.
 *
 * Returns:
 * Returns the address of the new pwrseq device or ERR_PTR() on failure.
 */
struct pwrseq_device *
devm_pwrseq_device_register(struct device *dev, struct pwrseq_config *config)
{
	struct pwrseq_device *pwrseq;
	int ret;

	pwrseq = pwrseq_device_register(config);
	if (IS_ERR(pwrseq))
		return pwrseq;

	ret = devm_add_action_or_reset(dev, devm_pwrseq_device_unregister,
				       pwrseq);
	if (ret)
		return ERR_PTR(ret);

	return pwrseq;
}
EXPORT_SYMBOL_GPL(devm_pwrseq_device_register);

/**
 * pwrseq_device_get_data() - Get the driver private data associated with this
 *                            sequencer.
 * @pwrseq: Power sequencer object.
 *
 * Returns:
 * Address of the private driver data.
 */
void *pwrseq_device_get_data(struct pwrseq_device *pwrseq)
{
	return pwrseq->drvdata;
}
EXPORT_SYMBOL_GPL(pwrseq_device_get_data);

struct pwrseq_match_data {
	struct pwrseq_device *matched;
	struct device *dev;
};

static int pwrseq_match_device(struct device *pwrseq_dev, void *data)
{
	struct pwrseq_device *pwrseq = to_pwrseq_device(pwrseq_dev);
	struct pwrseq_match_data *match_data = data;
	int ret;

	guard(rwsem_read)(&pwrseq->dev_sem);
	if (!device_is_registered(&pwrseq->dev))
		return 0;

	ret = pwrseq->match(pwrseq, match_data->dev);
	if (ret <= 0)
		return ret;

	match_data->matched = pwrseq;

	return 1;
}

/**
 * pwrseq_get() - Get the power sequencer associated with this device.
 * @dev: Device for which to get the sequencer.
 *
 * Returns:
 * New power sequencer descriptor for use by the consumer driver or ERR_PTR()
 * on failure.
 */
struct pwrseq_desc *pwrseq_get(struct device *dev)
{
	struct pwrseq_match_data match_data;
	struct pwrseq_device *pwrseq;
	int ret;

	struct pwrseq_desc *desc __free(kfree) = kzalloc(sizeof(*desc),
							 GFP_KERNEL);
	if (!desc)
		return ERR_PTR(-ENOMEM);

	match_data.matched = NULL;
	match_data.dev = dev;

	guard(rwsem_read)(&pwrseq_sem);

	ret = bus_for_each_dev(&pwrseq_bus, NULL, &match_data,
			       pwrseq_match_device);
	if (ret < 0)
		return ERR_PTR(ret);
	if (ret == 0)
		/* No device matched. */
		return ERR_PTR(-EPROBE_DEFER);

	pwrseq = match_data.matched;

	if (!try_module_get(pwrseq->owner))
		return ERR_PTR(-EPROBE_DEFER);

	desc->pwrseq = pwrseq_device_get(pwrseq);

	return no_free_ptr(desc);
}
EXPORT_SYMBOL_GPL(pwrseq_get);

/**
 * pwrseq_put() - Release the power sequencer descriptor.
 * @desc: Descriptor to release.
 */
void pwrseq_put(struct pwrseq_desc *desc)
{
	struct pwrseq_device *pwrseq;

	if (!desc)
		return;

	pwrseq = desc->pwrseq;

	if (desc->powered_up)
		pwrseq_power_off(desc);

	kfree(desc);
	module_put(pwrseq->owner);
	pwrseq_device_put(pwrseq);
}
EXPORT_SYMBOL_GPL(pwrseq_put);

static void devm_pwrseq_put(void *data)
{
	struct pwrseq_desc *desc = data;

	pwrseq_put(desc);
}

/**
 * devm_pwrseq_get() - Managed variant of pwrseq_get().
 * @dev: Device for which to get the sequencer and which also manages its
 *       lifetime.
 *
 * Returns:
 * New power sequencer descriptor for use by the consumer driver or ERR_PTR()
 * on failure.
 */
struct pwrseq_desc *devm_pwrseq_get(struct device *dev)
{
	struct pwrseq_desc *desc;
	int ret;

	desc = pwrseq_get(dev);
	if (IS_ERR(desc))
		return desc;

	ret = devm_add_action_or_reset(dev, devm_pwrseq_put, desc);
	if (ret)
		return ERR_PTR(ret);

	return desc;
}
EXPORT_SYMBOL_GPL(devm_pwrseq_get);

/**
 * pwrseq_power_on() - Issue a power-on request on behalf of the consumer
 *                     device.
 * @desc: Descriptor referencing the power sequencer.
 *
 * This function tells the power sequencer that the consumer wants to be
 * powered-up. The sequencer may already have powered-up the device in which
 * case the function returns 0. If the power-up sequence is already in
 * progress, the function will block until it's done and return 0. If this is
 * the first request, the device will be powered up.
 *
 * Returns:
 * 0 on success, negative error number on failure.
 */
int pwrseq_power_on(struct pwrseq_desc *desc)
{
	struct pwrseq_device *pwrseq;
	int ret;

	might_sleep();

	if (!desc || desc->powered_up)
		return 0;

	pwrseq = desc->pwrseq;

	guard(rwsem_read)(&pwrseq->dev_sem);
	if (!device_is_registered(&pwrseq->dev))
		return -ENODEV;

	guard(mutex)(&pwrseq->state_lock);

	pwrseq->pwrup_count++;
	if (pwrseq->pwrup_count != 1) {
		desc->powered_up = true;
		return 0;
	}

	ret = pwrseq->power_on(pwrseq);
	if (!ret)
		desc->powered_up = true;

	return ret;
}
EXPORT_SYMBOL_GPL(pwrseq_power_on);

/**
 * pwrseq_power_off() - Issue a power-off request on behalf of the consumer
 *                      device.
 * @desc: Descriptor referencing the power sequencer.
 *
 * This undoes the effects of pwrseq_power_on(). It issues a power-off request
 * on behalf of the consumer and when the last remaining user does so, the
 * power-down sequence will be started. If one is in progress, the function
 * will block until it's complete and then return.
 *
 * Returns:
 * 0 on success, negative error number on failure.
 */
int pwrseq_power_off(struct pwrseq_desc *desc)
{
	struct pwrseq_device *pwrseq;
	int ret;

	might_sleep();

	if (!desc || !desc->powered_up)
		return 0;

	pwrseq = desc->pwrseq;

	guard(rwsem_read)(&pwrseq->dev_sem);
	if (!device_is_registered(&pwrseq->dev))
		return -ENODEV;

	guard(mutex)(&pwrseq->state_lock);

	if (pwrseq->pwrup_count == 0) {
		WARN_ONCE(1, "Unmatched power-off\n");
		return -EBUSY;
	}

	pwrseq->pwrup_count--;
	if (pwrseq->pwrup_count != 0) {
		desc->powered_up = false;
		return 0;
	}

	ret = pwrseq->power_off(pwrseq);
	if (!ret)
		desc->powered_up = false;

	return ret;
}
EXPORT_SYMBOL_GPL(pwrseq_power_off);

static int __init pwrseq_init(void)
{
	int ret;

	ret = bus_register(&pwrseq_bus);
	if (ret) {
		pr_err("Failed to register the power sequencer bus\n");
		return ret;
	}

	return 0;
}
subsys_initcall(pwrseq_init);

static void __exit pwrseq_exit(void)
{
	bus_unregister(&pwrseq_bus);
}
module_exit(pwrseq_exit);

MODULE_AUTHOR("Bartosz Golaszewski <bartosz.golaszewski@linaro.org>");
MODULE_DESCRIPTION("Power Sequencing subsystem core");
MODULE_LICENSE("GPL");
