// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024 Linaro Ltd.
 */

#include <linux/device.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/pci-pwrctl.h>
#include <linux/property.h>
#include <linux/slab.h>

static int pci_pwrctl_notify(struct notifier_block *nb, unsigned long action,
			     void *data)
{
	struct pci_pwrctl *pwrctl = container_of(nb, struct pci_pwrctl, nb);
	struct device *dev = data;

	if (dev_fwnode(dev) != dev_fwnode(pwrctl->dev))
		return NOTIFY_DONE;

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		device_set_of_node_from_dev(dev, pwrctl->dev);
		break;
	case BUS_NOTIFY_BOUND_DRIVER:
		pwrctl->link = device_link_add(dev, pwrctl->dev,
					       DL_FLAG_AUTOREMOVE_CONSUMER);
		if (!pwrctl->link)
			dev_err(pwrctl->dev, "Failed to add device link\n");
		break;
	case BUS_NOTIFY_UNBOUND_DRIVER:
		device_link_del(pwrctl->link);
		break;
	}

	return NOTIFY_DONE;
}

int pci_pwrctl_device_enable(struct pci_pwrctl *pwrctl)
{
	if (!pwrctl->dev)
		return -ENODEV;

	pwrctl->nb.notifier_call = pci_pwrctl_notify;
	bus_register_notifier(&pci_bus_type, &pwrctl->nb);

	pci_lock_rescan_remove();
	pci_rescan_bus(to_pci_dev(pwrctl->dev->parent)->bus);
	pci_unlock_rescan_remove();

	return 0;
}
EXPORT_SYMBOL_GPL(pci_pwrctl_device_enable);

void pci_pwrctl_device_disable(struct pci_pwrctl *pwrctl)
{
	bus_unregister_notifier(&pci_bus_type, &pwrctl->nb);
}
EXPORT_SYMBOL_GPL(pci_pwrctl_device_disable);

static void devm_pci_pwrctl_device_disable(void *data)
{
	struct pci_pwrctl *pwrctl = data;

	pci_pwrctl_device_disable(pwrctl);
}

int devm_pci_pwrctl_device_enable(struct device *dev,
				  struct pci_pwrctl *pwrctl)
{
	int ret;

	ret = pci_pwrctl_device_enable(pwrctl);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev, devm_pci_pwrctl_device_disable,
					pwrctl);
}
EXPORT_SYMBOL_GPL(devm_pci_pwrctl_device_enable);
