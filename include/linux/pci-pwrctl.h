/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Linaro Ltd.
 */

#ifndef __PCI_PWRCTL_H__
#define __PCI_PWRCTL_H__

#include <linux/notifier.h>

struct device;

struct pci_pwrctl {
	struct notifier_block nb;
	struct device *dev;
	struct device_link *link;
};

int pci_pwrctl_device_enable(struct pci_pwrctl *pwrctl);
void pci_pwrctl_device_disable(struct pci_pwrctl *pwrctl);
int devm_pci_pwrctl_device_enable(struct device *dev,
				  struct pci_pwrctl *pwrctl);

#endif /* __PCI_PWRCTL_H__ */
