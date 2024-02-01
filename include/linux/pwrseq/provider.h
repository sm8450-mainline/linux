/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Linaro Ltd.
 */

#ifndef __POWER_SEQUENCING_PROVIDER_H__
#define __POWER_SEQUENCING_PROVIDER_H__

struct device;
struct module;
struct pwrseq_device;

typedef int (*pwrseq_power_state_func)(struct pwrseq_device *);
typedef int (*pwrseq_match_func)(struct pwrseq_device *, struct device *);

/**
 * struct pwrseq_config - Configuration used for registering a new provider.
 * @parent: Parent device for the sequencer.
 * @owner: Module providing this device.
 * @drvdata: Private driver data.
 * @match: Provider callback used to match the consumer device to the sequencer.
 * @power_on: Callback running the power-on sequence.
 * @power_off: Callback running the power-off sequence.
 */
struct pwrseq_config {
	struct device *parent;
	struct module *owner;
	void *drvdata;
	pwrseq_match_func match;
	pwrseq_power_state_func power_on;
	pwrseq_power_state_func power_off;
};

struct pwrseq_device *pwrseq_device_register(struct pwrseq_config *config);
void pwrseq_device_unregister(struct pwrseq_device *pwrseq);
struct pwrseq_device *
devm_pwrseq_device_register(struct device *dev, struct pwrseq_config *config);

void *pwrseq_device_get_data(struct pwrseq_device *pwrseq);

#endif /* __POWER_SEQUENCING_PROVIDER_H__ */
