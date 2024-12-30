/*
 * fts.c
 *
 * FTS Capacitive touch screen controller (FingerTipS)
 *
 * Copyright (C) 2016, STMicroelectronics Limited.
 * Authors: AMG(Analog Mems Group)
 *
 * 		marco.cali@st.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
 */

/*!
* \file fts.c
* \brief It is the main file which contains all the most important functions generally used by a device driver the driver
*/
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spi.h>
#include <linux/completion.h>
#ifdef CONFIG_SECURE_TOUCH
#include <linux/atomic.h>
#include <linux/sysfs.h>
#include <linux/hardirq.h>
#endif

#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>

#include <linux/fb.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>

#include "fts.h"
#include "fts_lib/ftsCompensation.h"
#include "fts_lib/ftsCore.h"
#include "fts_lib/ftsIO.h"
#include "fts_lib/ftsError.h"
#include "fts_lib/ftsFlash.h"
#include "fts_lib/ftsFrame.h"
#include "fts_lib/ftsGesture.h"
#include "fts_lib/ftsTest.h"
#include "fts_lib/ftsTime.h"
#include "fts_lib/ftsTool.h"
#include <linux/power_supply.h>
#include <linux/rtc.h>
#include <linux/time.h>
#include <linux/time64.h>

/**
 * Event handler installer helpers
 */
#define event_id(_e) (EVT_ID_##_e >> 4)
#define handler_name(_h) fts_##_h##_event_handler

#define install_handler(_i, _evt, _hnd)                                        \
	do {                                                                   \
		_i->event_dispatch_table[event_id(_evt)] = handler_name(_hnd); \
	} while (0)

#ifdef KERNEL_ABOVE_2_6_38
#define TYPE_B_PROTOCOL
#endif
extern SysInfo systemInfo;
extern TestToDo tests;

char tag[8] = "[ FTS ]\0";
#define FTS_EVENT_QUEUE_NAME "fts-event-queue"
#define FTS_IRQ_QUEUE_NAME "fts-irq-queue"
#define FTS_FPS_QUEUE_NAME "fts-fps-queue"
#define FTS_FWU_QUEUE_NAME "fts-fwu-queue"
#define FTS_DEBUGFS_DIR_NAME "tp_debug"
#define FTS_TOUCH_DEV_NAME "tp_dev"
#define FTS_TP_LOCKDOWN_INFO_NAME "tp_lockdown_info"
#define FTS_TP_SELFTEST_NAME "tp_selftest"
#define FTS_TP_DATA_DUMP_NAME "tp_data_dump"
#define FTS_TP_FW_VERSION_NAME "tp_fw_version"
#define FTS_TOUCH_FEATURE_QUEUE_NAME "fts-touch-feature"

/* buffer which store the input device name assigned by the kernel  */
char fts_ts_phys[64];
/* buffer used to store the command sent from the MP device file node  */
static u32 typeOfComand[CMD_STR_LEN] = { 0 };

/* number of parameter passed through the MP device file node  */
static int numberParameters;
#ifdef USE_ONE_FILE_NODE
static int feature_feasibility = ERROR_OP_NOT_ALLOW;
#endif

extern spinlock_t fts_int;
struct fts_ts_info *fts_info;

static int fts_init_sensing(struct fts_ts_info *info);
static int fts_mode_handler(struct fts_ts_info *info, int force);
static int fts_chip_initialization(struct fts_ts_info *info, int init_type);
static irqreturn_t fts_event_handler(int irq, void *ts_info);
static int fts_enable_reg(struct fts_ts_info *info, bool enable);
extern int power_supply_is_system_supplied(void);

/**
* Release all the touches in the linux input subsystem
* @param info pointer to fts_ts_info which contains info about the device and its hw setup
*/
void release_all_touches(struct fts_ts_info *info)
{
	unsigned int type = MT_TOOL_FINGER;
	int i;

	for (i = 0; i < TOUCH_ID_MAX; i++) {
		input_mt_slot(info->input_dev, i);
		input_mt_report_slot_state(info->input_dev, type, 0);
		input_report_abs(info->input_dev, ABS_MT_TRACKING_ID, -1);
		info->last_x[i] = info->last_y[i] = 0;
	}
	input_sync(info->input_dev);
	info->touch_id = 0;
	info->temp_touch_id = 0;
	info->width_major = 0;
	info->width_minor = 0;
	info->orientation = 0;
	logError(1, "%s release all touches\n", tag);
}

/**
 * @defgroup file_nodes Driver File Nodes
 * Driver publish a series of file nodes used to provide several utilities to the host and give him access to different API.
 * @{
 */

/**
 * @defgroup device_file_nodes Device File Nodes
 * @ingroup file_nodes
 * Device File Nodes \n
 * There are several file nodes that are associated to the device and which are designed to be used by the host to enable/disable features or trigger some system specific actions \n
 * Usually their final path depend on the definition of device tree node of the IC (e.g /sys/devices/soc.0/f9928000.i2c/i2c-6/6-0049)
 * @{
 */
/***************************************** FW UPGGRADE ***************************************************/

/**
 * File node function to Update firmware from shell \n
 * echo path_to_fw X Y > fwupdate   perform a fw update \n
 * where: \n
 * path_to_fw = file name or path of the the FW to burn, if "NULL" the default approach selected in the driver will be used\n
 * X = 0/1 to force the FW update whichever fw_version and config_id; 0=perform a fw update only if the fw in the file is newer than the fw in the chip \n
 * Y = 0/1 keep the initialization data; 0 = will erase the initialization data from flash, 1 = will keep the initialization data
 * the string returned in the shell is made up as follow: \n
 * { = start byte \n
 * X1X2X3X4 = 4 bytes in HEX format which represent an error code (00000000 no error) \n
 * } = end byte
 */
static ssize_t fts_fwupdate_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int ret, mode[2];
	char path[100];
	struct fts_ts_info *info = dev_get_drvdata(dev);

	/* by default(if not specified by the user) set the force = 0 and keep_cx to 1 */
	mode[0] = 0;
	mode[1] = 1;

	/* reading out firmware upgrade parameters */
	sscanf(buf, "%99s %d %d", path, &mode[0], &mode[1]);
	logError(1, "%s fts_fwupdate_store: mode = %s \n", tag, path);

	ret = flashProcedure(path, mode[0], mode[1]);

	info->fwupdate_stat = ret;

	if (ret < OK)
		logError(1, "%s  %s Unable to upgrade firmware! ERROR %08X\n",
			 tag, __func__, ret);
	return count;
}

static ssize_t fts_fwupdate_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);

	/*fwupdate_stat: ERROR code Returned by flashProcedure. */
	return snprintf(buf, PAGE_SIZE, "{ %08X }\n", info->fwupdate_stat);
}

/***************************************** UTILITIES (current fw_ver/conf_id, active mode, file fw_ver/conf_id)  ***************************************************/
/**
* File node to show on terminal external release version in Little Endian (first the less significant byte) \n
* cat appid			show on the terminal external release version of the FW running in the IC
*/
static ssize_t fts_appid_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int error;
	char temp[100] = {
		0x00,
	};

	error = snprintf(buf, PAGE_SIZE, "%s\n",
			 printHex("EXT Release = ", systemInfo.u8_releaseInfo,
				  EXTERNAL_RELEASE_INFO_SIZE, temp));

	return error;
}

/**
 * File node to show on terminal the mode that is active on the IC \n
 * cat mode_active		    to show the bitmask which indicate the modes/features which are running on the IC in a specific instant of time
 * the string returned in the shell is made up as follow: \n
 * { = start byte \n
 * X1 = 1 byte in HEX format which represent the actual running scan mode (@link scan_opt Scan Mode Options @endlink) \n
 * X2 = 1 byte in HEX format which represent the bitmask on which is running the actual scan mode \n
 * X3X4 = 2 bytes in HEX format which represent a bitmask of the features that are enabled at this moment (@link feat_opt Feature Selection Options @endlink) \n
 * } = end byte
 * @see fts_mode_handler()
 */
static ssize_t fts_mode_active_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);

	logError(1, "%s Current mode active = %08X\n", tag, info->mode);
	return snprintf(buf, PAGE_SIZE, "{ %08X }\n", info->mode);
}

/**
 * File node to show the fw_ver and config_id of the FW file
 * cat fw_file_test			show on the kernel log fw_version and config_id of the FW stored in the fw file/header file
 */
static ssize_t fts_fw_test_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	Firmware fw;
	int ret;
	char temp[100] = {
		0x00,
	};
	struct fts_ts_info *info = dev_get_drvdata(dev);

	fw.data = NULL;
	ret = readFwFile(info->board->default_fw_name, &fw, 0);

	if (ret < OK) {
		logError(1, "%s Error during reading FW file! ERROR %08X\n",
			 tag, ret);
	} else {
		logError(1, "%s %s, size = %d bytes\n", tag,
			 printHex("EXT Release = ", systemInfo.u8_releaseInfo,
				  EXTERNAL_RELEASE_INFO_SIZE, temp),
			 fw.data_size);
	}

	kfree(fw.data);
	return 0;
}

/***************************************** PRODUCTION TEST ***************************************************/

/**
 * File node to execute the Mass Production Test or to get data from the IC (raw or ms/ss init data)
 * echo cmd > stm_fts_cmd		to execute a command \n
 * cat stm_fts_cmd				to show the result of the command \n
 * echo cmd > stm_fts_cmd; cat stm_fts_cmd 		to execute and show the result in just one call \n
 * the string returned in the shell is made up as follow: \n
 * { = start byte \n
 * X1X2X3X4 = 4 bytes in HEX format which represent an error_code (00000000 = OK)\n
 * (optional) data = data coming from the command executed represented as HEX string \n
 *                   Not all the command return additional data \n
 * } = end byte
 * \n
 * Possible commands (cmd): \n
 * - 00 = MP Test -> return erro_code \n
 * - 01 = ITO Test -> return error_code \n
 * - 03 = MS Raw Test -> return error_code \n
 * - 04 = MS Init Data Test -> return error_code \n
 * - 05 = SS Raw Test -> return error_code \n
 * - 06 = SS Init Data Test -> return error_code \n
 * - 13 = Read 1 MS Raw Frame -> return additional data: MS frame row after row \n
 * - 14 = Read MS Init Data -> return additional data: MS init data row after row \n
 * - 15 = Read 1 SS Raw Frame -> return additional data: SS frame, force channels followed by sense channels \n
 * - 16 = Read SS Init Data -> return additional data: SS Init data, first IX for force and sense channels and then CX for force and sense channels \n
 * - F0 = Perform a system reset -> return error_code \n
 * - F1 = Perform a system reset and reenable the sensing and the interrupt
 */
static ssize_t stm_fts_cmd_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	int n;
	char *p = (char *)buf;

	memset(typeOfComand, 0, CMD_STR_LEN * sizeof(u32));

	logError(1, "%s \n", tag);
	for (n = 0; n < (count + 1) / 3; n++) {
		sscanf(p, "%02X ", &typeOfComand[n]);
		p += 3;
		logError(1, "%s typeOfComand[%d] = %02X \n", tag, n,
			 typeOfComand[n]);
	}

	numberParameters = n;
	logError(1, "%s Number of Parameters = %d \n", tag, numberParameters);
	return count;
}

static ssize_t stm_fts_cmd_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int res, j, doClean = 0, count = 0, index = 0;
	char buff[CMD_STR_LEN] = { 0 };

	int size = (6 * 2) + 1;
	int init_type = SPECIAL_PANEL_INIT;
	u8 *all_strbuff = NULL;
	const char *limit_file_name = NULL;
	struct fts_ts_info *info = dev_get_drvdata(dev);

	MutualSenseData compData;
	SelfSenseData comData;
	MutualSenseFrame frameMS;
	SelfSenseFrame frameSS;

	limit_file_name = fts_get_limit(info);

	if (numberParameters >= 1) {
		res = fts_disableInterrupt();
		if (res < 0) {
			logError(0, "%s fts_disableInterrupt: ERROR %08X \n",
				 tag, res);
			res = (res | ERROR_DISABLE_INTER);
			goto END;
		}
		switch (typeOfComand[0]) {
			/*ITO TEST */
		case 0x01:
			res = production_test_ito(limit_file_name, &tests);
			break;
			/*PRODUCTION TEST */
		case 0x00:

			if (systemInfo.u8_cfgAfeVer != systemInfo.u8_cxAfeVer) {
				res = ERROR_OP_NOT_ALLOW;
				logError(
					0,
					"%s Miss match in CX version! MP test not allowed with wrong CX memory! ERROR %08X \n",
					tag, res);
				break;
			}

			res = production_test_main(limit_file_name, 1,
						   init_type, &tests);
			break;
			/*read mutual raw */
		case 0x13:
			logError(0, "%s Get 1 MS Frame \n", tag);
			setScanMode(SCAN_MODE_ACTIVE, 0x01);
			mdelay(WAIT_FOR_FRESH_FRAMES);
			setScanMode(SCAN_MODE_ACTIVE, 0x00);
			mdelay(WAIT_AFTER_SENSEOFF);
			flushFIFO();
			res = getMSFrame3(MS_RAW, &frameMS);
			if (res < 0) {
				logError(
					0,
					"%s Error while taking the MS frame... ERROR %08X \n",
					tag, res);

			} else {
				logError(0, "%s The frame size is %d words\n",
					 tag, res);
				size = (res * (sizeof(short) * 2 + 1)) + 10;
				res = OK;
				print_frame_short(
					"MS frame =",
					array1dTo2d_short(
						frameMS.node_data,
						frameMS.node_data_size,
						frameMS.header.sense_node),
					frameMS.header.force_node,
					frameMS.header.sense_node);
			}
			break;
			/*read self raw */
		case 0x15:
			logError(0, "%s Get 1 SS Frame \n", tag);
			setScanMode(SCAN_MODE_ACTIVE, 0x01);
			mdelay(WAIT_FOR_FRESH_FRAMES);
			setScanMode(SCAN_MODE_ACTIVE, 0x00);
			mdelay(WAIT_AFTER_SENSEOFF);
			flushFIFO();
			res = getSSFrame3(SS_RAW, &frameSS);

			if (res < OK) {
				logError(
					0,
					"%s Error while taking the SS frame... ERROR %08X \n",
					tag, res);

			} else {
				logError(0, "%s The frame size is %d words\n",
					 tag, res);
				size = (res * (sizeof(short) * 2 + 1)) + 10;
				res = OK;
				print_frame_short(
					"SS force frame =",
					array1dTo2d_short(
						frameSS.force_data,
						frameSS.header.force_node, 1),
					frameSS.header.force_node, 1);
				print_frame_short(
					"SS sense frame =",
					array1dTo2d_short(
						frameSS.sense_data,
						frameSS.header.sense_node,
						frameSS.header.sense_node),
					1, frameSS.header.sense_node);
			}

			break;

		case 0x14:
			logError(0, "%s Get MS Compensation Data \n", tag);
			res = readMutualSenseCompensationData(LOAD_CX_MS_TOUCH,
							      &compData);

			if (res < 0) {
				logError(
					0,
					"%s Error reading MS compensation data ERROR %08X \n",
					tag, res);
			} else {
				logError(
					0,
					"%s MS Compensation Data Reading Finished! \n",
					tag);
				size = (compData.node_data_size * sizeof(u8)) *
					       3 +
				       1;
				print_frame_i8(
					"MS Data (Cx2) =",
					array1dTo2d_i8(
						compData.node_data,
						compData.node_data_size,
						compData.header.sense_node),
					compData.header.force_node,
					compData.header.sense_node);
			}
			break;

		case 0x16:
			logError(0, "%s Get SS Compensation Data... \n", tag);
			res = readSelfSenseCompensationData(LOAD_CX_SS_TOUCH,
							    &comData);
			if (res < 0) {
				logError(
					0,
					"%s Error reading SS compensation data ERROR %08X\n",
					tag, res);
			} else {
				logError(
					0,
					"%s SS Compensation Data Reading Finished! \n",
					tag);
				size = ((comData.header.force_node +
					 comData.header.sense_node) *
						2 +
					12) * sizeof(u8) *
					       2 +
				       1;
				print_frame_u8(
					"SS Data Ix2_fm = ",
					array1dTo2d_u8(
						comData.ix2_fm,
						comData.header.force_node, 1),
					comData.header.force_node, 1);
				print_frame_i8(
					"SS Data Cx2_fm = ",
					array1dTo2d_i8(
						comData.cx2_fm,
						comData.header.force_node, 1),
					comData.header.force_node, 1);
				print_frame_u8(
					"SS Data Ix2_sn = ",
					array1dTo2d_u8(
						comData.ix2_sn,
						comData.header.sense_node,
						comData.header.sense_node),
					1, comData.header.sense_node);
				print_frame_i8(
					"SS Data Cx2_sn = ",
					array1dTo2d_i8(
						comData.cx2_sn,
						comData.header.sense_node,
						comData.header.sense_node),
					1, comData.header.sense_node);
			}
			break;

		case 0x03:
			res = fts_system_reset();
			if (res >= OK)
				res = production_test_ms_raw(limit_file_name, 1,
							     &tests);
			break;

		case 0x04:
			res = fts_system_reset();
			if (res >= OK)
				res = production_test_ms_cx(limit_file_name, 1,
							    &tests);
			break;

		case 0x05:
			res = fts_system_reset();
			if (res >= OK)
				res = production_test_ss_raw(limit_file_name, 1,
							     &tests);
			break;

		case 0x06:
			res = fts_system_reset();
			if (res >= OK)
				res = production_test_ss_ix_cx(limit_file_name,
							       1, &tests);
			break;

		case 0xF0:
		case 0xF1:
			doClean = (int)(typeOfComand[0] & 0x01);
			res = cleanUp(doClean);
			break;

		default:
			logError(
				1,
				"%s COMMAND NOT VALID!! Insert a proper value ...\n",
				tag);
			res = ERROR_OP_NOT_ALLOW;
			break;
		}

		doClean = fts_mode_handler(info, 1);
		if (typeOfComand[0] != 0xF0)
			doClean |= fts_enableInterrupt();
		if (doClean < 0) {
			logError(0, "%s %s: ERROR %08X \n", tag, __func__,
				 (doClean | ERROR_ENABLE_INTER));
		}
	} else {
		logError(
			1,
			"%s NO COMMAND SPECIFIED!!! do: 'echo [cmd_code] [args] > stm_fts_cmd' before looking for result!\n",
			tag);
		res = ERROR_OP_NOT_ALLOW;
	}
END:
	all_strbuff = (u8 *)kzalloc(size, GFP_KERNEL);

	if (res >= OK) {
		/*all the other cases are already fine printing only the res. */
		switch (typeOfComand[0]) {
		case 0x13:
			snprintf(all_strbuff, size, "ms_frame\n");
			for (j = 0; j < frameMS.node_data_size; j++) {
				if ((j + 1) % frameMS.header.sense_node)
					snprintf(buff, sizeof(buff), "%04d ",
						 frameMS.node_data[j]);
				else
					snprintf(buff, sizeof(buff), "%04d\n",
						 frameMS.node_data[j]);

				strlcat(all_strbuff, buff, size);
			}

			kfree(frameMS.node_data);
			frameMS.node_data = NULL;
			break;

		case 0x15:
			snprintf(all_strbuff, size, "ss_frame\n");
			for (j = 0; j < frameSS.header.force_node - 1; j++) {
				snprintf(buff, sizeof(buff), "%04d ",
					 frameSS.force_data[j]);
				strlcat(all_strbuff, buff, size);
			}

			if (j == frameSS.header.force_node - 1) {
				snprintf(buff, sizeof(buff), "%04d\n",
					 frameSS.force_data[j]);
				strlcat(all_strbuff, buff, size);
			}

			for (j = 0; j < frameSS.header.sense_node - 1; j++) {
				snprintf(buff, sizeof(buff), "%04d ",
					 frameSS.sense_data[j]);
				strlcat(all_strbuff, buff, size);
			}

			if (j == frameSS.header.sense_node - 1) {
				snprintf(buff, sizeof(buff), "%04d\n",
					 frameSS.sense_data[j]);
				strlcat(all_strbuff, buff, size);
			}

			kfree(frameSS.force_data);
			kfree(frameSS.sense_data);

			break;

		case 0x14:
			snprintf(buff, sizeof(buff), "%02X",
				 (u8)compData.header.force_node);
			strlcat(all_strbuff, buff, size);
			snprintf(buff, sizeof(buff), "%02X",
				 (u8)compData.header.sense_node);
			strlcat(all_strbuff, buff, size);
			snprintf(buff, sizeof(buff), "%02X", compData.cx1);
			strlcat(all_strbuff, buff, size);

			for (j = 0; j < compData.node_data_size; j++) {
				snprintf(buff, sizeof(buff), "%02X",
					 *(compData.node_data + j));
				strlcat(all_strbuff, buff, size);
			}

			kfree(compData.node_data);
			compData.node_data = NULL;

			break;

		case 0x16:
			snprintf(buff, sizeof(buff), "%02X",
				 comData.header.force_node);
			strlcat(all_strbuff, buff, size);
			snprintf(buff, sizeof(buff), "%02X",
				 comData.header.sense_node);
			strlcat(all_strbuff, buff, size);
			snprintf(buff, sizeof(buff), "%02X", comData.f_ix1);
			strlcat(all_strbuff, buff, size);
			snprintf(buff, sizeof(buff), "%02X", comData.s_ix1);
			strlcat(all_strbuff, buff, size);
			snprintf(buff, sizeof(buff), "%02X", comData.f_cx1);
			strlcat(all_strbuff, buff, size);
			snprintf(buff, sizeof(buff), "%02X", comData.s_cx1);
			strlcat(all_strbuff, buff, size);

			for (j = 0; j < comData.header.force_node; j++) {
				snprintf(buff, sizeof(buff), "%02X",
					 comData.ix2_fm[j]);
				strlcat(all_strbuff, buff, size);
			}

			for (j = 0; j < comData.header.sense_node; j++) {
				snprintf(buff, sizeof(buff), "%02X",
					 comData.ix2_sn[j]);
				strlcat(all_strbuff, buff, size);
			}

			for (j = 0; j < comData.header.force_node; j++) {
				snprintf(buff, sizeof(buff), "%02X",
					 comData.cx2_fm[j]);
				strlcat(all_strbuff, buff, size);
			}

			for (j = 0; j < comData.header.sense_node; j++) {
				snprintf(buff, sizeof(buff), "%02X",
					 comData.cx2_sn[j]);
				strlcat(all_strbuff, buff, size);
			}

			kfree(comData.ix2_fm);
			kfree(comData.ix2_sn);
			kfree(comData.cx2_fm);
			kfree(comData.cx2_sn);

			break;

		default:
			snprintf(&all_strbuff[index], 11, "{ %08X", res);
			index += 10;
			snprintf(&all_strbuff[index], 3, " }");
			index += 2;

			break;
		}
	} else {
		snprintf(&all_strbuff[index], 11, "{ %08X", res);
		index += 10;
		snprintf(&all_strbuff[index], 3, " }");
		index += 2;
	}

	count = snprintf(buf, TSP_BUF_SIZE, "%s\n", all_strbuff);
	numberParameters = 0;
	kfree(all_strbuff);

	return count;
}

static ssize_t fts_lockdown_info_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);
	int ret;
	ret = fts_get_lockdown_info(info->lockdown_info, info);

	if (ret != OK) {
		logError(1, "%s get lockdown info error\n", tag);
		return 0;
	}

	return snprintf(
		buf, PAGE_SIZE,
		"0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\n",
		info->lockdown_info[0], info->lockdown_info[1],
		info->lockdown_info[2], info->lockdown_info[3],
		info->lockdown_info[4], info->lockdown_info[5],
		info->lockdown_info[6], info->lockdown_info[7]);
}

static ssize_t fts_lockdown_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int n, i, ret;
	char *p = (char *)buf;
	u8 *typecomand = NULL;

	memset(typeOfComand, 0, CMD_STR_LEN * sizeof(u32));
	logError(1, "%s \n", tag);
	for (n = 0; n < (count + 1) / 3; n++) {
		sscanf(p, "%02X ", &typeOfComand[n]);
		p += 3;
		logError(1, "%s command_sequence[%d] = %02X\n", tag, n,
			 typeOfComand[n]);
	}
	numberParameters = n;
	if (numberParameters < 3)
		goto END;
	logError(1, "%s %d = %d \n", tag, n, numberParameters);

	typecomand =
		(u8 *)kmalloc((numberParameters - 2) * sizeof(u8), GFP_KERNEL);
	if (typecomand != NULL) {
		for (i = 0; i < numberParameters - 2; i++) {
			typecomand[i] = (u8)typeOfComand[i + 2];
			logError(1, "%s typecomand[%d] = %X \n", tag, i,
				 typecomand[i]);
		}
	} else {
		goto END;
	}

	ret = writeLockDownInfo(typecomand, numberParameters - 2,
				typeOfComand[0]);
	if (ret < 0) {
		logError(1, "%s fts_lockdown_store failed\n", tag);
	}
	kfree(typecomand);
END:
	logError(1, "%s Number of Parameters = %d \n", tag, numberParameters);

	return count;
}

static ssize_t fts_lockdown_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int i, ret;
	int size = 0, count = 0;
	u8 type;
	u8 *temp_buffer = NULL;

	temp_buffer = (u8 *)kmalloc(LOCKDOWN_LENGTH * sizeof(u8), GFP_KERNEL);
	if (temp_buffer == NULL || numberParameters < 2) {
		count += snprintf(&buf[count], PAGE_SIZE,
				  "prepare read lockdown failded\n");
		return count;
	}
	type = typeOfComand[0];
	size = (int)(typeOfComand[1]);
	count += snprintf(&buf[count], PAGE_SIZE, "read lock down code:\n");
	ret = readLockDownInfo(temp_buffer, type, size);
	if (ret < OK) {
		count += snprintf(&buf[count], PAGE_SIZE,
				  "read lockdown failded\n");
		goto END;
	}
	for (i = 0; i < size; i++) {
		count += snprintf(&buf[count], PAGE_SIZE, "%02X ",
				  temp_buffer[i]);
	}
	count += snprintf(&buf[count], PAGE_SIZE, "\n");

END:
	numberParameters = 0;
	kfree(temp_buffer);
	return count;
}

static ssize_t fts_selftest_info_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	int res = 0, i = 0, count = 0, force_node = 0, sense_node = 0, pos = 0,
	    last_pos = 0;
	MutualSenseFrame frameMS;
	char buff[80];
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);

	res = fts_disableInterrupt();
	if (res < OK)
		goto END;

	setScanMode(SCAN_MODE_ACTIVE, 0x01);
	mdelay(WAIT_FOR_FRESH_FRAMES);
	setScanMode(SCAN_MODE_ACTIVE, 0x00);
	mdelay(WAIT_AFTER_SENSEOFF);
	flushFIFO();
	res = getMSFrame3(MS_RAW, &frameMS);
	if (res < 0) {
		logError(0,
			 "%s Error while taking the MS frame... ERROR %08X \n",
			 tag, res);
		goto END;
	}
	fts_mode_handler(info, 1);

	sense_node = frameMS.header.sense_node;
	force_node = frameMS.header.force_node;

	for (i = 0; i < RELEASE_INFO_SIZE; i++) {
		if (i == 0) {
			pos += snprintf(buff + last_pos, PAGE_SIZE, "0x%02x",
					systemInfo.u8_releaseInfo[i]);
			last_pos = pos;
		} else {
			pos += snprintf(buff + last_pos, PAGE_SIZE, "%02x",
					systemInfo.u8_releaseInfo[i]);
			last_pos = pos;
		}
	}
	count = snprintf(
		buf, PAGE_SIZE,
		"Device address:,0x49\nChip Id:,0x%04x\nFw version:,0x%04x\nConfig version:,0x%04x\nChip serial number:,%s\nForce lines count:,%02d\nSense lines count:,%02d\n\n",
		systemInfo.u16_chip0Id, systemInfo.u16_fwVer,
		systemInfo.u16_cfgVer, buff, force_node, sense_node);
END:
	fts_enableInterrupt();
	return count;
}

static ssize_t fts_ms_raw_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int res = 0, count = 0, j = 0, sense_node = 0, force_node = 0, pos = 0,
	    last_pos = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	MutualSenseFrame frameMS;
	int buf_size;

	res = fts_disableInterrupt();
	if (res < OK)
		goto END;

	setScanMode(SCAN_MODE_ACTIVE, 0x01);
	mdelay(WAIT_FOR_FRESH_FRAMES);
	setScanMode(SCAN_MODE_ACTIVE, 0x00);
	mdelay(WAIT_AFTER_SENSEOFF);
	flushFIFO();
	res = getMSFrame3(MS_RAW, &frameMS);

	fts_mode_handler(info, 1);
	sense_node = frameMS.header.sense_node;
	force_node = frameMS.header.force_node;
	buf_size = sense_node * force_node * 5 + (sense_node + force_node) * 4 +
		   50;

	info->data_dump_buf = kvmalloc(buf_size, GFP_KERNEL);
	if (!info->data_dump_buf) {
		logError(1, "%s %s alloc all_strbuff fail\n", tag, __func__);
		goto END;
	} else
		memset(info->data_dump_buf, 0, buf_size);
	pos += snprintf(info->data_dump_buf + last_pos, buf_size,
			"MsTouchRaw,%2d,%2d\n ,", force_node, sense_node);
	last_pos = pos;
	if (res >= OK) {
		for (j = 0; j < sense_node; j++)
			if ((j + 1) % sense_node) {
				pos += snprintf(info->data_dump_buf + last_pos,
						buf_size, "C%02d,", j);
				last_pos = pos;
			} else {
				pos += snprintf(info->data_dump_buf + last_pos,
						buf_size, "C%02d\nR00,", j);
				last_pos = pos;
			}
		for (j = 0; j < sense_node * force_node; j++) {
			if ((j + 1) % sense_node) {
				pos += snprintf(info->data_dump_buf + last_pos,
						buf_size, "%4d,",
						frameMS.node_data[j]);
				last_pos = pos;
			} else {
				if ((j + 1) / sense_node != force_node)
					pos += snprintf(info->data_dump_buf +
								last_pos,
							buf_size, "%4d\nR%02d,",
							frameMS.node_data[j],
							(j + 1) / sense_node);
				else
					pos += snprintf(info->data_dump_buf +
								last_pos,
							buf_size, "%4d\n",
							frameMS.node_data[j]);
				last_pos = pos;
			}
		}
		if (frameMS.node_data) {
			kfree(frameMS.node_data);
			frameMS.node_data = NULL;
		}
	}
	count = strlen(info->data_dump_buf);
	logError(1, "%s %s len:%d\n", tag, __func__, count);
	memcpy(buf, info->data_dump_buf, count);
	kvfree(info->data_dump_buf);
	info->data_dump_buf = NULL;
END:
	fts_enableInterrupt();
	return count;
}

static ssize_t fts_mutual_raw_ito_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	MutualSenseFrame msRawFrame;
	int last_pos = 0;
	int pos = 0;
	int j = 0, count = 0;
	int sense_node = 0, force_node = 0;
	int res = OK;
	u8 sett[2] = { 0x00, 0x00 };
	int buf_size;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);

	res = fts_disableInterrupt();
	if (res < OK)
		return res;
	logError(1, "%s ITO Production test is starting...\n", tag);
	memset(&msRawFrame, 0x00, sizeof(msRawFrame));
	res = fts_system_reset();

	if (res < 0) {
		logError(1, "%s %s: ERROR %08X \n", tag, __func__,
			 ERROR_PROD_TEST_ITO);
		goto ERROR;
	}

	sett[0] = SPECIAL_TUNING_IOFF;
	logError(1, "%s Trimming Ioff... \n", tag);
	res = writeSysCmd(SYS_CMD_SPECIAL_TUNING, sett, 2);

	if (res < OK) {
		logError(1, "%s production_test_ito: Trimm Ioff ERROR %08X \n",
			 tag, (res | ERROR_PROD_TEST_ITO));
		goto ERROR;
	}

	sett[0] = 0xFF;
	sett[1] = 0x01;
	logError(1, "%s ITO Check command sent... \n", tag);
	res = writeSysCmd(SYS_CMD_ITO, sett, 2);

	if (res < OK) {
		logError(1, "%s production_test_ito: ERROR %08X \n", tag,
			 (res | ERROR_PROD_TEST_ITO));
		goto ERROR;
	}

	logError(1, "%s ITO Command = OK! \n", tag);

	logError(1, "%s MS RAW ITO ADJ TEST: \n", tag);
	logError(1, "%s Collecting MS Raw data... \n", tag);
	res |= getMSFrame3(MS_RAW, &msRawFrame);

	if (res < OK) {
		logError(1, "%s %s: getMSFrame failed... ERROR %08X \n", tag,
			 __func__, ERROR_PROD_TEST_ITO);
		goto ERROR;
	}
	sense_node = msRawFrame.header.sense_node;
	force_node = msRawFrame.header.force_node;
	buf_size = sense_node * force_node * 5 + (sense_node + force_node) * 4 +
		   50;

	info->data_dump_buf = kvmalloc(buf_size, GFP_KERNEL);
	if (!info->data_dump_buf) {
		logError(1, "%s %s alloc all_strbuff fail\n", tag, __func__);
		goto ERROR;
	} else
		memset(info->data_dump_buf, 0, buf_size);

	logError(1, "%s MS RAW ITO ADJ HORIZONTAL\n", tag);
	pos += snprintf(info->data_dump_buf + last_pos, buf_size,
			"MsRawITO,%2d,%2d\n ,", force_node, sense_node);
	last_pos = pos;

	for (j = 0; j < sense_node; j++) {
		if ((j + 1) % sense_node) {
			pos += snprintf(info->data_dump_buf + last_pos,
					buf_size, "C%02d,", j);
			last_pos = pos;
		} else {
			pos += snprintf(info->data_dump_buf + last_pos,
					buf_size, "C%02d\nR00,", j);
			last_pos = pos;
		}
	}
	for (j = 0; j < sense_node * force_node; j++) {
		if ((j + 1) % sense_node) {
			pos += snprintf(info->data_dump_buf + last_pos,
					buf_size, "%4d,",
					(short)msRawFrame.node_data[j]);
			last_pos = pos;
		} else {
			if ((j + 1) / sense_node != force_node)
				pos += snprintf(info->data_dump_buf + last_pos,
						buf_size, "%4d\nR%02d,",
						(short)msRawFrame.node_data[j],
						(j + 1) / sense_node);
			else
				pos += snprintf(info->data_dump_buf + last_pos,
						buf_size, "%4d\n",
						(short)msRawFrame.node_data[j]);
			last_pos = pos;
		}
	}
	count = strlen(info->data_dump_buf);
	logError(1, "%s %s len:%d\n", tag, __func__, count);
	memcpy(buf, info->data_dump_buf, count);
	kvfree(info->data_dump_buf);
	info->data_dump_buf = NULL;
ERROR:
	if (msRawFrame.node_data != NULL) {
		kfree(msRawFrame.node_data);
		msRawFrame.node_data = NULL;
	}
	if (res < OK) {
		logError(1, "%s production_test_ito_horizontal: ERROR %08X \n",
			 tag, ERROR_PROD_TEST_ITO);
	}
	res = fts_system_reset();
	setScanMode(SCAN_MODE_ACTIVE, 0x01);
	fts_enableInterrupt();
	return count;
}

static ssize_t fts_ms_cx_total_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int res = 0, pos = 0, last_pos = 0, count = 0, j = 0, sense_node = 0,
	    force_node = 0;
	int buf_size;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	TotMutualSenseData totCompData;

	res = fts_disableInterrupt();
	if (res < OK)
		goto END;
	res = readTotMutualSenseCompensationData(LOAD_PANEL_CX_TOT_MS_TOUCH,
						 &totCompData);
	if (res >= OK) {
		sense_node = totCompData.header.sense_node;
		force_node = totCompData.header.force_node;
		buf_size = sense_node * force_node * 5 +
			   (sense_node + force_node) * 4 + 50;
		info->data_dump_buf = kvmalloc(buf_size, GFP_KERNEL);
		if (!info->data_dump_buf) {
			logError(1, "%s %s alloc all_strbuff fail\n", tag,
				 __func__);
			goto END;
		} else
			memset(info->data_dump_buf, 0, buf_size);
		pos += snprintf(info->data_dump_buf + last_pos, buf_size,
				"MsTouchTotalCx,%2d,%2d\n ,", force_node,
				sense_node);
		last_pos = pos;
		for (j = 0; j < sense_node; j++)
			if ((j + 1) % sense_node) {
				pos += snprintf(info->data_dump_buf + last_pos,
						buf_size, "C%02d,", j);
				last_pos = pos;
			} else {
				pos += snprintf(info->data_dump_buf + last_pos,
						buf_size, "C%02d\nR00,", j);
				last_pos = pos;
			}
		for (j = 0; j < sense_node * force_node; j++) {
			if ((j + 1) % sense_node) {
				pos += snprintf(info->data_dump_buf + last_pos,
						buf_size, "%4d,",
						totCompData.node_data[j]);
				last_pos = pos;
			} else {
				if ((j + 1) / sense_node != force_node)
					pos += snprintf(
						info->data_dump_buf + last_pos,
						buf_size, "%4d\nR%02d,",
						totCompData.node_data[j],
						(j + 1) / sense_node);
				else
					pos += snprintf(
						info->data_dump_buf + last_pos,
						buf_size, "%4d\n",
						totCompData.node_data[j]);
				last_pos = pos;
			}
		}
		if (totCompData.node_data) {
			kfree(totCompData.node_data);
			totCompData.node_data = NULL;
		}
		count = strlen(info->data_dump_buf);
		logError(1, "%s %s len:%d\n", tag, __func__, count);
		memcpy(buf, info->data_dump_buf, count);
		kvfree(info->data_dump_buf);
		info->data_dump_buf = NULL;
	} else {
		count = snprintf(buf, PAGE_SIZE, "%s\n",
				 "ms_cx_total test fail");
	}
END:
	fts_enableInterrupt();
	return count;
}

static ssize_t fts_ms_cx2_lp_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int res = 0, pos = 0, last_pos = 0, count = 0, j = 0, sense_node = 0,
	    force_node = 0;
	int buf_size;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	MutualSenseData msCompData;

	res = fts_disableInterrupt();
	if (res < OK)
		goto END;
	res = readMutualSenseCompensationData(LOAD_CX_MS_LOW_POWER,
					      &msCompData);
	if (res >= OK) {
		sense_node = msCompData.header.sense_node;
		force_node = msCompData.header.force_node;
		buf_size = sense_node * force_node * 5 +
			   (sense_node + force_node) * 4 + 50;
		info->data_dump_buf = kvmalloc(buf_size, GFP_KERNEL);
		if (!info->data_dump_buf) {
			logError(1, "%s %s alloc all_strbuff fail\n", tag,
				 __func__);
			goto END;
		} else
			memset(info->data_dump_buf, 0, buf_size);
		pos += snprintf(info->data_dump_buf + last_pos, buf_size,
				"MsTouchCx2Lp,%2d,%2d\n ,", force_node,
				sense_node);
		last_pos = pos;
		for (j = 0; j < sense_node; j++)
			if ((j + 1) % sense_node) {
				pos += snprintf(info->data_dump_buf + last_pos,
						buf_size, "C%02d,", j);
				last_pos = pos;
			} else {
				pos += snprintf(info->data_dump_buf + last_pos,
						buf_size, "C%02d\nR00,", j);
				last_pos = pos;
			}
		for (j = 0; j < sense_node * force_node; j++) {
			if ((j + 1) % sense_node) {
				pos += snprintf(info->data_dump_buf + last_pos,
						buf_size, "%4d,",
						msCompData.node_data[j]);
				last_pos = pos;
			} else {
				if ((j + 1) / sense_node != force_node)
					pos += snprintf(info->data_dump_buf +
								last_pos,
							buf_size, "%4d\nR%02d,",
							msCompData.node_data[j],
							(j + 1) / sense_node);
				else
					pos += snprintf(
						info->data_dump_buf + last_pos,
						buf_size, "%4d\n",
						msCompData.node_data[j]);
				last_pos = pos;
			}
		}
		if (msCompData.node_data) {
			kfree(msCompData.node_data);
			msCompData.node_data = NULL;
		}
		count = strlen(info->data_dump_buf);
		logError(1, "%s %s len:%d\n", tag, __func__, count);
		memcpy(buf, info->data_dump_buf, count);
		kvfree(info->data_dump_buf);
		info->data_dump_buf = NULL;
	} else {
		count = snprintf(buf, PAGE_SIZE, "%s\n", "ms_cx2_lp test fail");
	}
END:
	fts_enableInterrupt();
	return count;
}

static ssize_t fts_ms_cx2_lp_total_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int res = 0, pos = 0, last_pos = 0, count = 0, j = 0, sense_node = 0,
	    force_node = 0;
	int buf_size;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	TotMutualSenseData totCompData;

	res = fts_disableInterrupt();
	if (res < OK)
		goto END;
	res = readTotMutualSenseCompensationData(LOAD_PANEL_CX_TOT_MS_LOW_POWER,
						 &totCompData);
	if (res >= OK) {
		sense_node = totCompData.header.sense_node;
		force_node = totCompData.header.force_node;
		buf_size = sense_node * force_node * 5 +
			   (sense_node + force_node) * 4 + 50;
		info->data_dump_buf = kvmalloc(buf_size, GFP_KERNEL);
		if (!info->data_dump_buf) {
			logError(1, "%s %s alloc all_strbuff fail\n", tag,
				 __func__);
			goto END;
		} else
			memset(info->data_dump_buf, 0, buf_size);
		pos += snprintf(info->data_dump_buf + last_pos, buf_size,
				"MsTouchTxotalCx2Lp,%2d,%2d\n ,", force_node,
				sense_node);
		last_pos = pos;
		for (j = 0; j < sense_node; j++)
			if ((j + 1) % sense_node) {
				pos += snprintf(info->data_dump_buf + last_pos,
						buf_size, "C%02d,", j);
				last_pos = pos;
			} else {
				pos += snprintf(info->data_dump_buf + last_pos,
						buf_size, "C%02d\nR00,", j);
				last_pos = pos;
			}
		for (j = 0; j < sense_node * force_node; j++) {
			if ((j + 1) % sense_node) {
				pos += snprintf(info->data_dump_buf + last_pos,
						buf_size, "%4d,",
						totCompData.node_data[j]);
				last_pos = pos;
			} else {
				if ((j + 1) / sense_node != force_node)
					pos += snprintf(
						info->data_dump_buf + last_pos,
						buf_size, "%4d\nR%02d,",
						totCompData.node_data[j],
						(j + 1) / sense_node);
				else
					pos += snprintf(
						info->data_dump_buf + last_pos,
						buf_size, "%4d\n",
						totCompData.node_data[j]);
				last_pos = pos;
			}
		}
		if (totCompData.node_data) {
			kfree(totCompData.node_data);
			totCompData.node_data = NULL;
		}
		count = strlen(info->data_dump_buf);
		logError(1, "%s %s len:%d\n", tag, __func__, count);
		memcpy(buf, info->data_dump_buf, count);
		kvfree(info->data_dump_buf);
		info->data_dump_buf = NULL;
	} else {
		count = snprintf(buf, PAGE_SIZE, "%s\n",
				 "ms_cx2_lp_total test fail");
	}
END:
	fts_enableInterrupt();
	return count;
}

static ssize_t fts_ss_ix_total_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int ret = 0, pos = 0, last_pos = 0, count = 0, j = 0, sense_node = 0,
	    force_node = 0;
	char *all_strbuff = NULL;
	TotSelfSenseData totCompData;

	ret = fts_disableInterrupt();
	if (ret < OK)
		goto END;
	all_strbuff = kvmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!all_strbuff) {
		logError(1, "%s %s alloc all_strbuff fail\n", tag, __func__);
		goto END;
	} else {
		memset(all_strbuff, 0, PAGE_SIZE);
	}
	ret = readTotSelfSenseCompensationData(LOAD_PANEL_CX_TOT_SS_TOUCH,
					       &totCompData);
	if (ret < 0) {
		logError(
			1,
			"%s production_test_data: readTotSelfSenseCompensationData failed... ERROR %08X \n",
			tag, ERROR_PROD_TEST_DATA);
		goto END;
	}

	sense_node = 1;
	force_node = totCompData.header.force_node;

	pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
			"SsTouchForceTotalIx,%2d,1\n ,C00\n", force_node);
	last_pos = pos;
	for (j = 0; j < force_node; j++) {
		pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
				"R%02d,%4d\n", j, totCompData.ix_fm[j]);
		last_pos = pos;
	}

	pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
			"SsTouchForceTotalCx,%2d,1\n ,C00\n", force_node);
	last_pos = pos;
	for (j = 0; j < force_node; j++) {
		pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
				"R%02d,%4d\n", j, totCompData.cx_fm[j]);
		last_pos = pos;
	}

	sense_node = totCompData.header.sense_node;
	force_node = 1;

	pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
			"SsTouchsenseTotalIx,%2d,1\n ,C00\n", sense_node);
	last_pos = pos;
	for (j = 0; j < sense_node; j++) {
		pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
				"R%02d,%4d\n", j, totCompData.ix_sn[j]);
		last_pos = pos;
	}

	pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
			"SsTouchsenseTotalCx,%2d,1\n ,C00\n", sense_node);
	last_pos = pos;
	for (j = 0; j < sense_node; j++) {
		pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
				"R%02d,%4d\n", j, totCompData.cx_sn[j]);
		last_pos = pos;
	}

	if (totCompData.ix_fm != NULL) {
		kfree(totCompData.ix_fm);
		totCompData.ix_fm = NULL;
	}

	if (totCompData.cx_fm != NULL) {
		kfree(totCompData.cx_fm);
		totCompData.cx_fm = NULL;
	}

	if (totCompData.ix_sn != NULL) {
		kfree(totCompData.ix_sn);
		totCompData.ix_sn = NULL;
	}

	if (totCompData.cx_sn != NULL) {
		kfree(totCompData.cx_sn);
		totCompData.cx_sn = NULL;
	}

	count = snprintf(buf, PAGE_SIZE, "%s\n", all_strbuff);
END:
	if (all_strbuff) {
		kvfree(all_strbuff);
		all_strbuff = NULL;
	}
	fts_enableInterrupt();
	return count;
}

static ssize_t fts_ss_raw_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int res = 0, count = 0, j = 0, sense_node = 0, force_node = 0, pos = 0,
	    last_pos = 0;
	char *all_strbuff = NULL;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	SelfSenseFrame frameSS;

	res = fts_disableInterrupt();
	if (res < OK)
		goto END;
	all_strbuff = kvmalloc(PAGE_SIZE * 4, GFP_KERNEL);
	if (!all_strbuff) {
		logError(1, "%s %s alloc all_strbuff fail\n", tag, __func__);
		goto END;
	} else
		memset(all_strbuff, 0, PAGE_SIZE);
	setScanMode(SCAN_MODE_ACTIVE, 0x01);
	mdelay(WAIT_FOR_FRESH_FRAMES);
	setScanMode(SCAN_MODE_ACTIVE, 0x00);
	mdelay(WAIT_AFTER_SENSEOFF);
	flushFIFO();
	res = getSSFrame3(SS_RAW, &frameSS);

	fts_mode_handler(info, 1);
	sense_node = frameSS.header.sense_node;
	force_node = frameSS.header.force_node;
	pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
			"SsTouchRaw,%2d,%2d\n", force_node, sense_node);
	last_pos = pos;
	if (res >= OK) {
		pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
				"SS force frame\n ,");
		last_pos = pos;

		for (j = 0; j < frameSS.header.force_node - 1; j++) {
			pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
					"%04d,", frameSS.force_data[j]);
			last_pos = pos;
		}

		if (j == frameSS.header.force_node - 1) {
			pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
					"%04d\n", frameSS.force_data[j]);
			last_pos = pos;
		}

		pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
				"SS sense frame\n ,");
		last_pos = pos;

		for (j = 0; j < frameSS.header.sense_node - 1; j++) {
			pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
					"%04d,", frameSS.sense_data[j]);
			last_pos = pos;
		}

		if (j == frameSS.header.sense_node - 1) {
			pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
					"%04d\n", frameSS.sense_data[j]);
			last_pos = pos;
		}

		if (frameSS.force_data) {
			kfree(frameSS.force_data);
			frameSS.force_data = NULL;
		}
		if (frameSS.sense_data) {
			kfree(frameSS.sense_data);
			frameSS.sense_data = NULL;
		}
	}

	count = snprintf(buf, PAGE_SIZE, "%s\n", all_strbuff);
	kvfree(all_strbuff);
	all_strbuff = NULL;
END:
	fts_enableInterrupt();
	return count;
}

static ssize_t fts_strength_frame_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	MutualSenseFrame frame;
	int res = 0, count = 0, j = 0, size = 0;
	char *all_strbuff = NULL;
	char buff[CMD_STR_LEN] = { 0 };
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	frame.node_data = NULL;

	res = fts_disableInterrupt();
	if (res < OK)
		goto END;

	res = getMSFrame3(MS_STRENGTH, &frame);

	if (res < OK) {
		logError(1, "%s %s: could not get the frame! ERROR %08X \n",
			 tag, __func__, res);
		goto END;
	}
	size = (res * 5) + 11;

	/*
	   flushFIFO();
	 */
	fts_mode_handler(info, 1);
	all_strbuff = (char *)kmalloc(size * sizeof(char), GFP_KERNEL);

	if (all_strbuff != NULL) {
		memset(all_strbuff, 0, size);
		snprintf(all_strbuff, size, "ms_differ\n");
		if (res >= OK) {
			for (j = 0; j < frame.node_data_size; j++) {
				if ((j + 1) % frame.header.sense_node)
					snprintf(buff, sizeof(buff), "%4d,",
						 frame.node_data[j]);
				else
					snprintf(buff, sizeof(buff), "%4d\n",
						 frame.node_data[j]);

				strlcat(all_strbuff, buff, size);
			}

			kfree(frame.node_data);
			frame.node_data = NULL;
		}

		count = snprintf(buf, PAGE_SIZE, "%s\n", all_strbuff);
		kfree(all_strbuff);
	} else {
		logError(1,
			 "%s %s: Unable to allocate all_strbuff! ERROR %08X\n",
			 tag, __func__, ERROR_ALLOC);
	}

END:
	fts_enableInterrupt();

	return count;
}

#define CRC32_POLYNOMIAL 0xE89061DB

/***********************************************************************************
 * @brief Return CRC value of data
 * @param s32_message Pointer to message
 * @param s32_len Length of message in word (4-byte)
 * @return int32_t CRC value
 ***********************************************************************************/

static inline int32_t thp_crc32_check(int s32_message[], int s32_len)
{
	int i;
	int j;
	int k;
	int s32_remainder;
	u8 u8_byteData;

	s32_remainder = 0UL;

	for (i = 0; i < s32_len; i++) {
		for (j = 3; j >= 0; j--) {
			/*Get the correct byte ordering*/
			u8_byteData = s32_message[i] >> (j * 8);
			/*Bring the next byte into the remainder*/
			s32_remainder ^= (u8_byteData << 24);
			/*Perform modulo-2 division, a bit at a time*/
			for (k = 8; k > 0; --k) {
				/*Try to divide the current data bit*/
				if (s32_remainder & (1UL << 31)) {
					s32_remainder = (s32_remainder << 1) ^
							CRC32_POLYNOMIAL;
				} else {
					s32_remainder = (s32_remainder << 1);
				}
			}
		}
	}

	return s32_remainder;
}

static const char *fts_get_config(struct fts_ts_info *info);

int fts_enable_touch_delta(bool en)
{
	if (en)
		fts_info->enable_touch_delta = true;
	else
		fts_info->enable_touch_delta = false;
	logError(1, "%s %s enable touch delta:%d\n", tag, __func__,
		 fts_info->enable_touch_delta);
	return 0;
}

int fts_hover_auto_tune(struct fts_ts_info *info)
{
	int res = OK;
	u8 sett[2];
	logError(0, "%s start...\n", tag, __func__);

	fts_disableInterrupt();

	sett[0] = 0x02;
	sett[1] = 0x00;
	res = writeSysCmd(SYS_CMD_SPECIAL_TUNING, sett, 2);
	if (res < OK) {
		logError(
			1,
			"%s fts_hover_autotune Ioffset tuning 02 00 failed ERROR %08X\n",
			tag, (res | ERROR_PROD_TEST_INITIALIZATION));
		return res | ERROR_PROD_TEST_INITIALIZATION;
	}
	sett[0] = 0x00;
	sett[1] = 0x01;
	res = writeSysCmd(SYS_CMD_CX_TUNING, sett, 2);
	if (res < OK) {
		logError(
			1,
			"%s fts_hover_autotune autotune hover 00 01 failed ERROR %08X\n",
			tag, (res | ERROR_PROD_TEST_INITIALIZATION));
		return res | ERROR_PROD_TEST_INITIALIZATION;
	}
	sett[0] = 0x06;
	res = writeSysCmd(SYS_CMD_SAVE_FLASH, sett, 1);
	if (res < OK) {
		logError(
			1,
			"%s fts_hover_autotune save flash 06  failed ERROR %08X\n",
			tag, (res | ERROR_PROD_TEST_INITIALIZATION));
		return res | ERROR_PROD_TEST_INITIALIZATION;
	}
	logError(0, "%s end...\n", tag, __func__);

	fts_enableInterrupt();

	return res;
}

static ssize_t fts_hover_autotune_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);
	int on;
	int ret = 0;

	sscanf(buf, "%u", &on);
	logError(1, " %s %s\n", tag, __func__);
	if (on)
		ret = fts_hover_auto_tune(info);
	if (ret < OK)
		return -EIO;

	return count;
}

static ssize_t fts_hover_raw_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int res = 0, count = 0, j = 0, sense_node = 0, force_node = 0, pos = 0,
	    last_pos = 0;
	char *all_strbuff = NULL;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	SelfSenseFrame frameSS;
	TotSelfSenseData ssHoverCompData;
	u8 hover_cnt[4] = { 0xa8, 0x0b, 0x01, 0x00 };

	res = fts_disableInterrupt();
	if (res < OK)
		goto END;
	all_strbuff = kvmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!all_strbuff) {
		logError(1, "%s %s alloc all_strbuff fail\n", tag, __func__);
		goto END;
	} else {
		memset(all_strbuff, 0, PAGE_SIZE);
	}
	res = fts_write_dma_safe(hover_cnt, sizeof(hover_cnt));
	if (res != OK) {
		logError(1, "%s hover clear count ERROR = %d\n", tag, res);
		goto END;
	}

	setScanMode(SCAN_MODE_ACTIVE, 0xFF);

	res = getSSFrame3(SS_HVR_RAW, &frameSS);

	sense_node = frameSS.header.sense_node;
	force_node = frameSS.header.force_node;
	pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
			"SsHoverTouchRaw,%2d,%2d\n", force_node, sense_node);
	last_pos = pos;

	pos += snprintf(all_strbuff + last_pos, PAGE_SIZE, "TxRaw\n");
	last_pos = pos;

	if (res >= OK) {
		pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
				"SS Hover force frame\n ,");
		last_pos = pos;

		for (j = 0; j < frameSS.header.force_node - 1; j++) {
			pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
					"%04d,", frameSS.force_data[j]);
			last_pos = pos;
		}

		if (j == frameSS.header.force_node - 1) {
			pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
					"%04d\n", frameSS.force_data[j]);
			last_pos = pos;
		}

		pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
				"SS Hover sense frame\n ,");
		last_pos = pos;

		for (j = 0; j < frameSS.header.sense_node - 1; j++) {
			pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
					"%04d,", frameSS.sense_data[j]);
			last_pos = pos;
		}

		if (j == frameSS.header.sense_node - 1) {
			pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
					"%04d\n", frameSS.sense_data[j]);
			last_pos = pos;
		}

		if (frameSS.force_data) {
			kfree(frameSS.force_data);
			frameSS.force_data = NULL;
		}
		if (frameSS.sense_data) {
			kfree(frameSS.sense_data);
			frameSS.sense_data = NULL;
		}
	}

	res = getSSFrame3(SS_HVR_FILTER, &frameSS);

	sense_node = frameSS.header.sense_node;
	force_node = frameSS.header.force_node;

	pos += snprintf(all_strbuff + last_pos, PAGE_SIZE, "TxFilter\n");
	last_pos = pos;

	if (res >= OK) {
		pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
				"SS Hover force frame\n ,");
		last_pos = pos;

		for (j = 0; j < frameSS.header.force_node - 1; j++) {
			pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
					"%04d,", frameSS.force_data[j]);
			last_pos = pos;
		}

		if (j == frameSS.header.force_node - 1) {
			pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
					"%04d\n", frameSS.force_data[j]);
			last_pos = pos;
		}

		pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
				"SS Hover sense frame\n ,");
		last_pos = pos;

		for (j = 0; j < frameSS.header.sense_node - 1; j++) {
			pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
					"%04d,", frameSS.sense_data[j]);
			last_pos = pos;
		}

		if (j == frameSS.header.sense_node - 1) {
			pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
					"%04d\n", frameSS.sense_data[j]);
			last_pos = pos;
		}

		if (frameSS.force_data) {
			kfree(frameSS.force_data);
			frameSS.force_data = NULL;
		}
		if (frameSS.sense_data) {
			kfree(frameSS.sense_data);
			frameSS.sense_data = NULL;
		}
	}

	res = getSSFrame3(SS_HVR_BASELINE, &frameSS);

	sense_node = frameSS.header.sense_node;
	force_node = frameSS.header.force_node;

	pos += snprintf(all_strbuff + last_pos, PAGE_SIZE, "TxBaseline\n");
	last_pos = pos;

	if (res >= OK) {
		pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
				"SS Hover force frame\n ,");
		last_pos = pos;

		for (j = 0; j < frameSS.header.force_node - 1; j++) {
			pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
					"%04d,", frameSS.force_data[j]);
			last_pos = pos;
		}

		if (j == frameSS.header.force_node - 1) {
			pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
					"%04d\n", frameSS.force_data[j]);
			last_pos = pos;
		}

		pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
				"SS Hover sense frame\n ,");
		last_pos = pos;

		for (j = 0; j < frameSS.header.sense_node - 1; j++) {
			pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
					"%04d,", frameSS.sense_data[j]);
			last_pos = pos;
		}

		if (j == frameSS.header.sense_node - 1) {
			pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
					"%04d\n", frameSS.sense_data[j]);
			last_pos = pos;
		}

		if (frameSS.force_data) {
			kfree(frameSS.force_data);
			frameSS.force_data = NULL;
		}

		if (frameSS.sense_data) {
			kfree(frameSS.sense_data);
			frameSS.sense_data = NULL;
		}
	}
	pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
			"SS Hover IX Data\n ,");
	last_pos = pos;

	res = readTotSelfSenseCompensationData(
		STAPI_HOST_DATA_ID_PANEL_CX_SS_HVR, &ssHoverCompData);

	pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
			"SS Hover IX force frame\n ,");
	last_pos = pos;
	for (j = 0; j < ssHoverCompData.header.force_node - 1; j++) {
		pos += snprintf(all_strbuff + last_pos, PAGE_SIZE, "%04d,",
				ssHoverCompData.ix_fm[j]);
		last_pos = pos;
	}

	pos += snprintf(all_strbuff + last_pos, PAGE_SIZE,
			"\nSS Hover IX sense frame\n ,");
	last_pos = pos;

	for (j = 0; j < ssHoverCompData.header.sense_node - 1; j++) {
		pos += snprintf(all_strbuff + last_pos, PAGE_SIZE, "%04d,",
				ssHoverCompData.ix_sn[j]);
		last_pos = pos;
	}

	if (ssHoverCompData.ix_fm != NULL)
		kfree(ssHoverCompData.ix_fm);
	if (ssHoverCompData.ix_sn != NULL)
		kfree(ssHoverCompData.ix_sn);
	if (ssHoverCompData.cx_fm != NULL)
		kfree(ssHoverCompData.cx_fm);
	if (ssHoverCompData.cx_sn != NULL)
		kfree(ssHoverCompData.cx_sn);

	count = snprintf(buf, PAGE_SIZE, "%s\n", all_strbuff);
	kvfree(all_strbuff);
	all_strbuff = NULL;
END:
	fts_mode_handler(info, 1);
	fts_enableInterrupt();
	return count;
}

static ssize_t fts_ellipse_data_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int res;
	SelfSenseFrame frameSS;
	int force_node;
	int sense_node;

	logError(1, "%s %s\n", tag, __func__);
	res = fts_disableInterrupt();
	if (res < OK) {
		logError(1, "%s %s disable irq error\n", tag, __func__);
	}
	setScanMode(SCAN_MODE_ACTIVE, 0x01);
	mdelay(WAIT_FOR_FRESH_FRAMES);
	setScanMode(SCAN_MODE_ACTIVE, 0x00);
	mdelay(WAIT_AFTER_SENSEOFF);
	flushFIFO();
	res = getSSFrame3(SS_RAW, &frameSS);
	if (res < OK) {
		logError(1,
			 "%s Error while taking the SS frame... ERROR %08X \n",
			 tag, res);
		fts_enableInterrupt();
		return 0;
	}
	force_node = frameSS.header.force_node;
	sense_node = frameSS.header.sense_node;
	fts_mode_handler(fts_info, 1);
	fts_enableInterrupt();

	return snprintf(buf, PAGE_SIZE, "%d %d %d %d %d\n",
			frameSS.force_data[force_node / 4],
			frameSS.force_data[force_node * 3 / 4],
			frameSS.sense_data[sense_node / 4],
			frameSS.sense_data[sense_node / 2],
			frameSS.sense_data[sense_node * 3 / 4]);
}

#ifdef CONFIG_SECURE_TOUCH
static void fts_secure_touch_notify(struct fts_ts_info *info)
{
	/*might sleep*/
	sysfs_notify(&info->dev->kobj, NULL, "secure_touch");
	logError(1, "%s %s SECURE_NOTIFY:notify secure_touch\n", tag, __func__);
}

static int fts_secure_stop(struct fts_ts_info *info, bool block)
{
	struct fts_secure_info *scr_info = info->secure_info;

	logError(1, "%s %s SECURE_STOP: block = %d\n", tag, __func__,
		 (int)block);
	if (atomic_read(&scr_info->st_enabled) == 0) {
		logError(1, "%s %s secure touch is already disabled\n", tag,
			 __func__);
		return OK;
	}

	atomic_set(&scr_info->st_pending_irqs, -1);
	fts_secure_touch_notify(info);
	if (block) {
		if (wait_for_completion_interruptible(
			    &scr_info->st_powerdown) == -ERESTARTSYS) {
			logError(
				1,
				"%s %s SECURE_STOP:st_powerdown be interrupted\n",
				tag, __func__);
		} else {
			logError(
				1,
				"%s %s SECURE_STOP:st_powerdown be completed\n",
				tag, __func__);
		}
	}
	return OK;
}

static void fts_secure_work(struct fts_secure_info *scr_info)
{
	struct fts_ts_info *info = (struct fts_ts_info *)scr_info->fts_info;

	fts_secure_touch_notify(info);
	atomic_set(&scr_info->st_1st_complete, 1);
	if (wait_for_completion_interruptible(&scr_info->st_irq_processed) ==
	    -ERESTARTSYS) {
		logError(
			1,
			"%s %s SECURE_FILTER:st_irq_processed be interrupted\n",
			tag, __func__);
	} else {
		logError(1,
			 "%s %s SECURE_FILTER:st_irq_processed be completed\n",
			 tag, __func__);
	}

	fts_enableInterrupt();
	logError(1, "%s %s SECURE_FILTER:enable irq\n", tag, __func__);
}

static void fts_palm_store_delay(struct fts_secure_info *scr_info)
{
	struct fts_ts_info *info = scr_info->fts_info;

	logError(1, "%s %s IN", tag, __func__);
	fts_palm_sensor_cmd(scr_info->scr_delay.palm_value);
	logError(1, "%s %s OUT", tag, __func__);
}

static void fts_flush_delay_task(struct fts_secure_info *scr_info)
{
	if (scr_info->scr_delay.palm_pending) {
		fts_palm_store_delay(scr_info);
		scr_info->scr_delay.palm_pending = false;
	}
}

static int fts_secure_filter_interrupt(struct fts_ts_info *info)
{
	struct fts_secure_info *scr_info = info->secure_info;

	/*inited and enable first*/
	if (!scr_info->secure_inited ||
	    atomic_read(&scr_info->st_enabled) == 0) {
		return -EPERM;
	}

	fts_disableInterruptNoSync();
	logError(1, "%s %s SECURE_FILTER:disable irq\n", tag, __func__);
	/*check and change irq pending state
	 *change irq pending here, secure_touch_show, secure_touch_enable_store
	 *completion st_irq_processed at secure_touch_show, secure_touch_enable_stroe
	 */
	logError(1, "%s %s SECURE_FILTER:st_pending_irqs = %d\n", tag, __func__,
		 atomic_read(&scr_info->st_pending_irqs));
	if (atomic_cmpxchg(&scr_info->st_pending_irqs, 0, 1) == 0) {
		fts_secure_work(scr_info);
		logError(1, "%s %s SECURE_FILTER:secure_work return\n", tag,
			 __func__);
	}

	return 0;
}

static ssize_t fts_secure_touch_enable_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);
	struct fts_secure_info *scr_info = info->secure_info;

	logError(1, "%s %s SECURE_TOUCH_ENABLE[R]:st_enabled = %d\n", tag,
		 __func__, atomic_read(&scr_info->st_enabled));
	return scnprintf(buf, PAGE_SIZE, "%d",
			 atomic_read(&scr_info->st_enabled));
}

/* 	echo 0 > secure_touch_enable to disable secure touch
 * 	echo 1 > secure_touch_enable to enable secure touch
 */
static ssize_t fts_secure_touch_enable_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	int ret;
	unsigned long value;
	struct fts_ts_info *info = dev_get_drvdata(dev);
	struct fts_secure_info *scr_info = info->secure_info;

	atomic_set(&scr_info->st_1st_complete, 0);
	logError(1, "%s %s SECURE_TOUCH_ENABLE[W]:st_1st_complete=0\n", tag,
		 __func__);
	logError(1, "%s %s SECURE_TOUCH_ENABLE[W]:parse parameter\n", tag,
		 __func__);
	/*check and get cmd*/
	if (count > 2)
		return -EINVAL;
	ret = kstrtoul(buf, 10, &value);
	if (ret != 0)
		return ret;

	if (!scr_info->secure_inited)
		return -EIO;

	ret = count;

	logError(1, "%s %s SECURE_TOUCH_ENABLE[W]:st_enabled = %d\n", tag,
		 __func__, value);
	switch (value) {
	case 0:
		if (atomic_read(&scr_info->st_enabled) == 0) {
			logError(1, "%s %s secure touch is already disabled\n",
				 tag, __func__);
			return ret;
		}
		mutex_lock(&scr_info->palm_lock);
		atomic_set(&scr_info->st_enabled, 0);
		fts_secure_touch_notify(info);
		complete(&scr_info->st_irq_processed);
		fts_event_handler(info->client->irq, info);
		complete(&scr_info->st_powerdown);
		fts_flush_delay_task(scr_info);
		mutex_unlock(&scr_info->palm_lock);
		logError(
			1,
			"%s %s SECURE_TOUCH_ENABLE[W]:disable secure touch successful\n",
			tag, __func__);
		break;
	case 1:
		if (atomic_read(&scr_info->st_enabled) == 1) {
			logError(1, "%s %s secure touch is already enabled\n",
				 tag, __func__);
			return ret;
		}
		mutex_lock(&scr_info->palm_lock);
		/*wait until finish process all normal irq*/
		synchronize_irq(info->client->irq);

		/*enable secure touch*/
		reinit_completion(&scr_info->st_powerdown);
		reinit_completion(&scr_info->st_irq_processed);
		atomic_set(&scr_info->st_pending_irqs, 0);
		atomic_set(&scr_info->st_enabled, 1);
		mutex_unlock(&scr_info->palm_lock);
		logError(
			1,
			"%s %s SECURE_TOUCH_ENABLE[W]:enable secure touch successful\n",
			tag, __func__);
		break;
	default:
		logError(1, "%s %s %d in secure_touch_enable is not support\n",
			 tag, __func__, value);
		break;
	}
	return ret;
}

static ssize_t fts_secure_touch_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);
	struct fts_secure_info *scr_info = info->secure_info;
	int value = 0;

	logError(1, "%s %s SECURE_TOUCH[R]:st_1st_complete = %d\n", tag,
		 __func__, atomic_read(&scr_info->st_1st_complete));
	logError(1, "%s %s SECURE_TOUCH[R]:st_pending_irqs = %d\n", tag,
		 __func__, atomic_read(&scr_info->st_pending_irqs));

	if (atomic_read(&scr_info->st_enabled) == 0) {
		return -EBADF;
	}

	if (atomic_cmpxchg(&scr_info->st_pending_irqs, -1, 0) == -1)
		return -EINVAL;

	if (atomic_cmpxchg(&scr_info->st_pending_irqs, 1, 0) == 1) {
		value = 1;
	} else if (atomic_cmpxchg(&scr_info->st_1st_complete, 1, 0) == 1) {
		complete(&scr_info->st_irq_processed);
		logError(1,
			 "%s %s SECURE_TOUCH[R]:comlpetion st_irq_processed\n",
			 tag, __func__);
	}
	return scnprintf(buf, PAGE_SIZE, "%d", value);
}
#endif

static ssize_t fts_cmdfifo_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int cmd_addr = 0x001093DC;
	int node_data_size = 256;
	int ret;
	int pos = 0;
	int i;
	int j;
	int count = 0;
	u8 *read_buf = NULL;
	u8 *all_strbuff = NULL;
	u8 cmd[6] = { FTS_CMD_HW_REG_W, 0x20, 0x00, 0x00, 0x68, 0x08 };

	pr_debug("%s %s: enter\n", tag, __func__);

	read_buf = (u8 *)kzalloc(node_data_size, GFP_KERNEL);
	if (!read_buf) {
		pr_err("%s %s: memory alloc fail\n", tag, __func__);
		goto end;
	}
	all_strbuff = kvmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!all_strbuff) {
		logError(1, "%s %s alloc all_strbuff fail\n", tag, __func__);
		goto end;
	} else {
		memset(all_strbuff, 0, PAGE_SIZE);
	}

	if (fts_write_dma_safe(cmd, ARRAY_SIZE(cmd)) < OK) {
		logError(1, "%s memory sleep fail\n", tag, ERROR_BUS_W);
		goto end;
	}

	ret = fts_writeReadU8UX(FTS_CMD_HW_REG_R, ADDR_SIZE_HW_REG, cmd_addr,
				read_buf, node_data_size, DUMMY_FRAMEBUFFER);
	if (ret < OK) {
		logError(1, "%s %s: error while reading thp frame %08X\n", tag,
			 __func__, ret);
		goto end;
	}

	for (i = 0; i < 16; i++) {
		for (j = 0; j < 16; j++) {
			pos += snprintf(all_strbuff + pos, PAGE_SIZE, "%02x ",
					read_buf[i * 16 + j]);
		}
		all_strbuff[pos++] = '\n';
	}
	count = snprintf(buf, PAGE_SIZE, "%s\n", all_strbuff);

end:
	if (all_strbuff)
		kvfree(all_strbuff);
	if (read_buf)
		kfree(read_buf);

	all_strbuff = NULL;
	read_buf = NULL;

	return count;
}

static DEVICE_ATTR(fts_lockdown, (S_IRUGO | S_IWUSR | S_IWGRP),
		   fts_lockdown_show, fts_lockdown_store);
static DEVICE_ATTR(fwupdate, (S_IRUGO | S_IWUSR | S_IWGRP), fts_fwupdate_show,
		   fts_fwupdate_store);
static DEVICE_ATTR(ms_strength, (S_IRUGO), fts_strength_frame_show, NULL);
static DEVICE_ATTR(lockdown_info, (S_IRUGO), fts_lockdown_info_show, NULL);
static DEVICE_ATTR(appid, (S_IRUGO), fts_appid_show, NULL);
static DEVICE_ATTR(mode_active, (S_IRUGO), fts_mode_active_show, NULL);
static DEVICE_ATTR(fw_file_test, (S_IRUGO), fts_fw_test_show, NULL);
static DEVICE_ATTR(selftest_info, (S_IRUGO), fts_selftest_info_show, NULL);
static DEVICE_ATTR(ms_raw, (S_IRUGO), fts_ms_raw_show, NULL);
static DEVICE_ATTR(mutual_raw_ito, (S_IRUGO), fts_mutual_raw_ito_show, NULL);
static DEVICE_ATTR(ss_raw, (S_IRUGO), fts_ss_raw_show, NULL);
static DEVICE_ATTR(ms_cx_total, (S_IRUGO), fts_ms_cx_total_show, NULL);
static DEVICE_ATTR(ms_cx2_lp, (S_IRUGO), fts_ms_cx2_lp_show, NULL);
static DEVICE_ATTR(ms_cx2_lp_total, (S_IRUGO), fts_ms_cx2_lp_total_show, NULL);
static DEVICE_ATTR(ss_ix_total, (S_IRUGO), fts_ss_ix_total_show, NULL);
static DEVICE_ATTR(ss_hover, (S_IRUGO), fts_hover_raw_show, NULL);
static DEVICE_ATTR(stm_fts_cmd, (S_IRUGO | S_IWUSR | S_IWGRP), stm_fts_cmd_show,
		   stm_fts_cmd_store);

static DEVICE_ATTR(hover_tune, (S_IRUGO | S_IWUSR | S_IWGRP), NULL,
		   fts_hover_autotune_store);

static DEVICE_ATTR(cmd_fifo, (S_IRUGO | S_IWUSR | S_IWGRP), fts_cmdfifo_show,
		   NULL);
static struct attribute *fts_attr_group[] = {
	&dev_attr_fwupdate.attr,
	&dev_attr_appid.attr,
	&dev_attr_mode_active.attr,
	&dev_attr_fw_file_test.attr,
	&dev_attr_stm_fts_cmd.attr,
	&dev_attr_fts_lockdown.attr,
	&dev_attr_lockdown_info.attr,
	&dev_attr_selftest_info.attr,
	&dev_attr_ms_raw.attr,
	&dev_attr_ss_raw.attr,
	&dev_attr_mutual_raw_ito.attr,
	&dev_attr_ms_cx_total.attr,
	&dev_attr_ms_cx2_lp.attr,
	&dev_attr_ms_cx2_lp_total.attr,
	&dev_attr_ss_ix_total.attr,
	&dev_attr_ms_strength.attr,
	&dev_attr_ss_hover.attr,
	&dev_attr_hover_tune.attr,
	&dev_attr_cmd_fifo.attr,
	NULL,
};

static DEVICE_ATTR(ellipse_data, (S_IRUGO), fts_ellipse_data_show, NULL);

#ifdef CONFIG_SECURE_TOUCH
DEVICE_ATTR(secure_touch_enable, (S_IRUGO | S_IWUSR | S_IWGRP),
	    fts_secure_touch_enable_show, fts_secure_touch_enable_store);
DEVICE_ATTR(secure_touch, (S_IRUGO | S_IWUSR | S_IWGRP), fts_secure_touch_show,
	    NULL);
#endif
/**@}*/
/**@}*/

/**
 * @defgroup isr Interrupt Service Routine (Event Handler)
 * The most important part of the driver is the ISR (Interrupt Service Routine) called also as Event Handler \n
 * As soon as the interrupt pin goes low, fts_interrupt_handler() is called and the chain to read and parse the event read from the FIFO start.\n
 * For any different kind of EVT_ID there is a specific event handler which will take the correct action to report the proper info to the host. \n
 * The most important events are the one related to touch informations, status update or user report.
 * @{
 */

/**
 * Report to the linux input system the pressure and release of a button handling concurrency
 * @param info pointer to fts_ts_info which contains info about the device and its hw setup
 * @param key_code	button value
 */
void fts_input_report_key(struct fts_ts_info *info, int key_code)
{
	mutex_lock(&info->input_report_mutex);
	input_report_key(info->input_dev, key_code, 1);
	input_sync(info->input_dev);
	input_report_key(info->input_dev, key_code, 0);
	input_sync(info->input_dev);
	mutex_unlock(&info->input_report_mutex);
}

/**
* Event Handler for no events (EVT_ID_NOEVENT)
*/
static void fts_nop_event_handler(struct fts_ts_info *info,
				  unsigned char *event)
{
	logError(
		1,
		"%s %s Doing nothing for event = %02X %02X %02X %02X %02X %02X %02X %02X\n",
		tag, __func__, event[0], event[1], event[2], event[3], event[4],
		event[5], event[6], event[7]);
}

/**
* Event handler for enter and motion events (EVT_ID_ENTER_POINT, EVT_ID_MOTION_POINT )
* report to the linux input system touches with their coordinated and additional informations
*/
static void fts_enter_pointer_event_handler(struct fts_ts_info *info,
					    unsigned char *event)
{
	unsigned char touchId;
	unsigned int touch_condition = 1, tool = MT_TOOL_FINGER;
	int x, y, distance, angle, major, minor;
	u8 touchType, eventid;
	const struct fts_hw_platform_data *bdata = fts_info->board;
	if (!info->resume_bit)
		goto no_report;
	if (info->sensor_sleep) {
		logError(1, "%s %s sensor sleep, skip touch down event\n", tag,
			 __func__);
		return;
	}

	eventid = event[0];
	touchType = event[1] & 0x0F;
	touchId = (event[1] & 0xF0) >> 4;

	if (!bdata->support_super_resolution) {
		x = (((int)event[3] & 0x0F) << 8) | (event[2]);
		y = ((int)event[4] << 4) | ((event[3] & 0xF0) >> 4);
	} else {
		x = (((int)event[3]) << 8) | (event[2]);
		y = (((int)event[5]) << 8) | (event[4]);
	}
	distance = 0;
	angle = (signed char)event[5];
	major = (((event[0] & 0x0C) << 2) | ((event[6] & 0xF0) >> 4));
	minor = (((event[7] & 0xC0) >> 2) | (event[6] & 0x0F));
	if ((event[7] & 0x1f) >= 4) {
		logError(1, "%s %s %02X %02X %02X %02X %02X %02X %02X %02X\n",
			 tag, __func__, event[0], event[1], event[2], event[3],
			 event[4], event[5], event[6], event[7]);
	}
	if (x >= info->board->x_max)
		x = info->board->x_max;

	if (y >= info->board->y_max)
		y = info->board->y_max;
	if (info->board->swap_x)
		x = info->board->x_max - x;
	if (info->board->swap_y)
		y = info->board->y_max - y;
	input_mt_slot(info->input_dev, touchId);
	switch (touchType) {
	case TOUCH_TYPE_FINGER:
		/*logError(0, "%s  %s : It is a finger!\n",tag,__func__); */
	case TOUCH_TYPE_GLOVE:
		/*logError(0, "%s  %s : It is a glove!\n",tag,__func__); */
	case TOUCH_TYPE_PALM:
		/*logError(0, "%s  %s : It is a palm!\n",tag,__func__); */
		tool = MT_TOOL_FINGER;
		touch_condition = 1;
		if (test_bit(touchId, &info->temp_touch_id)) {
			logError(
				1,
				"%s  %s : NOTE: same slot in one irq, slot = %d !\n",
				tag, __func__, touchId);
			input_sync(info->input_dev);
		}
		__set_bit(touchId, &info->touch_id);
		__set_bit(touchId, &info->temp_touch_id);
		break;

	case TOUCH_TYPE_HOVER:
		tool = MT_TOOL_FINGER;
		touch_condition = 0;
		__set_bit(touchId, &info->touch_id);
		__set_bit(touchId, &info->temp_touch_id);
		distance = DISTANCE_MAX;
		break;

	case TOUCH_TYPE_INVALID:
	default:
		logError(1, "%s  %s : Invalid touch type = %d ! No Report...\n",
			 tag, __func__, touchType);
		goto no_report;
	}

	input_mt_report_slot_state(info->input_dev, tool, 1);
	input_report_key(info->input_dev, BTN_TOUCH, touch_condition);
	if (touch_condition)
		input_report_key(info->input_dev, BTN_TOOL_FINGER, 1);

	input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
	if ((info->last_x[touchId] != x) || (info->last_y[touchId] != y)) {
		input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR,
				 major * 16);
		input_report_abs(info->input_dev, ABS_MT_TOUCH_MINOR,
				 minor * 16);
		input_report_abs(info->input_dev, ABS_MT_ORIENTATION, angle);
		input_report_abs(info->input_dev, ABS_MT_DISTANCE, distance);
	}
	info->last_x[touchId] = x;
	info->last_y[touchId] = y;
	dev_dbg(info->dev,
		"%s  %s :  Event 0x%02x - ID[%d], (x, y, major, minor, angle) = (%3d, %3d, %3d, %3d, %3d) type = %d\n",
		tag, __func__, *event, touchId, x, y, major, minor, angle,
		touchType);
	if (eventid == 0x13) {
		logError(1, "%s  %s :  Event 0x%02x - Press ID[%d] type = %d\n",
			 tag, __func__, event[0], touchId, touchType);
	}

no_report:
	return;
}

/**
* Event handler for leave event (EVT_ID_LEAVE_POINT )
* Report to the linux input system that one touch left the display
*/
static void fts_leave_pointer_event_handler(struct fts_ts_info *info,
					    unsigned char *event)
{
	unsigned char touchId = 0;
	unsigned int tool = MT_TOOL_FINGER;
	unsigned int touch_condition = 0;
	u8 touchType;

	touchType = event[1] & 0x0F;
	touchId = (event[1] & 0xF0) >> 4;

	input_mt_slot(info->input_dev, touchId);
	switch (touchType) {
	case TOUCH_TYPE_FINGER:
		/*logError(0, "%s  %s : It is a finger!\n",tag,__func__); */
	case TOUCH_TYPE_GLOVE:
		/*logError(0, "%s  %s : It is a glove!\n",tag,__func__); */
	case TOUCH_TYPE_PALM:
		/*logError(0, "%s  %s : It is a palm!\n",tag,__func__); */
		tool = MT_TOOL_FINGER;
		touch_condition = 0;
		if (test_bit(touchId, &info->temp_touch_id)) {
			logError(
				1,
				"%s  %s : NOTE: down and up in one irq, slot = %d !\n",
				tag, __func__, touchId);
			input_sync(info->input_dev);
		}
		__clear_bit(touchId, &info->touch_id);
		__clear_bit(touchId, &info->temp_touch_id);
		break;
	case TOUCH_TYPE_HOVER:
		tool = MT_TOOL_FINGER;
		touch_condition = 1;
		__clear_bit(touchId, &info->touch_id);
		__clear_bit(touchId, &info->temp_touch_id);
		break;

	case TOUCH_TYPE_INVALID:
	default:
		logError(1, "%s  %s : Invalid touch type = %d ! No Report...\n",
			 tag, __func__, touchType);
		return;
	}
	input_mt_report_slot_state(info->input_dev, tool, 0);
	if (info->touch_id == 0) {
		input_report_key(info->input_dev, BTN_TOUCH, touch_condition);
		if (!touch_condition)
			input_report_key(info->input_dev, BTN_TOOL_FINGER, 0);
	}

	info->last_x[touchId] = info->last_y[touchId] = 0;
	input_report_abs(info->input_dev, ABS_MT_TRACKING_ID, -1);
}

/* EventId : EVT_ID_MOTION_POINT */
#define fts_motion_pointer_event_handler fts_enter_pointer_event_handler

/**
* Event handler for error events (EVT_ID_ERROR)
* Handle unexpected error events implementing recovery strategy and restoring the sensing status that the IC had before the error occured
*/
static void fts_error_event_handler(struct fts_ts_info *info,
				    unsigned char *event)
{
	int error = 0;
	logError(
		1,
		"%s %s Received event %02X %02X %02X %02X %02X %02X %02X %02X\n",
		tag, __func__, event[0], event[1], event[2], event[3], event[4],
		event[5], event[6], event[7]);

	switch (event[1]) {
	case EVT_TYPE_ERROR_ESD: {
		release_all_touches(info);

		fts_chip_powercycle(info);

		error = fts_system_reset();
		error |= fts_mode_handler(info, 0);
		error |= fts_enableInterrupt();
		if (error < OK) {
			logError(1,
				 "%s %s Cannot restore the device ERROR %08X\n",
				 tag, __func__, error);
		}
	} break;
	case EVT_TYPE_ERROR_WATCHDOG: {
		dumpErrorInfo(NULL, 0);
		release_all_touches(info);
		error = fts_system_reset();
		error |= fts_mode_handler(info, 0);
		error |= fts_enableInterrupt();
		if (error < OK) {
			logError(1,
				 "%s %s Cannot reset the device ERROR %08X\n",
				 tag, __func__, error);
		}
	} break;
	}
}

/**
* Event handler for controller ready event (EVT_ID_CONTROLLER_READY)
* Handle controller events received after unexpected reset of the IC updating the resets flag and restoring the proper sensing status
*/
static void fts_controller_ready_event_handler(struct fts_ts_info *info,
					       unsigned char *event)
{
	int error;

	logError(
		1,
		"%s %s Received event %02X %02X %02X %02X %02X %02X %02X %02X\n",
		tag, __func__, event[0], event[1], event[2], event[3], event[4],
		event[5], event[6], event[7]);
	release_all_touches(info);
	setSystemResetedUp(1);
	setSystemResetedDown(1);
	error = fts_mode_handler(info, 0);
	if (error < OK) {
		logError(1,
			 "%s %s Cannot restore the device status ERROR %08X\n",
			 tag, __func__, error);
	}
}

/**
* Event handler for status events (EVT_ID_STATUS_UPDATE)
* Handle status update events
*/
static void fts_status_event_handler(struct fts_ts_info *info,
				     unsigned char *event)
{
	switch (event[1]) {
	case EVT_TYPE_STATUS_ECHO:
		logError(
			0,
			"%s %s Echo event of command = %02X %02X %02X %02X %02X %02X\n",
			tag, __func__, event[2], event[3], event[4], event[5],
			event[6], event[7]);
		break;

	case EVT_TYPE_STATUS_FORCE_CAL:
		switch (event[2]) {
		case 0x00:
			logError(
				1,
				"%s %s Continuous frame drop Force cal = %02X %02X %02X %02X %02X %02X\n",
				tag, __func__, event[2], event[3], event[4],
				event[5], event[6], event[7]);
			break;

		case 0x01:
			logError(
				1,
				"%s %s Mutual negative detect Force cal = %02X %02X %02X %02X %02X %02X\n",
				tag, __func__, event[2], event[3], event[4],
				event[5], event[6], event[7]);
			break;

		case 0x02:
			logError(
				1,
				"%s %s Mutual calib deviation Force cal = %02X %02X %02X %02X %02X %02X\n",
				tag, __func__, event[2], event[3], event[4],
				event[5], event[6], event[7]);
			break;

		case 0x11:
			logError(
				1,
				"%s %s SS negative detect Force cal = %02X %02X %02X %02X %02X %02X\n",
				tag, __func__, event[2], event[3], event[4],
				event[5], event[6], event[7]);
			break;

		case 0x12:
			logError(
				1,
				"%s %s SS negative detect Force cal in Low Power mode = %02X %02X %02X %02X %02X %02X\n",
				tag, __func__, event[2], event[3], event[4],
				event[5], event[6], event[7]);
			break;

		case 0x13:
			logError(
				1,
				"%s %s SS negative detect Force cal in Idle mode = %02X %02X %02X %02X %02X %02X\n",
				tag, __func__, event[2], event[3], event[4],
				event[5], event[6], event[7]);
			break;

		case 0x20:
			logError(
				1,
				"%s %s SS invalid Mutual Strength soft Force cal = %02X %02X %02X %02X %02X %02X\n",
				tag, __func__, event[2], event[3], event[4],
				event[5], event[6], event[7]);
			break;

		case 0x21:
			logError(
				1,
				"%s %s SS invalid Self Strength soft Force cal = %02X %02X %02X %02X %02X %02X\n",
				tag, __func__, event[2], event[3], event[4],
				event[5], event[6], event[7]);
			break;

		case 0x22:
			logError(
				1,
				"%s %s SS invalid Self Island soft Force cal = %02X %02X %02X %02X %02X %02X\n",
				tag, __func__, event[2], event[3], event[4],
				event[5], event[6], event[7]);
			break;

		case 0x30:
			logError(
				1,
				"%s %s MS invalid Mutual Strength soft Force cal = %02X %02X %02X %02X %02X %02X\n",
				tag, __func__, event[2], event[3], event[4],
				event[5], event[6], event[7]);
			break;

		case 0x31:
			logError(
				1,
				"%s %s MS invalid Self Strength soft Force cal = %02X %02X %02X %02X %02X %02X\n",
				tag, __func__, event[2], event[3], event[4],
				event[5], event[6], event[7]);
			break;

		default:
			logError(
				1,
				"%s %s Force cal = %02X %02X %02X %02X %02X %02X\n",
				tag, __func__, event[2], event[3], event[4],
				event[5], event[6], event[7]);
		}
		break;

	case EVT_TYPE_STATUS_FRAME_DROP:
		switch (event[2]) {
		case 0x01:
			logError(
				1,
				"%s %s Frame drop noisy frame = %02X %02X %02X %02X %02X %02X\n",
				tag, __func__, event[2], event[3], event[4],
				event[5], event[6], event[7]);
			break;

		case 0x02:
			logError(
				1,
				"%s %s Frame drop bad R = %02X %02X %02X %02X %02X %02X\n",
				tag, __func__, event[2], event[3], event[4],
				event[5], event[6], event[7]);
			break;

		case 0x03:
			logError(
				1,
				"%s %s Frame drop invalid processing state = %02X %02X %02X %02X %02X %02X\n",
				tag, __func__, event[2], event[3], event[4],
				event[5], event[6], event[7]);
			break;

		default:
			logError(
				1,
				"%s %s Frame drop = %02X %02X %02X %02X %02X %02X\n",
				tag, __func__, event[2], event[3], event[4],
				event[5], event[6], event[7]);
		}
		break;

	case EVT_TYPE_STATUS_SS_RAW_SAT:
		if (event[2] == 1)
			logError(
				1,
				"%s %s SS Raw Saturated = %02X %02X %02X %02X %02X %02X\n",
				tag, __func__, event[2], event[3], event[4],
				event[5], event[6], event[7]);
		else
			logError(
				1,
				"%s %s SS Raw No more Saturated = %02X %02X %02X %02X %02X %02X\n",
				tag, __func__, event[2], event[3], event[4],
				event[5], event[6], event[7]);
		break;

	case EVT_TYPE_STATUS_WATER:
		if (event[2] == 1)
			logError(
				1,
				"%s %s Enter Water mode = %02X %02X %02X %02X %02X %02X\n",
				tag, __func__, event[2], event[3], event[4],
				event[5], event[6], event[7]);
		else
			logError(
				1,
				"%s %s Exit Water mode = %02X %02X %02X %02X %02X %02X\n",
				tag, __func__, event[2], event[3], event[4],
				event[5], event[6], event[7]);
		break;
	case 0x0c:
		break;
	default:
		logError(
			1,
			"%s %s Received unhandled status event = %02X %02X %02X %02X %02X %02X %02X %02X\n",
			tag, __func__, event[0], event[1], event[2], event[3],
			event[4], event[5], event[6], event[7]);
		break;
	}
}

static void fts_oval_event_handler(struct fts_ts_info *info,
				   unsigned char *event)
{
	info->width_major = event[2] << 8 | event[3];
	info->width_minor = event[4] << 8 | event[5];
	info->orientation = (signed char)event[6];
	/*logError(0, "%s %s info->width_major:%d,info->width_minor:%d,orieatiation:%d\n", tag, __func__, info->width_major, info->width_minor, info->orientation);*/
	return;
}

/**
 * Event handler for user report events (EVT_ID_USER_REPORT)
 * Handle user events reported by the FW due to some interaction triggered by an external user (press keys, perform gestures, etc.)
 */
static void fts_user_report_event_handler(struct fts_ts_info *info,
					  unsigned char *event)
{
	switch (event[1]) {
	case EVT_TYPE_USER_PROXIMITY:
		if (event[2] == 0) {
			logError(1, "%s %s No proximity!\n", tag, __func__);
		} else {
			logError(1, "%s %s Proximity Detected!\n", tag,
				 __func__);
		}
		break;
	case EVT_TYPE_USER_OVAL:
		fts_oval_event_handler(info, event);
		break;
	default:
		logError(
			1,
			"%s %s Received unhandled user report event = %02X %02X %02X %02X %02X %02X %02X %02X\n",
			tag, __func__, event[0], event[1], event[2], event[3],
			event[4], event[5], event[6], event[7]);
		break;
	}
}

static void fts_ts_sleep_work(struct work_struct *work)
{
	struct fts_ts_info *info =
		container_of(work, struct fts_ts_info, sleep_work);
	int error = 0, count = 0;
	unsigned char regAdd = FIFO_CMD_READALL;
	unsigned char data[FIFO_EVENT_SIZE * FIFO_DEPTH] = { 0 };
	unsigned char eventId;
	const unsigned char EVENTS_REMAINING_POS = 7;
	const unsigned char EVENTS_REMAINING_MASK = 0x1F;
	unsigned char events_remaining = 0;
	unsigned char *evt_data;
	static char pre_id[3];
	event_dispatch_handler_t event_handler;
	int r;
	if (info->tp_pm_suspend) {
		r = wait_for_completion_timeout(&info->pm_resume_completion,
						msecs_to_jiffies(500));
		if (!r) {
			logError(
				1,
				"%s pm_resume_completion timeout, i2c is closed",
				tag);
			pm_relax(info->dev);
			fts_enableInterrupt();
			return;
		} else {
			logError(
				1,
				"%s pm_resume_completion be completed, handling irq",
				tag);
		}
	}

	error = fts_writeReadU8UX(regAdd, 0, 0, data, FIFO_EVENT_SIZE,
				  DUMMY_FIFO);
	events_remaining = data[EVENTS_REMAINING_POS] & EVENTS_REMAINING_MASK;
	events_remaining = (events_remaining > FIFO_DEPTH - 1) ?
				   FIFO_DEPTH - 1 :
				   events_remaining;

	/*Drain the rest of the FIFO, up to 31 events*/
	if (error == OK && events_remaining > 0) {
		error = fts_writeReadU8UX(regAdd, 0, 0, &data[FIFO_EVENT_SIZE],
					  FIFO_EVENT_SIZE * events_remaining,
					  DUMMY_FIFO);
	}
	if (error != OK) {
		logError(
			1,
			"Error (%d) while reading from FIFO in fts_event_handler",
			error);
	} else {
		for (count = 0; count < events_remaining + 1; count++) {
			evt_data = &data[count * FIFO_EVENT_SIZE];
			if (pre_id[0] == EVT_ID_USER_REPORT &&
			    pre_id[1] == 0x02 && pre_id[2] == 0x18) {
				pre_id[0] = 0;
				pre_id[1] = 0;
				pre_id[2] = 0;
				continue;
			}
			if (evt_data[0] == EVT_ID_NOEVENT)
				break;
			eventId = evt_data[0] >> 4;
			/*Ensure event ID is within bounds*/
			if (eventId < NUM_EVT_ID) {
				event_handler =
					info->event_dispatch_table[eventId];
				event_handler(info, (evt_data));
				pre_id[0] = evt_data[0];
				pre_id[1] = evt_data[1];
				pre_id[2] = evt_data[2];
			}
		}
	}
	input_sync(info->input_dev);

	pm_relax(info->dev);
	fts_enableInterrupt();

	return;
}

/**
 * Bottom Half Interrupt Handler function
 * This handler is called each time there is at least one new event in the FIFO and the interrupt pin of the IC goes low.
 * It will read all the events from the FIFO and dispatch them to the proper event handler according the event ID
 */
static irqreturn_t fts_event_handler(int irq, void *ts_info)
{
	struct fts_ts_info *info = ts_info;
	int error = 0, count = 0;
	unsigned char regAdd = FIFO_CMD_READALL;
	unsigned char data[FIFO_EVENT_SIZE * FIFO_DEPTH] = { 0 };
	unsigned char eventId;
	const unsigned char EVENTS_REMAINING_POS = 7;
	const unsigned char EVENTS_REMAINING_MASK = 0x1F;
	unsigned char events_remaining = 0;
	unsigned char *evt_data;
	static char pre_id[3];
	event_dispatch_handler_t event_handler;
	static struct task_struct *touch_task = NULL;

	if (touch_task == NULL) {
		touch_task = current;
	}
	if (info->tp_pm_suspend || info->sensor_sleep) {
		logError(1, "%s %s debug for sleep", tag, __func__);
	}


	if (!info->tp_pm_suspend) {
		pm_stay_awake(info->dev);
	} else {
		logError(1, "%s device in suspend, schedue to work", tag);
		pm_wakeup_event(info->dev, 0);
		if (!work_pending(&info->sleep_work)) {
			pm_stay_awake(info->dev);
			queue_work(info->irq_wq, &info->sleep_work);
			fts_disableInterruptNoSync();
		}
		return IRQ_HANDLED;
	}
#ifdef CONFIG_SECURE_TOUCH
	if (!fts_secure_filter_interrupt(info)) {
		return IRQ_HANDLED;
	}
#endif
	info->temp_touch_id = 0;
	cpu_latency_qos_add_request(&info->pm_qos_req_irq, 0);
	error = fts_writeReadU8UX(regAdd, 0, 0, data, FIFO_EVENT_SIZE,
				  DUMMY_FIFO);
	events_remaining = data[EVENTS_REMAINING_POS] & EVENTS_REMAINING_MASK;
	events_remaining = (events_remaining > FIFO_DEPTH - 1) ?
				   FIFO_DEPTH - 1 :
				   events_remaining;

	/*Drain the rest of the FIFO, up to 31 events*/
	if (error == OK && events_remaining > 0) {
		error = fts_writeReadU8UX(regAdd, 0, 0, &data[FIFO_EVENT_SIZE],
					  FIFO_EVENT_SIZE * events_remaining,
					  DUMMY_FIFO);
	}
	if (error != OK) {
		logError(
			1,
			"Error (%d) while reading from FIFO in fts_event_handler",
			error);
	} else {
		for (count = 0; count < events_remaining + 1; count++) {
			evt_data = &data[count * FIFO_EVENT_SIZE];
			if (pre_id[0] == EVT_ID_USER_REPORT &&
			    pre_id[1] == 0x02 && pre_id[2] == 0x18) {
				pre_id[0] = 0;
				pre_id[1] = 0;
				pre_id[2] = 0;
				continue;
			}
			if (evt_data[0] == EVT_ID_NOEVENT)
				break;
			eventId = evt_data[0] >> 4;
			/*Ensure event ID is within bounds*/
			if (eventId < NUM_EVT_ID) {
				event_handler =
					info->event_dispatch_table[eventId];
				event_handler(info, (evt_data));
				pre_id[0] = evt_data[0];
				pre_id[1] = evt_data[1];
				pre_id[2] = evt_data[2];
			}
		}
	}
	input_sync(info->input_dev);

	cpu_latency_qos_remove_request(&info->pm_qos_req_irq);
	pm_relax(info->dev);
	return IRQ_HANDLED;
}

/**@}*/

static const char *fts_get_config(struct fts_ts_info *info)
{
	struct fts_hw_platform_data *pdata = info->board;
	int i = 0, ret = 0;

	if (!info->lockdown_is_ok) {
		logError(1, "%s can't read lockdown info", tag);
		return pdata->default_fw_name;
	}

	ret |= fts_enableInterrupt();

	for (i = 0; i < pdata->config_array_size; i++) {
		if (info->lockdown_info[1] ==
		    pdata->config_array[i].tp_vendor) {
			if (pdata->config_array[i].tp_module != U8_MAX &&
			    info->lockdown_info[7] ==
				    pdata->config_array[i].tp_module) {
				break;
			} else if (pdata->config_array[i].tp_module == U8_MAX)
				break;
		}
	}

	if (i >= pdata->config_array_size) {
		logError(1, "%s can't find right config, i:%d, array_size:%d",
			 tag, i, pdata->config_array_size);
		return pdata->default_fw_name;
	}

	logError(1, "%s Choose config %d: %s", tag, i,
		 pdata->config_array[i].fts_cfg_name);
	pdata->current_index = i;
	return pdata->config_array[i].fts_cfg_name;
}

const char *fts_get_limit(struct fts_ts_info *info)
{
	struct fts_hw_platform_data *pdata = info->board;
	int i = 0, ret = 0;

	if (!info->lockdown_is_ok) {
		logError(1, "%s can't read lockdown info", tag);
		return LIMITS_FILE;
	}

	ret |= fts_enableInterrupt();

	for (i = 0; i < pdata->config_array_size; i++) {
		if (info->lockdown_info[1] ==
		    pdata->config_array[i].tp_vendor) {
			if (pdata->config_array[i].tp_module != U8_MAX &&
			    info->lockdown_info[7] ==
				    pdata->config_array[i].tp_module) {
				break;
			} else if (pdata->config_array[i].tp_module == U8_MAX)
				break;
		}
	}

	if (i >= pdata->config_array_size) {
		logError(1, "%s can't find right limit", tag);
		return LIMITS_FILE;
	}

	logError(1, "%s Choose limit file %d: %s", tag, i,
		 pdata->config_array[i].fts_limit_name);
	pdata->current_index = i;
	return pdata->config_array[i].fts_limit_name;
}

/**
*	Implement the fw update and initialization flow of the IC that should be executed at every boot up.
*	The function perform a fw update of the IC in case of crc error or a new fw version and then understand if the IC need to be re-initialized again.
*	@return  OK if success or an error code which specify the type of error encountered
*/
int fts_fw_update(struct fts_ts_info *info, const char *fw_name, int force)
{
	u8 error_to_search[4] = { EVT_TYPE_ERROR_CRC_CX_HEAD,
				  EVT_TYPE_ERROR_CRC_CX,
				  EVT_TYPE_ERROR_CRC_CX_SUB_HEAD,
				  EVT_TYPE_ERROR_CRC_CX_SUB };
	int retval = 0;
	int retval1 = 0;
	int ret = 0;
	int crc_status = 0;
	int error = 0;
	int init_type = NO_INIT;
#ifdef PRE_SAVED_METHOD
	int keep_cx = 1;
#else
	int keep_cx = 0;
#endif

	logError(1, "%s Fw Auto Update is starting... \n", tag);
	ret = fts_crc_check();
	if (ret > OK) {
		logError(1, "%s %s: CRC Error or NO FW!\n", tag, __func__);
		crc_status = ret;
	} else {
		crc_status = 0;
		logError(
			1,
			"%s %s: NO CRC Error or Impossible to read CRC register! \n",
			tag, __func__);
	}

	if (fw_name == NULL) {
		fw_name = fts_get_config(info);
		if (fw_name == NULL)
			logError(1, "%s not found mached config!", tag);
	}

	if (fw_name) {
		if (force)
			retval = flashProcedure(fw_name, 1, keep_cx);
		else
			retval = flashProcedure(fw_name, crc_status, keep_cx);

		if ((retval & 0xFF000000) == ERROR_FLASH_PROCEDURE) {
			logError(
				1,
				"%s %s: firmware update failed and retry! ERROR %08X\n",
				tag, __func__, retval);
			fts_chip_powercycle(info);
			retval1 = flashProcedure(info->board->default_fw_name,
						 crc_status, keep_cx);
			if ((retval1 & 0xFF000000) == ERROR_FLASH_PROCEDURE) {
				logError(
					1,
					"%s %s: firmware update failed again!  ERROR %08X\n",
					tag, __func__, retval1);
				logError(1, "%s Fw Auto Update Failed!\n", tag);
			}
		}
	}

	logError(1, "%s %s: Verifying if CX CRC Error...\n", tag, __func__,
		 ret);
	ret = fts_system_reset();
	if (ret >= OK) {
		ret = pollForErrorType(error_to_search, 4);
		if (ret < OK) {
			logError(1, "%s %s: No Cx CRC Error Found! \n", tag,
				 __func__);
			logError(1, "%s %s: Verifying if Panel CRC Error... \n",
				 tag, __func__);
			error_to_search[0] = EVT_TYPE_ERROR_CRC_PANEL_HEAD;
			error_to_search[1] = EVT_TYPE_ERROR_CRC_PANEL;
			ret = pollForErrorType(error_to_search, 2);
			if (ret < OK) {
				logError(1,
					 "%s %s: No Panel CRC Error Found! \n",
					 tag, __func__);
				init_type = NO_INIT;
			} else {
				logError(
					1,
					"%s %s: Panel CRC Error FOUND! CRC ERROR = %02X\n",
					tag, __func__, ret);
				init_type = SPECIAL_PANEL_INIT;
			}
		} else {
			logError(
				1,
				"%s %s: Cx CRC Error FOUND! CRC ERROR = %02X\n",
				tag, __func__, ret);

			logError(
				1,
				"%s %s: Try to recovery with CX in fw file...\n",
				tag, __func__, ret);
			flashProcedure(info->board->default_fw_name, CRC_CX, 0);
			logError(1, "%s %s: Refresh panel init data... \n", tag,
				 __func__, ret);
		}
	} else {
		logError(
			1,
			"%s %s: Error while executing system reset! ERROR %08X\n",
			tag, __func__, ret);
	}

	if (init_type != SPECIAL_FULL_PANEL_INIT) {
#ifdef PRE_SAVED_METHOD
		if (systemInfo.u8_cfgAfeVer != systemInfo.u8_cxAfeVer) {
			init_type = SPECIAL_FULL_PANEL_INIT;
			logError(
				1,
				"%s %s: Different CX AFE Ver: %02X != %02X... Execute FULL Panel Init! \n",
				tag, __func__, systemInfo.u8_cfgAfeVer,
				systemInfo.u8_cxAfeVer);
		} else
#endif

			if (systemInfo.u8_cfgAfeVer !=
			    systemInfo.u8_panelCfgAfeVer) {
			init_type = SPECIAL_PANEL_INIT;
			logError(
				1,
				"%s %s: Different Panel AFE Ver: %02X != %02X... Execute Panel Init! \n",
				tag, __func__, systemInfo.u8_cfgAfeVer,
				systemInfo.u8_panelCfgAfeVer);
		} else {
			init_type = NO_INIT;
		}
	}

	if (init_type != NO_INIT) {
		error = fts_chip_initialization(info, init_type);
		if (error < OK) {
			logError(
				1,
				"%s %s Cannot initialize the chip ERROR %08X\n",
				tag, __func__, error);
		}
	}

	error = fts_init_sensing(info);
	if (error < OK) {
		logError(
			1,
			"%s Cannot initialize the hardware device ERROR %08X\n",
			tag, error);
	}

	logError(1, "%s Fw Update Finished! error = %08X\n", tag, error);
	return error;
}

#ifndef FW_UPDATE_ON_PROBE

/**
*	Function called by the delayed workthread executed after the probe in order to perform the fw update flow
*	@see  fts_fw_update()
*/
static void fts_fw_update_auto(struct work_struct *work)
{
	struct delayed_work *fwu_work =
		container_of(work, struct delayed_work, work);
	struct fts_ts_info *info =
		container_of(fwu_work, struct fts_ts_info, fwu_work);
	fts_fw_update(info, NULL, 0);
}
#endif

/**
*	Execute the initialization of the IC (supporting a retry mechanism), checking also the resulting data
*	@see  production_test_main()
*/
static int fts_chip_initialization(struct fts_ts_info *info, int init_type)
{
	int ret2 = 0;
	int retry;
	int initretrycnt = 0;

	for (retry = 0; retry <= RETRY_INIT_BOOT; retry++) {
		ret2 = production_test_initialization(init_type);
		if (ret2 == OK)
			break;
		initretrycnt++;
		logError(1,
			 "%s initialization cycle count = %04d - ERROR %08X \n",
			 tag, initretrycnt, ret2);
		fts_chip_powercycle(info);
	}

	if (ret2 < OK) {
		logError(1, "%s fts initialization failed 3 times \n", tag);
	}

	return ret2;
}

/**
 * @addtogroup isr
 * @{
 */
/**
*	Top half Interrupt handler function
*	Respond to the interrupt and schedule the bottom half interrupt handler in its work queue
*	@see fts_event_handler()
*/
/*
static irqreturn_t fts_interrupt_handler(int irq, void *handle)
{
	struct fts_ts_info *info = handle;
#ifdef CONFIG_SECURE_TOUCH
	if (!fts_secure_filter_interrupt(info)) {
		return IRQ_HANDLED;
	}
#endif
	disable_irq_nosync(info->client->irq);
	queue_work(info->event_wq, &info->work);

	return IRQ_HANDLED;
}
*/
/**
*	Initialize the dispatch table with the event handlers for any possible event ID and the interrupt routine behavior (triggered when the IRQ pin is low and associating the top half interrupt handler function).
*	@see fts_interrupt_handler()
*/
static int fts_interrupt_install(struct fts_ts_info *info)
{
	int i, error = 0;
#ifdef TOUCH_IRQ_CPU_AFFINITY
	struct cpumask cpu_mask;
#endif
	info->event_dispatch_table = kzalloc(
		sizeof(event_dispatch_handler_t) * NUM_EVT_ID, GFP_KERNEL);

	if (!info->event_dispatch_table) {
		logError(1, "%s OOM allocating event dispatch table\n", tag);
		return -ENOMEM;
	}

	for (i = 0; i < NUM_EVT_ID; i++)
		info->event_dispatch_table[i] = fts_nop_event_handler;

	install_handler(info, ENTER_POINT, enter_pointer);
	install_handler(info, LEAVE_POINT, leave_pointer);
	install_handler(info, MOTION_POINT, motion_pointer);
	install_handler(info, ERROR, error);
	install_handler(info, CONTROLLER_READY, controller_ready);
	install_handler(info, STATUS_UPDATE, status);
	install_handler(info, USER_REPORT, user_report);

	/* disable interrupts in any case */
	error = fts_disableInterrupt();
	logError(1, "%s Interrupt Mode\n", tag);
	if (request_threaded_irq(info->client->irq, NULL, fts_event_handler,
				 info->board->irq_flags, FTS_TS_DRV_NAME,
				 info)) {
		logError(1, "%s Request irq failed\n", tag);
		kfree(info->event_dispatch_table);
		error = -EBUSY;
	} else {
		disable_irq(info->client->irq);
	}
	return error;
}

/**
*	Clean the dispatch table and the free the IRQ.
*	This function is called when the driver need to be removed
*/
static void fts_interrupt_uninstall(struct fts_ts_info *info)
{
	fts_disableInterrupt();

	kfree(info->event_dispatch_table);

	free_irq(info->client->irq, info);
}

#ifndef I2C_INTERFACE
/* configure manually SPI4 because when no fw is running the chip use
 * SPI3 by default */

static int fts_spi4_mode_set(struct fts_ts_info *info)
{
	u8 cmd[1] = { 0x00 };
	int error;

	logError(1, "%s Setting SPI4 mode...\n", tag);
	cmd[0] = 0x10;
	error = fts_writeU8UX(FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG,
			      ADDR_GPIO_DIRECTION, cmd, 1);
	if (error < OK) {
		logError(1, "%s can not set gpio dir ERROR %08X\n", tag, error);
		return error;
	}

	cmd[0] = 0x02;
	error = fts_writeU8UX(FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG,
			      ADDR_GPIO_PULLUP, cmd, 1);
	if (error < OK) {
		logError(1, "%s can not set gpio pull-up ERROR %08X\n", tag,
			 error);
		return error;
	}
#if defined(ALIX) || defined(SALIXP)
#if defined(ALIX)
	cmd[0] = 0x70;
#else
	cmd[0] = 0x07;
#endif
	error = fts_writeU8UX(FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG,
			      ADDR_GPIO_CONFIG_REG3, cmd, 1);
	if (error < OK) {
		logError(1, "%s can not set gpio config ERROR %08X\n", tag,
			 error);
		return error;
	}
#else
	cmd[0] = 0x07;
	error = fts_writeU8UX(FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG,
			      ADDR_GPIO_CONFIG_REG2, cmd, 1);
	if (error < OK) {
		logError(1, "%s can not set gpio config ERROR %08X\n", tag,
			 error);
		return error;
	}
#endif
	cmd[0] = 0x30;
	error = fts_writeU8UX(FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG,
			      ADDR_GPIO_CONFIG_REG0, cmd, 1);
	if (error < OK) {
		logError(1, "%s can not set gpio config ERROR %08X\n", tag,
			 error);
		return error;
	}

	cmd[0] = SPI4_MASK;
	error = fts_writeU8UX(FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG, ADDR_ICR, cmd,
			      1);
	if (error < OK) {
		logError(1, "%s can not set spi4 mode ERROR %08X\n", tag,
			 error);
		return error;
	}
	return OK;
}
#endif

/**@}*/

/**
* This function try to attempt to communicate with the IC for the first time during the boot up process in order to acquire the necessary info for the following stages.
* The function execute a system reset, read fundamental info (system info) from the IC and install the interrupt
* @return OK if success or an error code which specify the type of error encountered
*/
static int fts_init(struct fts_ts_info *info)
{
	int error, retry;
	u8 readData[2];

	error = fts_system_reset();
#ifdef I2C_INTERFACE
	if (error < OK && isI2cError(error)) {
#else
	if (error < OK) {
#endif
		logError(1, "%s Cannot reset the device! ERROR %08X\n", tag,
			 error);
		logError(
			1,
			"%s check chip_id to judge there is no fw or no panel\n",
			tag);
#ifdef SPI4_WIRE
		error = fts_spi4_mode_set(info);
		if (error < OK) {
			logError(1, "%s change spi4 mode error\n", tag);
			return error;
		}
		msleep(1);
#endif
		logError(1, "%s Reading chip id\n", tag);
		for (retry = 0; retry < 3; retry++) {
			msleep(10); /* wait for the GPIO to stabilize */
			error = fts_writeReadU8UX(FTS_CMD_HW_REG_R,
						  ADDR_SIZE_HW_REG,
						  ADDR_DCHIP_ID, readData, 2,
						  DUMMY_FIFO);
			logError(1, "%s chip_id0:0x%x,chip_id1:0x%x\n", tag,
				 readData[0], readData[1]);
			if ((readData[0] == DCHIP_ID_0) &&
			    (readData[1] == DCHIP_ID_1))
				break;
		}
		if (retry == 3) {
			logError(1, "%s read chip error,no panel\n", tag);
			return ERROR_OP_NOT_ALLOW;
		} else {
			logError(
				1,
				"%s read chip ok, maybe no fw,force update fw\n",
				tag);
			return OK;
		}
	} else {
		if (error == (ERROR_TIMEOUT | ERROR_SYSTEM_RESET_FAIL)) {
			logError(1, "%s Setting default Sys INFO! \n", tag);
			error = defaultSysInfo(0);
		} else {
			error = readSysInfo(0);
			if (error < OK) {
				if (!isI2cError(error))
					error = OK;
				logError(
					1,
					"%s Cannot read Sys Info! ERROR %08X\n",
					tag, error);
			}
		}
	}

	return error;
}

/**
* Execute a power cycle in the IC, toggling the power lines (AVDD and DVDD)
* @param info pointer to fts_ts_info struct which contain information of the regulators
* @return 0 if success or another value if fail
*/
int fts_chip_powercycle(struct fts_ts_info *info)
{
	int error = 0;

	logError(1, "%s %s: Power Cycle Starting... \n", tag, __func__);
	logError(1, "%s %s: Disabling IRQ... \n", tag, __func__);

	fts_disableInterruptNoSync();

	if (info->vdd_reg) {
		error = regulator_disable(info->vdd_reg);
		if (error < 0) {
			logError(1, "%s %s: Failed to disable DVDD regulator\n",
				 tag, __func__);
		}
	}

	if (info->avdd_reg) {
		error = regulator_disable(info->avdd_reg);
		if (error < 0) {
			logError(1, "%s %s: Failed to disable AVDD regulator\n",
				 tag, __func__);
		}
	}

	if (info->board->avdd_gpio) {
		gpio_direction_output(info->board->avdd_gpio, 0);
	}

	if (info->board->reset_gpio != GPIO_NOT_DEFINED)
		gpio_set_value(info->board->reset_gpio, 0);
	else
		mdelay(300);

	if (info->vdd_reg) {
		error = regulator_enable(info->vdd_reg);
		if (error < 0) {
			logError(1, "%s %s: Failed to enable DVDD regulator\n",
				 tag, __func__);
		}
	}

	mdelay(1);

	if (info->avdd_reg) {
		error = regulator_enable(info->avdd_reg);
		if (error < 0) {
			logError(1, "%s %s: Failed to enable AVDD regulator\n",
				 tag, __func__);
		}
	}

	if (info->board->avdd_gpio) {
		gpio_direction_output(info->board->avdd_gpio, 1);
	}

	mdelay(6);

	if (info->board->reset_gpio != GPIO_NOT_DEFINED) {
		mdelay(10);
		gpio_set_value(info->board->reset_gpio, 1);
	}

	release_all_touches(info);

	logError(1, "%s %s: Power Cycle Finished! ERROR CODE = %08x\n", tag,
		 __func__, error);
	setSystemResetedUp(1);
	setSystemResetedDown(1);
	return error;
}

/**
 * Complete the boot up process, initializing the sensing of the IC according to the current setting chosen by the host and register the notifier for the suspend/resume actions and the event handler
 * @return OK if success or an error code which specify the type of error encountered
 */
static int fts_init_sensing(struct fts_ts_info *info)
{
	int error = 0;
	error |= fts_interrupt_install(info);
	error |= fts_mode_handler(info, 0);
	if (error < OK) {
		logError(1, "%s %s Init after Probe error (ERROR = %08X)\n",
			 tag, __func__, error);
		return error;
	}
	error |= fts_enableInterrupt();

	return error;
}

/**
 * @ingroup mode_section
 * @{
 */
/**
 * The function handle the switching of the mode in the IC enabling/disabling the sensing and the features set from the host
 * @param info pointer to fts_ts_info which contains info about the device and its hw setup
 * @param force if 1, the enabling/disabling command will be send even if the feature was alredy enabled/disabled otherwise it will judge if the feature changed status or the IC had s system reset and therefore the features need to be restored
 * @return OK if success or an error code which specify the type of error encountered
 */
static int fts_mode_handler(struct fts_ts_info *info, int force)
{
	int res = OK;
	int ret = OK;
	u8 settings[4] = { 0 };

	info->mode = MODE_NOTHING;
	logError(0, "%s %s: Mode Handler starting... \n", tag, __func__);
	switch (info->resume_bit) {
	case 0:
		logError(0, "%s %s: Screen OFF... \n", tag, __func__);
		logError(1, "%s %s: Sense OFF! \n", tag, __func__);
		ret = setScanMode(SCAN_MODE_ACTIVE, 0x00);
		res |= ret;
		setSystemResetedDown(0);
		break;

	case 1:
		logError(1, "%s %s: Screen ON... \n", tag, __func__);
		settings[0] = 0x01;
		logError(1, "%s %s: Sense ON! \n", tag, __func__);
		res |= setScanMode(SCAN_MODE_ACTIVE, settings[0]);
		info->mode |= (SCAN_MODE_ACTIVE << 24);
		MODE_ACTIVE(info->mode, settings[0]);
		setSystemResetedUp(0);
		break;

	default:
		logError(1,
			 "%s %s: invalid resume_bit value = %d! ERROR %08X \n",
			 tag, __func__, info->resume_bit, ERROR_OP_NOT_ALLOW);
		res = ERROR_OP_NOT_ALLOW;
	}

	logError(0, "%s %s: Mode Handler finished! res = %08X mode = %08X \n",
		 tag, __func__, res, info->mode);
	return res;
}

/**
 * Resume work function which perform a system reset, clean all the touches from the linux input system and prepare the ground for enabling the sensing
 */
static void fts_resume_work(struct work_struct *work)
{
	struct fts_ts_info *info;
	int r;

	info = container_of(work, struct fts_ts_info, resume_work);

	if (!info->probe_ok)
		return;

	pm_stay_awake(info->dev);
	if (info->tp_pm_suspend) {
		pm_wakeup_event(info->dev, 0);
		r = wait_for_completion_timeout(&info->pm_resume_completion,
						msecs_to_jiffies(500));
		if (!r) {
			logError(
				1,
				"%s pm_resume_completion timeout, i2c is closed",
				tag);
			pm_relax(info->dev);
			return;
		} else {
			logError(1, "%s pm_resume_completion be completed",
				 tag);
		}
	}

	info->resume_bit = 1;

	fts_disableInterrupt();
#ifdef CONFIG_SECURE_TOUCH
	fts_secure_stop(info, true);
#endif

	fts_system_reset();
	release_all_touches(info);
	fts_mode_handler(info, 0);
	msleep(12);
	info->sensor_sleep = false;

	fts_enableInterrupt();

	pm_relax(info->dev);
}

/**
 * Suspend work function which clean all the touches from Linux input system and prepare the ground to disabling the sensing or enter in gesture mode
 */
static void fts_suspend_work(struct work_struct *work)
{
	struct fts_ts_info *info;
	int r;

	info = container_of(work, struct fts_ts_info, suspend_work);

	if (!info->probe_ok)
		return;

	pm_stay_awake(info->dev);

	if (info->tp_pm_suspend) {
		pm_wakeup_event(info->dev, 0);
		r = wait_for_completion_timeout(&info->pm_resume_completion,
						msecs_to_jiffies(500));
		if (!r) {
			logError(
				1,
				"%s pm_resume_completion timeout, i2c is closed",
				tag);
			pm_relax(info->dev);
			return;
		} else {
			logError(1, "%s pm_resume_completion be completed",
				 tag);
		}
	}
	info->resume_bit = 0;

#ifdef CONFIG_SECURE_TOUCH
	fts_secure_stop(info, true);
#endif
	fts_disableInterrupt();
	fts_mode_handler(info, 0);
	release_all_touches(info);
	info->sensor_sleep = true;
	pm_relax(info->dev);
}

/**
 * From the name of the power regulator get/put the actual regulator structs (copying their references into fts_ts_info variable)
 * @param info pointer to fts_ts_info which contains info about the device and its hw setup
 * @param get if 1, the regulators are get otherwise they are put (released) back to the system
 * @return OK if success or an error code which specify the type of error encountered
 */
static int fts_get_reg(struct fts_ts_info *info, bool get)
{
	int retval;
	const struct fts_hw_platform_data *bdata = info->board;

	if (!get) {
		retval = 0;
		goto regulator_put;
	}

	if ((bdata->vdd_reg_name != NULL) && (*bdata->vdd_reg_name != 0)) {
		info->vdd_reg = regulator_get(info->dev, bdata->vdd_reg_name);
		if (IS_ERR(info->vdd_reg)) {
			logError(1, "%s %s: Failed to get pullup regulator\n",
				 tag, __func__);
			retval = PTR_ERR(info->vdd_reg);
			goto regulator_put;
		}
	}

	if ((bdata->avdd_reg_name != NULL) && (*bdata->avdd_reg_name != 0)) {
		info->avdd_reg = regulator_get(info->dev, bdata->avdd_reg_name);
		if (IS_ERR(info->avdd_reg)) {
			logError(1,
				 "%s %s: Failed to get bus power regulator\n",
				 tag, __func__);
			retval = PTR_ERR(info->avdd_reg);
			goto regulator_put;
		}
	}
	return OK;

regulator_put:
	if (info->vdd_reg) {
		regulator_put(info->vdd_reg);
		info->vdd_reg = NULL;
	}

	if (info->avdd_reg) {
		regulator_put(info->avdd_reg);
		info->avdd_reg = NULL;
	}

	return retval;
}

/**
 * Enable or disable the power regulators
 * @param info pointer to fts_ts_info which contains info about the device and its hw setup
 * @param enable if 1, the power regulators are turned on otherwise they are turned off
 * @return OK if success or an error code which specify the type of error encountered
 */
static int fts_enable_reg(struct fts_ts_info *info, bool enable)
{
	int retval;

	if (!enable) {
		retval = 0;
		goto disable_pwr_reg;
	}

	if (info->vdd_reg) {
		retval = regulator_enable(info->vdd_reg);
		if (retval < 0) {
			logError(1, "%s %s: Failed to enable bus regulator\n",
				 tag, __func__);
			goto exit;
		}
	}

	if (info->avdd_reg) {
		retval = regulator_enable(info->avdd_reg);
		if (retval < 0) {
			logError(1, "%s %s: Failed to enable power regulator\n",
				 tag, __func__);
			goto disable_bus_reg;
		}
	}

	if (info->board->avdd_gpio) {
		gpio_direction_output(info->board->avdd_gpio, 1);
	}

	return OK;

disable_pwr_reg:
	if (info->avdd_reg)
		regulator_disable(info->avdd_reg);
	if (info->board->avdd_gpio)
		gpio_direction_output(info->board->avdd_gpio, 0);

disable_bus_reg:
	if (info->vdd_reg)
		regulator_disable(info->vdd_reg);

exit:
	return retval;
}

/**
 * Configure a GPIO according to the parameters
 * @param gpio gpio number
 * @param config if true, the gpio is set up otherwise it is free
 * @param dir direction of the gpio, 0 = in, 1 = out
 * @param state initial value (if the direction is in, this parameter is ignored)
 * return error code
 */
static int fts_gpio_setup(int gpio, bool config, int dir, int state)
{
	int retval = 0;
	unsigned char buf[16];

	if (config) {
		if (!fts_info->gpio_has_request) {
			snprintf(buf, 16, "fts_gpio_%u\n", gpio);
			retval = gpio_request(gpio, buf);
			if (retval) {
				logError(
					1,
					"%s %s: Failed to get gpio %d (code: %d)",
					tag, __func__, gpio, retval);
				return retval;
			}
		}

		if (dir == 0)
			retval = gpio_direction_input(gpio);
		else
			retval = gpio_direction_output(gpio, state);
		if (retval) {
			logError(1, "%s %s: Failed to set gpio %d direction",
				 tag, __func__, gpio);
			return retval;
		}
	} else {
		gpio_free(gpio);
	}

	return retval;
}

/**
 * Setup the IRQ and RESET (if present) gpios.
 * If the Reset Gpio is present it will perform a cycle HIGH-LOW-HIGH in order to assure that the IC has been reset properly
 */
static int fts_set_gpio(struct fts_ts_info *info, bool alway_output_low)
{
	int retval;
	struct fts_hw_platform_data *bdata = info->board;

	retval = fts_gpio_setup(bdata->irq_gpio, true, alway_output_low ? 1 : 0,
				0);
	if (retval < 0) {
		logError(1, "%s %s: Failed to configure irq GPIO\n", tag,
			 __func__);
		goto err_gpio_irq;
	}

	if (bdata->reset_gpio >= 0) {
		retval = fts_gpio_setup(bdata->reset_gpio, true, 1,
					alway_output_low ? 0 : 1);
		if (retval < 0) {
			logError(1, "%s %s: Failed to configure reset GPIO\n",
				 tag, __func__);
			goto err_gpio_reset;
		}
	}
	info->gpio_has_request = true;
	return OK;

err_gpio_reset:
	fts_gpio_setup(bdata->irq_gpio, false, 0, 0);
	bdata->reset_gpio = GPIO_NOT_DEFINED;
err_gpio_irq:
	return retval;
}

static int fts_pinctrl_init(struct fts_ts_info *info)
{
	int retval = 0;
	/* Get pinctrl if target uses pinctrl */
	info->ts_pinctrl = devm_pinctrl_get(info->dev);

	if (IS_ERR_OR_NULL(info->ts_pinctrl)) {
		retval = PTR_ERR(info->ts_pinctrl);
		dev_err(info->dev, "Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	info->pinctrl_state_active =
		pinctrl_lookup_state(info->ts_pinctrl, PINCTRL_STATE_ACTIVE);

	if (IS_ERR_OR_NULL(info->pinctrl_state_active)) {
		retval = PTR_ERR(info->pinctrl_state_active);
		dev_err(info->dev, "Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	info->pinctrl_state_suspend =
		pinctrl_lookup_state(info->ts_pinctrl, PINCTRL_STATE_SUSPEND);

	if (IS_ERR_OR_NULL(info->pinctrl_state_suspend)) {
		retval = PTR_ERR(info->pinctrl_state_suspend);
		dev_dbg(info->dev, "Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	return 0;
err_pinctrl_lookup:
	devm_pinctrl_put(info->ts_pinctrl);
err_pinctrl_get:
	info->ts_pinctrl = NULL;
	return retval;
}

/**
 * Retrieve and parse the hw information from the device tree node defined in the system.
 * the most important information to obtain are: IRQ and RESET gpio numbers, power regulator names
 * In the device file node is possible to define additional optional information that can be parsed here.
 */
static int parse_dt(struct device *dev, struct fts_hw_platform_data *bdata)
{
	int retval;
	const char *name;
	struct device_node *temp, *np = dev->of_node;
	struct fts_config_info *config_info;
	u32 temp_val;

	bdata->irq_gpio = of_get_named_gpio(np, "fts,irq-gpio", 0);

	logError(0, "%s irq_gpio = %d\n", tag, bdata->irq_gpio);
	retval = of_property_read_string(np, "fts,pwr-reg-name", &name);
	if (retval == -EINVAL)
		bdata->avdd_reg_name = NULL;
	else if (retval < 0)
		return retval;
	else {
		bdata->avdd_reg_name = name;
		logError(0, "%s pwr_reg_name = %s\n", tag, name);
	}

	retval = of_property_read_string(np, "fts,bus-reg-name", &name);
	if (retval == -EINVAL)
		bdata->vdd_reg_name = NULL;
	else if (retval < 0)
		return retval;
	else {
		bdata->vdd_reg_name = name;
		logError(0, "%s bus_reg_name = %s\n", tag, name);
	}

	retval = of_get_named_gpio(np, "fts,avdd-gpio", 0);
	if (retval < 0) {
		logError(0,"%s can't find avdd-gpio[%d]\n", tag, retval);
		bdata->avdd_gpio = 0;
	} else {
		logError(0,"%s get avdd-gpio[%d] from dt\n", tag, retval);
		bdata->avdd_gpio = retval;
	}

	if (of_property_read_bool(np, "fts,reset-gpio-enable")) {
		bdata->reset_gpio =
			of_get_named_gpio(np, "fts,reset-gpio", 0);
		logError(0, "%s reset_gpio =%d\n", tag, bdata->reset_gpio);
	} else {
		bdata->reset_gpio = GPIO_NOT_DEFINED;
	}

	retval = of_property_read_u32(np, "fts,irq-flags", &temp_val);
	if (retval < 0)
		return retval;
	else
		bdata->irq_flags = temp_val;
	retval = of_property_read_u32(np, "fts,x-max", &temp_val);
	if (retval < 0)
		bdata->x_max = X_AXIS_MAX;
	else
		bdata->x_max = temp_val;

	retval = of_property_read_u32(np, "fts,y-max", &temp_val);
	if (retval < 0)
		bdata->y_max = Y_AXIS_MAX;
	else
		bdata->y_max = temp_val;
	retval = of_property_read_string(np, "fts,default-fw-name",
					 &bdata->default_fw_name);
	bdata->swap_x = of_property_read_bool(np, "fts,swap-x");
	bdata->swap_y = of_property_read_bool(np, "fts,swap-y");
	bdata->support_vsync_mode =
		of_property_read_bool(np, "fts,support-vsync-mode");

	retval = of_property_read_u32(np, "fts,config-array-size",
				      (u32 *)&bdata->config_array_size);

	if (retval) {
		logError(1, "%s Unable to get array size\n", tag);
		return retval;
	}

	bdata->config_array = devm_kzalloc(
		dev, bdata->config_array_size * sizeof(struct fts_config_info),
		GFP_KERNEL);

	if (!bdata->config_array) {
		logError(1, "%s Unable to allocate memory\n", tag);
		return -ENOMEM;
	}

	config_info = bdata->config_array;
	for_each_child_of_node (np, temp) {
		retval = of_property_read_u32(temp, "fts,tp-vendor", &temp_val);

		if (retval) {
			logError(1, "%s Unable to read tp vendor\n", tag);
		} else {
			config_info->tp_vendor = (u8)temp_val;
			logError(1, "%s %s:tp vendor: %u", tag, __func__,
				 config_info->tp_vendor);
		}
		retval = of_property_read_u32(temp, "fts,tp-color", &temp_val);
		if (retval) {
			logError(1, "%s Unable to read tp color\n", tag);
		} else {
			config_info->tp_color = (u8)temp_val;
			logError(1, "%s %s:tp color: %u", tag, __func__,
				 config_info->tp_color);
		}

		retval = of_property_read_u32(temp, "fts,tp-module", &temp_val);
		if (retval) {
			logError(1, "%s Unable to read tp module\n", tag);
			config_info->tp_module = U8_MAX;
		} else {
			config_info->tp_module = (u8)temp_val;
			logError(1, "%s %s:tp module: %u", tag, __func__,
				 config_info->tp_module);
		}

		retval = of_property_read_u32(temp, "fts,tp-hw-version",
					      &temp_val);

		if (retval) {
			logError(1, "%s Unable to read tp hw version\n", tag);
		} else {
			config_info->tp_hw_version = (u8)temp_val;
			logError(1, "%s %s:tp color: %u", tag, __func__,
				 config_info->tp_hw_version);
		}

		retval = of_property_read_string(temp, "fts,fw-name",
						 &config_info->fts_cfg_name);

		if (retval && (retval != -EINVAL)) {
			config_info->fts_cfg_name = NULL;
			logError(1, "%s Unable to read cfg name\n", tag);
		} else {
			logError(1, "%s %s:fw_name: %s", tag, __func__,
				 config_info->fts_cfg_name);
		}
		retval = of_property_read_string(temp, "fts,limit-name",
						 &config_info->fts_limit_name);

		if (retval && (retval != -EINVAL)) {
			config_info->fts_limit_name = NULL;
			logError(1, "%s Unable to read limit name\n", tag);
		} else {
			logError(1, "%s %s:limit_name: %s", tag, __func__,
				 config_info->fts_limit_name);
		}
		config_info++;
	}
	return OK;
}

static int fts_short_open_test(void)
{
	TestToDo selftests;
	int res = -1;
	int init_type = SPECIAL_PANEL_INIT;
	const char *limit_file_name = NULL;
	limit_file_name = fts_get_limit(fts_info);

	memset(&selftests, 0x00, sizeof(TestToDo));

	/* Hover Test */
	selftests.SelfHoverForceRaw = 0; /*  SS Hover Force Raw min/Max test */
	selftests.SelfHoverSenceRaw = 0; /*  SS Hover Sence Raw min/Max test */
	selftests.SelfHoverForceIxTotal =
		0; /*  SS Hover Total Force Ix min/Max (for each node)* test */
	selftests.SelfHoverSenceIxTotal = 0;

	selftests.MutualRawAdjITO = 0;
	selftests.MutualRaw = 0;
	selftests.MutualRawEachNode = 1;
	selftests.MutualRawGap = 0;
	selftests.MutualRawAdj = 0;
	selftests.MutualRawLP = 0;
	selftests.MutualRawGapLP = 0;
	selftests.MutualRawAdjLP = 0;
	selftests.MutualCx1 = 0;
	selftests.MutualCx2 = 0;
	selftests.MutualCx2Adj = 0;
	selftests.MutualCxTotal = 0;
	selftests.MutualCxTotalAdj = 0;
	selftests.MutualCx1LP = 0;
	selftests.MutualCx2LP = 0;
	selftests.MutualCx2AdjLP = 0;
	selftests.MutualCxTotalLP = 0;
	selftests.MutualCxTotalAdjLP = 0;
	selftests.MutualKeyRaw = 0;
	selftests.MutualKeyCx1 = 0;
	selftests.MutualKeyCx2 = 0;
	selftests.MutualKeyCxTotal = 0;
	selftests.SelfForceRaw = 1;
	selftests.SelfForceRawGap = 0;
	selftests.SelfForceRawLP = 0;
	selftests.SelfForceRawGapLP = 0;
	selftests.SelfForceIx1 = 0;
	selftests.SelfForceIx2 = 0;
	selftests.SelfForceIx2Adj = 0;
	selftests.SelfForceIxTotal = 0;
	selftests.SelfForceIxTotalAdj = 0;
	selftests.SelfForceCx1 = 0;
	selftests.SelfForceCx2 = 0;
	selftests.SelfForceCx2Adj = 0;
	selftests.SelfForceCxTotal = 0;
	selftests.SelfForceCxTotalAdj = 0;
	selftests.SelfSenseRaw = 1;
	selftests.SelfSenseRawGap = 0;
	selftests.SelfSenseRawLP = 0;
	selftests.SelfSenseRawGapLP = 0;
	selftests.SelfSenseIx1 = 0;
	selftests.SelfSenseIx2 = 0;
	selftests.SelfSenseIx2Adj = 0;
	selftests.SelfSenseIxTotal = 0;
	selftests.SelfSenseIxTotalAdj = 0;
	selftests.SelfSenseCx1 = 0;
	selftests.SelfSenseCx2 = 0;
	selftests.SelfSenseCx2Adj = 0;
	selftests.SelfSenseCxTotal = 0;
	selftests.SelfSenseCxTotalAdj = 0;

	res = fts_disableInterrupt();
	if (res < 0) {
		logError(0, "%s fts_disableInterrupt: ERROR %08X \n", tag, res);
		res = (res | ERROR_DISABLE_INTER);
		goto END;
	}
	res = production_test_main(limit_file_name, 1, init_type, &selftests);
END:
	fts_mode_handler(fts_info, 1);
	fts_enableInterrupt();
	if (res == OK)
		return FTS_RESULT_PASS;
	else
		return FTS_RESULT_FAIL;
}

static int fts_i2c_test(void)
{
	int ret = 0;
	u8 data[SYS_INFO_SIZE] = { 0 };

	logError(0, "%s %s: Reading System Info...\n", tag, __func__);
	ret = fts_writeReadU8UX(FTS_CMD_FRAMEBUFFER_R, BITS_16,
				ADDR_FRAMEBUFFER, data, SYS_INFO_SIZE,
				DUMMY_FRAMEBUFFER);
	if (ret < OK) {
		logError(
			1,
			"%s %s: error while reading the system data ERROR %08X\n",
			tag, __func__, ret);
		return FTS_RESULT_FAIL;
	}

	return FTS_RESULT_PASS;
}

static ssize_t fts_selftest_read(struct file *file, char __user *buf,
				 size_t count, loff_t *pos)
{
	char tmp[5] = { 0 };
	int cnt;

	if (*pos != 0)
		return 0;
	cnt = snprintf(tmp, sizeof(fts_info->result_type), "%d\n",
		       fts_info->result_type);
	if (copy_to_user(buf, tmp, strlen(tmp))) {
		return -EFAULT;
	}
	*pos += cnt;
	return cnt;
}

#ifdef FTS_SELFTEST_FORCE_CAL
/*
*Do FORCE calibrate before CIT open/short sefltest
*/
static int fts_force_calibration(void)
{
	u8 param = 0x01;
	u8 res = OK;

	logError(1, "%s %s Enter\n", tag, __func__);
	res = production_test_initialization(SPECIAL_FULL_PANEL_INIT);
	if (res < 0) {
		logError(1,
			 "%s Error during  INITIALIZATION TEST! ERROR %08X\n",
			 tag, res);
	} else {
		logError(1, "%s do force calibration success", tag);
		res = cleanUp(param);
		if (res == OK)
			logError(1, "%s %s execute clean up success!", tag,
				 __func__);
		else
			logError(1, "%s %s execute clean up Failed!", tag,
				 __func__);
	}
	logError(1, "%s %s Exit\n", tag, __func__);
	return res;
}
#endif

static ssize_t fts_selftest_write(struct file *file, const char __user *buf,
				  size_t count, loff_t *pos)
{
	int retval = 0;
	char tmp[6];

	if (copy_from_user(tmp, buf, count)) {
		retval = -EFAULT;
		goto out;
	}
	if (!strncmp("short", tmp, 5) || !strncmp("open", tmp, 4)) {
#ifdef FTS_SELFTEST_FORCE_CAL
		fts_force_calibration();
#endif
		retval = fts_short_open_test();
	} else if (!strncmp("i2c", tmp, 3))
		retval = fts_i2c_test();

	fts_info->result_type = retval;
out:
	if (retval >= 0)
		retval = count;

	return retval;
}

static const struct proc_ops fts_selftest_ops = {
	.proc_read = fts_selftest_read,
	.proc_write = fts_selftest_write,
};

static int fts_datadump_show(struct seq_file *m, void *v)
{
	char *tmp = (char *)m->private;
	seq_puts(m, tmp);
	logError(1, "%s %s show data dump", tag, __func__);
	return 0;
}

static int fts_datadump_open(struct inode *inode, struct file *file)
{
	int ret = 0, cnt1 = 0, cnt2 = 0, cnt3 = 0;
	char *tmp;

	tmp = kvmalloc(PAGE_SIZE * 3, GFP_KERNEL);
	logError(1, "%s %s vmalloc data dump memory, addr=%d", tag, __func__,
		 tmp);
	if (tmp == NULL)
		return -1;
	else
		memset(tmp, 0, PAGE_SIZE * 3);

	cnt1 = fts_strength_frame_show(fts_info->dev, NULL, tmp);
	if (cnt1 == 0) {
		ret = 0;
		return -1;
	}

	ret = stm_fts_cmd_store(fts_info->dev, NULL, "13", 2);
	if (ret == 0)
		return -1;
	cnt2 = stm_fts_cmd_show(fts_info->dev, NULL, tmp + cnt1);
	if (cnt2 == 0) {
		ret = 0;
		return -1;
	}

	ret = stm_fts_cmd_store(fts_info->dev, NULL, "15", 2);
	if (ret == 0)
		return -1;
	cnt3 = stm_fts_cmd_show(fts_info->dev, NULL, tmp + cnt1 + cnt2);
	if (cnt3 == 0) {
		ret = 0;
		return -1;
	}

	return single_open(file, fts_datadump_show, tmp);
}

static int fts_datadump_release(struct inode *inode, struct file *file)
{
	if (((struct seq_file *)file->private_data)->private != NULL) {
		logError(1, "%s %s vfree data dump memory, addr=%d", tag,
			 __func__,
			 ((struct seq_file *)file->private_data)->private);
		kvfree(((struct seq_file *)file->private_data)->private);
		((struct seq_file *)file->private_data)->private = NULL;
	}
	return single_release(inode, file);
}

static const struct proc_ops fts_datadump_ops = {
	.proc_open = fts_datadump_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = fts_datadump_release,
};

#define TP_INFO_MAX_LENGTH 50

static ssize_t fts_fw_version_read(struct file *file, char __user *buf,
				   size_t count, loff_t *pos)
{
	int cnt = 0, ret = 0;
	char tmp[TP_INFO_MAX_LENGTH];

	if (*pos != 0)
		return 0;

	cnt = snprintf(tmp, TP_INFO_MAX_LENGTH, "%x.%x\n", systemInfo.u16_fwVer,
		       systemInfo.u16_cfgVer);
	ret = copy_to_user(buf, tmp, cnt);
	*pos += cnt;
	if (ret != 0)
		return 0;
	else
		return cnt;
}

static const struct proc_ops fts_fw_version_ops = {
	.proc_read = fts_fw_version_read,
};

static ssize_t fts_lockdown_info_read(struct file *file, char __user *buf,
				      size_t count, loff_t *pos)
{
	int cnt = 0, ret = 0;
	char tmp[TP_INFO_MAX_LENGTH];

	if (*pos != 0)
		return 0;

	ret = fts_get_lockdown_info(fts_info->lockdown_info, fts_info);
	if (ret != OK) {
		logError(1, "%s %s get lockdown info error\n", tag, __func__);
		goto out;
	}

	cnt = snprintf(
		tmp, TP_INFO_MAX_LENGTH,
		"0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\n",
		fts_info->lockdown_info[0], fts_info->lockdown_info[1],
		fts_info->lockdown_info[2], fts_info->lockdown_info[3],
		fts_info->lockdown_info[4], fts_info->lockdown_info[5],
		fts_info->lockdown_info[6], fts_info->lockdown_info[7]);
	ret = copy_to_user(buf, tmp, cnt);
out:
	*pos += cnt;
	if (ret != 0)
		return 0;
	else
		return cnt;
}

static const struct proc_ops fts_lockdown_info_ops = {
	.proc_read = fts_lockdown_info_read,
};

#ifdef CONFIG_PM
static int fts_pm_suspend(struct device *dev)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);
	logError(1, "%s %s\n", tag, __func__);

	if (!info) {
		logError(1, "%s fts_pm_suspend failed, return\n", tag);
		return 0;
	}

	info->tp_pm_suspend = true;
	reinit_completion(&info->pm_resume_completion);

	return 0;
}

static int fts_pm_resume(struct device *dev)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);
	logError(1, "%s %s\n", tag, __func__);

	info->tp_pm_suspend = false;
	complete_all(&info->pm_resume_completion);

	return 0;
}

static const struct dev_pm_ops fts_dev_pm_ops = {
	.suspend = fts_pm_suspend,
	.resume = fts_pm_resume,
};
#endif

#ifdef FTS_DEBUG_FS
static void tpdbg_shutdown(struct fts_ts_info *info, bool sleep)
{
	u8 settings[4] = { 0 };
	info->mode = MODE_NOTHING;

	if (sleep) {
		logError(0, "%s %s: Sense OFF! \n", tag, __func__);
		setScanMode(SCAN_MODE_ACTIVE, 0x00);
	} else {
		settings[0] = 0x01;
		logError(0, "%s %s: Sense ON! \n", tag, __func__);
		setScanMode(SCAN_MODE_ACTIVE, settings[0]);
		info->mode |= (SCAN_MODE_ACTIVE << 24);
		MODE_ACTIVE(info->mode, settings[0]);
	}
}

static void tpdbg_suspend(struct fts_ts_info *info, bool enable)
{
	if (enable)
		queue_work(info->event_wq, &info->suspend_work);
	else
		queue_work(info->event_wq, &info->resume_work);
}

static int tpdbg_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	return 0;
}

static ssize_t tpdbg_read(struct file *file, char __user *buf, size_t size,
			  loff_t *ppos)
{
	const char *str = "cmd support as below:\n \
				\necho \"irq-disable\" or \"irq-enable\" to ctrl irq\n \
				\necho \"tp-sd-en\" of \"tp-sd-off\" to ctrl panel in or off sleep mode\n \
				\necho \"tp-suspend-en\" or \"tp-suspend-off\" to ctrl panel in or off suspend status\n";

	loff_t pos = *ppos;
	int len = strlen(str);

	if (pos < 0)
		return -EINVAL;
	if (pos >= len)
		return 0;

	if (copy_to_user(buf, str, len))
		return -EFAULT;

	*ppos = pos + len;

	return len;
}

static ssize_t tpdbg_write(struct file *file, const char __user *buf,
			   size_t size, loff_t *ppos)
{
	struct fts_ts_info *info = file->private_data;
	char *cmd = kzalloc(size + 1, GFP_KERNEL);
	int ret = size;

	if (!cmd)
		return -ENOMEM;

	if (copy_from_user(cmd, buf, size)) {
		ret = -EFAULT;
		goto out;
	}

	cmd[size] = '\0';

	if (!strncmp(cmd, "irq-disable", 11)) {
		logError(1, "%s %s irq disable\n", tag, __func__);
		fts_disableInterrupt();
	} else if (!strncmp(cmd, "irq-enable", 10)) {
		logError(1, "%s %s irq enable\n", tag, __func__);
		fts_enableInterrupt();
	} else if (!strncmp(cmd, "tp-sd-en", 8))
		tpdbg_shutdown(info, true);
	else if (!strncmp(cmd, "tp-sd-off", 9))
		tpdbg_shutdown(info, false);
	else if (!strncmp(cmd, "tp-suspend-en", 13))
		tpdbg_suspend(info, true);
	else if (!strncmp(cmd, "tp-suspend-off", 14))
		tpdbg_suspend(info, false);
out:
	kfree(cmd);

	return ret;
}

static int tpdbg_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;

	return 0;
}

static const struct file_operations tpdbg_operations = {
	.owner = THIS_MODULE,
	.open = tpdbg_open,
	.read = tpdbg_read,
	.write = tpdbg_write,
	.release = tpdbg_release,
};
#endif

#ifdef CONFIG_SECURE_TOUCH
int fts_secure_init(struct fts_ts_info *info)
{
	int ret;
	struct fts_secure_info *scr_info =
		kmalloc(sizeof(*scr_info), GFP_KERNEL);
	if (!scr_info) {
		logError(1, "%s %s alloc fts_secure_info failed\n", tag,
			 __func__);
		return -ENOMEM;
	}

	logError(1, "%s fts_secure_init\n", tag);

	mutex_init(&scr_info->palm_lock);

	init_completion(&scr_info->st_powerdown);
	init_completion(&scr_info->st_irq_processed);

	atomic_set(&scr_info->st_enabled, 0);
	atomic_set(&scr_info->st_pending_irqs, 0);

	info->secure_info = scr_info;

	ret = sysfs_create_file(&info->dev->kobj,
				&dev_attr_secure_touch_enable.attr);
	if (ret < 0) {
		logError(
			1,
			"%s %s create sysfs attribute secure_touch_enable failed\n",
			tag, __func__);
		goto err;
	}

	ret = sysfs_create_file(&info->dev->kobj, &dev_attr_secure_touch.attr);
	if (ret < 0) {
		logError(1,
			 "%s %s create sysfs attribute secure_touch failed\n",
			 tag, __func__);
		goto err;
	}

	scr_info->fts_info = info;
	scr_info->secure_inited = true;

	return 0;

err:
	kfree(scr_info);
	info->secure_info = NULL;
	return ret;
}

void fts_secure_remove(struct fts_ts_info *info)
{
	struct fts_secure_info *scr_info = info->secure_info;

	sysfs_remove_file(&info->dev->kobj, &dev_attr_secure_touch_enable.attr);
	sysfs_remove_file(&info->dev->kobj, &dev_attr_secure_touch.attr);
	kfree(scr_info);
}

#endif

/**
 * Probe function, called when the driver it is matched with a device with the same name compatible name
 * This function allocate, initialize and define all the most important function and flow that are used by the driver to operate with the IC.
 * It allocates device variables, initialize queues and schedule works, registers the IRQ handler, suspend/resume callbacks, registers the device to the linux input subsystem etc.
 */
#ifdef I2C_INTERFACE
static int fts_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
#else
static int fts_probe(struct spi_device *client)
{
#endif

	struct fts_ts_info *info = NULL;
	int error = 0;
	struct device_node *dp = client->dev.of_node;
	int retval;
	int skip_5_1 = 0;
	u16 bus_type;

	logError(1, "%s %s: driver spi ver: %s\n", tag, __func__,
		 FTS_TS_DRV_VERSION);

#ifdef I2C_INTERFACE
	logError(1, "%s I2C interface... \n", tag);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		logError(1, "%s Unsupported I2C functionality\n", tag);
		error = -EIO;
		goto ProbeErrorExit_0;
	}

	logError(0, "%s i2c address: %x \n", tag, client->addr);
	bus_type = BUS_I2C;
#else
	logError(1, "%s SPI interface... \n", tag);
	client->mode = SPI_MODE_0;
#ifndef SPI4_WIRE
	client->mode |= SPI_3WIRE;
#endif
	client->max_speed_hz = SPI_CLOCK_FREQ;
	client->bits_per_word = 8;
	if (spi_setup(client) < 0) {
		logError(1, "%s Unsupported SPI functionality\n", tag);
		error = -EIO;
		goto ProbeErrorExit_0;
	}
	bus_type = BUS_SPI;
#endif

	logError(0, "%s SET Device driver INFO: \n", tag);

	info = kzalloc(sizeof(struct fts_ts_info), GFP_KERNEL);
	if (!info) {
		logError(
			1,
			"%s Out of memory... Impossible to allocate struct info!\n",
			tag);
		error = -ENOMEM;
		goto ProbeErrorExit_0;
	}

	fts_info = info;
	info->client = client;
	info->dev = &info->client->dev;
	dev_set_drvdata(info->dev, info);

	if (dp) {
		info->board = devm_kzalloc(&client->dev,
					   sizeof(struct fts_hw_platform_data),
					   GFP_KERNEL);
		if (!info->board) {
			logError(1, "%s ERROR:info.board kzalloc failed \n",
				 tag);
			error = -ENOMEM;
			goto ProbeErrorExit_1;
		}
		parse_dt(&client->dev, info->board);
	}
	logError(0, "%s SET GPIOS: \n", tag);
	info->gpio_has_request = false;
	retval = fts_set_gpio(info, true);
	if (retval < 0) {
		logError(1, "%s %s: ERROR Failed to set up GPIO's\n", tag,
			 __func__);
		error = retval;
		goto ProbeErrorExit_1;
	}

	error = fts_pinctrl_init(info);

	if (!error && info->ts_pinctrl) {
		error = pinctrl_select_state(info->ts_pinctrl,
					     info->pinctrl_state_active);

		if (error < 0) {
			dev_err(&client->dev,
				"%s: Failed to select %s pinstate %d\n",
				__func__, PINCTRL_STATE_ACTIVE, error);
		}
	} else {
		dev_err(&client->dev, "%s: Failed to init pinctrl\n", __func__);
		goto ProbeErrorExit_1;
	}

	logError(0, "%s SET Regulators: \n", tag);
	retval = fts_get_reg(info, true);
	if (retval < 0) {
		logError(1, "%s ERROR: %s: Failed to get regulators\n", tag,
			 __func__);
		error = retval;
		goto ProbeErrorExit_2;
	}

	retval = fts_enable_reg(info, true);
	if (retval < 0) {
		logError(1, "%s %s: ERROR Failed to enable regulators\n", tag,
			 __func__);
		error = retval;
		goto ProbeErrorExit_3;
	}

	mdelay(3);
	retval = fts_set_gpio(info, false);
	if (retval < 0) {
		logError(1, "%s %s: ERROR Failed to set up GPIO's\n", tag,
			 __func__);
		error = retval;
		goto ProbeErrorExit_3_1;
	}

	info->client->irq = gpio_to_irq(info->board->irq_gpio);
	logError(1, "%s gpio_num:%d, irq:%d\n", tag, info->board->irq_gpio,
		 info->client->irq);

	logError(0, "%s SET Event Handler: \n", tag);

	info->event_wq =
		alloc_workqueue(FTS_EVENT_QUEUE_NAME,
				WQ_UNBOUND | WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!info->event_wq) {
		logError(1, "%s ERROR: Cannot create work thread\n", tag);
		error = -ENOMEM;
		goto ProbeErrorExit_4;
	}

	info->irq_wq = alloc_workqueue(
		FTS_IRQ_QUEUE_NAME, WQ_UNBOUND | WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!info->irq_wq) {
		logError(1, "%s ERROR: Cannot create irq work thread\n", tag);
		error = -ENOMEM;
		goto ProbeErrorExit_4;
	}

	INIT_WORK(&info->resume_work, fts_resume_work);
	INIT_WORK(&info->suspend_work, fts_suspend_work);
	INIT_WORK(&info->sleep_work, fts_ts_sleep_work);
	init_completion(&info->tp_reset_completion);

	logError(0, "%s SET Input Device Property: \n", tag);
	info->dev = &info->client->dev;
	info->input_dev = input_allocate_device();
	if (!info->input_dev) {
		logError(1, "%s ERROR: No such input device defined! \n", tag);
		error = -ENODEV;
		goto ProbeErrorExit_5;
	}
	info->input_dev->dev.parent = &client->dev;
	info->input_dev->name = FTS_TS_DRV_NAME;
	snprintf(fts_ts_phys, sizeof(fts_ts_phys), "%s/input0",
		 info->input_dev->name);
	info->input_dev->phys = fts_ts_phys;
	info->input_dev->id.bustype = bus_type;
	info->input_dev->id.vendor = 0x0001;
	info->input_dev->id.product = 0x0002;
	info->input_dev->id.version = 0x0100;
	input_set_drvdata(info->input_dev, info);

	__set_bit(EV_SYN, info->input_dev->evbit);
	__set_bit(EV_KEY, info->input_dev->evbit);
	__set_bit(EV_ABS, info->input_dev->evbit);
	__set_bit(BTN_TOUCH, info->input_dev->keybit);
	__set_bit(BTN_TOOL_FINGER, info->input_dev->keybit);
	/*__set_bit(BTN_TOOL_PEN, info->input_dev->keybit);*/

	input_mt_init_slots(info->input_dev, TOUCH_ID_MAX, INPUT_MT_DIRECT);

	/*input_mt_init_slots(info->input_dev, TOUCH_ID_MAX); */

	input_set_abs_params(info->input_dev, ABS_MT_POSITION_X, X_AXIS_MIN,
			     info->board->x_max - 1, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y, Y_AXIS_MIN,
			     info->board->y_max - 1, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MAJOR, AREA_MIN,
			     info->board->x_max, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MINOR, AREA_MIN,
			     info->board->y_max, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_WIDTH_MINOR, AREA_MIN,
			     AREA_MAX, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_WIDTH_MAJOR, AREA_MIN,
			     AREA_MAX, 0, 0);

	input_set_abs_params(info->input_dev, ABS_MT_DISTANCE, DISTANCE_MIN,
			     DISTANCE_MAX, 0, 0);

	mutex_init(&(info->input_report_mutex));

	spin_lock_init(&fts_int);

	/* register the multi-touch input device */
	error = input_register_device(info->input_dev);
	if (error) {
		logError(1, "%s ERROR: No such input device\n", tag);
		error = -ENODEV;
		goto ProbeErrorExit_5_1;
	}

	skip_5_1 = 1;
	/* track slots */
	info->touch_id = 0;
	info->temp_touch_id = 0;

	info->resume_bit = 1;
	info->lockdown_is_ok = false;
	logError(0, "%s Init Core Lib: \n", tag);
	initCore(info);
	/* init hardware device */
	logError(0, "%s Device Initialization: \n", tag);
	error = fts_init(info);
	if (error < OK) {
		logError(1, "%s Cannot initialize the device ERROR %08X\n", tag,
			 error);
		error = -ENODEV;
		goto ProbeErrorExit_6;
	}
	info->vsync_fps = 120;
#ifdef CONFIG_SECURE_TOUCH
	logError(1, "%s %s create secure touch file...\n", tag, __func__);
	error = fts_secure_init(info);
	if (error < 0) {
		logError(1, "%s %s init secure touch failed\n", tag, __func__);
		goto ProbeErrorExit_7;
	}
	logError(1, "%s %s create secure touch file successful\n", tag,
		 __func__);
	fts_secure_stop(info, 1);
#endif

#ifdef CONFIG_I2C_BY_DMA
	/*dma buf init*/
	info->dma_buf = (struct fts_dma_buf *)kzalloc(sizeof(*info->dma_buf),
						      GFP_KERNEL);
	if (!info->dma_buf) {
		logError(1, "%s %s:Error alloc mem failed!", tag, __func__);
		goto ProbeErrorExit_7;
	}
	mutex_init(&info->dma_buf->dmaBufLock);
	info->dma_buf->rdBuf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!info->dma_buf->rdBuf) {
		logError(1, "%s %s:Error alloc mem failed!", tag, __func__);
		goto ProbeErrorExit_7;
	}
	info->dma_buf->wrBuf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!info->dma_buf->wrBuf) {
		logError(1, "%s %s:Error alloc mem failed!", tag, __func__);
		goto ProbeErrorExit_7;
	}
#endif
	error = fts_get_lockdown_info(info->lockdown_info, info);

	if (error < OK)
		logError(1, "%s can't get lockdown info", tag);
	else {
		logError(
			1,
			"%s Lockdown:0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\n",
			tag, info->lockdown_info[0], info->lockdown_info[1],
			info->lockdown_info[2], info->lockdown_info[3],
			info->lockdown_info[4], info->lockdown_info[5],
			info->lockdown_info[6], info->lockdown_info[7]);
		info->lockdown_is_ok = true;
	}
	info->tp_selftest_proc =
		proc_create(FTS_TP_SELFTEST_NAME, 0644, NULL, &fts_selftest_ops);

#ifdef FTS_FW_UPDATE
#ifdef FW_UPDATE_ON_PROBE
	logError(1, "%s FW Update and Sensing Initialization: \n", tag);
	error = fts_fw_update(info, NULL, 0);
	if (error < OK) {
		logError(1,
			 "%s Cannot execute fw upgrade the device ERROR %08X\n",
			 tag, error);
		error = -ENODEV;
		goto ProbeErrorExit_7;
	}
#else
	logError(0, "%s SET Auto Fw Update: \n", tag);
	info->fwu_workqueue = alloc_workqueue(
		FTS_FWU_QUEUE_NAME, WQ_UNBOUND | WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!info->fwu_workqueue) {
		logError(1, "%s ERROR: Cannot create fwu work thread\n", tag);
		goto ProbeErrorExit_7;
	}
	INIT_DELAYED_WORK(&info->fwu_work, fts_fw_update_auto);
#endif
#else
	error = fts_init_sensing(info);
	if (error < OK) {
		logError(
			1,
			"%s Cannot initialize the hardware device ERROR %08X\n",
			tag, error);
	}
#endif
	info->sensor_scan = true;

	logError(0, "%s SET Device File Nodes: \n", tag);
	/* sysfs stuff */
	info->attrs.attrs = fts_attr_group;
	error = sysfs_create_group(&client->dev.kobj, &info->attrs);
	if (error) {
		logError(1, "%s ERROR: Cannot create sysfs structure!\n", tag);
		error = -ENODEV;
		goto ProbeErrorExit_7;
	}

	error = fts_proc_init();
	if (error < OK)
		logError(1, "%s Error: can not create /proc file! \n", tag);

	device_init_wakeup(&client->dev, 1);
	init_completion(&info->pm_resume_completion);

#ifdef FTS_DEBUG_FS
	info->debugfs = debugfs_create_dir(FTS_DEBUGFS_DIR_NAME, NULL);
	if (info->debugfs) {
		debugfs_create_file("switch_state", 0660, info->debugfs, info,
				    &tpdbg_operations);
	}
#endif

	if (info->fts_tp_class == NULL)
		info->fts_tp_class = class_create("touch");
	info->fts_touch_dev = device_create(info->fts_tp_class, NULL,
					    DCHIP_ID_0, info, FTS_TOUCH_DEV_NAME);

	if (IS_ERR(info->fts_touch_dev)) {
		logError(1,
			 "%s ERROR: Failed to create device for the sysfs!\n",
			 tag);
		goto ProbeErrorExit_8;
	}

	dev_set_drvdata(info->fts_touch_dev, info);
	error = sysfs_create_file(&info->fts_touch_dev->kobj,
				  &dev_attr_ellipse_data.attr);
	if (error) {
		logError(
			1,
			"%s Error: Failed to create ellipse_data sysfs group!\n",
			tag);
	}
	info->tp_lockdown_info_proc = proc_create(FTS_TP_LOCKDOWN_INFO_NAME, 0444,
						  NULL, &fts_lockdown_info_ops);
	info->tp_data_dump_proc =
		proc_create(FTS_TP_DATA_DUMP_NAME, 0444, NULL, &fts_datadump_ops);
	info->tp_fw_version_proc =
		proc_create(FTS_TP_FW_VERSION_NAME, 0444, NULL, &fts_fw_version_ops);

#ifndef FW_UPDATE_ON_PROBE
	queue_delayed_work(info->fwu_workqueue, &info->fwu_work,
			   msecs_to_jiffies(EXP_FN_WORK_DELAY_MS));
#endif
	info->probe_ok = true;

	logError(1, "%s Probe Finished! \n", tag);
	return OK;
ProbeErrorExit_8:
	fts_disableInterrupt();
	device_destroy(info->fts_tp_class, DCHIP_ID_0);
	if (info->tp_lockdown_info_proc)
		remove_proc_entry(FTS_TP_LOCKDOWN_INFO_NAME, NULL);
	if (info->tp_data_dump_proc)
		remove_proc_entry(FTS_TP_DATA_DUMP_NAME, NULL);
	if (info->tp_fw_version_proc)
		remove_proc_entry(FTS_TP_FW_VERSION_NAME, NULL);
	info->tp_lockdown_info_proc = NULL;
	info->tp_data_dump_proc = NULL;
	info->tp_fw_version_proc = NULL;
/*
	class_destroy(info->fts_tp_class);
	info->fts_tp_class = NULL;
*/
ProbeErrorExit_7:
	if (info->tp_selftest_proc)
		remove_proc_entry(FTS_TP_SELFTEST_NAME, NULL);
	info->tp_selftest_proc = NULL;
#ifdef CONFIG_SECURE_TOUCH
	fts_secure_remove(info);
#endif
#ifdef CONFIG_I2C_BY_DMA
	if (info->dma_buf)
		kfree(info->dma_buf);
	if (info->dma_buf->rdBuf)
		kfree(info->dma_buf->rdBuf);
	if (info->dma_buf->wrBuf)
		kfree(info->dma_buf->wrBuf);
#endif
ProbeErrorExit_6:
	input_unregister_device(info->input_dev);

ProbeErrorExit_5_1:
	if (skip_5_1 != 1)
		input_free_device(info->input_dev);

ProbeErrorExit_5:
	destroy_workqueue(info->event_wq);

ProbeErrorExit_4:
	fts_gpio_setup(info->board->irq_gpio, false, 0, 0);
	fts_gpio_setup(info->board->reset_gpio, false, 0, 0);

ProbeErrorExit_3_1:
	fts_enable_reg(info, false);

ProbeErrorExit_3:
	fts_get_reg(info, false);

ProbeErrorExit_2:
	if (info->ts_pinctrl)
		devm_pinctrl_put(info->ts_pinctrl);

ProbeErrorExit_1:
	kfree(info);

ProbeErrorExit_0:
	logError(1, "%s Probe Failed!\n", tag);

	return error;
}

/**
 * Clear and free all the resources associated to the driver.
 * This function is called when the driver need to be removed.
 */
#ifdef I2C_INTERFACE
static void fts_remove(struct i2c_client *client)
{
#else
static void fts_remove(struct spi_device *client)
{
#endif

	struct fts_ts_info *info = dev_get_drvdata(&(client->dev));

	fts_proc_remove();
	if (info->tp_lockdown_info_proc)
		remove_proc_entry(FTS_TP_LOCKDOWN_INFO_NAME, NULL);
	if (info->tp_selftest_proc)
		remove_proc_entry(FTS_TP_SELFTEST_NAME, NULL);
	if (info->tp_data_dump_proc)
		remove_proc_entry(FTS_TP_DATA_DUMP_NAME, NULL);
	if (info->tp_fw_version_proc)
		remove_proc_entry(FTS_TP_FW_VERSION_NAME, NULL);
	info->tp_lockdown_info_proc = NULL;
	info->tp_selftest_proc = NULL;
	info->tp_data_dump_proc = NULL;
	info->tp_fw_version_proc = NULL;

	/* sysfs stuff */
	sysfs_remove_group(&client->dev.kobj, &info->attrs);
	/* remove interrupt and event handlers */
	fts_interrupt_uninstall(info);
	/* unregister the device */
	input_unregister_device(info->input_dev);

	/* Remove the work thread */
	destroy_workqueue(info->event_wq);
#ifndef FW_UPDATE_ON_PROBE
	destroy_workqueue(info->fwu_workqueue);
#endif
	device_destroy(info->fts_tp_class, DCHIP_ID_0);
	if (info->debugfs)
		debugfs_remove(info->debugfs);

	fts_enable_reg(info, false);
	fts_get_reg(info, false);
	fts_gpio_setup(info->board->irq_gpio, false, 0, 0);
	fts_gpio_setup(info->board->reset_gpio, false, 0, 0);
	fts_info = NULL;
#ifdef CONFIG_SECURE_TOUCH
	fts_secure_remove(info);
#endif
	/* free all */
	kfree(info);
}

/**
* Struct which contains the compatible names that need to match with the definition of the device in the device tree node
*/
static struct of_device_id fts_of_match_table[] = {
	{
		.compatible = "st,spi",
	},
	{},
};
MODULE_DEVICE_TABLE(of, fts_of_match_table);

#ifdef I2C_INTERFACE
static const struct i2c_device_id fts_device_id[] = {
	{
		"st,spi", 0
	},
	{}
};
MODULE_DEVICE_TABLE(i2c, fts_device_id);

static struct i2c_driver fts_i2c_driver = {
	.driver = {
		   .name = FTS_TS_DRV_NAME,
		   .of_match_table = fts_of_match_table,
#ifdef CONFIG_PM
		   .pm = &fts_dev_pm_ops,
#endif
		   },
	.probe = fts_probe,
	.remove = fts_remove,
	.id_table = fts_device_id,
};
#else
static const struct spi_device_id fts_device_id[] = {
	{
		"st,spi", 0
	},
	{}
};

MODULE_DEVICE_TABLE(spi, fts_device_id);

static struct spi_driver fts_spi_driver = {
	.driver = {
		   .name = FTS_TS_DRV_NAME,
		   .of_match_table = fts_of_match_table,
#ifdef CONFIG_PM
		   .pm = &fts_dev_pm_ops,
#endif
		   .owner = THIS_MODULE,
		   },
	.probe = fts_probe,
	.remove = fts_remove,
	.id_table = fts_device_id,
};
#endif

static int __init fts_driver_init(void)
{
#ifdef I2C_INTERFACE
	return i2c_add_driver(&fts_i2c_driver);
#else
	return spi_register_driver(&fts_spi_driver);
#endif
}

static void __exit fts_driver_exit(void)
{
#ifdef I2C_INTERFACE
	i2c_del_driver(&fts_i2c_driver);
#else
	spi_unregister_driver(&fts_spi_driver);
#endif
}

MODULE_DESCRIPTION("STMicroelectronics MultiTouch IC Driver");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL");

late_initcall(fts_driver_init);
module_exit(fts_driver_exit);
