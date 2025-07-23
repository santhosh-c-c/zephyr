/*
 * Copyright (c) 2025 Linumiz GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_mspm0_dac

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>

/* TI Driverlib includes */
#include <ti/devices/msp/peripherals/hw_dac12.h>
#include <ti/driverlib/dl_dac12.h>

/* 12-bit DAC Binary Representation Range */
#define DAC12_BINARY_MAX 4095
#define DAC12_BINARY_MIN 0

/* 12-bit DAC Two's Complement Representation Range */
#define DAC12_TWOS_COMP_MAX 2047
#define DAC12_TWOS_COMP_MIN (-2048)

/* 8-bit DAC Binary Representation Range */
#define DAC8_BINARY_MAX 255
#define DAC8_BINARY_MIN 0

/* 8-bit DAC Two's Complement Representation Range */
#define DAC8_TWOS_COMP_MAX 127
#define DAC8_TWOS_COMP_MIN (-128)

/* Timeout to wait until the DAC module is ready after enabling */
#define DAC_MOD_RDY_TIMEOUT K_MSEC(CONFIG_DAC_MSPM0_TIMEOUT_MS)

struct dac_mspm0_config {
	DAC12_Regs *dac_base;
	DL_DAC12_REPRESENTATION dac_repr;
	DL_DAC12_VREF_SOURCE dac_vref_src;
	DL_DAC12_FIFO dac_fifo_en;
	DL_DAC12_FIFO_TRIGGER dac_fifo_trig;
	DL_DAC12_DMA_TRIGGER dac_dma_trig;
	DL_DAC12_FIFO_THRESHOLD dac_fifo_th;
	DL_DAC12_SAMPLETIMER dac_samp_timer;
	DL_DAC12_SAMPLES_PER_SECOND dac_samp_ps;
	void (*irq_config_func)(const struct device *dev);
};

struct dac_mspm0_data {
	const struct device *dev;
	uint8_t resolution;
	uint32_t copy_value;
	struct k_sem mod_rdy;
	struct k_work fifo_fill_work;
};

/* TODO remove print, DMA DONE ? use sem/mutex ?, work queue for fifo filling like k_work submit? 
 * Link fifo_full with DMA - Don't trigDMA when fifo full
 */
static void dac_mspm0_isr(const struct device *dev)
{
	const struct dac_mspm0_config *config = dev->config;
	struct dac_mspm0_data *data = dev->data;
	DAC12_Regs *dac12 = config->dac_base;
	DL_DAC12_IIDX pending_intr;

	pending_intr = DL_DAC12_getPendingInterrupt(dac12);

	switch (pending_intr) {
	case DL_DAC12_IIDX_MODULE_READY:
		printk("*-----DAC Module Ready-----*\n");
		k_sem_give(&data->mod_rdy);
		break;

	case DL_DAC12_IIDX_FIFO_1_4_EMPTY:
		if (pending_intr == DL_DAC12_IIDX_FIFO_1_4_EMPTY) {
			printk("------DAC FIFO 1/4 Empty interrupt-------\n");
		}

	case DL_DAC12_IIDX_FIFO_1_2_EMPTY:
		if (pending_intr == DL_DAC12_IIDX_FIFO_1_2_EMPTY) {
			printk("------DAC FIFO 1/2 Empty interrupt------\n");
		}

	case DL_DAC12_IIDX_FIFO_3_4_EMPTY:
		if (pending_intr == DL_DAC12_IIDX_FIFO_3_4_EMPTY) {
			printk("------DAC FIFO 1/4 Empty interrupt-------\n");
		}

	case DL_DAC12_IIDX_FIFO_EMPTY:
		if (pending_intr == DL_DAC12_IIDX_FIFO_EMPTY) {
			printk("------DAC FIFO Empty interrupt------\n");
		}

	case DL_DAC12_IIDX_FIFO_UNDERRUN:
		if (pending_intr == DL_DAC12_IIDX_FIFO_UNDERRUN) {
			printk("------DAC FIFO Empty interrupt------\n");
		}

	ret = k_work_submit(&data->fifo_fill_work);
		break;

	default:
		break;
	}
}

/*TODO remove written, remove print, remove test add pointer onto data given by user to fill fifo*/
static void fifo_fill_work_handler(struct k_work *work)
{
	struct dac_mspm0_data *data = CONTAINER_OF(work, struct dac_mspm0_data, fifo_fill_work);
	const struct device *dev = data->dev;
	
	const struct dac_mspm0_config *config = dev->config;
	DAC12_Regs *dac12 = config->dac_base;
	
	uint32_t count = 4, written = 0;

	if (data->resolution == 12) {
		uint16_t temp_buffer[4];
		for (int i = 0; i < count; i++) {
			temp_buffer[i] = (uint16_t)data->copy_value;
		}
		written = DL_DAC12_fillFIFO12(dac12, temp_buffer, count);

	} else {
		uint8_t temp_buffer[4];
		for (int i = 0; i < count; i++) {
			temp_buffer[i] = (uint8_t)data->copy_value;
		}
		written = DL_DAC12_fillFIFO8(dac12, temp_buffer, count);
	}

	printk("------FIFO filled with %u samples------\n", written);
	// Check if sample time generator is enabled

	if (!DL_DAC12_isSampleTimeGeneratorEnabled(dac12)) {
		printk("Sample Timer Generator is DISABLED. Enabling now...\n");
		DL_DAC12_enableSampleTimeGenerator(dac12);
	}

#if 1
				/**************TEST***********/
		DL_DAC12_FIFO_TRIGGER fifo_trig = DL_DAC12_getFIFOTriggerSource(dac12);

		switch (fifo_trig) {
		case DL_DAC12_FIFO_TRIGGER_SAMPLETIMER:
			printk("FIFO Trigger Source: Sample Timer Generator\n");
			break;
		case DL_DAC12_FIFO_TRIGGER_HWTRIG0:
			printk("FIFO Trigger Source: Hardware Trigger\n");
			break;
		default:
			break;
		}

		// Check if sample time generator is enabled
		if (!DL_DAC12_isSampleTimeGeneratorEnabled(dac12)) {
			printk("Sample Timer Generator is DISABLED. Enabling now...\n");
			DL_DAC12_enableSampleTimeGenerator(dac12);
		} else {
			printk("Sample Timer Generator is already ENABLED.\n");
		}

		// Get and print the sample rate
		DL_DAC12_SAMPLES_PER_SECOND rate = DL_DAC12_getSampleRate(dac12);
		switch (rate) {
		case DL_DAC12_SAMPLES_PER_SECOND_500:
			printk("Sample Timer Rate: 500 sample/sec\n");
			break;
		case DL_DAC12_SAMPLES_PER_SECOND_1K:
			printk("Sample Timer Rate: 1K samples/sec\n");
		default:
			break;
		}
				/**************TEST***********/
#endif

}

static int dac_mspm0_channel_setup(const struct device *dev,
				   const struct dac_channel_cfg *channel_cfg)
{
	const struct dac_mspm0_config *config = dev->config;
	struct dac_mspm0_data *data = dev->data;
	DAC12_Regs *dac12 = config->dac_base;

	if (channel_cfg->channel_id != 0) {
		return -EINVAL;
	}

	if (channel_cfg->resolution != 8 && channel_cfg->resolution != 12) {
		return -ENOTSUP;
	}

	data->resolution = channel_cfg->resolution;

	DL_DAC12_RESOLUTION dac_res = (channel_cfg->resolution == 12) ?
		DL_DAC12_RESOLUTION_12BIT : DL_DAC12_RESOLUTION_8BIT;

	DL_DAC12_AMP dac_amp_set = channel_cfg->buffered ?
		DL_DAC12_AMP_ON : DL_DAC12_AMP_OFF_TRISTATE;

	DL_DAC12_OUTPUT dac_op_en = channel_cfg->internal ?
		DL_DAC12_OUTPUT_ENABLED : DL_DAC12_OUTPUT_DISABLED;

	DL_DAC12_Config cfg = {
		.outputEnable 		   = dac_op_en,
		.resolution                = dac_res,
		.amplifierSetting          = dac_amp_set,
		.representation            = config->dac_repr,
		.voltageReferenceSource    = config->dac_vref_src,
		.fifoEnable                = config->dac_fifo_en,
		.fifoTriggerSource         = config->dac_fifo_trig,
		.dmaTriggerEnable          = config->dac_dma_trig,
		.dmaTriggerThreshold       = config->dac_fifo_th,
		.sampleTimeGeneratorEnable = config->dac_samp_timer,
		.sampleRate                = config->dac_samp_ps,
	};

	/* Ensure DAC is disabled before configuration */
	if (DL_DAC12_isEnabled(dac12)) {
		DL_DAC12_disable(dac12);
	}

	DL_DAC12_init(dac12, &cfg);
	k_sem_reset(&data->mod_rdy);

	DL_DAC12_enable(dac12);
	if (!DL_DAC12_isEnabled(dac12)) {
		return -EAGAIN;
	}

	if (k_sem_take(&data->mod_rdy, DAC_MOD_RDY_TIMEOUT) != 0) {
		return -ETIMEDOUT;
	}

	DL_DAC12_performSelfCalibrationBlocking(dac12);
	DL_DAC12_enableInterrupt(dac12, DL_DAC12_INTERRUPT_FIFO_EMPTY
			| DL_DAC12_INTERRUPT_FIFO_THREE_QTRS_EMPTY
			| DL_DAC12_INTERRUPT_FIFO_TWO_QTRS_EMPTY
			| DL_DAC12_INTERRUPT_FIFO_ONE_QTR_EMPTY
			| DL_DAC12_INTERRUPT_FIFO_UNDERRUN);
 
			//| DL_DAC12_INTERRUPT_FIFO_FULL

	/*-----------------TEST-----------------*/
	if(DL_DAC12_isOutputPinEnabled(dac12))
	{
		printk("\nOP ENABLE\n");
	}
	if(DL_DAC12_isFIFOEnabled(dac12))
	{
		printk("FIFO ENABLE\n");
	}
	if(DL_DAC12_isSampleTimeGeneratorEnabled(dac12))
	{
		printk("STG ENABLE\n");
	}
	DL_DAC12_AMP amp_setting = DL_DAC12_getAmplifier(dac12);
	if (amp_setting == DL_DAC12_AMP_OFF_TRISTATE) {
		printk("Amplifier setting: DL_DAC12_AMP_OFF_TRISTATE (High impedance)\n");
	} else if (amp_setting == DL_DAC12_AMP_OFF_0V) {
		printk("Amplifier setting: DL_DAC12_AMP_OFF_0V (Pulled to ground)\n");
	} else if (amp_setting == DL_DAC12_AMP_ON) {
		printk("Amplifier setting: DL_DAC12_AMP_ON (Enabled)\n");
	}
	/*-----------------TEST-----------------*/

	return 0;
}

static int dac_mspm0_write_value(const struct device *dev, uint8_t channel, uint32_t value)
{
	const struct dac_mspm0_config *config = dev->config;
	struct dac_mspm0_data *data = dev->data;
	DAC12_Regs *dac12 = config->dac_base;

	if (!DL_DAC12_isEnabled(dac12)) {
		return -EAGAIN;
	}

	if (channel != 0) {
		return -EINVAL;
	}
	
	data->copy_value = value;

	if (data->resolution == 12) {
		if ((config->dac_repr == DL_DAC12_REPRESENTATION_BINARY) &&
		    (value > DAC12_BINARY_MAX || value < DAC12_BINARY_MIN)) {
			return -EINVAL;
		}

		if ((config->dac_repr == DL_DAC12_REPRESENTATION_TWOS_COMPLEMENT) &&
		    ((int32_t)value > DAC12_TWOS_COMP_MAX ||
		     (int32_t)value < DAC12_TWOS_COMP_MIN)) {
			return -EINVAL;
		}

		DL_DAC12_output12(dac12, value);

	} else if (data->resolution == 8) {
		if ((config->dac_repr == DL_DAC12_REPRESENTATION_BINARY) &&
		    (value > DAC8_BINARY_MAX || value < DAC8_BINARY_MIN)) {
			return -EINVAL;
		}

		if ((config->dac_repr == DL_DAC12_REPRESENTATION_TWOS_COMPLEMENT) &&
		    ((int32_t)value > DAC8_TWOS_COMP_MAX || (int32_t)value < DAC8_TWOS_COMP_MIN)) {
			return -EINVAL;
		}

		DL_DAC12_output8(dac12, (uint8_t)value);

	} else {
		/* Setup must be done with valid resolution */
		return -EINVAL;
	}
	
	printk("\ndata->copy_value : %d\n", data->copy_value);
	return 0;
}

static int dac_mspm0_init(const struct device *dev)
{
	const struct dac_mspm0_config *config = dev->config;
	struct dac_mspm0_data *data = dev->data;
	DAC12_Regs *dac12 = config->dac_base;

	DL_DAC12_enablePower(dac12);
	if (!DL_DAC12_isPowerEnabled(dac12)) {
		return -EAGAIN;
	}

	k_sem_init(&data->mod_rdy, 0, 1);
	k_work_init(&data->fifo_fill_work, fifo_fill_work_handler);	

	DL_DAC12_disableInterrupt(dac12,
			DL_DAC12_INTERRUPT_MODULE_READY | DL_DAC12_INTERRUPT_FIFO_EMPTY
			| DL_DAC12_INTERRUPT_FIFO_THREE_QTRS_EMPTY
			| DL_DAC12_INTERRUPT_FIFO_TWO_QTRS_EMPTY
			| DL_DAC12_INTERRUPT_FIFO_ONE_QTR_EMPTY
			| DL_DAC12_INTERRUPT_FIFO_FULL
			| DL_DAC12_INTERRUPT_FIFO_UNDERRUN);

	DL_DAC12_clearInterruptStatus(dac12, 
			DL_DAC12_INTERRUPT_MODULE_READY | DL_DAC12_INTERRUPT_FIFO_EMPTY
			| DL_DAC12_INTERRUPT_FIFO_THREE_QTRS_EMPTY
			| DL_DAC12_INTERRUPT_FIFO_TWO_QTRS_EMPTY
			| DL_DAC12_INTERRUPT_FIFO_ONE_QTR_EMPTY
			| DL_DAC12_INTERRUPT_FIFO_FULL
			| DL_DAC12_INTERRUPT_FIFO_UNDERRUN);

	config->irq_config_func(dev);
	DL_DAC12_enableInterrupt(dac12, DL_DAC12_INTERRUPT_MODULE_READY);

	return 0;
}

static DEVICE_API(dac, dac_mspm0_driver_api) = {
	.channel_setup = dac_mspm0_channel_setup,
	.write_value   = dac_mspm0_write_value
};

#define DAC_MSPM0_DEFINE(id)								\
											\
	static void dac_mspm0_irq_config_##id(const struct device *dev)			\
	{										\
		IRQ_CONNECT(DT_INST_IRQN(id), DT_INST_IRQ(id, priority), dac_mspm0_isr, \
				DEVICE_DT_INST_GET(id), 0);				\
		irq_enable(DT_INST_IRQN(id));						\
	};										\
											\
	static const struct dac_mspm0_config dac_mspm0_config_##id = {			\
		.dac_base = (DAC12_Regs *)DT_INST_REG_ADDR(id),				\
											\
		.dac_repr = IS_ENABLED(CONFIG_DAC_MSPM0_DFM_TWOS_COMPLEMENT)		\
					? DL_DAC12_REPRESENTATION_TWOS_COMPLEMENT	\
					: DL_DAC12_REPRESENTATION_BINARY,		\
											\
		.dac_vref_src   = DL_DAC12_VREF_SOURCE_VDDA_VSSA,			\
		.dac_fifo_en    = DL_DAC12_FIFO_DISABLED,				\
		.dac_fifo_trig  = DL_DAC12_FIFO_TRIGGER_SAMPLETIMER,			\
		.dac_dma_trig   = DL_DAC12_DMA_TRIGGER_DISABLED,			\
		.dac_fifo_th    = DL_DAC12_FIFO_THRESHOLD_THREE_QTRS_EMPTY,		\
		.dac_samp_timer = DL_DAC12_SAMPLETIMER_DISABLE,				\
		.dac_samp_ps    = DL_DAC12_SAMPLES_PER_SECOND_1K,			\
		.irq_config_func = dac_mspm0_irq_config_##id				\
	};										\
											\
	static struct dac_mspm0_data dac_mspm0_data_##id = {				\
		/* Config at channel setup */						\
		.resolution = 0,							\
		.copy_value = 0,							\
	};										\
											\
	DEVICE_DT_INST_DEFINE(id, &dac_mspm0_init, NULL, &dac_mspm0_data_##id,		\
			&dac_mspm0_config_##id,	POST_KERNEL, CONFIG_DAC_INIT_PRIORITY,	\
			&dac_mspm0_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DAC_MSPM0_DEFINE)
