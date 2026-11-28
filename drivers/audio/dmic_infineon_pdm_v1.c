/*
 * Copyright (c) 2026 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * PSOC 6 MXAUDIOSS v1 PDM-PCM, not the MXPDM v2 peripheral.
 */
#define DT_DRV_COMPAT infineon_pdm_v1

#include <cy_pdm_pcm.h>
#include <cy_sysclk.h>
#include <errno.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dmic_infineon_pdm_v1, CONFIG_AUDIO_DMIC_LOG_LEVEL);

BUILD_ASSERT(CY_IP_MXAUDIOSS_VERSION == 1, "This driver requires MXAUDIOSS v1");

struct pdm_v1_config {
	PDM_Type *base;
	const struct pinctrl_dev_config *pinctrl;
	void (*connect_irq)(const struct device *dev);
};

struct pdm_v1_data {
	volatile enum dmic_state state;
	struct k_mem_slab *slab;
	struct k_msgq queue;
	char queue_storage[CONFIG_DMIC_INFINEON_PDM_V1_QUEUE_SIZE * sizeof(void *)];
	void *active;
	size_t used;
	size_t block_size;
	uint16_t discard_words;
	/* dmic_read() calls between their state snapshot and return. */
	uint8_t readers;
};

/* Called with interrupts locked, or from this device's ISR. */
static void quiesce(const struct device *dev)
{
	const struct pdm_v1_config *config = dev->config;

	Cy_PDM_PCM_SetInterruptMask(config->base, 0);
	Cy_PDM_PCM_Disable(config->base);
	Cy_PDM_PCM_ClearInterrupt(config->base, CY_PDM_PCM_INTR_MASK);
}

static void release_buffers(const struct device *dev)
{
	struct pdm_v1_data *data = dev->data;
	void *buffer;

	if (data->active != NULL) {
		k_mem_slab_free(data->slab, data->active);
		data->active = NULL;
	}
	while (k_msgq_get(&data->queue, &buffer, K_NO_WAIT) == 0) {
		/* NULL is a wakeup sentinel, not a slab block. */
		if (buffer != NULL) {
			k_mem_slab_free(data->slab, buffer);
		}
	}
	data->used = 0;
}

/*
 * Wake a reader blocked in pdm_v1_read() once capture has stopped. The NULL
 * sentinel makes it return -EIO and is never handed to the caller. If the
 * queue is full, no reader can be waiting on it. Sentinels left without a
 * reader are purged by configure and START.
 */
static void wake_reader(struct pdm_v1_data *data)
{
	void *sentinel = NULL;

	(void)k_msgq_put(&data->queue, &sentinel, K_NO_WAIT);
}

static void capture_error(const struct device *dev)
{
	struct pdm_v1_data *data = dev->data;

	quiesce(dev);
	data->state = DMIC_STATE_ERROR;
	/* Keep queued and active slabs until STOP/RESET, never recycle a caller's slab. */
	wake_reader(data);
}

static void pdm_v1_isr(const struct device *dev)
{
	const struct pdm_v1_config *config = dev->config;
	struct pdm_v1_data *data = dev->data;
	uint32_t status = Cy_PDM_PCM_GetInterruptStatusMasked(config->base);

	if (data->state != DMIC_STATE_ACTIVE) {
		Cy_PDM_PCM_ClearInterrupt(config->base, status);
		return;
	}
	if (status & (CY_PDM_PCM_INTR_RX_OVERFLOW | CY_PDM_PCM_INTR_RX_UNDERFLOW)) {
		capture_error(dev);
		return;
	}
	/* Process complete L/R pairs only. Never reset FIFO while the converter is active. */
	uint8_t words = Cy_PDM_PCM_GetNumInFifo(config->base) & ~1U;

	for (uint8_t i = 0; i < words; i += 2) {
		uint16_t left = (uint16_t)Cy_PDM_PCM_ReadFifo(config->base);
		uint16_t right = (uint16_t)Cy_PDM_PCM_ReadFifo(config->base);

		if (data->discard_words != 0) {
			data->discard_words -= 2;
			continue;
		}
		uint16_t *samples = (uint16_t *)((uint8_t *)data->active + data->used);

		samples[0] = left;
		samples[1] = right;
		data->used += 4;
		if (data->used == data->block_size) {
			if (k_msgq_put(&data->queue, &data->active, K_NO_WAIT) != 0) {
				capture_error(dev);
				return;
			}
			data->active = NULL;
			data->used = 0;
			if (k_mem_slab_alloc(data->slab, &data->active, K_NO_WAIT) != 0) {
				capture_error(dev);
				return;
			}
		}
	}
	Cy_PDM_PCM_ClearInterrupt(config->base, status);
}

static int pdm_v1_configure(const struct device *dev, struct dmic_cfg *requested)
{
	const struct pdm_v1_config *config = dev->config;
	struct pdm_v1_data *data = dev->data;
	const uint32_t stereo_map = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT) |
				    dmic_build_channel_map(1, 0, PDM_CHAN_RIGHT);

	if (requested == NULL || requested->streams == NULL) {
		return -EINVAL;
	}
	if (data->state == DMIC_STATE_ACTIVE || data->state == DMIC_STATE_PAUSED) {
		return -EBUSY;
	}
	struct pcm_stream_cfg *stream = &requested->streams[0];

	if (requested->channel.req_num_streams != 1 || requested->channel.req_num_chan != 2 ||
	    requested->channel.req_chan_map_lo != stereo_map ||
	    requested->channel.req_chan_map_hi != 0 || stream->pcm_rate != 15625 ||
	    stream->pcm_width != 16 || stream->mem_slab == NULL || stream->block_size == 0 ||
	    stream->block_size % 4 != 0 || stream->block_size > stream->mem_slab->info.block_size) {
		return -EINVAL;
	}
	if (requested->io.pdm_clk_pol || requested->io.pdm_data_pol || requested->io.pdm_clk_skew ||
	    requested->io.min_pdm_clk_freq > 2000000 || requested->io.max_pdm_clk_freq < 2000000 ||
	    requested->io.min_pdm_clk_dc > 50 || requested->io.max_pdm_clk_dc < 50) {
		return -ENOTSUP;
	}
	/* HF1 belongs to AUDIOSS. The board clock driver owns its routing, not DMIC.
	 * HF0/FLL and the CM4, CM0+, PERI clocks are deliberately left unchanged.
	 */
	if (!Cy_SysClk_ClkHfIsEnabled(1) || Cy_SysClk_ClkHfGetFrequency(1) != 8000000) {
		LOG_ERR("PDMv1 requires enabled 8 MHz HF1");
		return -EIO;
	}
	int ret = pinctrl_apply_state(config->pinctrl, PINCTRL_STATE_DEFAULT);

	if (ret != 0) {
		return ret;
	}
	cy_stc_pdm_pcm_config_t pdl = {
		.clkDiv = CY_PDM_PCM_CLK_DIV_BYPASS,
		.mclkDiv = CY_PDM_PCM_CLK_DIV_BYPASS,
		.ckoDiv = 3, /* 8 MHz / (3 + 1) = 2 MHz, 50% duty */
		.ckoDelay = 3,
		.sincDecRate = 64, /* 2 MHz / (2 * 64) = 15625 stereo frames/s */
		.chanSelect = CY_PDM_PCM_OUT_STEREO,
		.chanSwapEnable = false,
		.highPassFilterGain = 11,
		.highPassDisable = false,
		.softMuteCycles = CY_PDM_PCM_SOFT_MUTE_CYCLES_96,
		.softMuteFineGain = 0,
		.softMuteEnable = false,
		.wordLen = CY_PDM_PCM_WLEN_16_BIT,
		.signExtension = true,
		.gainLeft = CY_PDM_PCM_BYPASS,
		.gainRight = CY_PDM_PCM_BYPASS,
		.rxFifoTriggerLevel = 63,
		.dmaTriggerEnable = false,
		.interruptMask = 0,
	};
	unsigned int key = irq_lock();

	/* A reader in flight reports the block size it snapshotted; keep it valid. */
	if (data->readers != 0) {
		irq_unlock(key);
		return -EBUSY;
	}
	quiesce(dev);
	release_buffers(dev);
	if (Cy_PDM_PCM_Init(config->base, &pdl) != CY_PDM_PCM_SUCCESS) {
		data->slab = NULL;
		data->block_size = 0;
		data->state = DMIC_STATE_INITIALIZED;
		irq_unlock(key);
		return -EIO;
	}
	data->slab = stream->mem_slab;
	data->block_size = stream->block_size;
	data->state = DMIC_STATE_CONFIGURED;
	requested->channel.act_num_streams = 1;
	requested->channel.act_num_chan = 2;
	requested->channel.act_chan_map_lo = stereo_map;
	requested->channel.act_chan_map_hi = 0;
	irq_unlock(key);
	return 0;
}

static int pdm_v1_trigger(const struct device *dev, enum dmic_trigger command)
{
	const struct pdm_v1_config *config = dev->config;
	struct pdm_v1_data *data = dev->data;
	int ret = 0;
	unsigned int key = irq_lock();

	switch (command) {
	case DMIC_TRIGGER_START:
		if (data->state != DMIC_STATE_CONFIGURED) {
			ret = -EIO;
			break;
		}
		release_buffers(dev); /* Purge stale wakeup sentinels. */
		ret = k_mem_slab_alloc(data->slab, &data->active, K_NO_WAIT);
		if (ret != 0) {
			break;
		}
		data->used = 0;
		data->discard_words = 128; /* Discard 64 stereo startup frames (>45 PDL cycles). */
		Cy_PDM_PCM_ClearFifo(config->base);
		Cy_PDM_PCM_ClearInterrupt(config->base, CY_PDM_PCM_INTR_MASK);
		data->state = DMIC_STATE_ACTIVE;
		Cy_PDM_PCM_SetInterruptMask(config->base, CY_PDM_PCM_INTR_RX_TRIGGER |
								  CY_PDM_PCM_INTR_RX_OVERFLOW |
								  CY_PDM_PCM_INTR_RX_UNDERFLOW);
		Cy_PDM_PCM_Enable(config->base);
		break;
	case DMIC_TRIGGER_STOP:
	case DMIC_TRIGGER_RESET:
		quiesce(dev);
		release_buffers(dev);
		if (command == DMIC_TRIGGER_RESET) {
			data->slab = NULL;
			data->block_size = 0;
		}
		data->state = data->slab != NULL ? DMIC_STATE_CONFIGURED : DMIC_STATE_INITIALIZED;
		wake_reader(data);
		break;
	default:
		ret = -ENOTSUP;
		break;
	}
	irq_unlock(key);
	return ret;
}

static int pdm_v1_read(const struct device *dev, uint8_t stream, void **buffer, size_t *size,
		       int32_t timeout)
{
	struct pdm_v1_data *data = dev->data;

	if (stream != 0 || buffer == NULL || size == NULL) {
		return -EINVAL;
	}
	*buffer = NULL;
	*size = 0;
	unsigned int key = irq_lock();
	enum dmic_state state = data->state;
	size_t block_size = data->block_size;

	if (state == DMIC_STATE_ACTIVE) {
		data->readers++;
	}
	irq_unlock(key);
	if (state != DMIC_STATE_ACTIVE) {
		return -EIO;
	}

	/* Wait without holding the IRQ lock so the ISR can complete blocks. */
	void *block;
	int ret = k_msgq_get(&data->queue, &block, SYS_TIMEOUT_MS(timeout));

	key = irq_lock();
	data->readers--;
	irq_unlock(key);
	if (ret != 0) {
		return ret;
	}
	if (block == NULL) {
		/* Woken by a capture error, STOP or RESET. */
		return -EIO;
	}
	/*
	 * Only configure() and RESET change block_size. configure() returns
	 * -EBUSY while a reader is in flight, and RESET leaves a wakeup sentinel
	 * that only configure() or START (after configure()) can purge, so a
	 * dequeued block always matches the size snapshotted with the state.
	 */
	*buffer = block;
	*size = block_size;
	return 0;
}

static int pdm_v1_init(const struct device *dev)
{
	const struct pdm_v1_config *config = dev->config;
	struct pdm_v1_data *data = dev->data;

	k_msgq_init(&data->queue, data->queue_storage, sizeof(void *),
		    CONFIG_DMIC_INFINEON_PDM_V1_QUEUE_SIZE);
	quiesce(dev);
	data->state = DMIC_STATE_INITIALIZED;
	config->connect_irq(dev);
	return 0;
}

static DEVICE_API(dmic, pdm_v1_api) = {
	.configure = pdm_v1_configure,
	.trigger = pdm_v1_trigger,
	.read = pdm_v1_read,
};

#define PDM_V1_DEFINE(inst)                                                                        \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static void pdm_v1_irq_##inst(const struct device *dev)                                    \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), pdm_v1_isr,           \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}                                                                                          \
	static struct pdm_v1_data pdm_v1_data_##inst;                                              \
	static const struct pdm_v1_config pdm_v1_config_##inst = {                                 \
		.base = (PDM_Type *)DT_INST_REG_ADDR(inst),                                        \
		.pinctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                   \
		.connect_irq = pdm_v1_irq_##inst,                                                  \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, pdm_v1_init, NULL, &pdm_v1_data_##inst, &pdm_v1_config_##inst, \
			      POST_KERNEL, CONFIG_AUDIO_DMIC_INIT_PRIORITY, &pdm_v1_api);

DT_INST_FOREACH_STATUS_OKAY(PDM_V1_DEFINE)
