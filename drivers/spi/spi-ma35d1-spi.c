// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Nuvoton MA35D1 SPI controller driver
 *
 * Copyright (c) 2026 Nuvoton Technology Corp.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/platform_data/dma-ma35d1.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/scatterlist.h>
#include <linux/spi/spi.h>

/* SPI register offsets */
#define MA35D1_SPI_CTL			0x00
#define MA35D1_SPI_CLKDIV		0x04
#define MA35D1_SPI_SSCTL		0x08
#define MA35D1_SPI_PDMACTL		0x0c
#define MA35D1_SPI_FIFOCTL		0x10
#define MA35D1_SPI_STATUS		0x14
#define MA35D1_SPI_STATUS2		0x18
#define MA35D1_SPI_FIFOCTL2		0x1c
#define MA35D1_SPI_TX			0x20
#define MA35D1_SPI_RX			0x30

/* SPI Control Register */
#define MA35D1_SPI_CTL_DATDIR		BIT(20)
#define MA35D1_SPI_CTL_REORDER		BIT(19)
#define MA35D1_SPI_CTL_SLAVE		BIT(18)
#define MA35D1_SPI_CTL_UNITIEN		BIT(17)
#define MA35D1_SPI_CTL_RXONLY		BIT(15)
#define MA35D1_SPI_CTL_HALFDPX		BIT(14)
#define MA35D1_SPI_CTL_LSB		BIT(13)
#define MA35D1_SPI_CTL_DWIDTH_MASK	GENMASK(12, 8)
#define MA35D1_SPI_CTL_SUSPITV_MASK	GENMASK(7, 4)
#define MA35D1_SPI_CTL_CLKPOL		BIT(3)
#define MA35D1_SPI_CTL_TXNEG		BIT(2)
#define MA35D1_SPI_CTL_RXNEG		BIT(1)
#define MA35D1_SPI_CTL_SPIEN		BIT(0)

/* SPI Clock Divider Register */
#define MA35D1_SPI_CLKDIV_MASK		GENMASK(8, 0)
#define MA35D1_SPI_CLKDIV_MAX		FIELD_MAX(MA35D1_SPI_CLKDIV_MASK)

/* SPI Slave Select Control Register */
#define MA35D1_SPI_SSCTL_AUTOSS		BIT(3)
#define MA35D1_SPI_SSCTL_SSACTPOL	BIT(2)
#define MA35D1_SPI_SSCTL_SS1		BIT(1)
#define MA35D1_SPI_SSCTL_SS0		BIT(0)

/* SPI PDMA Control Register */
#define MA35D1_SPI_PDMACTL_PDMARST	BIT(2)
#define MA35D1_SPI_PDMACTL_RXPDMAEN	BIT(1)
#define MA35D1_SPI_PDMACTL_TXPDMAEN	BIT(0)

/* SPI FIFO Control Register */
#define MA35D1_SPI_FIFOCTL_TXTH_MASK	GENMASK(31, 28)
#define MA35D1_SPI_FIFOCTL_RXTH_MASK	GENMASK(27, 24)
#define MA35D1_SPI_FIFOCTL_TXFBCLR	BIT(9)
#define MA35D1_SPI_FIFOCTL_RXFBCLR	BIT(8)
#define MA35D1_SPI_FIFOCTL_TXRST	BIT(1)
#define MA35D1_SPI_FIFOCTL_RXRST	BIT(0)

/* SPI Status Register */
#define MA35D1_SPI_STATUS_TXRXRST	BIT(23)
#define MA35D1_SPI_STATUS_FIFOCLR	BIT(22)
#define MA35D1_SPI_STATUS_TXFULL	BIT(17)
#define MA35D1_SPI_STATUS_TXEMPTY	BIT(16)
#define MA35D1_SPI_STATUS_SPIENSTS	BIT(15)
#define MA35D1_SPI_STATUS_RXEMPTY	BIT(8)
#define MA35D1_SPI_STATUS_BUSY		BIT(0)

/* SPI FIFO2 Control Register */
#define MA35D1_SPI_FIFOCTL2_TXTHPDMA_MASK	GENMASK(28, 24)
#define MA35D1_SPI_FIFOCTL2_RXTHPDMA_MASK	GENMASK(20, 16)

#define MA35D1_SPI_NUM_CS		2
#define MA35D1_SPI_MIN_BPW		8
#define MA35D1_SPI_MAX_BPW		32
#define MA35D1_SPI_MAX_MASTER_HZ	100000000U

/*
 * PDMA hardware transfer count is limited to 65536 transfers per
 * descriptor. Limit each SPI-core DMA SG segment to 64 KiB so every
 * segment fits for all supported PDMA widths (1, 2 and 4 bytes).
 */
#define MA35D1_SPI_MAX_DMA_SEGMENT		0x10000
#define MA35D1_SPI_DMA_MIN_BYTES	64

#define MA35D1_SPI_FIFO_THRESHOLD	4
#define MA35D1_SPI_POLL_TIMEOUT_US	1000000

struct ma35d1_spi {
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
	dma_addr_t phys_base;

	struct dma_chan *dma_tx;
	struct dma_chan *dma_rx;
	struct ma35d1_peripheral tx_peripheral;
	struct ma35d1_peripheral rx_peripheral;
	struct completion dma_tx_done;
	struct completion dma_rx_done;

	u32 parent_rate;
};

static u32 ma35d1_spi_read(struct ma35d1_spi *hw, u32 reg)
{
	return readl(hw->regs + reg);
}

static void ma35d1_spi_write(struct ma35d1_spi *hw, u32 reg, u32 val)
{
	writel(val, hw->regs + reg);
}

static void ma35d1_spi_update_bits(struct ma35d1_spi *hw, u32 reg,
				   u32 mask, u32 val)
{
	u32 tmp;

	tmp = ma35d1_spi_read(hw, reg);
	tmp &= ~mask;
	tmp |= val & mask;
	ma35d1_spi_write(hw, reg, tmp);
}

static int ma35d1_spi_wait_idle(struct ma35d1_spi *hw)
{
	u32 val;

	return readl_poll_timeout(hw->regs + MA35D1_SPI_STATUS, val,
				  !(val & MA35D1_SPI_STATUS_BUSY) &&
				  (val & MA35D1_SPI_STATUS_TXEMPTY) &&
				  (val & MA35D1_SPI_STATUS_RXEMPTY),
				  1, MA35D1_SPI_POLL_TIMEOUT_US);
}

static int ma35d1_spi_disable(struct ma35d1_spi *hw)
{
	u32 val;

	ma35d1_spi_update_bits(hw, MA35D1_SPI_CTL,
			       MA35D1_SPI_CTL_SPIEN, 0);

	return readl_poll_timeout(hw->regs + MA35D1_SPI_STATUS, val,
				  !(val & MA35D1_SPI_STATUS_SPIENSTS),
				  1, MA35D1_SPI_POLL_TIMEOUT_US);
}

static int ma35d1_spi_enable(struct ma35d1_spi *hw)
{
	u32 val;

	ma35d1_spi_update_bits(hw, MA35D1_SPI_CTL,
			       MA35D1_SPI_CTL_SPIEN,
			       MA35D1_SPI_CTL_SPIEN);

	return readl_poll_timeout(hw->regs + MA35D1_SPI_STATUS, val,
				  val & MA35D1_SPI_STATUS_SPIENSTS,
				  1, MA35D1_SPI_POLL_TIMEOUT_US);
}

static int ma35d1_spi_clear_fifos(struct ma35d1_spi *hw)
{
	u32 val;

	ma35d1_spi_update_bits(hw, MA35D1_SPI_FIFOCTL,
			       MA35D1_SPI_FIFOCTL_TXFBCLR |
			       MA35D1_SPI_FIFOCTL_RXFBCLR,
			       MA35D1_SPI_FIFOCTL_TXFBCLR |
			       MA35D1_SPI_FIFOCTL_RXFBCLR);

	return readl_poll_timeout(hw->regs + MA35D1_SPI_STATUS, val,
				  !(val & MA35D1_SPI_STATUS_FIFOCLR),
				  1, MA35D1_SPI_POLL_TIMEOUT_US);
}

static int ma35d1_spi_reset_fifos(struct ma35d1_spi *hw)
{
	u32 val;

	ma35d1_spi_update_bits(hw, MA35D1_SPI_FIFOCTL,
			       MA35D1_SPI_FIFOCTL_TXRST |
			       MA35D1_SPI_FIFOCTL_RXRST,
			       MA35D1_SPI_FIFOCTL_TXRST |
			       MA35D1_SPI_FIFOCTL_RXRST);

	return readl_poll_timeout(hw->regs + MA35D1_SPI_STATUS, val,
				  !(val & MA35D1_SPI_STATUS_TXRXRST),
				  1, MA35D1_SPI_POLL_TIMEOUT_US);
}

static int ma35d1_spi_reset_pdma(struct ma35d1_spi *hw)
{
	u32 val;

	ma35d1_spi_write(hw, MA35D1_SPI_PDMACTL,
			 MA35D1_SPI_PDMACTL_PDMARST);

	return readl_poll_timeout(hw->regs + MA35D1_SPI_PDMACTL, val,
				  !(val & MA35D1_SPI_PDMACTL_PDMARST),
				  1, MA35D1_SPI_POLL_TIMEOUT_US);
}

static unsigned int ma35d1_spi_word_bytes(unsigned int bits_per_word)
{
	if (bits_per_word <= 8)
		return 1;
	if (bits_per_word <= 16)
		return 2;

	return 4;
}

static u32 ma35d1_spi_dwidth(unsigned int bits_per_word)
{
	if (bits_per_word == 32)
		return 0;

	return bits_per_word;
}

static int ma35d1_spi_set_speed(struct ma35d1_spi *hw, u32 speed_hz,
				u32 *effective_speed_hz)
{
	u32 divider;
	u32 reg;

	if (!speed_hz)
		return -EINVAL;

	/*
	 * SPI_CLK = parent / (DIVIDER + 1), and hardware only permits odd
	 * values in DIVIDER. Therefore, the actual divisor must be even.
	 */
	divider = DIV_ROUND_UP(hw->parent_rate, speed_hz);
	divider = max(divider, 2U);
	if (divider & 1)
		divider++;

	if (divider > MA35D1_SPI_CLKDIV_MAX + 1)
		return -EINVAL;

	reg = divider - 1;
	ma35d1_spi_update_bits(hw, MA35D1_SPI_CLKDIV,
			       MA35D1_SPI_CLKDIV_MASK,
			       FIELD_PREP(MA35D1_SPI_CLKDIV_MASK, reg));

	*effective_speed_hz = hw->parent_rate / divider;

	return 0;
}

static int ma35d1_spi_config_transfer(struct ma35d1_spi *hw,
				      struct spi_device *spi,
				      struct spi_transfer *xfer,
				      unsigned int *bits_per_word)
{
	u32 ctl_mask;
	u32 ctl = 0;
	u32 speed_hz;
	u32 bpw;
	int ret;

	bpw = xfer->bits_per_word ?: spi->bits_per_word;
	if (!bpw)
		bpw = 8;

	if (bpw < MA35D1_SPI_MIN_BPW || bpw > MA35D1_SPI_MAX_BPW)
		return -EINVAL;

	if (xfer->len % ma35d1_spi_word_bytes(bpw))
		return -EINVAL;

	speed_hz = xfer->speed_hz ?: spi->max_speed_hz;

	ret = ma35d1_spi_wait_idle(hw);
	if (ret) {
		dev_err(hw->dev, "controller did not become idle: %d\n", ret);
		return ret;
	}

	ret = ma35d1_spi_disable(hw);
	if (ret) {
		dev_err(hw->dev, "failed to disable controller: %d\n", ret);
		return ret;
	}

	ret = ma35d1_spi_set_speed(hw, speed_hz, &xfer->effective_speed_hz);
	if (ret) {
		dev_err(hw->dev, "cannot generate SPI clock %u Hz: %d\n",
			speed_hz, ret);
		return ret;
	}

	ctl_mask = MA35D1_SPI_CTL_DATDIR |
		   MA35D1_SPI_CTL_REORDER |
		   MA35D1_SPI_CTL_SLAVE |
		   MA35D1_SPI_CTL_UNITIEN |
		   MA35D1_SPI_CTL_RXONLY |
		   MA35D1_SPI_CTL_HALFDPX |
		   MA35D1_SPI_CTL_LSB |
		   MA35D1_SPI_CTL_DWIDTH_MASK |
		   MA35D1_SPI_CTL_CLKPOL |
		   MA35D1_SPI_CTL_TXNEG |
		   MA35D1_SPI_CTL_RXNEG;

	ctl |= FIELD_PREP(MA35D1_SPI_CTL_DWIDTH_MASK,
			  ma35d1_spi_dwidth(bpw));

	if (spi->mode & SPI_CPOL)
		ctl |= MA35D1_SPI_CTL_CLKPOL;

	/*
	 * Mode 0/3: transmit on falling edge, receive on rising edge.
	 * Mode 1/2: transmit on rising edge, receive on falling edge.
	 */
	if (!!(spi->mode & SPI_CPOL) == !!(spi->mode & SPI_CPHA))
		ctl |= MA35D1_SPI_CTL_TXNEG;
	else
		ctl |= MA35D1_SPI_CTL_RXNEG;

	if (spi->mode & SPI_LSB_FIRST)
		ctl |= MA35D1_SPI_CTL_LSB;

	ma35d1_spi_update_bits(hw, MA35D1_SPI_CTL, ctl_mask, ctl);

	ret = ma35d1_spi_clear_fifos(hw);
	if (ret) {
		dev_err(hw->dev, "failed to clear FIFOs: %d\n", ret);
		return ret;
	}

	ret = ma35d1_spi_enable(hw);
	if (ret) {
		dev_err(hw->dev, "failed to enable controller: %d\n", ret);
		return ret;
	}

	*bits_per_word = bpw;

	return 0;
}

static u32 ma35d1_spi_load_word(const void *buf, size_t offset,
				unsigned int word_bytes)
{
	u32 val = 0;

	if (buf)
		memcpy(&val, (const u8 *)buf + offset, word_bytes);

	return val;
}

static void ma35d1_spi_store_word(void *buf, size_t offset,
				  unsigned int word_bytes, u32 val)
{
	if (buf)
		memcpy((u8 *)buf + offset, &val, word_bytes);
}

static int ma35d1_spi_pio_transfer(struct ma35d1_spi *hw,
				   struct spi_transfer *xfer,
				   unsigned int bits_per_word)
{
	unsigned int word_bytes = ma35d1_spi_word_bytes(bits_per_word);
	u32 data_mask = U32_MAX;
	u32 status;
	size_t offset;
	int ret;

	if (bits_per_word < 32)
		data_mask = GENMASK(bits_per_word - 1, 0);

	for (offset = 0; offset < xfer->len; offset += word_bytes) {
		ret = readl_poll_timeout(hw->regs + MA35D1_SPI_STATUS, status,
					 !(status & MA35D1_SPI_STATUS_TXFULL),
					 1, MA35D1_SPI_POLL_TIMEOUT_US);
		if (ret) {
			dev_err(hw->dev, "TX FIFO timeout: %d\n", ret);
			return ret;
		}

		ma35d1_spi_write(hw, MA35D1_SPI_TX,
				 ma35d1_spi_load_word(xfer->tx_buf, offset,
						      word_bytes) & data_mask);

		ret = readl_poll_timeout(hw->regs + MA35D1_SPI_STATUS, status,
					 !(status & MA35D1_SPI_STATUS_RXEMPTY),
					 1, MA35D1_SPI_POLL_TIMEOUT_US);
		if (ret) {
			dev_err(hw->dev, "RX FIFO timeout: %d\n", ret);
			return ret;
		}

		ma35d1_spi_store_word(xfer->rx_buf, offset, word_bytes,
				      ma35d1_spi_read(hw, MA35D1_SPI_RX) &
				      data_mask);
	}

	ret = ma35d1_spi_wait_idle(hw);
	if (ret) {
		dev_err(hw->dev, "PIO transfer did not complete: %d\n", ret);
		return ret;
	}

	return 0;
}

static enum dma_slave_buswidth ma35d1_spi_dma_width(unsigned int bits_per_word)
{
	switch (bits_per_word) {
	case 8:
		return DMA_SLAVE_BUSWIDTH_1_BYTE;
	case 16:
		return DMA_SLAVE_BUSWIDTH_2_BYTES;
	case 32:
		return DMA_SLAVE_BUSWIDTH_4_BYTES;
	default:
		return DMA_SLAVE_BUSWIDTH_UNDEFINED;
	}
}

static bool ma35d1_spi_can_dma(struct spi_controller *ctlr,
			       struct spi_device *spi,
			       struct spi_transfer *xfer)
{
	struct ma35d1_spi *hw = spi_controller_get_devdata(ctlr);
	unsigned int bpw;
	unsigned int align;

	if (!hw->dma_tx || !hw->dma_rx ||
	    xfer->len < MA35D1_SPI_DMA_MIN_BYTES)
		return false;

	bpw = xfer->bits_per_word ?: spi->bits_per_word;
	if (!bpw)
		bpw = 8;

	switch (bpw) {
	case 8:
		align = 1;
		break;
	case 16:
		align = 2;
		break;
	case 32:
		align = 4;
		break;
	default:
		return false;
	}

	if (xfer->len % align)
		return false;

	if (xfer->tx_buf && !IS_ALIGNED((unsigned long)xfer->tx_buf, align))
		return false;
	if (xfer->rx_buf && !IS_ALIGNED((unsigned long)xfer->rx_buf, align))
		return false;

	return true;
}

static void ma35d1_spi_dma_complete(void *arg)
{
	complete(arg);
}

static unsigned long ma35d1_spi_dma_timeout(struct spi_transfer *xfer)
{
	u64 ms;
	u32 speed_hz = xfer->effective_speed_hz ?: xfer->speed_hz;

	if (!speed_hz)
		speed_hz = 100000;

	ms = DIV_ROUND_UP_ULL((u64)xfer->len * 8 * MSEC_PER_SEC, speed_hz);
	ms = ms * 2 + 200;
	ms = max_t(u64, ms, 1000);
	ms = min_t(u64, ms, UINT_MAX);

	return msecs_to_jiffies((unsigned int)ms);
}

static int ma35d1_spi_config_dma(struct ma35d1_spi *hw,
				 enum dma_slave_buswidth width)
{
	struct dma_slave_config config = { };
	int ret;

	config.direction = DMA_DEV_TO_MEM;
	config.src_addr = hw->phys_base + MA35D1_SPI_RX;
	config.src_addr_width = width;
	config.src_maxburst = 1;
	config.peripheral_config = &hw->rx_peripheral;
	config.peripheral_size = sizeof(hw->rx_peripheral);

	ret = dmaengine_slave_config(hw->dma_rx, &config);
	if (ret) {
		dev_err(hw->dev, "failed to configure RX DMA: %d\n", ret);
		return ret;
	}

	memset(&config, 0, sizeof(config));
	config.direction = DMA_MEM_TO_DEV;
	config.dst_addr = hw->phys_base + MA35D1_SPI_TX;
	config.dst_addr_width = width;
	config.dst_maxburst = 1;
	config.peripheral_config = &hw->tx_peripheral;
	config.peripheral_size = sizeof(hw->tx_peripheral);

	ret = dmaengine_slave_config(hw->dma_tx, &config);
	if (ret) {
		dev_err(hw->dev, "failed to configure TX DMA: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ma35d1_spi_dma_transfer(struct ma35d1_spi *hw,
				   struct spi_transfer *xfer,
				   unsigned int bits_per_word)
{
	struct dma_async_tx_descriptor *rxdesc;
	struct dma_async_tx_descriptor *txdesc;
	enum dma_slave_buswidth width;
	dma_cookie_t cookie;
	unsigned long timeout;
	int ret;

	width = ma35d1_spi_dma_width(bits_per_word);
	if (width == DMA_SLAVE_BUSWIDTH_UNDEFINED)
		return -EINVAL;

	ret = ma35d1_spi_config_dma(hw, width);
	if (ret)
		return ret;

	reinit_completion(&hw->dma_rx_done);
	reinit_completion(&hw->dma_tx_done);

	rxdesc = dmaengine_prep_slave_sg(hw->dma_rx,
					xfer->rx_sg.sgl, xfer->rx_sg.nents,
					DMA_DEV_TO_MEM,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!rxdesc)
		return -EIO;

	rxdesc->callback = ma35d1_spi_dma_complete;
	rxdesc->callback_param = &hw->dma_rx_done;
	cookie = dmaengine_submit(rxdesc);
	ret = dma_submit_error(cookie);
	if (ret)
		goto err_terminate;

	/*
	 * Submit RX first so an RX descriptor does not remain unsubmitted if
	 * allocating the TX descriptor fails. Neither channel starts until
	 * dma_async_issue_pending() is called below.
	 */
	txdesc = dmaengine_prep_slave_sg(hw->dma_tx,
					xfer->tx_sg.sgl, xfer->tx_sg.nents,
					DMA_MEM_TO_DEV,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!txdesc) {
		ret = -EIO;
		goto err_terminate;
	}

	txdesc->callback = ma35d1_spi_dma_complete;
	txdesc->callback_param = &hw->dma_tx_done;
	cookie = dmaengine_submit(txdesc);
	ret = dma_submit_error(cookie);
	if (ret)
		goto err_terminate;

	/* Arm both DMA channels before allowing the SPI peripheral to request. */
	dma_async_issue_pending(hw->dma_rx);
	dma_async_issue_pending(hw->dma_tx);

	/*
	 * For master full-duplex operation the hardware requires TX PDMA to be
	 * enabled no later than RX PDMA. Enable both in one register write.
	 */
	ma35d1_spi_write(hw, MA35D1_SPI_PDMACTL,
			 MA35D1_SPI_PDMACTL_TXPDMAEN |
			 MA35D1_SPI_PDMACTL_RXPDMAEN);

	timeout = ma35d1_spi_dma_timeout(xfer);

	if (!wait_for_completion_timeout(&hw->dma_tx_done, timeout)) {
		ret = -ETIMEDOUT;
		dev_err(hw->dev, "TX DMA timeout\n");
		goto err_stop;
	}

	if (!wait_for_completion_timeout(&hw->dma_rx_done, timeout)) {
		ret = -ETIMEDOUT;
		dev_err(hw->dev, "RX DMA timeout\n");
		goto err_stop;
	}

	/* Stop RX and TX requests together as required for full-duplex mode. */
	ma35d1_spi_write(hw, MA35D1_SPI_PDMACTL, 0);

	ret = ma35d1_spi_wait_idle(hw);
	if (ret) {
		dev_err(hw->dev, "SPI did not become idle after DMA\n");
		goto err_terminate;
	}

	return 0;

err_stop:
	ma35d1_spi_write(hw, MA35D1_SPI_PDMACTL, 0);
err_terminate:
	dmaengine_terminate_sync(hw->dma_tx);
	dmaengine_terminate_sync(hw->dma_rx);
	ma35d1_spi_reset_pdma(hw);
	ma35d1_spi_reset_fifos(hw);

	return ret;
}

static int ma35d1_spi_transfer_one(struct spi_controller *ctlr,
				   struct spi_device *spi,
				   struct spi_transfer *xfer)
{
	struct ma35d1_spi *hw = spi_controller_get_devdata(ctlr);
	unsigned int bits_per_word;
	int ret;

	if (!xfer->len)
		return 0;

	ret = ma35d1_spi_config_transfer(hw, spi, xfer, &bits_per_word);
	if (ret)
		return ret;

	if (ma35d1_spi_can_dma(ctlr, spi, xfer))
		return ma35d1_spi_dma_transfer(hw, xfer, bits_per_word);

	return ma35d1_spi_pio_transfer(hw, xfer, bits_per_word);
}

static void ma35d1_spi_set_cs(struct spi_device *spi, bool level)
{
	struct ma35d1_spi *hw = spi_controller_get_devdata(spi->controller);
	u32 chip_select = spi_get_chipselect(spi, 0);
	u32 ss_mask;
	u32 val;
	bool active;

	if (chip_select >= MA35D1_SPI_NUM_CS) {
		dev_err(hw->dev, "invalid chip select %u\n", chip_select);
		return;
	}

	/*
	 * The SPI core passes the physical CS level to this callback. The
	 * callback may run in atomic context, so it must not sleep or poll.
	 */
	active = (spi->mode & SPI_CS_HIGH) ? level : !level;
	ss_mask = chip_select ? MA35D1_SPI_SSCTL_SS1 :
				MA35D1_SPI_SSCTL_SS0;

	val = ma35d1_spi_read(hw, MA35D1_SPI_SSCTL);
	val &= ~MA35D1_SPI_SSCTL_AUTOSS;

	if (!active) {
		val &= ~ss_mask;
		ma35d1_spi_write(hw, MA35D1_SPI_SSCTL, val);
		return;
	}

	/*
	 * Program the active polarity while the selected line is inactive,
	 * then assert the native chip select. This avoids changing polarity
	 * and assertion state in the same register write.
	 */
	val &= ~(MA35D1_SPI_SSCTL_SSACTPOL | ss_mask);
	if (spi->mode & SPI_CS_HIGH)
		val |= MA35D1_SPI_SSCTL_SSACTPOL;
	ma35d1_spi_write(hw, MA35D1_SPI_SSCTL, val);

	ma35d1_spi_write(hw, MA35D1_SPI_SSCTL, val | ss_mask);
}

static void ma35d1_spi_release_dma(void *data)
{
	struct ma35d1_spi *hw = data;

	if (hw->dma_tx) {
		dma_release_channel(hw->dma_tx);
		hw->dma_tx = NULL;
	}

	if (hw->dma_rx) {
		dma_release_channel(hw->dma_rx);
		hw->dma_rx = NULL;
	}
}

static int ma35d1_spi_request_dma(struct spi_controller *ctlr,
				  struct ma35d1_spi *hw)
{
	struct device *dev = hw->dev;
	int ret;

	if (!device_property_present(dev, "dmas"))
		return 0;

	hw->dma_tx = dma_request_chan(dev, "tx");
	if (IS_ERR(hw->dma_tx)) {
		ret = PTR_ERR(hw->dma_tx);
		hw->dma_tx = NULL;
		return dev_err_probe(dev, ret,
				     "failed to request TX DMA channel\n");
	}

	hw->dma_rx = dma_request_chan(dev, "rx");
	if (IS_ERR(hw->dma_rx)) {
		ret = PTR_ERR(hw->dma_rx);
		hw->dma_rx = NULL;
		dma_release_channel(hw->dma_tx);
		hw->dma_tx = NULL;
		return dev_err_probe(dev, ret,
				     "failed to request RX DMA channel\n");
	}

	ret = device_property_read_u32(dev, "nuvoton,pdma-reqsel-tx",
				       &hw->tx_peripheral.reqsel);
	if (ret) {
		ret = dev_err_probe(dev, ret,
				    "missing TX PDMA request selector\n");
		goto err_release;
	}

	ret = device_property_read_u32(dev, "nuvoton,pdma-reqsel-rx",
				       &hw->rx_peripheral.reqsel);
	if (ret) {
		ret = dev_err_probe(dev, ret,
				    "missing RX PDMA request selector\n");
		goto err_release;
	}

	if (hw->tx_peripheral.reqsel > 0xff ||
	    hw->rx_peripheral.reqsel > 0xff) {
		ret = dev_err_probe(dev, -EINVAL,
				    "invalid PDMA request selector\n");
		goto err_release;
	}

	init_completion(&hw->dma_tx_done);
	init_completion(&hw->dma_rx_done);

	ret = devm_add_action_or_reset(dev, ma35d1_spi_release_dma, hw);
	if (ret)
		return ret;

	ctlr->dma_tx = hw->dma_tx;
	ctlr->dma_rx = hw->dma_rx;
	ctlr->can_dma = ma35d1_spi_can_dma;
	ctlr->max_dma_len = MA35D1_SPI_MAX_DMA_SEGMENT;

	return 0;

err_release:
	dma_release_channel(hw->dma_rx);
	dma_release_channel(hw->dma_tx);
	hw->dma_rx = NULL;
	hw->dma_tx = NULL;

	return ret;
}

static void ma35d1_spi_handle_err(struct spi_controller *ctlr,
				  struct spi_message *message)
{
	struct ma35d1_spi *hw = spi_controller_get_devdata(ctlr);
	int ret;

	(void)message;

	ma35d1_spi_write(hw, MA35D1_SPI_PDMACTL, 0);

	if (hw->dma_tx)
		dmaengine_terminate_sync(hw->dma_tx);
	if (hw->dma_rx)
		dmaengine_terminate_sync(hw->dma_rx);

	ret = ma35d1_spi_reset_pdma(hw);
	if (ret)
		dev_warn(hw->dev, "failed to reset PDMA after transfer error: %d\n",
			 ret);

	ret = ma35d1_spi_reset_fifos(hw);
	if (ret)
		dev_warn(hw->dev, "failed to reset FIFOs after transfer error: %d\n",
			 ret);
}

static int ma35d1_spi_hw_init(struct ma35d1_spi *hw)
{
	u32 fifoctl;
	u32 fifoctl2;
	u32 ctl;
	int ret;

	ret = ma35d1_spi_disable(hw);
	if (ret)
		return ret;

	ret = ma35d1_spi_reset_pdma(hw);
	if (ret)
		return ret;

	/* Manual chip-select mode, both native CS lines inactive. */
	ma35d1_spi_update_bits(hw, MA35D1_SPI_SSCTL,
			       MA35D1_SPI_SSCTL_AUTOSS |
			       MA35D1_SPI_SSCTL_SSACTPOL |
			       MA35D1_SPI_SSCTL_SS0 |
			       MA35D1_SPI_SSCTL_SS1,
			       0);

	/* Keep the reset suspend interval and initialize to SPI mode 0, 8-bit. */
	ctl = ma35d1_spi_read(hw, MA35D1_SPI_CTL);
	ctl &= MA35D1_SPI_CTL_SUSPITV_MASK;
	ctl |= FIELD_PREP(MA35D1_SPI_CTL_DWIDTH_MASK, 8) |
	       MA35D1_SPI_CTL_TXNEG;
	ma35d1_spi_write(hw, MA35D1_SPI_CTL, ctl);

	fifoctl = ma35d1_spi_read(hw, MA35D1_SPI_FIFOCTL);
	fifoctl &= ~(MA35D1_SPI_FIFOCTL_TXTH_MASK |
		     MA35D1_SPI_FIFOCTL_RXTH_MASK);
	fifoctl |= FIELD_PREP(MA35D1_SPI_FIFOCTL_TXTH_MASK,
			      MA35D1_SPI_FIFO_THRESHOLD) |
		   FIELD_PREP(MA35D1_SPI_FIFOCTL_RXTH_MASK,
			      MA35D1_SPI_FIFO_THRESHOLD);
	ma35d1_spi_write(hw, MA35D1_SPI_FIFOCTL, fifoctl);

	fifoctl2 = FIELD_PREP(MA35D1_SPI_FIFOCTL2_TXTHPDMA_MASK,
			       MA35D1_SPI_FIFO_THRESHOLD) |
		   FIELD_PREP(MA35D1_SPI_FIFOCTL2_RXTHPDMA_MASK,
			       MA35D1_SPI_FIFO_THRESHOLD);
	ma35d1_spi_write(hw, MA35D1_SPI_FIFOCTL2, fifoctl2);

	ret = ma35d1_spi_reset_fifos(hw);
	if (ret)
		return ret;

	return ma35d1_spi_enable(hw);
}

static void ma35d1_spi_hw_disable(void *data)
{
	struct ma35d1_spi *hw = data;

	ma35d1_spi_write(hw, MA35D1_SPI_PDMACTL, 0);
	if (hw->dma_tx)
		dmaengine_terminate_sync(hw->dma_tx);
	if (hw->dma_rx)
		dmaengine_terminate_sync(hw->dma_rx);
	ma35d1_spi_disable(hw);
}

static int ma35d1_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *ctlr;
	struct ma35d1_spi *hw;
	struct resource *res;
	unsigned long rate;
	int ret;

	ctlr = devm_spi_alloc_host(dev, sizeof(*hw));
	if (!ctlr)
		return -ENOMEM;

	ctlr->dev.of_node = dev->of_node;

	hw = spi_controller_get_devdata(ctlr);
	hw->dev = dev;

	hw->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(hw->regs))
		return dev_err_probe(dev, PTR_ERR(hw->regs),
				     "failed to map registers\n");

	hw->phys_base = res->start;

	hw->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(hw->clk))
		return dev_err_probe(dev, PTR_ERR(hw->clk),
				     "failed to get and enable SPI clock\n");

	rate = clk_get_rate(hw->clk);
	if (!rate || rate > U32_MAX)
		return dev_err_probe(dev, -EINVAL,
				     "invalid SPI parent clock rate %lu\n", rate);
	hw->parent_rate = rate;

	ctlr->num_chipselect = MA35D1_SPI_NUM_CS;
	ctlr->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST;
	ctlr->bits_per_word_mask = SPI_BPW_RANGE_MASK(MA35D1_SPI_MIN_BPW,
						      MA35D1_SPI_MAX_BPW);
	ctlr->flags = SPI_CONTROLLER_MUST_TX | SPI_CONTROLLER_MUST_RX;
	ctlr->min_speed_hz = DIV_ROUND_UP(hw->parent_rate,
					  MA35D1_SPI_CLKDIV_MAX + 1);
	ctlr->max_speed_hz = min_t(u32, hw->parent_rate / 2,
					   MA35D1_SPI_MAX_MASTER_HZ);
	ctlr->set_cs = ma35d1_spi_set_cs;
	ctlr->transfer_one = ma35d1_spi_transfer_one;
	ctlr->handle_err = ma35d1_spi_handle_err;

	ret = ma35d1_spi_request_dma(ctlr, hw);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, ma35d1_spi_hw_disable, hw);
	if (ret)
		return ret;

	ret = ma35d1_spi_hw_init(hw);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to initialize SPI controller\n");

	platform_set_drvdata(pdev, ctlr);

	ret = devm_spi_register_controller(dev, ctlr);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register SPI controller\n");

	return 0;
}

static const struct of_device_id ma35d1_spi_of_match[] = {
	{ .compatible = "nuvoton,ma35d0-spi" },
	{ .compatible = "nuvoton,ma35d1-spi" },
	{ .compatible = "nuvoton,ma35h0-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, ma35d1_spi_of_match);

static struct platform_driver ma35d1_spi_driver = {
	.probe = ma35d1_spi_probe,
	.driver = {
		.name = "ma35d1-spi",
		.of_match_table = ma35d1_spi_of_match,
	},
};
module_platform_driver(ma35d1_spi_driver);

MODULE_AUTHOR("Chi-Wen Weng <cwweng@nuvoton.com>");
MODULE_DESCRIPTION("Nuvoton MA35D1 SPI controller driver");
MODULE_LICENSE("GPL");
