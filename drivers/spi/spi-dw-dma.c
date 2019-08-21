
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

#include "spi-dw.h"

#include <linux/platform_data/dma-dw.h>
#include <linux/delay.h>


#define DESC_DATA_SIZE   (4*1024)
#define RX_BUSY     0
#define TX_BUSY     1

static int transfer (struct dw_spi *dws, struct spi_transfer *xfer);

static int spi_check_status (struct dw_spi *dws)
{
	uint8_t sr = dw_readl(dws, DW_SPI_SR);
	uint8_t busy = (sr & (1 << 0));    /* SSI Busy Flag. */
	return busy;
}

static irqreturn_t transfer_handler (struct dw_spi *dws)
{
	u16 irq_status = dw_readl(dws, DW_SPI_ISR);
	if (!irq_status)
		return IRQ_NONE;

	dw_readl(dws, DW_SPI_ICR);
	spi_reset_chip(dws);

	dev_err(&dws->master->dev, "%s: FIFO overrun/underrun\n", __func__);
	dws->master->cur_msg->status = -EIO;

	spi_finalize_current_transfer(dws->master);
	return IRQ_HANDLED;
}

static enum dma_slave_buswidth convert_dma_width (u32 dma_width)
{
	if (dma_width == 1)
		return DMA_SLAVE_BUSWIDTH_1_BYTE;
	else if (dma_width == 2)
		return DMA_SLAVE_BUSWIDTH_2_BYTES;
	return DMA_SLAVE_BUSWIDTH_UNDEFINED;
}

static void spi_wait_status(struct dw_spi *dws)
{
	/*
	time(freg) = len*8/freq
	time(8k) = len*8/8k = len/1k [sec] = len*1000 [us]
	*/
	long int us = dws->len * 1000;
	while (spi_check_status(dws) && us--)
		udelay(1);
}

static void tx_done (void *arg)
{
	/* dws->dma_chan_busy is set before the dma transfer starts,
	 callback for tx channel will clear a corresponding bit. */

	struct dw_spi *dws = arg;
	spi_wait_status(dws);
	clear_bit(TX_BUSY, &dws->dma_chan_busy);
	if (test_bit(RX_BUSY, &dws->dma_chan_busy))
		return;
	spi_finalize_current_transfer(dws->master);
}

static void rx_done (void *arg)
{
	/* dws->dma_chan_busy is set before the dma transfer starts,
	callback for rx channel will clear a corresponding bit. */

	struct dw_spi *dws = arg;
	spi_wait_status(dws);

	size_t len = min(dws->len, DESC_DATA_SIZE);
	dws->len -= len;
	dws->rx  += len;

	if(!dws->len){
		clear_bit(RX_BUSY, &dws->dma_chan_busy);
		if (test_bit(TX_BUSY, &dws->dma_chan_busy))
			return;
		spi_finalize_current_transfer(dws->master);
	}else{
		/* next part */
		transfer(dws, NULL);   //xfer = null, because not used
	}
}

static struct dma_async_tx_descriptor *prepare_tx (
	struct dw_spi       *dws,
	struct spi_transfer *xfer)
{
	if (!dws->tx)
		return NULL;


	/* slave config */
	struct dma_slave_config config;
	memset(&config, 0, sizeof(config));
	config.direction      = DMA_MEM_TO_DEV;
	config.device_fc      = false;

	config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.src_maxburst   = dws->fifo_len/2;
	config.src_addr       = dws->tx;

	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.dst_maxburst   = dws->fifo_len/2;
	config.dst_addr       = CPHYSADDR(dws->dma_addr);

	dmaengine_slave_config(dws->txchan, &config);

	/* descriptor */
	struct dma_async_tx_descriptor *desc;
	desc = dmaengine_prep_slave_single(
		dws->txchan,				// chan
		CPHYSADDR(dws->tx),			// dws->tx, buf_tx
		dws->len,				// len
		DMA_MEM_TO_DEV,				// dir
		DMA_PREP_INTERRUPT | DMA_CTRL_ACK);	// flags
	if (!desc)
		return NULL;

	/* callback */
	desc->callback = tx_done;
	desc->callback_param = dws;

	return desc;
}

static struct dma_async_tx_descriptor *prepare_rx (
	struct dw_spi       *dws,
	struct spi_transfer *xfer)
{
	if (!dws->rx)
		return NULL;

	/* slave config */
	struct dma_slave_config config;
	memset(&config, 0, sizeof(config));
	config.direction      = DMA_DEV_TO_MEM;
	config.device_fc      = false;

	config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.src_maxburst   = dws->fifo_len/2;
	config.src_addr       = CPHYSADDR(dws->dma_addr);

	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	config.dst_maxburst   = dws->fifo_len/2;
	config.dst_addr       = dws->rx;

	dmaengine_slave_config(dws->rxchan, &config);

	size_t len = min(dws->len,DESC_DATA_SIZE);
	spi_enable_chip(dws, 0);
	dw_writel(dws, DW_SPI_CTRL1, len-1);
	spi_enable_chip(dws, 1);

	/* descriptor */
	struct dma_async_tx_descriptor *desc;
	desc = dmaengine_prep_slave_single(
		dws->rxchan,				// chan
		CPHYSADDR(dws->rx),			// dws->rx, buf_rx
		len,					// len
		DMA_DEV_TO_MEM,				// dir
		DMA_PREP_INTERRUPT | DMA_CTRL_ACK);	// flags
	if (!desc)
		return NULL;

	/* callback */
	desc->callback = rx_done;
	desc->callback_param = dws;

	return desc;
}

static int init (struct dw_spi *dws)
{
	struct device *dev = &(dws->master->dev);

	dws->rxchan = dma_request_slave_channel(dev, "rx");
	dws->txchan = dma_request_slave_channel(dev, "tx");

	if (!dws->rxchan || !dws->txchan){
		if(dws->txchan)
			dma_release_channel(dws->txchan);
		if(dws->rxchan)
			dma_release_channel(dws->rxchan);
		return -EBUSY;
	}

	dws->master->dma_rx = dws->rxchan;
	dws->master->dma_tx = dws->txchan;
	dws->transfer_handler = transfer_handler;

	dws->dma_inited = 1;

	return 0;
}

static void exit (struct dw_spi *dws)
{
	if (!dws->dma_inited)
		return;

	dmaengine_terminate_all(dws->txchan);
	dmaengine_terminate_all(dws->rxchan);

	dma_release_channel(dws->txchan);
	dma_release_channel(dws->rxchan);
}

static bool can (
	struct spi_master *master,
	struct spi_device *spi,
	struct spi_transfer *xfer)
{
	struct dw_spi *dws = spi_master_get_devdata(master);
	return (dws->dma_inited) && (xfer->len > dws->fifo_len);
}

static int setup (struct dw_spi *dws, struct spi_transfer *xfer)
{
	/* clear */
	dw_writel(dws, DW_SPI_SER, 0);
	dw_writel(dws, DW_SPI_DMACR, 0);

	/* MODE */
	uint32_t tmode;
	if (dws->rx && dws->tx)
		tmode = SPI_TMOD_TR;
	else if (dws->rx)
		tmode = SPI_TMOD_RO;
	else
		tmode = SPI_TMOD_TO;

	/* CTRL0 */
	uint32_t cr0;
	cr0 = dw_readl(dws, DW_SPI_CTRL0);
	cr0 &= ~SPI_TMOD_MASK;
	cr0 |= (tmode << SPI_TMOD_OFFSET);
	dw_writel(dws, DW_SPI_CTRL0, cr0);

	/* DMATDLR */
	dw_writel(dws, DW_SPI_DMATDLR, dws->fifo_len/2);
	dw_writel(dws, DW_SPI_DMARDLR, dws->fifo_len/2 -1);

	/* DMACR */
	uint16_t dma_ctrl = 0;
	if(dws->tx)
		dma_ctrl |= SPI_DMA_TDMAE;
	if(dws->rx)
		dma_ctrl |= SPI_DMA_RDMAE;
	dw_writel(dws, DW_SPI_DMACR, dma_ctrl);

	return 0;
}

static int transfer (struct dw_spi *dws, struct spi_transfer *xfer)
{
	struct dma_async_tx_descriptor *rxdesc;
	struct dma_async_tx_descriptor *txdesc;

	/* rx must be started before tx due to spi instinct */
	rxdesc = prepare_rx(dws, xfer);
	if (rxdesc) {
		set_bit(RX_BUSY, &dws->dma_chan_busy);
		dmaengine_submit(rxdesc);
		dma_async_issue_pending(dws->rxchan);    // start
	}

	txdesc = prepare_tx(dws, xfer);
	if (txdesc) {
		set_bit(TX_BUSY, &dws->dma_chan_busy);
		dmaengine_submit(txdesc);
		dma_async_issue_pending(dws->txchan);    // start
	}

	if (!dws->tx && dws->rx)
		dw_writel(dws, DW_SPI_DR, 0);  // write dummy data to start read-only mode
	dw_writel(dws, DW_SPI_SER, 1<<1);  // start spi


	return 0;
}

static void stop (struct dw_spi *dws)
{
	if (test_bit(TX_BUSY, &dws->dma_chan_busy)) {
		dmaengine_terminate_all(dws->txchan);
		clear_bit(TX_BUSY, &dws->dma_chan_busy);
	}
	if (test_bit(RX_BUSY, &dws->dma_chan_busy)) {
		dmaengine_terminate_all(dws->rxchan);
		clear_bit(RX_BUSY, &dws->dma_chan_busy);
	}
}

static struct dw_spi_dma_ops  dma_ops = {
	.dma_init     = init,
	.dma_exit     = exit,
	.can_dma      = can,
	.dma_setup    = setup,
	.dma_transfer = transfer,
	.dma_stop     = stop,
};



void spi_dma_init (struct dw_spi *dws)
{
	dws->dma_ops = &dma_ops;
}
