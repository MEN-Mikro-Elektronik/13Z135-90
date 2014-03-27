/*
 * MEN 16z135 High Speed UART
 *
 * Copyright (C) 2014 MEN Mikroelektronik GmbH (www.men.de)
 * Author: Johannes Thumshirn <johannes.thumshirn@men.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; version 2 of the License.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ":" fmt

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/serial_core.h>
#include <linux/ioport.h>
#include <linux/tty_flip.h>

#include <MEN/men_chameleon.h>

#define MEN_Z135_BASECLK		3250000
#define MEN_Z135_FIFO_SIZE		1024
#define MEN_Z135_NUM_MSI_VECTORS	2

#define MEN_Z135_STAT_REG		0x0
#define MEN_Z135_RX_RAM		0x4
#define MEN_Z135_TX_RAM		0x400
#define MEN_Z135_RX_CTRL		0x800
#define MEN_Z135_TX_CTRL		0x804
#define MEN_Z135_CONF_REG		0x808
#define MEN_Z135_UART_FREQ		0x80c
#define MEN_Z135_BAUD_REG		0x810
#define MENZ135_TIMEOUT		0x814

#define MEN_Z135_MEM_SIZE		0x818

#define IRQ_PENDING(x) ((x) & 1)
#define IRQ_ID(x) (((x) >> 1) & 7)

#define RXCIEN BIT(0)		/* TX Space IRQ enable */
#define TXCIEN BIT(1)		/* RX Space IRQ enable */
#define RLSIEN BIT(2)		/* Receiver Line Status IRQ enable */
#define MSIEN  BIT(3)		/* Modem Status IRQ enable */

#define DTR BIT(24)
#define RTS BIT(25)
#define OUT1 BIT(26)
#define OUT2 BIT(27)
#define LOOP BIT(28)
#define RCFC BIT(29)

#define BYTES_TO_ALIGN(x) ((unsigned long) (x) & 0x3)

static int line = 0;

static int budget = 32;
module_param(budget, int, S_IRUGO);

enum {
	IRQ_RX = 0,
	IRQ_TX,
	NUM_IRQS,		/* Always last */
};

struct men_z135_port {
	struct uart_port port;
	struct tasklet_struct intrs[NUM_IRQS];
	struct pci_dev *pdev;
	struct uart_driver *men_z135_driver;
	CHAMELEON_UNIT_T *chu;
	int msi;
	int tx_empty;
	unsigned int rxb;
	unsigned int txb;
	u32 stat_reg;
};
#define to_men_z135(port) container_of(port, struct men_z135_port, port)

/**
 * men_z135_reg_set() - Set value in register
 * @uart: The UART port
 * @addr: Register address
 * @val: value to set
 */
static inline void men_z135_reg_set(struct men_z135_port *uart,
				    u32 addr, u32 val)
{
	struct uart_port *port = &uart->port;
	u32 reg;

	reg = ioread32(port->membase + addr);
	reg |= val;
	iowrite32(reg, port->membase + addr);
}

/**
 * men_z135_reg_clr() - Unset value in register
 * @uart: The UART port
 * @addr: Register address
 * @val: value to clear
 */
static inline void men_z135_reg_clr(struct men_z135_port *uart,
				    u32 addr, u32 val)
{
	struct uart_port *port = &uart->port;
	u32 reg;

	reg = ioread32(port->membase + addr);
	reg &= ~val;
	iowrite32(reg, port->membase + addr);
}

/**
 * men_z135_intr_msi_tx() - MSI IRQ handler for TX side
 * @irq: The IRQ number
 * @data: Pointer to UART port
 *
 * men_z135_intr_msi_tx() just calls the TX tasklet, thats it.
 */
static irqreturn_t men_z135_intr_msi_tx(int irq, void *data)
{
	struct men_z135_port *uart = (struct men_z135_port *)data;

	men_z135_reg_clr(uart, MEN_Z135_CONF_REG, TXCIEN);
	tasklet_schedule(&uart->intrs[IRQ_TX]);

	return IRQ_HANDLED;
}

/**
 * men_z135_intr_msi_rx() - MSI IRQ handler for RX side
 * @irq: The IRQ number
 * @data: Pointer to UART port
 *
 * Deactivate RX IRQ, the RX tasklet will reactivate it when it is done
 * processing the FIFO contents.
 */
static irqreturn_t men_z135_intr_msi_rx(int irq, void *data)
{
	struct men_z135_port *uart = (struct men_z135_port *)data;

	men_z135_reg_clr(uart, MEN_Z135_CONF_REG, RXCIEN);
	tasklet_schedule(&uart->intrs[IRQ_RX]);

	return IRQ_HANDLED;
}

/**
 * men_z135_intr() - Handle legacy IRQs
 * @irq: The IRQ number
 * @data: Pointer to UART port
 *
 * Check IIR register to see which tasklet to start.
 */
static irqreturn_t men_z135_intr(int irq, void *data)
{
	struct men_z135_port *uart = (struct men_z135_port *)data;
	struct uart_port *port = &uart->port;
	int irq_id;

	uart->stat_reg = ioread32(port->membase + MEN_Z135_STAT_REG);

	/* IRQ pending is low active */
	if (IRQ_PENDING(uart->stat_reg))
		return IRQ_NONE;

	irq_id = IRQ_ID(uart->stat_reg);

	if (irq_id == 0 || irq_id == 1) {
		men_z135_reg_clr(uart, MEN_Z135_CONF_REG, TXCIEN);
		tasklet_schedule(&uart->intrs[IRQ_TX]);

	} else if (irq_id == 2 || irq_id == 3 || irq_id == 6) {
		men_z135_reg_clr(uart, MEN_Z135_CONF_REG, RXCIEN);
		tasklet_schedule(&uart->intrs[IRQ_RX]);
	} else {
		dev_warn(&uart->pdev->dev, "Unknown IRQ id %d\n", irq_id);
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

/**
 * get_rx_fifo_content() - Get the number of bytes in RX FIFO
 * @uart: The UART port
 *
 * Read RXC register from hardware and return current FIFO fill size.
 */
static u16 get_rx_fifo_content(struct men_z135_port *uart)
{
	struct uart_port *port = &uart->port;
	u16 rxc;

	uart->stat_reg = ioread32(port->membase + MEN_Z135_STAT_REG);

	rxc = uart->stat_reg >> 24;
	rxc |= uart->stat_reg & 0x300;

	return rxc;
}

/**
 * men_z135_handle_rx() - RX tasklet routine
 * @arg: Pointer to struct men_z135_port
 *
 * Poll UART's RX FIFOs for data. Copy up to @budget FIFO contents in polling
 * mode until interrupts get re-enabled. The port therefore is in locked state.
 * When done with polling, men_z135_handle_rx() re-activates the RX IRQ.
 *
 * TODO: Acknowledge copied bits
 */
static void men_z135_handle_rx(unsigned long arg)
{
	struct men_z135_port *uart = (struct men_z135_port *) arg;
	struct uart_port *port = &uart->port;
	struct tty_port *tport = &uart->port.state->port;
	char buf[MEN_Z135_FIFO_SIZE];
	int i;
	u16 size;

	for (i = 0; i < budget; i++) {
		size = get_rx_fifo_content(uart);
		if (size == 0 || size > MEN_Z135_FIFO_SIZE)
			break;

		memcpy_fromio(buf, port->membase + MEN_Z135_RX_RAM, size);

		tty_insert_flip_string(tport, buf, size);
		tty_flip_buffer_push(tport);

		uart->rxb += size;
	}
	men_z135_reg_set(uart, MEN_Z135_CONF_REG, RXCIEN);
}

/**
 * men_z135_handle_tx() - TX tasklet routine
 * @arg: Pointer to struct men_z135_port
 *
 * TODO: Acknowledge copied bits
 */
static void men_z135_handle_tx(unsigned long arg)
{
	struct men_z135_port *uart = (struct men_z135_port *) arg;
	struct uart_port *port = &uart->port;
	struct circ_buf *xmit = &port->state->xmit;
	int qlen;
	int n;
	int tail;

	if (uart_circ_empty(xmit) || uart_tx_stopped(port))
		return;

	if (port->x_char)
		return;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	/* calculate bytes to copy */
	qlen = uart_circ_chars_pending(xmit);

	/* if we're not aligned, it's better to copy only 1 byte and then the
	 * rest
	 */
	if (BYTES_TO_ALIGN(qlen))
		n = BYTES_TO_ALIGN(qlen);
	else
		n = min(MEN_Z135_FIFO_SIZE, qlen);

	tail = xmit->tail & (UART_XMIT_SIZE - 1);

	memcpy_toio(port->membase + MEN_Z135_TX_RAM, xmit->buf + tail, n);
	xmit->tail = (tail + n) & (UART_XMIT_SIZE - 1);

	men_z135_reg_set(uart, MEN_Z135_CONF_REG, TXCIEN);

	/* ACK number of bytes copied, this actually kicks off the copying
	 * process in HW
	 */
	iowrite32(n & 0x3ff, port->membase + MEN_Z135_TX_CTRL);
}

/**
 * men_z135_request_msi() - Request a MSI block for RX and TX IRQs
 * @uart: The UART port
 *
 * Request a block of 2 MSI vectors for handling RX and TX IRQs.
 * Return 0 on success, -ENOTSUPP otherwise.
 */
static int men_z135_request_msi(struct men_z135_port *uart)
{
	CHAMELEON_UNIT_T *chu = uart->chu;
	int err = 0;

	err = pci_enable_msi_block(uart->pdev, MEN_Z135_NUM_MSI_VECTORS);
	if (err != 2) {
		dev_warn(&uart->pdev->dev,
				 "Failed to request 2 MSI vectors, falling back to legacy IRQs\n");
		return -ENOTSUPP;
	}
	err = request_irq(chu->irq, men_z135_intr_msi_rx, IRQF_SHARED,
			  "men_z135_intr_msi_rx", uart);
	if (err) {
		pci_disable_msi(uart->pdev);
		return -ENOTSUPP;
	}

	err = request_irq(chu->irq + 1, men_z135_intr_msi_tx, IRQF_SHARED,
			  "men_z135_intr_msi_tx", uart);
	if (err) {
		pci_disable_msi(uart->pdev);
		free_irq(chu->irq, (void *)uart);
		return -ENOTSUPP;
	}

	uart->msi = 1;

	return 0;
}

/**
 * men_z135_request_irq() - Request IRQ for 16z135 core
 * @uart: z135 private uart port structure
 *
 * Request an IRQ for 16z135 to use. First try using MSI, if it fails
 * fall back to using legacy interrupts.
 */
static int men_z135_request_irq(struct men_z135_port *uart)
{
	CHAMELEON_UNIT_T *chu = uart->chu;
	struct device *dev = &uart->pdev->dev;
	int err = 0;

	err = men_z135_request_msi(uart);
	if (!err)
		goto request_done;

	err = request_irq(chu->irq, men_z135_intr, IRQF_SHARED,
			  "men_z135_intr", uart);
	if (err)
		dev_err(dev, "Error %d getting interrupt\n", err);

request_done:
	return err;
}

/**
 * men_z135_tx_empty() - Handle tx_empty call
 * @port: The UART port
 *
 * This function tests whether the TX FIFO and shifter for the port
 * described by @port is empty.
 */
static unsigned int men_z135_tx_empty(struct uart_port *port)
{
	struct men_z135_port *uart = to_men_z135(port);

	if (uart->tx_empty)
		return TIOCSER_TEMT;
	else
		return 0;
}

/**
 * men_z135_set_mctrl() - Set modem control lines
 * @port: The UART port
 * @mctrl: The modem control lines
 *
 * This function sets the modem control lines for a port described by @port
 * to the state described by @mctrl
 */
static void men_z135_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct men_z135_port *uart = to_men_z135(port);
	u32 conf_reg = 0;

	if (mctrl & TIOCM_RTS)
		conf_reg |= RTS;
	if (mctrl & TIOCM_DTR)
		conf_reg |= DTR;
	if (mctrl & TIOCM_OUT1)
		conf_reg |= OUT1;
	if (mctrl & TIOCM_OUT2)
		conf_reg |= OUT2;
	if (mctrl & TIOCM_LOOP)
		conf_reg |= LOOP;

	men_z135_reg_set(uart, MEN_Z135_CONF_REG, conf_reg);
}

/**
 * men_z135_get_mctrl() - Get modem control lines
 * @port: The UART port
 *
 * Retruns the current state of modem control inputs.
 */
static unsigned int men_z135_get_mctrl(struct uart_port *port)
{
	unsigned int mctrl = 0;
	u32 conf_reg;

	conf_reg = ioread32(port->membase + MEN_Z135_CONF_REG);

	if (conf_reg & RTS)
		mctrl |= TIOCM_RTS;
	if (conf_reg & DTR)
		mctrl |= TIOCM_DTR;
	if (conf_reg & OUT1)
		mctrl |= TIOCM_OUT1;
	if (conf_reg & OUT2)
		mctrl |= TIOCM_OUT2;
	if (conf_reg & LOOP)
		mctrl |= TIOCM_LOOP;

	return mctrl;
}

/**
 * men_z135_stop_tx() - Stop transmitting characters
 * @port: The UART port
 *
 * Stop transmitting characters. This might be due to CTS line becomming
 * inactive or the tty layer indicating we want to stop transmission due to
 * an XOFF character.
 */
static void men_z135_stop_tx(struct uart_port *port)
{
	struct men_z135_port *uart = to_men_z135(port);

	men_z135_reg_clr(uart, MEN_Z135_CONF_REG, TXCIEN);
}

/**
 * men_z135_start_tx() - Start transmitting characters
 * @port: The UART port
 *
 * Start transmitting character. This actually doesn't transmit anything, but
 * fires off the TX tasklet.
 */
static void men_z135_start_tx(struct uart_port *port)
{
	struct men_z135_port *uart = to_men_z135(port);

	tasklet_schedule(&uart->intrs[IRQ_TX]);
}

/**
 * men_z135_flush_buffer() - Flush write buffers
 * @port: The UART port
 *
 * Flush write buffers by setting the TX FIFO's write pointer to 0.
 */
static void men_z135_flush_buffer(struct uart_port *port)
{
	struct men_z135_port *uart = to_men_z135(port);

	men_z135_reg_set(uart, MEN_Z135_TX_CTRL, 0);
}

/**
 * men_z135_stop_rx() - Stop receiving characters
 * @port: The UART port
 *
 * Stop receiving characters; the prot is in the process of being closed.
 */
static void men_z135_stop_rx(struct uart_port *port)
{
	struct men_z135_port *uart = to_men_z135(port);

	men_z135_reg_clr(uart, MEN_Z135_CONF_REG, RXCIEN);
}

/**
 * men_z135_enable_ms() - Enable Modem Status
 * port:
 *
 * Enable Modem Status IRQ.
 */
static void men_z135_enable_ms(struct uart_port *port)
{
	struct men_z135_port *uart = to_men_z135(port);

	men_z135_reg_set(uart, MEN_Z135_CONF_REG, MSIEN);
}

static int men_z135_startup(struct uart_port *port)
{
	struct men_z135_port *uart = to_men_z135(port);
	int err;
	u32 conf_reg = 0;

	err = men_z135_request_irq(uart);
	if (err)
		return -ENODEV;

	conf_reg |= (RXCIEN | RLSIEN | MSIEN);
	men_z135_reg_set(uart, MEN_Z135_CONF_REG, conf_reg);

	return 0;
}

static void men_z135_shutdown(struct uart_port *port)
{
	struct men_z135_port *uart = to_men_z135(port);
	u32 conf_reg = 0;

	conf_reg |= (RXCIEN | RLSIEN | MSIEN);
	men_z135_reg_clr(uart, MEN_Z135_CONF_REG, conf_reg);

	free_irq(uart->port.irq, uart);
	if (uart->msi) {
		free_irq(uart->port.irq + 1, uart);
	}
}

static void men_z135_set_termios(struct uart_port *port,
				 struct ktermios *termios,
				 struct ktermios *old)
{
}

static const char *men_z135_type(struct uart_port *port)
{
	return KBUILD_MODNAME;
}

static void men_z135_release_port(struct uart_port *port)
{
	iounmap(port->membase);
	port->membase = 0;

	release_mem_region(port->mapbase, MEN_Z135_MEM_SIZE);
}

static int men_z135_request_port(struct uart_port *port)
{
	int size = MEN_Z135_MEM_SIZE;

	if (!request_mem_region(port->mapbase, size, "men_z135_port"))
		return -EBUSY;

	port->membase = ioremap(port->mapbase, MEN_Z135_MEM_SIZE);
	if (port->membase == NULL) {
		release_mem_region(port->mapbase, MEN_Z135_MEM_SIZE);
		return -ENOMEM;
	}

	return 0;
}

static void men_z135_config_port(struct uart_port *port, int type)
{

	port->type = 29;	/* XXX: Use correct port type */
	men_z135_request_port(port);
}

static int men_z135_verify_port(struct uart_port *port,
				struct serial_struct *serinfo)
{
	return -EINVAL;
}

static struct uart_ops men_z135_ops = {
	.tx_empty = men_z135_tx_empty,
	.set_mctrl = men_z135_set_mctrl,
	.get_mctrl = men_z135_get_mctrl,
	.stop_tx = men_z135_stop_tx,
	.start_tx = men_z135_start_tx,
	.flush_buffer = men_z135_flush_buffer,
	.stop_rx = men_z135_stop_rx,
	.enable_ms = men_z135_enable_ms,
	.startup = men_z135_startup,
	.shutdown = men_z135_shutdown,
	.set_termios = men_z135_set_termios,
	.type = men_z135_type,
	.release_port = men_z135_release_port,
	.request_port = men_z135_request_port,
	.config_port = men_z135_config_port,
	.verify_port = men_z135_verify_port,
};

/**
 * men_z135_probe() - Probe a z135 instance
 * @chu: The chameleon unit
 *
 * men_z135_probe does the basic setup of hardware resources and registers the
 * new uart port to the tty layer.
 */
static int men_z135_probe(CHAMELEON_UNIT_T *chu)
{
	struct men_z135_port *uart;
	struct device *dev;
	struct uart_driver *men_z135_driver;
	char dname[20];
	int err;

	dev = &chu->pdev->dev;

	uart = devm_kzalloc(dev, sizeof(struct men_z135_port), GFP_KERNEL);
	if (!uart)
		return -ENOMEM;

	men_z135_driver = devm_kzalloc(dev, sizeof(struct uart_driver),
				       GFP_KERNEL);
	if (!men_z135_driver)
		return -ENOMEM;

	snprintf(dname, sizeof(dname), "men_z135_port%d", line);

	men_z135_driver->owner = THIS_MODULE;
	men_z135_driver->driver_name = dname;
	men_z135_driver->dev_name = "ttyHSU";
	men_z135_driver->major = 0;
	men_z135_driver->minor = 0;
	men_z135_driver->nr = 1;

	chu->driver_data = uart;

	uart->port.uartclk = MEN_Z135_BASECLK * 16;
	uart->port.fifosize = MEN_Z135_FIFO_SIZE;
	uart->port.iotype = UPIO_MEM;
	uart->port.ops = &men_z135_ops;
	uart->port.irq = chu->irq;
	uart->port.flags = UPF_BOOT_AUTOCONF | UPF_IOREMAP;
	uart->port.line = line++;
	uart->port.dev = dev;
	uart->pdev = chu->pdev;
	uart->chu = chu;
	uart->men_z135_driver = men_z135_driver;
	uart->port.mapbase = (phys_addr_t) chu->phys;
	uart->port.membase = NULL;

	spin_lock_init(&uart->port.lock);

	tasklet_init(&uart->intrs[IRQ_RX], men_z135_handle_rx,
		     (unsigned long)uart);
	tasklet_init(&uart->intrs[IRQ_TX], men_z135_handle_tx,
		     (unsigned long)uart);

	err = uart_register_driver(men_z135_driver);
	if (err)
		goto err_mem;

	err = uart_add_one_port(men_z135_driver, &uart->port);
	if (err)
		goto err_add;

	return 0;

err_add:
	uart_unregister_driver(men_z135_driver);

err_mem:
	dev_err(dev, "Failed to register UART: %d\n", err);

	return err;
}

/**
 * men_z135_remove() - Remove a z135 instance from the system
 *
 * @chu: The chameleon unit
 */
static int men_z135_remove(CHAMELEON_UNIT_T *chu)
{
	struct men_z135_port *uart;
	struct uart_driver *men_z135_driver;

	uart = chu->driver_data;
	men_z135_driver = uart->men_z135_driver;

	uart_remove_one_port(men_z135_driver, &uart->port);
	uart_unregister_driver(men_z135_driver);

	return 0;
}

static u16 mod_code_arr[] = { 0xbd,
			      CHAMELEON_MODCODE_END };

static CHAMELEON_DRIVER_T cham_driver = {
	.name = "men_z135-uart",
	.modCodeArr = mod_code_arr,
	.probe = men_z135_probe,
	.remove = men_z135_remove,
};

/**
 * men_z135_init() - Driver Registration Routine
 *
 * men_z135_init is the first routine called when the driver is loaded. All it
 * does is register with the legacy MEN Chameleon subsystem.
 */
static int __init men_z135_init(void)
{
	men_chameleon_register_driver(&cham_driver);
	return 0;
}
module_init(men_z135_init);

/**
 * men_z135_exit() - Driver Exit Routine
 *
 * men_z135_exit is called just before the driver is removed from memory.
 */
static void __exit men_z135_exit(void)
{
	men_chameleon_unregister_driver(&cham_driver);
}
module_exit(men_z135_exit);

MODULE_AUTHOR("Johannes Thumshirn <johannes.thumshirn@men.de>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MEN 16z135 High Speed UART");
