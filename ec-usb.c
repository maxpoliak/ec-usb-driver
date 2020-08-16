/*
 * Embedded controller driver
 *
 * Copyright (c) 2019 YADRO
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <asm/atomic.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/usb.h>
#include <linux/poll.h>
#include <linux/ioctl.h>
#include <linux/interrupt.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <asm/errno.h>
#include <linux/circ_buf.h>
#include <linux/types.h>

/* Driver magic word for receive control commands from user space */
#define EC_USB_MINOR	192
#define EC_DRIVER_NAME	"bmc_mcu"
#define DRIVER_VERSION	"Embedded Controller Driver v1.0"

/* IOCTL commands for this driver */
enum {
	EC_CAN_CTRL_ID = 0,
	GET_TIMEOUT_ID,
	SET_TIMEOUT_ID,
	FLTR_HDR_ID,
	EC_USB_RESET_ID,
	MAX_LIMIT_EC_GROUP
};

#define EC_CAN_CTRL	_IOWR(EC_USB_MINOR, EC_CAN_CTRL_ID, unsigned int)
#define EC_SERV_TIMEOUT_GET	_IOR(EC_USB_MINOR, GET_TIMEOUT_ID, int*)
#define EC_SERV_TIMEOUT_SET	_IOWR(EC_USB_MINOR, SET_TIMEOUT_ID, int)
#define EC_HBP_CAN_FLTR_SET	_IOWR(EC_USB_MINOR, FLTR_HDR_ID, u32)
#define EC_USB_RESET		_IOWR(EC_USB_MINOR, EC_USB_RESET_ID, int)

/* Macro for system events */
#define SND_MSG_EVENT(c, event)	set_bit((int)event, (void*)&c->flags)
#define TST_MSG_EVENT(c, event)	test_bit((int)event, (void*)&c->flags)
#define CLR_MSG_EVENT(c, event)	clear_bit((int)event, (void*)&c->flags)
#define INVALID_DESC_EVENT(ev)	(ev >= MAX_NUM_EVENT);

/* The delay before removing the main device structure in ms */
#define EC_REMOVE_WAIT_MS	100

/* If the ring buffer is full, the system pauses */
#define EC_RING_BUFFER_PAUSE_US	560

/* Macro to get dev struct pointer from current Kref */
#define TO_EC_DEV(d)	container_of(d, struct ec_dev, kref)

/* Size of attr header in COU packet */
#define COU_ATTR_HEADER_SIZE	4

/* Size of data in COU packet */
#define COU_DATA_SIZE	8
#define COU_BYTES_NUM	COU_DATA_SIZE

/* Size of all COU packet */
#define COU_PACKET_SIZE	(COU_DATA_SIZE + COU_ATTR_HEADER_SIZE)

/* Packet engine completion timeout */
#define COU_ENGINE_TIMEOUT_MS	5

/* Period of block interrupt request (us) */
#define INTERRUPT_URB_PERIOD_US	20

/* Service timeout to read packet from bulk-in ep mcu (ms) */
#define SERVICE_TIMEOUT_MIN	50
#define SERVICE_TIMEOUT		500
#define SERVICE_TIMEOUT_MAX	5000

/* Statistics */
#define STAT_BUF_SIZE		400
#define STAT_INTERVAL_UPDATE	1000

/* Macro of mask for reserved bits */
#define MASK_RES_BITS		(0x1F << 11 | 0x7)
#define MASK_SRV_RES_BITS	0x7

/* Ring packet buffer */
#define CIRCULAR_NUM_ENTRY	1024
#define CONTEXT_PCKT_BUF_SIZE	12
#define CNTR_BOUNDARIES(c)			\
	do {					\
		if ((c) >= CIRCULAR_NUM_ENTRY)	\
			(c) = 0;		\
	} while (0)

#define MOVE_POSITION(c)		\
	do {				\
		++(c);			\
		CNTR_BOUNDARIES(c);	\
	} while (0)

/* Macros to display log-info in console or log-file */
#define LOG_BANER	"[ ec ]: "
#define EC_ERR(s, args...)	printk(KERN_ERR LOG_BANER s, ##args)
#define EC_WARN(s, args...)	printk(KERN_WARNING LOG_BANER s, ##args)
#define EC_PRINT_USB_EP_PARAM(pendp, param, buff)			\
	do {								\
		buff = pendp->param,					\
		printk(KERN_INFO "         "#param" = %d \n", buff);	\
	} while (0)

#ifdef SET_LOG
#define EC_LOG(s, args...)	printk(KERN_INFO LOG_BANER s, ##args)
#define EC_INFO(s, args...)	printk(KERN_INFO LOG_BANER s, ##args)
#define EC_GDB(s, args...)	printk(KERN_DEBUG LOG_BANER s, ##args)
#else
#define EC_LOG(s, args...)	{}
#define EC_INFO(s, args...)	{}
#define EC_GDB(s, args...)	{}
#endif /* SET_LOG */

/* Signal descriptor */
enum {
	DISCONNECT_DEV = 0,
	USB_HARD_ERR,
	INVALID_PACKET,
	RECEIVE_PACKET,
	HI_PRIO_EVENT,
	USER_EVENT,
	MAX_NUM_EVENT
};

/**
 * struct ring_pckt_buff - ring buffer attributes structure
 *
 * @buff:        fragmented buffer
 * @packet_size: packet size in the specified buffer fragment
 * @head:        head of ring buffer
 * @tail:        tail of ring buffer
 *
 */
typedef struct ring_packet_buff {
	u8 buff[CIRCULAR_NUM_ENTRY][CONTEXT_PCKT_BUF_SIZE];
	u8 packet_size[CIRCULAR_NUM_ENTRY];
	int head;
	int tail;
} ring_pckt_buff_t;

typedef int (*msg_cb_t)(void *, u8, void *);
typedef int (*sig_cb_t)(u32, void *);

/**
 * struct cb_list - callbaks list
 * @list:   callbacks list
 * @msg_cb: callback to copy packet
 * @sig_cb: callback to send signal
 * @proc_context_data: context of current process
 *
 */
struct cb_list {
	struct list_head hndl_list;
	rwlock_t list_lock;
};

/**
 * struct cb_data
 * @list:   callbacks list
 * @msg_cb: callback to copy packet
 * @sig_cb: callback to send signal
 * @proc_context_data: context of current process
 *
 */
struct cb_data {
	struct list_head list;
	msg_cb_t msg_cb;
	sig_cb_t sig_cb;
	void *proc_context_data;
};

/**
 * struct cou_stat - the COU statistics structure
 * @pckt_rcv:    all received packets
 * @pckt_snd:    all sent packets
 * @bytes_rcv:   all received bytes
 * @bytes_snd:   all sent bytes
 * @pps_rcv:     packet per second was received
 * @pps_snd:     packet per second was sent
 * @bps_rcv:     bytes per second was received
 * @bps_snd:     bytes per second was sent
 * @drop_pckt:   packet was drop
 * @lost_bytes:  bytes was lost
 */
struct cou_stat {
	u64 pckt_rcv;
	u64 pckt_snd;
	u64 bytes_rcv;
	u64 bytes_snd;
	u64 pps_rcv;
	u64 pps_snd;
	u64 bps_rcv;
	u64 bps_snd;
	u64 drop_pckt;
	u64 lost_bytes;
};

/**
 * struct cou_data - the COU-proto data structure
 * @mutex:               mutex for shared access to the endpoint
 *                       buffer
 * @a_servise_timeout:   service timeout
 * @read_exec:           driver-specific packet write
 *                       callback
 * @write_exec:          driver-specific packet read
 *                       callback
 * @copy_exec:           driver-specific callback to copy packet
 *                       to local ring buffer
 * @driver_data:         driver-specific callback argument
 * @pckt_proc:
 * @stat_proc:
 * @statistics:          statistics
 * @stat_pckt_rcv_cnt:   counter of received packets
 * @stat_pckt_snd_cnt:   counter of sent packets
 * @stat_bytes_rcv_cnt:  counter of received bytes
 * @stat_bytes_snd_cnt:  counter of sent bytes
 * @stat_pckt_drop_cnt:  counter of drop packets
 * @stat_bytes_lost_cnt: counter of drop bytes
 */
struct cou_data {
	struct mutex mutex;
	atomic_t a_servise_timeout;
	struct task_struct *pckt_proc;
	struct task_struct *stat_proc;
	struct cou_stat statistics;
	atomic_t stat_pckt_rcv_cnt;
	atomic_t stat_pckt_snd_cnt;
	atomic_t stat_bytes_rcv_cnt;
	atomic_t stat_bytes_snd_cnt;
	atomic_t stat_pckt_drop_cnt;
	atomic_t stat_bytes_lost_cnt;
};

/**
 * struct ec_usb_endpoit - endpoint parameters structure
 *
 * @usb_endpoint_descriptor: usb device descriptor
 * @struct urb:              usb request block structure
 * @buff:                    buffer for transfer/receive data
 * @buff_size:               buffer size
 *
 */
struct ec_usb_ep {
	struct usb_endpoint_descriptor *desc;
	struct urb *urb;
	int pipe;
	u8 *buff;
	size_t buff_size;
};

/**
 * struct ec_dev - main ec control structure
 *
 * @usb_dev:     usb device data
 * @interface:   the interface for this device
 * @kref:        reference counter
 * @ep_bulk_in:  data structure of the bulk-in usb endpoint
 * @ep_bulk_out: data structure of the bulk-out usb endpoint
 * @ep_int_in:   data structure of the interrupt-in usb endpoint
 * @async_event: asynchronous queue for high
 * @cou:         CAN over USB protocol data sturecture
 * @cb_list:     callbacks list
 *
 */
struct ec_dev {
	struct usb_device *usb_dev;
	struct usb_interface *interface;
	struct kref kref;
	struct completion hold;
	struct ec_usb_ep ep_bulk_in;
	struct ec_usb_ep ep_bulk_out;
	struct ec_usb_ep ep_int_in;
	struct fasync_struct *async_event;
	struct cou_data cou;
	struct cb_list cb_list;
};

/**
 * struct process_context_t - process context attribute structure
 *
 * @ec:        ec device data
 * @handl_cb:  data structure of callback parameters
 * @flags:     flags of process
 * @event:     events wait queue
 * @ring_buff: ring buffer
 */
struct process_context {
	struct ec_dev *ec;
	struct cb_data *cb_data;
	u32 flags;
	wait_queue_head_t event;
	struct ring_packet_buff ring_buff;
};

/* table of devices that work with this driver */
static struct usb_device_id ec_id_table[] = {
	{USB_DEVICE(0x0483, 0xfff0)},
	{}
};

MODULE_DEVICE_TABLE(usb, ec_id_table);
static DEFINE_MUTEX(usb_dev_mutex);

/* declaration ec driver structure for current device */
static struct usb_driver ec_usb_driver;

/**
 * ring_buff_is_empty() - checks buffer contents - empty or not?
 *
 * @ring_buff: ring buffer data structure
 *
 * Returns true if the circular buffer is empty.
 */
static bool ring_buff_is_empty(ring_pckt_buff_t *ring_buff)
{
	return ring_buff->tail == ring_buff->head;
}

/**
 * ring_buff_is_full() - checks buffer contents - full or not?
 *
 * @ring_buff: ring buffer data structure
 *
 * Returns true if the circular buffer is full
 */
static bool ring_buff_is_full(ring_pckt_buff_t* ring_buff)
{
	if (!CIRC_SPACE(ring_buff->head, ring_buff->tail, CIRCULAR_NUM_ENTRY))
		return true;
	return false;
}

/**
 * ring_buff_look_up_u32() - match packet header in buffer
 *
 * @ring_buff: ring buffer data structure
 * @hdr:       the header at the beginning of the package for
 *             searching in the buffer.
 * @size:      size of value
 *
 * This procedure searches for a packet header in the ring buffer.
 * The procedure will read packets from the buffer and move the position of
 * tail until the required header is found.
 *
 * Returns true if the values match or false otherwise.
 */
static bool ring_buff_look_up_u32(ring_pckt_buff_t *ring_buff,
                                  u32 hdr,
                                  u32 mask)
{
	while (!ring_buff_is_empty(ring_buff)) {
		u32 compared, reference;
		reference = hdr | mask;
		compared = *((u32*)ring_buff->buff[ring_buff->tail]);
		compared |= mask;
		if (compared == reference)
			return true;
		MOVE_POSITION(ring_buff->tail);
	}
	return  false;
}

/**
 * ring_buff_write_packet() - write data to ring buffer
 *
 * @ring_buff:  ring buffer data structure
 * @write_from: write data from this memory
 * @size:       size of data
 *
 * This procedure copies the packet from the shared buffer to the local
 * ring buffer of the current process context
 *
 * Returns:
 * -ENOBUFS - ring buffer is full
 * -EINVAL  - invalid parameter < size >
 * 0 - no errors
 */
static int ring_buff_write_packet(ring_pckt_buff_t *ring_buff,
				  void *write_from,
				  u8 size)
{
	if (size > CONTEXT_PCKT_BUF_SIZE)
		return  -EINVAL;
	if (ring_buff_is_full(ring_buff))
		return  -ENOBUFS;
	memcpy(ring_buff->buff[ring_buff->head], write_from, size);
	ring_buff->packet_size[ring_buff->head] = size;

	MOVE_POSITION(ring_buff->head);
	return 0;
}

/**
 * ring_buff_read_packet() - read data from ring buffer
 *
 * @ring_buff: ring buffer data structure
 * @read_to:   copy data to this memory
 * @size:      size of data
 *
 * This procedure copies the earliest packet from the local ring buffer to
 * <read_to> memory.
 *
 * Returns:
 * -ENOBUFS - ring buffer is empty
 * 0 - no errors
 */
static int ring_buff_read_packet(ring_pckt_buff_t *ring_buff,
				 void *read_to,
				 u8 *count)
{
	if (ring_buff_is_empty(ring_buff))
		return -ENOBUFS;

	if (*count > ring_buff->packet_size[ring_buff->tail])
		*count = ring_buff->packet_size[ring_buff->tail];
	memcpy(read_to, ring_buff->buff[ring_buff->tail], *count);
	MOVE_POSITION(ring_buff->tail);
	return 0;
}

/**
 * ring_buff_user_read_packet() - read data from ring buffer to user space
 *
 * @ring_buff:     ring buffer data structure
 * @read_to_user:  copy data to this memory
 * @size:          size of data
 *
 * This procedure copies the earliest packet from the local ring buffer to
 * <read_to_user> memory in user space.
 *
 * Returns:
 * -ENOBUFS - ring buffer is empty
 * -EFAULT  - error when copying data to user space
 * 0 - no errors
 */
static int ring_buff_user_read_packet(ring_pckt_buff_t *ring_buff,
				      void __user *read_to_user,
				      u8 *count)
{
	int rv = 0;
	if (ring_buff_is_empty(ring_buff)) {
		EC_ERR("Ring buffer is empty!\n");
		return -ENOBUFS;
	}

	if (*count > ring_buff->packet_size[ring_buff->tail])
		*count = ring_buff->packet_size[ring_buff->tail];

	if (copy_to_user(read_to_user, ring_buff->buff[ring_buff->tail], *count)) {
		EC_ERR("copy_to_user: rv = -EFAULT \n");
		rv = -EFAULT;
	}
	MOVE_POSITION(ring_buff->tail);
	return rv;
}

/**
 * msg_cb_add() - add a new callback
 *
 * @cb:         callbacks data structure
 * @new_sig_cb: callback to send a signal
 * @new_msg_cb: callback to send a new message
 * @context:    context of process parameters
 *
 * Add a new specified structure with callbacks to the cb list
 *
 * Return: pointer to the created handlers structure
 * NULL_PTR - could not allocate memory
 */
static struct cb_data* msg_cb_add(struct process_context *context,
				  struct cb_list *cb,
				  msg_cb_t new_msg_cb,
				  sig_cb_t new_sig_cb)
{
	struct cb_data *new_cb_handler;

	new_cb_handler = devm_kzalloc(&context->ec->usb_dev->dev,
				      sizeof(struct cb_data),
				      GFP_KERNEL);
	if (!new_cb_handler)
		return NULL;

	new_cb_handler->proc_context_data = context;
	new_cb_handler->msg_cb = new_msg_cb;
	new_cb_handler->sig_cb = new_sig_cb;

	write_lock(&cb->list_lock);
	list_add_tail(&new_cb_handler->list, &cb->hndl_list);
	write_unlock(&cb->list_lock);
	return new_cb_handler;
}

/**
 * msg_cb_del() - delete callback
 *
 * @cb:         callbacks data structure
 * @new_sig_cb: callback to send a signal
 * @new_msg_cb: callback to send a new message
 * @context:    context of process parameters
 *
 * Delete the specified structure (old_hndl_cb) with callbacks from the cb
 * list
 */
static void msg_cb_del(struct process_context *context,
		       struct cb_list *cb,
		       struct cb_data *old_hndl_cb)
{
	write_lock(&cb->list_lock);
	list_del(&old_hndl_cb->list);
	write_unlock(&cb->list_lock);
	devm_kfree(&context->ec->usb_dev->dev, old_hndl_cb);
}

/**
 * msg_cb_rcv_pckt_engine() - copy the received packet to all ring buffers
 *
 * @cb:        callbacks data structure
 * @pckt_buf:  pointer to packets buffer with a new packet
 * @size:      size of this packet
 *
 * This procedure uses the callback of each process to copy the received
 * packet to the local circular buffer in the context of this process.
 *
 * Return: error status
 * 0 - OK
 * -ENOBUFS - the ring buffer of one of the processes is full
 */
static int msg_cb_rcv_pckt_engine(struct cb_list *cb, void *pckt_buf, u8 size)
{
	struct list_head *iter;
	struct cb_data *hndl_cb_entry;
	int rv = 0, all_rv = 0;

	read_lock(&cb->list_lock);
	list_for_each(iter, &cb->hndl_list) {
		hndl_cb_entry = list_entry(iter, struct cb_data, list);
		if (hndl_cb_entry->msg_cb) {
			rv = hndl_cb_entry->msg_cb(pckt_buf,
					size,
					hndl_cb_entry->proc_context_data);
			if (rv == -ENOBUFS)
				all_rv |= rv;
			else if (rv == -ENODEV) {
				all_rv = rv;
				break;
			}
		}
	}
	read_unlock(&cb->list_lock);
	return all_rv;
}

/**
 * cou_packet_rcv_process() - The packet handling procedure that
 *                            comes from mcu
 *
 * When the hardware buffer of the usb interface on the MCU side is full,
 * errors and mcu failure can occur. These are features of this system.
 * User-space application in linux cant provide real time. Therefore, the
 * software needs to read the packet via USB bus from the MCU as soon as
 * it enters the hardware buffer on the MCU side. cou_packet_rcv_process()
 * reads packets into a special circular buffer that is created when the
 * character device /dev/mcu is opened.
 *
 * @arg: pointer to <struct ec_dev>
 *
 * Return: error status
 * 0 - OK
 * -ENODEV  - the connection to the device was interrupted
 *
 */
static int cou_packet_rcv_process(void *arg)
{
	struct ec_dev *ec = (struct ec_dev*)arg;
	struct cou_data *cou = &ec->cou;
	int timeout, rv;
	u32 *raw_packet_header;
	int size = 0;

	EC_LOG("Start cou_packet_rcv_process!\n");
	while(!kthread_should_stop()) {
		/*
		 *  packet waiting timeout can be set from user space.
		 *  this parameter is called the <Service Timeout> and is
		 *  set by the function cou_srv_timeout_set(..)
		 */
		timeout = atomic_read(&cou->a_servise_timeout);
		rv = usb_bulk_msg(ec->usb_dev,
				  ec->ep_bulk_in.pipe,
				  ec->ep_bulk_in.buff,
				  ec->ep_bulk_in.buff_size,
				  &size,
				  timeout);
		switch(rv) {
		case 0:
			if ((size > COU_PACKET_SIZE) ||
			    (size < COU_ATTR_HEADER_SIZE)) {
				/* the packet size is not valid */
				atomic_inc(&cou->stat_pckt_drop_cnt);
				atomic_add(size, &cou->stat_bytes_lost_cnt);
				EC_ERR("The packet is not valid!\n");
				continue;
			}

			/* all reserved bits must be set to 1 */
			raw_packet_header = (u32*)ec->ep_bulk_in.buff;
			*raw_packet_header |= MASK_RES_BITS;

			rv = msg_cb_rcv_pckt_engine(&ec->cb_list,
						    ec->ep_bulk_in.buff,
						    (u8)size);
			if (rv) {
				atomic_inc(&cou->stat_pckt_drop_cnt);
				atomic_add(size, &cou->stat_bytes_lost_cnt);
				EC_ERR("Callbacks have occurred"
						"with errors!\n");
				break;
			}
			/* update statistics */
			atomic_inc(&cou->stat_pckt_rcv_cnt);
			atomic_add(size, &cou->stat_bytes_rcv_cnt);
			break;

		case -ETIMEDOUT:
			break;

		case -EPIPE:
			EC_ERR("USB pipe has been broken!: rv = %d\n", rv);
		case -ENODEV:
		case -ESHUTDOWN:
		case -ECONNRESET:
		case -ENOENT:
		case -EILSEQ:
			EC_ERR("MCU device is not responding! rv = %d\n", rv);
			EC_INFO("Exit from cou_packet_rcv_process!\n");
			return -ENODEV;

		case -EPROTO:
			EC_ERR("USB protocol error! rv = %d\n", rv);
			break;

		default:
			EC_ERR("unknown status received: rv = %d\n", rv);
			break;

		} /* end switch(rv) */

	} /* end while( !kthread_should_stop() ... */

	EC_INFO("Exit from cou_packet_rcv_process!\n");
	cou->pckt_proc = NULL;
	return 0;
}

/**
 * ec_usb_data_complete()
 */
static void cou_data_transfer_complete(struct urb *urb)
{
	int status = urb->status;
	struct ec_dev *ec = (struct ec_dev*) urb->context;

	if (status && !(status == -ENOENT ||
			status == -ECONNRESET ||
			status == -ESHUTDOWN)) {
		/* sync/async unlink faults aren't errors */
		EC_ERR("nonzero write bulk status received: %d\n", status);
	}
	complete(&ec->hold);
}

/**
 * cou_packet_engine() - transmit packet
 * @ec:           main ec data structure
 * @packet_buff:  packet buffer
 * @size:         size of buffer
 * @jiffies_wait: cpu jiffies for waiting
 *
 * This function creates and transmits packet of COU protocol to main mcu
 * in can network
 *
 * Return: error status and URB status
 * 0            - OK
 * -ENODEV      - incorrectly specified device structure or device
 *                is not detected
 * -EFUTEX      - error with locking futex
 * -EOVERFLOW   - data package size is too large
 * -ERESTARTSYS - interrupted by a signal
 * -EINVAL      - the packet header is not valid
 * -ETIMEDOUT   - timeout
 * and URB error status
 */
static int cou_packet_engine(struct ec_dev *ec,
			     void *raw_packet,
			     u8 size,
			     unsigned long jiffies_wait)
{
	int rv;
	struct cou_data *cou = &ec->cou;
	u32 *raw_packet_header;
	unsigned long compl_timeout;

	if ((size > COU_PACKET_SIZE) || (size < COU_ATTR_HEADER_SIZE)) {
		/* the packet size is not valid */
		EC_ERR("The packet header is not valid size = %i!\n",
		       (int)size);
		return -EINVAL;
	}
	rv = mutex_lock_interruptible(&cou->mutex);
	if (rv) {
		/* error with locking */
		EC_INFO("Mutex was not captured! Error = %d \n", rv);
		return rv;
	}

	do {
		/* all reserved bits must be set to 1 */
		raw_packet_header = (u32*)raw_packet;
		*raw_packet_header |= MASK_RES_BITS;

		memcpy(ec->ep_bulk_out.buff, raw_packet, size);
		ec->ep_bulk_out.urb->transfer_buffer_length = size;
		ec->ep_bulk_out.urb->complete = cou_data_transfer_complete;
		ec->ep_bulk_out.urb->context = ec;
		/*
		 * usb-core uses the buffer pointed to by the setup_dma
		 * variable instead of the setup_packet variable
		 */
		ec->ep_bulk_out.urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		reinit_completion(&ec->hold);
		rv = usb_submit_urb(ec->ep_bulk_out.urb, GFP_KERNEL);
		if (rv)
			break;

		/* to block process for waiting completion from callback */
		compl_timeout = msecs_to_jiffies(COU_ENGINE_TIMEOUT_MS);
		rv = wait_for_completion_interruptible_timeout(&ec->hold,
							       compl_timeout);
		if (rv <= 0) {
			/* timeout or interruptible from signal */
			rv = rv ? -ERESTARTSYS : -ETIMEDOUT;
			break;
		}
		mutex_unlock(&cou->mutex);
		/* update statistics */
		atomic_inc(&cou->stat_pckt_snd_cnt);
		atomic_add(size, &cou->stat_bytes_snd_cnt);
		/* return urb status */
		return ec->ep_bulk_out.urb->status;
	} while(false);

	complete_all(&ec->hold);
	usb_kill_urb(ec->ep_bulk_out.urb);
	mutex_unlock(&cou->mutex);
	/* update lost bytes */
	atomic_inc(&cou->stat_pckt_drop_cnt);
	atomic_add(size, &cou->stat_bytes_lost_cnt);
	EC_ERR("Packet transfer error = %d!\n", rv);
	return  rv;
}

/**
 * cou_stat_pps_bps_update() - update packet statistic for COU protocol
 * @cou: data structure of the COU protocol
 */
static void cou_stat_pps_bps_update(struct cou_data *cou)
{
	cou->statistics.pps_rcv = atomic_read(&cou->stat_pckt_rcv_cnt);
	cou->statistics.pps_snd = atomic_read(&cou->stat_pckt_snd_cnt);
	cou->statistics.bps_rcv = atomic_read(&cou->stat_bytes_rcv_cnt);
	cou->statistics.bps_snd = atomic_read(&cou->stat_bytes_snd_cnt);

	cou->statistics.pckt_rcv += atomic_read(&cou->stat_pckt_rcv_cnt);
	cou->statistics.pckt_snd += atomic_read(&cou->stat_pckt_snd_cnt);
	cou->statistics.bytes_rcv += atomic_read(&cou->stat_bytes_rcv_cnt);
	cou->statistics.bytes_snd += atomic_read(&cou->stat_bytes_snd_cnt);

	cou->statistics.drop_pckt += atomic_read(&cou->stat_pckt_drop_cnt);
	cou->statistics.lost_bytes += atomic_read(&cou->stat_bytes_lost_cnt);

	atomic_set(&cou->stat_pckt_rcv_cnt, 0);
	atomic_set(&cou->stat_pckt_snd_cnt, 0);
	atomic_set(&cou->stat_bytes_rcv_cnt, 0);
	atomic_set(&cou->stat_bytes_snd_cnt, 0);
	atomic_set(&cou->stat_pckt_drop_cnt, 0);
	atomic_set(&cou->stat_bytes_lost_cnt, 0);
}

/**
 * cou_stat_upd_process() - update statistic
 */
static int cou_stat_upd_process(void *context)
{
	struct cou_data *cou = (struct cou_data*) context;
	unsigned long timeout;
	int rv = 0;

	EC_INFO("Start cou_stat_upd_process!\n");
	while(!kthread_should_stop()) {
		/* 1 sec soft delay */
		timeout = msecs_to_jiffies(STAT_INTERVAL_UPDATE);
		schedule_timeout_interruptible(timeout);

		if (kthread_should_stop())
			break;
		cou_stat_pps_bps_update(cou);
	}
	EC_INFO("Stop cou_stat_upd_process!\n");
	cou->stat_proc = NULL;
	return rv;
}

/**
 * cou_srv_timeout_set() - set service timeout
 * @cou:     data structure of the COU protocol
 * @timeout: service timeout
 *
 * Return: error status
 * 0 - OK
 * -EINVAL  - invalid argument
 */
static int cou_srv_timeout_set(struct cou_data *cou, unsigned int timeout)
{
	if ((timeout < SERVICE_TIMEOUT_MIN) ||
	    (timeout > SERVICE_TIMEOUT_MAX)) {
		EC_ERR("Invalid timeout value! rv = %d\n", timeout);
		return -EINVAL;
	}
	atomic_set(&cou->a_servise_timeout, timeout);
	return 0;
}

/**
 * cou_srv_timeout_get() - get service timeout
 * @cou: data structure of the COU protocol
 *
 * Return: value of the service timeout
 */
static unsigned int cou_srv_timeout_get(struct cou_data *cou)
{
	return (unsigned int)atomic_read(&cou->a_servise_timeout);
}

/**
 * cou_stat_get() - Copy statistics to buffer
 * @cou:  data structure of the COU protocol
 * @buf:  buffer to copy
 * @size: size of buffer
 *
 * Return: the actual size of the statistics in the buffer.
 */
static ssize_t cou_stat_get(struct cou_data *cou, char *buf, size_t size)
{
	struct cou_stat statistics;
	memcpy(&statistics, &cou->statistics, sizeof(struct cou_stat));
	return snprintf(buf,
			size,
			"received:   %llu bytes\n"
			"            %llu packets\n"
			"            %llu bps\n"
			"            %llu pps\n"
			"sent:       %llu bytes\n"
			"            %llu packets\n"
			"            %llu bps\n"
			"            %llu pps\n"
			"lost:       %llu bytes\n"
			"            %llu packets\n\n"
			"waiting to be sent:\n",
			statistics.bytes_rcv,
			statistics.pckt_rcv,
			statistics.bps_rcv,
			statistics.pps_rcv,
			statistics.bytes_snd,
			statistics.pckt_snd,
			statistics.bps_snd,
			statistics.pps_snd,
			statistics.lost_bytes,
			statistics.drop_pckt);
}

/**
 * cou_stat_reset() - Resets statistics on received/sent/lost packets
 * @cou: data structure of the COU protocol
 */
static void cou_stat_reset(struct cou_data *cou)
{
	memset(&cou->statistics, 0, sizeof(struct cou_stat));
}

/**
 * ec_usb_data_arrived_impl()
 */
static int ec_usb_data_arrived_impl(void *usb_ep_buf,
                                    u8 size,
                                    void *user_proc_context)
{
	int rv, ev;
	unsigned long cb_context_buff_full_timeout;
	struct process_context *context =
			(struct process_context*)user_proc_context;

	if (ring_buff_is_full(&context->ring_buff)) {
		/*
		 *  if the buffer is full at the moment the packet arrives,
		 *  the system pauses to allow the application a little more
		 *  time to process the received packets
		 */
		cb_context_buff_full_timeout =
				usecs_to_jiffies(EC_RING_BUFFER_PAUSE_US);

		EC_ERR("Ring buffer is full! Pause %d us!\n",
		       EC_RING_BUFFER_PAUSE_US);

		rv = wait_event_interruptible_timeout(
				context->event,
				(ev = TST_MSG_EVENT(context, DISCONNECT_DEV)),
				cb_context_buff_full_timeout);
		if (ev)
			return -ENODEV;
		rv = 0;
	}
	/*
	 *  copy to loacal ring buffer in data context
	 *  of the current process
	 */
	rv = ring_buff_write_packet(&context->ring_buff, usb_ep_buf, size);
	if (rv == -ENOBUFS) {
		/* packet ring buffer is full */
		EC_WARN("Write packet to local context buffer is failed. "
			"Ring buffer is full!\n");
	}
	SND_MSG_EVENT(context, RECEIVE_PACKET);
	wake_up_interruptible(&context->event);
	return rv;
}

/**
 * ec_usb_signal_impl()
 */
static int ec_usb_signal_impl(u32 event_type, void *user_proc_context)
{
	struct process_context *context =
			(struct process_context *) user_proc_context;
	SND_MSG_EVENT(context, event_type);
	wake_up_interruptible(&context->event);
	return 0;
}

/**
 * hbp_filter_passed()
 */
static int hbp_filter_passed(struct process_context *self, u8 *cou_packet)
{
	int rv = 0;
	u32 packet_hdr, mask;
	u8  packet_size = COU_PACKET_SIZE;

	memcpy(&packet_hdr, cou_packet, COU_ATTR_HEADER_SIZE);
	/* set mask for priority */
	mask = MASK_RES_BITS;
	if (TST_MSG_EVENT(self, DISCONNECT_DEV))
		return -ENODEV;
	if (TST_MSG_EVENT(self, RECEIVE_PACKET)) {
		/* cou packet was received */
		if (!ring_buff_is_empty(&self->ring_buff)) {
			/* packet ring buffer is not empty */
			rv = -EINPROGRESS;
			if (!cou_packet) {
				/* use header based filter */
				if (ring_buff_look_up_u32(&self->ring_buff,
							  packet_hdr,
							  mask))
					ring_buff_read_packet(&self->ring_buff,
							      cou_packet,
							      &packet_size);
				else
					/* no matches found */
					rv = 0;
			}
		}
		CLR_MSG_EVENT(self, RECEIVE_PACKET);
	}
	return rv;
}

/**
 * hb_filter_wait_packet_timeout()
 */
static int hbp_filter_wait_packet_timeout(struct process_context *self,
                                          u8 *packet,
                                          unsigned long jiffies_wait)
{
	int rv, ev = 0;
	if (jiffies_wait) {
		/* wait with timeout */
		rv = wait_event_interruptible_timeout(
				self->event,
				(ev = hbp_filter_passed(self, packet)),
				jiffies_wait);
		if (rv <= 0)
			return rv ? -ERESTARTSYS : -ETIMEDOUT;
	} else {
		rv = wait_event_interruptible(
					self->event,
					(ev = hbp_filter_passed(self,packet)));
		if (rv)
			return -ERESTARTSYS;
	}
	if (ev == -ENODEV)
		return ev;
	return 0;
}

/**
 * ec_usb_hot_reset() - Use this function to reset the USB interface
 *                      from the host to the microcontroller
 *
 * @ec: ec control structure
 */
static int ec_usb_hot_reset(struct ec_dev *ec)
{
	struct usb_interface *ec_usb_iface = ec->interface;
	EC_INFO("Reset USB interface to the controller! ...\n");

	/*
	 * Reset EC USB device from an atomic context
	 * where usb_reset_device won't work (as it blocks).
	 * There is no no need to lock/unlock the reset_ws as
	 * schedule_work does its own.
	 */
	usb_queue_reset_device(ec_usb_iface);

	EC_INFO("...Done!\n");
	return 0;
}

/**
 * ec_delete()
 */
static void ec_delete(struct kref *kref)
{
	struct ec_dev *ec  = TO_EC_DEV(kref);

	EC_INFO("Delete device from devfs table!...");
	if (ec->ep_bulk_out.desc) {
		if (ec->ep_bulk_out.buff) {
			/* free up allocated buffer */
			usb_free_coherent(ec->usb_dev,
					  ec->ep_bulk_out.buff_size,
					  ec->ep_bulk_out.buff,
					  ec->ep_bulk_out.urb->transfer_dma);
		}
		usb_free_urb(ec->ep_bulk_out.urb);
		ec->ep_bulk_out.desc = NULL;
	}

	ec->ep_bulk_in.desc = NULL;

	if (ec->ep_int_in.desc) {
		/* deactivate dma buffer */
		if (ec->ep_int_in.buff) {
			usb_free_coherent(ec->usb_dev,
					  ec->ep_int_in.buff_size,
					  ec->ep_int_in.buff,
					  ec->ep_int_in.urb->transfer_dma);
		}
		/* free urb for interrupt-in endpoint */
		usb_free_urb(ec->ep_int_in.urb);
		ec->ep_int_in.desc = NULL;
	}
	usb_put_dev(ec->usb_dev);
	schedule_timeout_interruptible(msecs_to_jiffies(EC_REMOVE_WAIT_MS));
	EC_WARN(" ...done!\n");
}

/**
 * ec_open()
 */
static int ec_open(struct inode *inode, struct file *file)
{
	struct ec_dev *ec;
	struct usb_interface *interface;
	int sub_minor;
	struct process_context *proc_context;

	EC_INFO("Open file of ec device pid = %d <%s>\n", current->pid,
			current->comm);
	mutex_lock(&usb_dev_mutex);
	sub_minor = iminor(inode);
	interface = usb_find_interface(&ec_usb_driver, sub_minor);
	if (!interface) {
		mutex_unlock(&usb_dev_mutex);
		dev_err(&interface->dev,
		        "Can't find device for minor %d\n",
		        sub_minor);
		return -ENODEV;
	}

	ec = (struct ec_dev*) usb_get_intfdata(interface);
	if (!ec) {
		dev_err(&interface->dev, "Device not found\n");
		mutex_unlock(&usb_dev_mutex);
		return -ENODEV;
	}

	proc_context = devm_kzalloc(&ec->usb_dev->dev,
				    sizeof(struct process_context),
				    GFP_KERNEL);
	if (!proc_context) {
		/* error with alloc */
		mutex_unlock(&usb_dev_mutex);
		EC_ERR("Could not allocate memory for process context - %d!\n",
		       current->pid);
		return -ENOMEM;
	}

	proc_context->ec = ec;
	proc_context->flags = 0;
	init_waitqueue_head(&proc_context->event);
	/* register callbacks for this process */
	proc_context->cb_data = msg_cb_add(proc_context,
					   &ec->cb_list,
					   ec_usb_data_arrived_impl,
					   ec_usb_signal_impl);
	if (!proc_context->cb_data) {
		mutex_unlock(&usb_dev_mutex);
		EC_ERR("Error when registering a callback!\n");
		devm_kfree(&ec->usb_dev->dev, proc_context);
		return -ENOMEM;
	}

	/* put process context to private structure */
	file->private_data = proc_context;

	/* increment our usage count for the device */
	kref_get(&ec->kref);
	mutex_unlock(&usb_dev_mutex);
	EC_INFO("Open file of ec device\n");
	return 0;
}

/**
 * ec_release()
 */
static int ec_release(struct inode *inode, struct file *file)
{
	struct ec_dev *ec;
	struct process_context *proc_context;

	EC_INFO("Release file of ec device\n");
	mutex_lock(&usb_dev_mutex);
	proc_context = (struct process_context*)file->private_data;
	ec = proc_context->ec;

	msg_cb_del(proc_context, &ec->cb_list, proc_context->cb_data);
	devm_kfree(&ec->usb_dev->dev, proc_context);
	file->private_data = NULL;

	mutex_unlock(&usb_dev_mutex);
	kref_put(&ec->kref, ec_delete);
	return  0;
}

/**
 * ec_can_packet_read()
 */
static ssize_t ec_can_packet_read(struct file *file,
				  char __user *user_data_p,
				  size_t count,
				  loff_t *unused)
{
	register struct process_context *self_context;
	int rv = 0;
	u8 packet_size = (u8)count;

	self_context = (struct process_context*)file->private_data;
	/* TODO: need to add a check: the device is disconnected or not */

	if (ring_buff_is_empty(&self_context->ring_buff)) {
		if (!(file->f_flags & O_NONBLOCK)) {
			/* wait signal about receiving packet */
			rv = hbp_filter_wait_packet_timeout(self_context,
								NULL, false);
			if (rv) {
				/* interruptible from signal */
				EC_ERR("Waiting has been interrupted! rv=%d\n",
					rv);
				return -ERESTARTSYS;
			}
		} else {
			/* non blocking call */
			rv = hbp_filter_passed(self_context, NULL);
			if (rv == -ENODEV)
				return -ENODEV;
			else if (rv != -EINPROGRESS)
				return -EAGAIN;
		}
	}
	rv = ring_buff_user_read_packet(&self_context->ring_buff,
	                                user_data_p,
	                                &packet_size);
	if      (!rv)
		rv = packet_size;
	else if (rv == -EFAULT) {
		EC_ERR("Error when copying data to user space! rv = %d\n", rv);
		return  rv;
	} else if (rv == -ENOBUFS) {
		/* error with using ring buffer */
		EC_ERR("Error with using ring buffer! rv = %d\n", rv);
		return rv;
	}
	return rv;
}

/**
 * ec_can_packet_write()
 */
static ssize_t ec_can_packet_write(struct file *file,
				   const char __user *user_data_p,
				   size_t count,
				   loff_t *unused)
{
	struct process_context *proc_context =
				(struct process_context*) file->private_data;
	u8 can_raw_packet[COU_PACKET_SIZE];
	int rv;

	/* TODO: need to add a check: the device is disconnected or not */
	if (!count)
		return count;

	memset(can_raw_packet, 0, sizeof(can_raw_packet));
	rv = copy_from_user(&can_raw_packet, user_data_p, count);
	if (rv) {
		/* error occurred while copying from user space */
		EC_ERR("Error with copying data from user space! rv = %d\n",
		       rv);
		return -EFAULT;
	}
	rv = cou_packet_engine(proc_context->ec, &can_raw_packet, count, 0);
	if (rv)
		return rv;
	else
		return count;
}

/**
 * ec_event_poll()
 */
static unsigned int ec_event_poll(struct file *file, poll_table *wait)
{
	struct process_context *proc_context =
				(struct process_context*)file->private_data;
	unsigned int mask = 0;

	do {
		/* TODO: need to add a check: the device is disconnected */
		if (!ring_buff_is_empty(&proc_context->ring_buff)) {
			/*
			 *  do not wait if there are packets in the ring buffer
			 *  it is necessary to quickly subtract all messages
			 */
			mask = POLLIN | POLLRDNORM;
			break;
		}
		/*  wait until get interrupt message from ec */
		poll_wait(file, &proc_context->event, wait);
		if (TST_MSG_EVENT(proc_context, DISCONNECT_DEV)) {
			/* the device failure event occurred */
			mask = POLLERR;
			break;
		}

		if (!ring_buff_is_empty(&proc_context->ring_buff))
			mask = POLLIN | POLLRDNORM;

	} while (false);

	if (mask == POLLERR) {
		/* device is not detected */
		EC_ERR("Device is not detected! "
		       "Error with reading private data!\n");
		return  POLLERR | POLLHUP;
	}

	/*
	 *  define the type of event. In the future, the number
	 *  of types of events can increase.
	 */
	if (TST_MSG_EVENT(proc_context, HI_PRIO_EVENT))
		mask |= POLLPRI;
	if (TST_MSG_EVENT(proc_context, USB_HARD_ERR))
		mask |= POLLERR;

	return  mask;
}

/**
 * ec_msg_fasync()
 */
static int ec_msg_fasync(int fd, struct file *file, int mode)
{
	struct process_context *proc_context =
			(struct process_context *)file->private_data;
	struct ec_dev *ec = proc_context->ec;

	/* TODO: need to add a check: the device is disconnected */
	/* create asynchronous event queue */
	EC_INFO("Call ec_msg_fasync: pid = %d\n", current->pid);
	return fasync_helper(fd, file, mode, &ec->async_event);
}

/**
 * ec_cmd_ioctl()
 */
static long ec_cmd_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rv = 0;
	struct process_context *proc_context;
	struct ec_dev *ec;
	u32 *raw_packet_header;
	u8 cmd_packet[COU_PACKET_SIZE];

	proc_context = (struct process_context *)file->private_data;

	/* TODO: need to add a check: the device is disconnected */
	ec = proc_context->ec;
	switch(cmd) {
	case EC_CAN_CTRL:
		break;

	case EC_SERV_TIMEOUT_SET:
		if (copy_from_user(&rv, (void __user*)arg, sizeof(int))) {
			/* error occurred while copying from user space */
			EC_ERR("Error with copying data from "
			       "user space! rv = %d\n",
			       rv);
			return  -EFAULT;
		} else  rv = cou_srv_timeout_set(&ec->cou, rv);
		break;

	case EC_SERV_TIMEOUT_GET:
		rv = cou_srv_timeout_get(&ec->cou);
		if (copy_to_user((void __user*)arg, &rv, sizeof(int))) {
			/* error occurred while copying to user space */
			EC_ERR("Error when copying data"
			       "to user space! rv = %d\n",
			       rv);
			return -EFAULT;
		} else
			rv = 0;
		break;

	case EC_HBP_CAN_FLTR_SET:
		memset(cmd_packet,0,COU_PACKET_SIZE);
		rv = copy_from_user(&cmd_packet,
				    (void __user*)arg,
				    COU_PACKET_SIZE);
		if (rv) {
			/* error occurred while copying from user space */
			EC_ERR("Error with copying data from"
			       "user space! rv = %d\n",
			       rv);
			return  -EFAULT;
		}

		/* all reserved bits must be set to 1 */
		raw_packet_header = (u32*)ec->ep_bulk_in.buff;
		*raw_packet_header |= MASK_RES_BITS;
		
		if (!(file->f_flags & O_NONBLOCK)) {
			rv = hbp_filter_wait_packet_timeout(proc_context,
							    cmd_packet,
							    false);
		} else {
			/* non blocking call */
			rv = hbp_filter_passed(proc_context, cmd_packet);
			if (rv == -ENODEV)
				return -ENODEV;
			else if (rv != -EINPROGRESS)
				return -EAGAIN;
			rv = 0;
		}
		if (!rv) {
			/* packet passed through filter */
			rv = copy_to_user((void __user*)arg,
					  &cmd_packet,
					  COU_PACKET_SIZE);
			if (rv) {
				/* err occurred while copying to user space */
				EC_ERR("Error when copying data to"
				       "user space! rv = %d\n",
				       rv);
				return -EFAULT;
			}
		}
		break;

	case EC_USB_RESET:
		ec_usb_hot_reset(ec);
		break;

	default:
		EC_ERR("Unknown command! %i \n", cmd);
		return  -ENOIOCTLCMD;
	}
	return rv;
}

static const struct file_operations ec_fops = {
	.owner = THIS_MODULE,
	.read = ec_can_packet_read,
	.write = ec_can_packet_write,
	.open = ec_open,
	.unlocked_ioctl = ec_cmd_ioctl,
	.release = ec_release,
	.poll = ec_event_poll,
	.fasync = ec_msg_fasync,
	.llseek = noop_llseek,
};

static struct usb_class_driver ec_class = {
	.name = "mcu%d",
	.fops = &ec_fops,
	.minor_base = EC_USB_MINOR,
};

/**
 * cou_usb_interrupt_ep_handler()
 */
static void ec_usb_interrupt_ep_handler(struct urb *urb)
{
	int rv, status = urb->status;

	EC_INFO(": ec_usb_interrupt_ep_handler(): enter!\n");
	switch (status) {
	case 0:
		break;

	case -EOVERFLOW:
		EC_ERR("Overflow with length\n");

	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
	case -EILSEQ:
		/* The device is terminated, clean up */
		break;

	case -EPROTO:
		EC_ERR("Host EC device has been reset signal: %d\n", status);
		EC_INFO(": ec_usb_interrupt_ep_handler(): exit!\n");
		return;

	default:
		EC_ERR("unknown status received: %d\n", status);
		break;
	}

	/* send new interrupt-urb to usb stack */
	rv = usb_submit_urb(urb, GFP_ATOMIC);
	if (rv) {
		/* error with transmiting URB */
		EC_ERR("usb_submit_urb failed: %d\n", rv);
	}
	EC_INFO(": ec_usb_interrupt_ep_handler(): exit!\n");
}

/**
 * ec_sysfs_statistics_show()
 */
static ssize_t ec_sysfs_statistics_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct usb_interface *interface = to_usb_interface(dev);
	struct ec_dev *ec;

	if (!interface)
		return -ENODEV;
	/* print COU-statistic on screen */
	ec = (struct ec_dev *)usb_get_intfdata(interface);
	return cou_stat_get(&ec->cou, buf, STAT_BUF_SIZE);
}

/**
 * ec_sysfs_statistics_reset()
 */
static ssize_t ec_sysfs_statistics_reset(struct device* dev,
					 struct device_attribute* attr,
					 const char* buf,
					 size_t count)
{
	struct usb_interface *interface = to_usb_interface(dev);
	struct ec_dev *ec;

	if (!interface)
		return -ENODEV;
	ec = (struct ec_dev *)usb_get_intfdata(interface);
	/* clear COU statistics */
	cou_stat_reset(&ec->cou);
	return count;
}

/*
 * ec_sysfs_statistics
 */
static struct device_attribute ec_sysfs_statistics = {
	.attr.name = "statistic",
	.attr.mode = S_IRUGO | S_IWUSR,
	.show = ec_sysfs_statistics_show,
	.store = ec_sysfs_statistics_reset,
};

/**
 * ec_sysfs_usb_hot_reset() - reset the ec device by writing
 *                            "1" value to the "hreset" file
 *                            in the sysfs
 * Returns:
 * count   - success;
 * -ENODEV - dev isn`t found;
 */
static ssize_t ec_sysfs_usb_hot_reset(struct device* dev,
				      struct device_attribute* attr,
				      const char* buf,
				      size_t count)
{
	struct usb_interface *interface = to_usb_interface(dev);
	int rv = -ENODEV;
	struct ec_dev *ec;

	if (!interface)
		return rv;

	dev_info(dev, "Hot reset!\n");
	ec = (struct ec_dev *)usb_get_intfdata(interface);

	/* reset device */
	ec_usb_hot_reset(ec);

	return count;
}

/**
 * ec_sysfs_hot_reset
 */
static struct device_attribute ec_sysfs_hot_reset = {
	.attr.name = "hreset",
	.attr.mode = S_IWUSR,
	.show = NULL,
	.store = ec_sysfs_usb_hot_reset,
};

/**
 * ec_sysfs_dev_attrs[]
 */
static struct attribute *ec_sysfs_dev_attrs[] = {
	&ec_sysfs_statistics.attr,
	&ec_sysfs_hot_reset.attr,
	NULL
};

/**
 * ec_sysfs_dev_attr_grp
 */
static const struct attribute_group ec_sysfs_dev_attr_grp = {
	.attrs = ec_sysfs_dev_attrs,
};

/**
 * print_ep_info()
 */
static void print_ep_info(struct usb_endpoint_descriptor *ep_dscr)
{
	unsigned int temp;
	EC_PRINT_USB_EP_PARAM(ep_dscr, bLength, temp);
	EC_PRINT_USB_EP_PARAM(ep_dscr, bDescriptorType, temp);
	EC_PRINT_USB_EP_PARAM(ep_dscr, bEndpointAddress, temp);
	EC_PRINT_USB_EP_PARAM(ep_dscr, bmAttributes, temp);
	EC_PRINT_USB_EP_PARAM(ep_dscr, bInterval, temp);
	EC_PRINT_USB_EP_PARAM(ep_dscr, wMaxPacketSize, temp);
	EC_PRINT_USB_EP_PARAM(ep_dscr, bRefresh, temp);
	EC_PRINT_USB_EP_PARAM(ep_dscr, bSynchAddress, temp);
}

/**
 * ec_usb_ep_bulk_out_init()
 */
static int ec_usb_ep_bulk_out_init(struct usb_device *usb_dev,
				   struct ec_usb_ep *ep,
				   struct usb_endpoint_descriptor *ep_desc)
{
	ep->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!ep->urb)
		return -ENOMEM;

	/* allocate buffer and configure dma */
	ep->buff_size = ep_desc->wMaxPacketSize;
	ep->buff = (u8*)usb_alloc_coherent(usb_dev,
					   ep->buff_size,
					   GFP_KERNEL,
					   &ep->urb->transfer_dma);
	/* configure transfer pipe for bulk-out endpoint */
	ep->pipe = usb_sndbulkpipe(usb_dev, ep_desc->bEndpointAddress);
	/* determine urb parameters */
	ep->urb->dev = usb_dev;
	ep->urb->transfer_buffer = ep->buff;
	ep->urb->pipe = ep->pipe;
	ep->desc = ep_desc;
	return 0;
}

/**
 * ec_usb_ep_bulk_in_init()
 */
static int ec_usb_ep_bulk_in_init(struct usb_device *usb_dev,
				  struct ec_usb_ep *ep,
				  struct usb_endpoint_descriptor *ep_desc)
{
	/*
	 *  the module needs to get information about the size
	 *  of the hardware buffer on the usb side of the device
	 */
	ep->buff_size = ep_desc->wMaxPacketSize;
	ep->buff = (u8*)devm_kzalloc(&usb_dev->dev, ep->buff_size, GFP_KERNEL);
	if (!ep->buff)
		return -ENOMEM;
	/* configure recieve pipe */
	ep->pipe = usb_rcvbulkpipe(usb_dev, ep_desc->bEndpointAddress);
	ep->desc = ep_desc;
	return 0;
}

/**
 * ec_usb_ep_bulk_in_init()
 */
static int ec_usb_ep_interrupt_in_init(struct usb_device *usb_dev,
				       struct ec_usb_ep *ep,
				       struct usb_endpoint_descriptor *ep_desc)
{
	ep->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!ep->urb)
		return -ENOMEM;

	ep->buff_size = usb_endpoint_maxp(ep_desc);
	ep->buff = (u8*)usb_alloc_coherent(usb_dev,
					   ep->buff_size,
					   GFP_KERNEL,
					   &ep->urb->transfer_dma);
	if (!ep->buff)
		return -ENOMEM;

	ep->pipe = usb_rcvintpipe(usb_dev, ep_desc->bEndpointAddress);
	/* create usb request block for interrupt in endpoint */
	usb_fill_int_urb(ep->urb,
			 usb_dev,
			 ep->pipe,
			 ep->buff,
			 ep->buff_size,
			 ec_usb_interrupt_ep_handler,
			 ep,
			 ep_desc->bInterval);
	/* use dma */
	ep->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	ep->desc = ep_desc;
	return 0;
}

/**
 * ec_probe()
 */
static int ec_probe(struct usb_interface *interface,
		    const struct usb_device_id *id)
{
	struct ec_dev *ec = NULL;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *ep_descriptor;
	struct usb_device *new_usb_dev;
	int i, rv = -ENOMEM;

	new_usb_dev = usb_get_dev(interface_to_usbdev(interface));
	dev_info(&interface->dev, "Probe ec device!\n");
	ec = (struct ec_dev*) devm_kzalloc(&new_usb_dev->dev,
					   sizeof(struct ec_dev),
					   GFP_KERNEL);
	do {
		if (!ec) {
			/* error with allocate */
			dev_err(&interface->dev, "Out of memory\n");
			break;
		}
		/*
		 * the driver supports a large number of ec devices
		 * use the reference count for the numbering of each device
		 */
		kref_init(&ec->kref);

		/* init usb device parameters */
		ec->usb_dev = new_usb_dev;
		ec->interface = interface;
		iface_desc = interface->cur_altsetting;
		dev_info(&interface->dev,
			 "device have %d endpoints\n",
			 iface_desc->desc.bNumEndpoints);

		for(i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
			/* get information about endpoint */
			ep_descriptor = &iface_desc->endpoint[i].desc;
			print_ep_info(ep_descriptor);
			/* check builk-out ep */
			if (usb_endpoint_is_bulk_out(ep_descriptor) &&
			    !ec->ep_bulk_out.desc) {
				rv = ec_usb_ep_bulk_out_init(ec->usb_dev,
							     &ec->ep_bulk_out,
							     ep_descriptor);
				if (rv)
					break;
				dev_info(&interface->dev,
					 "ep-bulk-out num: i = %d \n",
					 i);
			}

			/* check builk-in ep */
			if (usb_endpoint_is_bulk_in(ep_descriptor) &&
			    !ec->ep_bulk_in.desc) {
				rv = ec_usb_ep_bulk_in_init(ec->usb_dev,
							    &ec->ep_bulk_in,
							    ep_descriptor);
				if (rv)
					break;
				dev_info(&interface->dev,
					 "ep-bulk-in num: i = %d \n",
					 i);
			}

			/* check builk-in ep */
			if (usb_endpoint_is_int_in(ep_descriptor) &&
			    !ec->ep_int_in.desc) {
				rv = ec_usb_ep_interrupt_in_init(ec->usb_dev,
								 &ec->ep_int_in,
								 ep_descriptor);
				if (rv)
					break;
				dev_info(&interface->dev,
					 "ep-int-in num: i = %d \n",
					 i);
			}

		} /* endfor(i = 0; i < iface_desc->desc.bNumEndpoints; ++i) */

		if (!ec->ep_bulk_out.desc ||
		    !ec->ep_bulk_in.desc ||
		    !ec->ep_int_in.desc) {
			dev_err(&interface->dev,
				"Some USB end-point was not correctly"
				"initialized!\n");
			rv = -EIO;
			break;
		}

		/* register new usb ec device */
		usb_set_intfdata(interface, ec);
		rv = usb_register_dev(interface, &ec_class);
		if (rv) {
			/* something prevented us from registering this drv */
			dev_err(&interface->dev,
				"Not able to get a minor for this device\n");
			usb_set_intfdata(interface, NULL);
			break;
		}
		dev_info(&interface->dev,
			 "Embedded Controller now attached to /dev/mcu%d\n",
#ifdef CONFIG_USB_DYNAMIC_MINORS
			 interface->minor);
#else
			 interface->minor - mcu_class.minor_base);
#endif
		/* can over usb protocol init */
		init_completion(&ec->hold);
		mutex_init(&ec->cou.mutex);
		atomic_set(&ec->cou.a_servise_timeout, SERVICE_TIMEOUT);
		ec->cou.stat_proc = kthread_create(cou_stat_upd_process,
		                                   &ec->cou,
		                                   "cou_stat_upd_process");
		if (ec->cou.stat_proc == ERR_PTR(-ENOMEM)) {
			/* error when creating a child thread */
			EC_ERR("Error when creating"
			       "<cou_stat_upd_process> task!\n");
			rv = -ECHILD;
			break;
		}
		ec->cou.pckt_proc = kthread_create(cou_packet_rcv_process,
		                                   ec,
		                                   "bmc_cou_process");
		if (ec->cou.pckt_proc == ERR_PTR(-ENOMEM)) {
			/* error when creating a child thread */
			EC_ERR("Error when creating <cou_processing> task!\n");
			rv = -ECHILD;
			break;
		}
		cou_stat_reset(&ec->cou);
		wake_up_process(ec->cou.stat_proc);
		wake_up_process(ec->cou.pckt_proc);

		/* callbaks list */
		rwlock_init(&ec->cb_list.list_lock);
		INIT_LIST_HEAD(&ec->cb_list.hndl_list);

		/* transmit interrupt-in urb to usb stack */
		rv = usb_submit_urb(ec->ep_int_in.urb, GFP_KERNEL);
		if (rv) {
			/* error with transfer */
			rv = -EIO;
			dev_err(&interface->dev,
			        "Could not submitting URB - err = %d\n",
			        rv);
			break;
		}

		/* create device attribute files in sysfs */
		rv = sysfs_create_group(&interface->dev.kobj, &ec_sysfs_dev_attr_grp);
		if (rv) {
			dev_err(&interface->dev,
				"Failed to create the device files in"
				"the sysfs! rv = %d \n",
				rv);
			break;
		}
		return 0;
	} while (false);

	dev_err(&interface->dev, " Probe error = %d!\n", rv);
	if (!ec)
		kref_put(&ec->kref, ec_delete);
	return  rv;
}

/**
 * ec_disconnect()
 */
static void ec_disconnect(struct usb_interface *interface)
{
	struct ec_dev *ec;
	int ec_minor = interface->minor;

	dev_info(&interface->dev, "ec_disconnect!\n");
	ec = (struct ec_dev*) usb_get_intfdata(interface);

	/* remove device file from sysfs */
	sysfs_remove_group(&interface->dev.kobj, &ec_sysfs_dev_attr_grp);

	/* stop all threads */
	if (ec->cou.pckt_proc) kthread_stop(ec->cou.pckt_proc);
	if (ec->cou.stat_proc) kthread_stop(ec->cou.stat_proc);
	/* clear interface descriptor */
	usb_set_intfdata(interface, NULL);
	ec->interface = NULL;
	/* call destructor for asynchronous event queue */
	kill_fasync(&ec->async_event, SIGIO, POLL_IN);
	/* give back our minor */
	usb_deregister_dev(interface, &ec_class);
	/* decrement our usage count */
	kref_put(&ec->kref, ec_delete);
#ifndef CONFIG_USB_DYNAMIC_MINORS
	ec_minor -= ec_class.minor_base;
#endif
	dev_info(&interface->dev, "/dev/mcu%d now disconnected\n", ec_minor);
}

/* ec driver structure for current device */
static struct usb_driver ec_usb_driver = {
	.name = EC_DRIVER_NAME,
	.probe = ec_probe,
	.disconnect = ec_disconnect,
	.id_table = ec_id_table,
};

/**
 * ec_module_init()
 */
static int __init ec_module_init(void)
{
	int rv;
	EC_INFO("Start Embedded Controller module! \n");
	/* register this driver with the USB subsystem */
	rv = usb_register(&ec_usb_driver);
	if (rv)
		EC_ERR("usb_register failed. Error number %d\n", rv);
	return  rv;
}

/**
 * ec_module_exit()
 */
static void __exit ec_module_exit(void)
{
	/* deregister this driver with the USB subsystem */
	EC_INFO("Remove Embedded Controller module!\n");
	usb_deregister(&ec_usb_driver);
}

module_init(ec_module_init);
module_exit(ec_module_exit);

MODULE_AUTHOR("Maxim Polyakov <m.polyakov@yadro.com>");
MODULE_DESCRIPTION("Embedded controller driver");
MODULE_DESCRIPTION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
