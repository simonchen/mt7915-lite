// SPDX-License-Identifier: ISC
/*
 * Copyright (C) 2019 Lorenzo Bianconi <lorenzo.bianconi83@gmail.com>
 */

#include <linux/wait.h>
#include <linux/rcupdate.h>
#include "mt76.h"
#include "mt76_connac_mcu.h"

/*
#define mt76_busy_wait_event_timeout(wq_head, condition, timeout)	\
do {									\
	unsigned long __last_qs = jiffies;				\
	unsigned long __end = __last_qs + (timeout);			\
	for (;;) {							\
		rmb();							\
		if (condition)						\
			break;						\
		if (time_after_eq(jiffies, __end))			\
			break;						\
		cond_resched();						\
		msleep(1); cpu_relax();				\
		rcu_softirq_qs_periodic(__last_qs);			\
	}								\
} while (0)
*/

#define ___mt76_wait_event(wq_head, condition, state, exclusive, ret, cmd)      \
({                                                                              \
        __label__ __out;                                                        \
        struct wait_queue_entry __wq_entry;                                     \
        long __ret = ret;       /* explicit shadow */                           \
	unsigned long __end = jiffies + (ret);					\
	unsigned int wq_flags = (exclusive || (cmd == MCU_EXT_CMD(GET_MIB_INFO))) ? WQ_FLAG_EXCLUSIVE : 0; \
										\
        init_wait_entry(&__wq_entry, wq_flags);					\
        for (;;) {                                                              \
                long __int = prepare_to_wait_event(&wq_head, &__wq_entry, state);\
                                                                                \
                if (condition)                                                  \
                        break;                                                  \
		if (time_after_eq(jiffies, __end))				\
			break;							\
                                                                                \
                if (___wait_is_interruptible(state) && __int) {                 \
                        __ret = __int;                                          \
                        goto __out;                                             \
                }                                                               \
                                                                                \
                cmd;                                                            \
        }                                                                       \
        finish_wait(&wq_head, &__wq_entry);                                     \
__out:  __ret;                                                                  \
})

#define __mt76_wait_event_timeout(wq_head, condition, timeout, exclusive)	\
	___mt76_wait_event(wq_head, ___wait_cond_timeout(condition),		\
		      TASK_UNINTERRUPTIBLE, exclusive, timeout,			\
		      cond_resched())

#define mt76_wait_event_timeout(wq_head, condition, timeout, exclusive)         \
({                                                                              \
        long __ret = timeout;                                                   \
        might_sleep();                                                          \
        if (!___wait_cond_timeout(condition))                                   \
                __ret = __mt76_wait_event_timeout(wq_head, condition, timeout, exclusive); \
        __ret;                                                                  \
})

struct sk_buff *
__mt76_mcu_msg_alloc(struct mt76_dev *dev, const void *data,
		     int len, int data_len, gfp_t gfp)
{
	const struct mt76_mcu_ops *ops = dev->mcu_ops;
	struct sk_buff *skb;

	len = max_t(int, len, data_len);
	len = ops->headroom + len + ops->tailroom;

	skb = alloc_skb(len, gfp);
	if (!skb)
		return NULL;

	memset(skb->head, 0, len);
	skb_reserve(skb, ops->headroom);

	if (data && data_len)
		skb_put_data(skb, data, data_len);

	return skb;
}
EXPORT_SYMBOL_GPL(__mt76_mcu_msg_alloc);

struct sk_buff *mt76_mcu_get_response(struct mt76_dev *dev,
				      unsigned long expires, int cmd)
{
	unsigned long timeout;
	int is_ext_cmd = ((cmd & __MCU_CMD_FIELD_ID) == MCU_CMD_EXT_CID);

	if (!time_is_after_jiffies(expires))
		return NULL;

	timeout = expires - jiffies;
	mt76_wait_event_timeout(dev->mcu.wait,
			   (!skb_queue_empty(&dev->mcu.res_q) ||
			    test_bit(MT76_MCU_RESET, &dev->phy.state)),
			   timeout, is_ext_cmd);
	return skb_dequeue(&dev->mcu.res_q);
}
EXPORT_SYMBOL_GPL(mt76_mcu_get_response);

bool mt76_get_test_mcu_restart(void)
{
	return rx_poll_retry_enable;
}
EXPORT_SYMBOL_GPL(mt76_get_test_mcu_restart);

void mt76_set_test_mcu_restart(bool ok)
{
	rx_poll_retry_enable = ok;
}
EXPORT_SYMBOL_GPL(mt76_set_test_mcu_restart);

void mt76_mcu_rx_event(struct mt76_dev *dev, struct sk_buff *skb)
{
	skb_queue_tail(&dev->mcu.res_q, skb);
	wake_up(&dev->mcu.wait);
}
EXPORT_SYMBOL_GPL(mt76_mcu_rx_event);

int mt76_mcu_send_and_get_msg(struct mt76_dev *dev, int cmd, const void *data,
			      int len, bool wait_resp, struct sk_buff **ret_skb)
{
	struct sk_buff *skb;

	if (dev->mcu_ops->mcu_send_msg)
		return dev->mcu_ops->mcu_send_msg(dev, cmd, data, len, wait_resp);

	skb = mt76_mcu_msg_alloc(dev, data, len);
	if (!skb)
		return -ENOMEM;

	return mt76_mcu_skb_send_and_get_msg(dev, skb, cmd, wait_resp, ret_skb);
}
EXPORT_SYMBOL_GPL(mt76_mcu_send_and_get_msg);

int mt76_mcu_skb_send_and_get_msg(struct mt76_dev *dev, struct sk_buff *skb,
				  int cmd, bool wait_resp,
				  struct sk_buff **ret_skb)
{
	unsigned long expires;
	int ret, seq;

	if (ret_skb)
		*ret_skb = NULL;

	mutex_lock(&dev->mcu.mutex);

	//if (MCU_EXT_CMD_GET_MIB_INFO == (FIELD_GET(__MCU_CMD_FIELD_EXT_ID, cmd)) && 
	if (dev->mcu.mib_access_time > 0 && 
			time_after(dev->mcu.mib_access_time + HZ/10, jiffies)) {
		ret = -EBUSY; 
		dev_kfree_skb(skb);
		goto out;
	}

	ret = dev->mcu_ops->mcu_skb_send_msg(dev, skb, cmd, &seq);
	if (ret < 0) {
		dev_kfree_skb(skb);
		goto out;
	}

	if (!wait_resp) {
		ret = 0;
		goto out;
	}

	if (dev->mcu_ops->mcu_set_timeout)
		dev->mcu_ops->mcu_set_timeout(dev, cmd);

	expires = jiffies + dev->mcu.timeout;

	do {
		skb = mt76_mcu_get_response(dev, expires, cmd);
		ret = dev->mcu_ops->mcu_parse_response(dev, cmd, skb, seq);
		if (!ret && ret_skb)
			*ret_skb = skb;
		else
			dev_kfree_skb(skb);
	} while (ret == -EAGAIN && cmd != MCU_EXT_CMD(GET_MIB_INFO));

out:
//	if (MCU_EXT_CMD_GET_MIB_INFO == (FIELD_GET(__MCU_CMD_FIELD_EXT_ID, cmd)))
		dev->mcu.mib_access_time = jiffies; // save the last MIB access time
	mutex_unlock(&dev->mcu.mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(mt76_mcu_skb_send_and_get_msg);

int __mt76_mcu_send_firmware(struct mt76_dev *dev, int cmd, const void *data,
			     int len, int max_len)
{
	int err, cur_len;

	while (len > 0) {
		cur_len = min_t(int, max_len, len);

		err = mt76_mcu_send_msg(dev, cmd, data, cur_len, false);
		if (err)
			return err;

		data += cur_len;
		len -= cur_len;

		if (dev->queue_ops->tx_cleanup)
			dev->queue_ops->tx_cleanup(dev,
						   dev->q_mcu[MT_MCUQ_FWDL],
						   false);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(__mt76_mcu_send_firmware);
