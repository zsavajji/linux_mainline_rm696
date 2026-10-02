// SPI data driven vibrator, the N9's (RM-696): its amplifier takes the waveform as a serial bit
// stream on McSPI2's data line. Nokia's drivers/input/misc/vibra_spi.c (Ilkka Koskinen, 2010) on
// DT: an input force-feedback device ("SPI vibrator", Harmattan's udev links it /dev/input/vibra)
// whose custom periodic effects are raw sample buffers; Immersion's immvibed uploads and plays them.

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#define VIBRA_EFFECTS		36
#define MAX_EFFECT_SIZE		1024	/* bytes */
#define WAIT_TIMEOUT		10	/* ms */
#define POWER_UP_TIME		1000	/* us, the amplifier's (board-rm696-peripherals.c) */

#define FF_EFFECT_QUEUED	BIT(0)
#define FF_EFFECT_PLAYING	BIT(1)
#define FF_EFFECT_ABORTING	BIT(2)
#define FF_EFFECT_UPLOADING	BIT(3)

enum vibra_status {
	IDLE = 0,
	STARTED,
	PLAYING,
	CLOSING,
};

struct vibra_effect_info {
	char		*buf;
	unsigned int	buflen;
	unsigned long	flags;
	int		remaining_ms;
	unsigned int	len_ms;
};

struct vibra_data {
	struct device			*dev;
	struct input_dev		*input_dev;
	struct workqueue_struct		*workqueue;
	struct work_struct		play_work;
	struct spi_device		*spi_dev;
	struct spi_transfer		t;
	struct spi_message		msg;
	u32				spi_max_speed_hz;
	enum vibra_status		status;
	struct vibra_effect_info	effects[VIBRA_EFFECTS];
	int				next_effect;
	int				current_effect;
	wait_queue_head_t		wq;
	struct gpio_desc		*power;
};

static void vibra_set_power(struct vibra_data *vibra, bool enable)
{
	gpiod_set_value_cansleep(vibra->power, enable);
	if (enable)
		usleep_range(POWER_UP_TIME, POWER_UP_TIME + 100);
}

static int vibra_spi_raw_write_effect(struct vibra_data *vibra)
{
	spi_message_init(&vibra->msg);
	memset(&vibra->t, 0, sizeof(vibra->t));

	vibra->t.tx_buf	= vibra->effects[vibra->current_effect].buf;
	vibra->t.len	= vibra->effects[vibra->current_effect].buflen;
	spi_message_add_tail(&vibra->t, &vibra->msg);

	return spi_sync(vibra->spi_dev, &vibra->msg);
}

static void vibra_play_work(struct work_struct *work)
{
	DECLARE_WAITQUEUE(wait, current);
	struct vibra_data *vibra = container_of(work, struct vibra_data, play_work);
	struct vibra_effect_info *curr;
	int ret;

	add_wait_queue(&vibra->wq, &wait);
	while (1) {
		long val;

		if (vibra->status == CLOSING)
			goto switch_off;

		vibra->current_effect = vibra->next_effect;
		curr = &vibra->effects[vibra->current_effect];

		if (curr->flags & FF_EFFECT_ABORTING)
			goto switch_off;

		if (curr->remaining_ms > 0) {
			spin_lock_irq(&vibra->input_dev->event_lock);
			curr->flags |= FF_EFFECT_PLAYING;
			curr->remaining_ms -= curr->len_ms;
			spin_unlock_irq(&vibra->input_dev->event_lock);

			if (vibra->status == STARTED) {
				vibra_set_power(vibra, true);
				vibra->status = PLAYING;
			}

			ret = vibra_spi_raw_write_effect(vibra);
			if (ret < 0) {
				dev_err(vibra->dev, "Error replaying an effect: %d", ret);
				goto switch_off;
			}

			spin_lock_irq(&vibra->input_dev->event_lock);
			curr->flags &= ~FF_EFFECT_PLAYING;
			spin_unlock_irq(&vibra->input_dev->event_lock);
			continue;
		}

		/* Nothing to play: power off unless data comes within the timeout */
		val = wait_event_interruptible_timeout(vibra->wq,
				vibra->effects[vibra->next_effect].remaining_ms > 0,
				msecs_to_jiffies(WAIT_TIMEOUT));
		if (val > 0)
			continue;
switch_off:
		vibra_set_power(vibra, false);
		vibra->status = IDLE;
		remove_wait_queue(&vibra->wq, &wait);
		return;
	}
}

/* Called with the input device's event_lock held and interrupts off (hence _irq elsewhere) */
static int vibra_spi_playback(struct input_dev *input, int effect_id, int value)
{
	struct vibra_data *vibra = input_get_drvdata(input);
	struct vibra_effect_info *einfo = &vibra->effects[effect_id];
	struct ff_effect *ff_effect = &input->ff->effects[effect_id];

	if (!vibra->workqueue)
		return -ENODEV;

	if (einfo->flags & FF_EFFECT_UPLOADING)
		return -EBUSY;

	if (value == 0) {
		/* Abort the given effect */
		if (einfo->flags & FF_EFFECT_PLAYING)
			einfo->flags |= FF_EFFECT_ABORTING;
		einfo->flags &= ~FF_EFFECT_QUEUED;
	} else {
		/* Make the given effect the next one */
		vibra->effects[vibra->next_effect].flags &= ~FF_EFFECT_QUEUED;
		vibra->next_effect = effect_id;
		einfo->flags |= FF_EFFECT_QUEUED;
		einfo->flags &= ~FF_EFFECT_ABORTING;
		einfo->remaining_ms = ff_effect->replay.length;

		if (vibra->status == IDLE) {
			vibra->status = STARTED;
			queue_work(vibra->workqueue, &vibra->play_work);
		}
		wake_up_interruptible(&vibra->wq);
	}

	return 0;
}

static int vibra_spi_upload(struct input_dev *input, struct ff_effect *effect,
			    struct ff_effect *old)
{
	struct vibra_data *vibra = input_get_drvdata(input);
	struct vibra_effect_info *einfo = &vibra->effects[effect->id];
	struct ff_periodic_effect *p = &effect->u.periodic;
	unsigned int datalen;
	int ret = 0;

	if (effect->type != FF_PERIODIC || p->waveform != FF_CUSTOM)
		return -EINVAL;

	spin_lock_irq(&vibra->input_dev->event_lock);
	if (einfo->flags & (FF_EFFECT_QUEUED | FF_EFFECT_PLAYING | FF_EFFECT_UPLOADING)) {
		spin_unlock_irq(&vibra->input_dev->event_lock);
		return -EBUSY;
	}
	einfo->flags |= FF_EFFECT_UPLOADING;
	spin_unlock_irq(&vibra->input_dev->event_lock);

	datalen = p->custom_len * sizeof(p->custom_data[0]);
	if (datalen > MAX_EFFECT_SIZE) {
		ret = -ENOSPC;
		goto exit;
	}

	if (einfo->buf && einfo->buflen != datalen) {
		kfree(einfo->buf);
		einfo->buf = NULL;
	}
	if (!einfo->buf) {
		einfo->buf = kzalloc(datalen, GFP_KERNEL);
		if (!einfo->buf) {
			ret = -ENOMEM;
			goto exit;
		}
	}

	/* The input core copied the effect from user space, but not its custom_data */
	if (copy_from_user(einfo->buf, p->custom_data, datalen)) {
		ret = -EFAULT;
		goto exit;
	}
	einfo->buflen = datalen;

	/* The sample's length in ms, at the bus's bit rate */
	einfo->len_ms = 1000 * datalen * BITS_PER_BYTE / vibra->spi_max_speed_hz;
	if (einfo->len_ms < 1)
		ret = -EINVAL;
exit:
	einfo->flags &= ~FF_EFFECT_UPLOADING;
	return ret;
}

static int vibra_spi_open(struct input_dev *input)
{
	struct vibra_data *vibra = input_get_drvdata(input);

	vibra->workqueue = alloc_ordered_workqueue("vibra", 0);
	return vibra->workqueue ? 0 : -ENOMEM;
}

static void vibra_spi_close(struct input_dev *input)
{
	struct vibra_data *vibra = input_get_drvdata(input);

	vibra->status = CLOSING;
	cancel_work_sync(&vibra->play_work);
	destroy_workqueue(vibra->workqueue);
	vibra->workqueue = NULL;
	vibra->status = IDLE;
}

static int vibra_spi_probe(struct spi_device *spi)
{
	struct vibra_data *vibra;
	struct ff_device *ff;
	int ret;

	/* Freed by input_ff_destroy() (ff->private) when the input device goes */
	vibra = kzalloc(sizeof(*vibra), GFP_KERNEL);
	if (!vibra)
		return -ENOMEM;

	vibra->power = devm_gpiod_get(&spi->dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(vibra->power)) {
		ret = dev_err_probe(&spi->dev, PTR_ERR(vibra->power), "no power gpio\n");
		goto err_free;
	}

	vibra->spi_max_speed_hz = spi->max_speed_hz;
	INIT_WORK(&vibra->play_work, vibra_play_work);
	init_waitqueue_head(&vibra->wq);
	vibra->dev = &spi->dev;
	vibra->spi_dev = spi;
	spi_set_drvdata(spi, vibra);

	spi->bits_per_word = 32;
	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "spi_setup failed\n");
		goto err_free;
	}

	vibra->input_dev = input_allocate_device();
	if (!vibra->input_dev) {
		ret = -ENOMEM;
		goto err_free;
	}

	input_set_drvdata(vibra->input_dev, vibra);
	vibra->input_dev->name		= "SPI vibrator";
	vibra->input_dev->id.version	= 1;
	vibra->input_dev->dev.parent	= &spi->dev;
	vibra->input_dev->open		= vibra_spi_open;
	vibra->input_dev->close		= vibra_spi_close;

	set_bit(FF_PERIODIC, vibra->input_dev->ffbit);
	set_bit(FF_CUSTOM, vibra->input_dev->ffbit);

	ret = input_ff_create(vibra->input_dev, VIBRA_EFFECTS);
	if (ret) {
		input_free_device(vibra->input_dev);
		goto err_free;
	}

	ff = vibra->input_dev->ff;
	ff->private = vibra;
	ff->upload = vibra_spi_upload;
	ff->playback = vibra_spi_playback;

	ret = input_register_device(vibra->input_dev);
	if (ret)
		input_free_device(vibra->input_dev);	/* its ff teardown frees vibra */
	return ret;

err_free:
	kfree(vibra);
	return ret;
}

static void vibra_spi_remove(struct spi_device *spi)
{
	struct vibra_data *vibra = spi_get_drvdata(spi);
	int i;

	for (i = 0; i < VIBRA_EFFECTS; i++)
		kfree(vibra->effects[i].buf);
	input_unregister_device(vibra->input_dev);
}

static const struct of_device_id vibra_spi_of_match[] = {
	{ .compatible = "nokia,vibra-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, vibra_spi_of_match);

static const struct spi_device_id vibra_spi_ids[] = {
	{ "vibra-spi" },
	{ }
};
MODULE_DEVICE_TABLE(spi, vibra_spi_ids);

static struct spi_driver vibra_spi_driver = {
	.driver = {
		.name = "vibra_spi",
		.of_match_table = vibra_spi_of_match,
	},
	.id_table = vibra_spi_ids,
	.probe = vibra_spi_probe,
	.remove = vibra_spi_remove,
};
module_spi_driver(vibra_spi_driver);

MODULE_DESCRIPTION("SPI data driven vibrator (Nokia N9)");
MODULE_AUTHOR("Ilkka Koskinen <ilkka.koskinen@nokia.com>");
MODULE_LICENSE("GPL");
