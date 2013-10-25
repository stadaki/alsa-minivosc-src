/*
 *  Minimal virtual oscillator (minivosc) soundcard
 *
 *  Based on Loopback soundcard (aloop-kernel.c):
 *  Original code:
 *  Copyright (c) by Jaroslav Kysela <perex@perex.cz>
 *
 *  More accurate positioning and full-duplex support:
 *  Copyright (c) Ahmet İnan <ainan at mathematik.uni-freiburg.de>
 *
 *  Major (almost complete) rewrite:
 *  Copyright (c) by Takashi Iwai <tiwai@suse.de>
 *
 *  with snippets from Ben Collins: Writing an ALSA driver
 *  http://ben-collins.blogspot.com/2010/04/writing-alsa-driver.html
 *
 *  minivosc specific parts:
 *  Copyright (c) by Smilen Dimitrov <sd at imi.aau.dk>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

static int debug = 1;
/* Use our own dbg macro http://www.n1ywb.com/projects/darts/darts-usb/darts-usb.c*/
#undef dbg
#define dbg(format, arg...) do { if (debug) printk(KERN_DEBUG __FILE__ ": " format "\n" , ## arg); } while (0)
#define dbg2(format, arg...) do { if (debug) printk( ": " format "\n" , ## arg); } while (0)


/* Here is our user defined breakpoint to */
/* initiate communication with remote (k)gdb */
/* don't use if not actually using kgdb */
#define BREAKPOINT() asm("   int $3");


// copy from aloop-kernel.c:
#include <linux/init.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <linux/version.h>

MODULE_AUTHOR("sdaau");
MODULE_DESCRIPTION("minivosc soundcard");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{ALSA,minivosc soundcard}}");

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;	/* Index 0-MAX */
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;	/* ID for this card */
static int enable[SNDRV_CARDS] = {1, [1 ... (SNDRV_CARDS - 1)] = 0};

static struct platform_device *devices[SNDRV_CARDS];

#define byte_pos(x)	((x) / HZ)
#define frac_pos(x)	((x) * HZ)

#define MAX_BUFFER (32 * 48)
static struct snd_pcm_hardware minivosc_pcm_hw =
{
	.info = (SNDRV_PCM_INFO_MMAP |
	SNDRV_PCM_INFO_INTERLEAVED |
	SNDRV_PCM_INFO_BLOCK_TRANSFER |
	SNDRV_PCM_INFO_MMAP_VALID),
	.formats          = SNDRV_PCM_FMTBIT_U8,
	.rates            = SNDRV_PCM_RATE_8000,
	.rate_min         = 8000,
	.rate_max         = 8000,
	.channels_min     = 1,
	.channels_max     = 1,
	.buffer_bytes_max = MAX_BUFFER, //(32 * 48) = 1536,
	.period_bytes_min = 48,
	.period_bytes_max = 48,
	.periods_min      = 1,
	.periods_max      = 32,
};


struct minivosc_device
{
	struct snd_card *card;
	struct snd_pcm *pcm;
	const struct minivosc_pcm_ops *timer_ops;
	/*
	* we have only one substream, so all data in this struct
	*/
	/* copied from struct loopback: */
	struct mutex cable_lock;
	/* copied from struct loopback_cable: */
	/* PCM parameters */
	unsigned int pcm_period_size;
	unsigned int pcm_bps;		/* bytes per second */
	/* flags */
	unsigned int valid;
	unsigned int running;
	unsigned int period_update_pending :1;
	/* timer stuff */
	unsigned int irq_pos;		/* fractional IRQ position */
	unsigned int period_size_frac;
	unsigned long last_jiffies;
	struct timer_list timer;
	/* copied from struct loopback_pcm: */
	struct snd_pcm_substream *substream;
	unsigned int pcm_buffer_size;
	unsigned int buf_pos;	/* position in buffer */
	unsigned int silent_size;
	/* added for waveform: */
	unsigned int wvf_pos;	/* position in waveform array */
	unsigned int wvf_lift;	/* lift of waveform array */
};

// waveform
#if defined(COPYALG_V1) || defined(COPYALG_V2) || defined(COPYALG_V3)
static char wvfdat[]={	20, 22, 24, 25, 24, 22, 21,
			19, 17, 15, 14, 15, 17, 19,
			20, 127, 22, 19, 17, 15, 19};
#endif
#if defined(COPYALG_V1) || defined(COPYALG_V2)
static char wvfdat2[]={	20, 22, 24, 25, 24, 22, 21,
			19, 17, 15, 14, 15, 17, 19,
			20, 127, 22, 19, 17, 15, 19};
#endif
#if defined(COPYALG_V1) || defined(COPYALG_V2) || defined(COPYALG_V3)
static unsigned int wvfsz=sizeof(wvfdat);//*sizeof(float) is included already
#endif
// * functions for driver/kernel module initialization
static void minivosc_unregister_all(void);
static int __init alsa_card_minivosc_init(void);
static void __exit alsa_card_minivosc_exit(void);

// * declare functions for this struct describing the driver (to be defined later):
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
static int __devinit minivosc_probe(struct platform_device *devptr);
static int __devexit minivosc_remove(struct platform_device *devptr);
#else
static int minivosc_probe(struct platform_device *devptr);
static int minivosc_remove(struct platform_device *devptr);
#endif


// * here declaration of functions that will need to be in _ops, before they are defined
static int minivosc_hw_params(struct snd_pcm_substream *ss,
                        struct snd_pcm_hw_params *hw_params);
static int minivosc_hw_free(struct snd_pcm_substream *ss);
static int minivosc_pcm_open(struct snd_pcm_substream *ss);
static int minivosc_pcm_close(struct snd_pcm_substream *ss);
static int minivosc_pcm_prepare(struct snd_pcm_substream *ss);
static int minivosc_pcm_trigger(struct snd_pcm_substream *ss,
                          int cmd);
static snd_pcm_uframes_t minivosc_pcm_pointer(struct snd_pcm_substream *ss);

static int minivosc_pcm_dev_free(struct snd_device *device);
static int minivosc_pcm_free(struct minivosc_device *chip);

// * declare timer functions - copied from aloop-kernel.c
static void minivosc_timer_start(struct minivosc_device *mydev);
static void minivosc_timer_stop(struct minivosc_device *mydev);
static void minivosc_pos_update(struct minivosc_device *mydev);
static void minivosc_timer_function(unsigned long data);
static void minivosc_xfer_buf(struct minivosc_device *mydev, unsigned int count);
static void minivosc_fill_capture_buf(struct minivosc_device *mydev, unsigned int bytes);


// note snd_pcm_ops can usually be separate _playback_ops and _capture_ops
static struct snd_pcm_ops minivosc_pcm_ops =
{
	.open      = minivosc_pcm_open,
	.close     = minivosc_pcm_close,
	.ioctl     = snd_pcm_lib_ioctl,
	.hw_params = minivosc_hw_params,
	.hw_free   = minivosc_hw_free,
	.prepare   = minivosc_pcm_prepare,
	.trigger   = minivosc_pcm_trigger,
	.pointer   = minivosc_pcm_pointer,
};

// specifies what func is called @ snd_card_free
// used in snd_device_new
static struct snd_device_ops dev_ops =
{
	.dev_free = minivosc_pcm_dev_free,
};


#define SND_MINIVOSC_DRIVER	"snd_minivosc"

// * we need a struct describing the driver:
static struct platform_driver minivosc_driver =
{
	.probe		= minivosc_probe,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
	.remove		= __devexit_p(minivosc_remove),
#else
	.remove		= minivosc_remove,
#endif
//~ #ifdef CONFIG_PM
	//~ .suspend	= minivosc_suspend,
	//~ .resume	= minivosc_resume,
//~ #endif
	.driver		= {
		.name	= SND_MINIVOSC_DRIVER,
		.owner = THIS_MODULE
	},
};


/*
 *
 * Probe/remove functions
 *
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
static int __devinit minivosc_probe(struct platform_device *devptr)
#else
static int minivosc_probe(struct platform_device *devptr)
#endif
{

	struct snd_card *card;
	struct minivosc_device *mydev;
	int ret;

	int nr_subdevs; // how many capture substreams we want
	struct snd_pcm *pcm;

	int dev = devptr->id; // from aloop-kernel.c

	dbg("%s: probe", __func__);


	// no need to kzalloc minivosc_device separately, if the sizeof is included here
	ret = snd_card_create(index[dev], id[dev],
	                      THIS_MODULE, sizeof(struct minivosc_device), &card);

	if (ret < 0)
		goto __nodev;

	mydev = card->private_data;
	mydev->card = card;
	// MUST have mutex_init here - else crash on mutex_lock!!
	mutex_init(&mydev->cable_lock);

	dbg2("-- mydev %p", mydev);

	sprintf(card->driver, "my_driver-%s", SND_MINIVOSC_DRIVER);
	sprintf(card->shortname, "MySoundCard Audio %s", SND_MINIVOSC_DRIVER);
	sprintf(card->longname, "%s", card->shortname);


	snd_card_set_dev(card, &devptr->dev); // present in dummy, not in aloop though


	ret = snd_device_new(card, SNDRV_DEV_LOWLEVEL, mydev, &dev_ops);

	if (ret < 0)
		goto __nodev;


	nr_subdevs = 1; // how many capture substreams we want
	// * we want 0 playback, and 1 capture substreams (4th and 5th arg) ..
	ret = snd_pcm_new(card, card->driver, 0, 0, nr_subdevs, &pcm);

	if (ret < 0)
		goto __nodev;


	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &minivosc_pcm_ops); // in both aloop-kernel.c and dummy.c, after snd_pcm_new...
	pcm->private_data = mydev; //here it should be dev/card struct (the one containing struct snd_card *card) - this DOES NOT end up in substream->private_data

	pcm->info_flags = 0;
	strcpy(pcm->name, card->shortname);

	/*
	trid to add this - but it crashes here:
	//mydev->substream->private_data = mydev;
	Well, first time real substream comes in, is in _open - so
	that has to be handled there.. That is: at this point, mydev->substream is null,
	and we first have a chance to set it ... in _open!
	*/

	ret = snd_pcm_lib_preallocate_pages_for_all(pcm,
	        SNDRV_DMA_TYPE_CONTINUOUS,
	        snd_dma_continuous_data(GFP_KERNEL),
	        MAX_BUFFER, MAX_BUFFER); // in both aloop-kernel.c and dummy.c, after snd_pcm_set_ops...

	if (ret < 0)
		goto __nodev;

	// * will use the snd_card_register form from aloop-kernel.c/dummy.c here..
	ret = snd_card_register(card);

	if (ret == 0)   // or... (!ret)
	{
		platform_set_drvdata(devptr, card);
		return 0; // success
	}

__nodev: // as in aloop/dummy...
	dbg("__nodev reached!!");
	snd_card_free(card); // this will autocall .dev_free (= minivosc_pcm_dev_free)
	return ret;
}

// from dummy/aloop:
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
static int __devexit minivosc_remove(struct platform_device *devptr)
#else
static int minivosc_remove(struct platform_device *devptr)
#endif
{
	dbg("%s", __func__);
	snd_card_free(platform_get_drvdata(devptr));
	platform_set_drvdata(devptr, NULL);
	return 0;
}


/*
 *
 * hw alloc/free functions
 *
 */
static int minivosc_hw_params(struct snd_pcm_substream *ss,
                        struct snd_pcm_hw_params *hw_params)
{
	dbg("%s", __func__);
	return snd_pcm_lib_malloc_pages(ss,
	                                params_buffer_bytes(hw_params));
}

static int minivosc_hw_free(struct snd_pcm_substream *ss)
{
	dbg("%s", __func__);
	return snd_pcm_lib_free_pages(ss);
}


/*
 *
 * PCM functions
 *
 */
static int minivosc_pcm_open(struct snd_pcm_substream *ss)
{
	struct minivosc_device *mydev = ss->private_data;

	//BREAKPOINT();
	dbg("%s", __func__);

	// copied from aloop-kernel.c:
	mutex_lock(&mydev->cable_lock);

	ss->runtime->hw = minivosc_pcm_hw;

	mydev->substream = ss; 	//save (system given) substream *ss, in our structure field
	ss->runtime->private_data = mydev;
	mydev->wvf_pos = 0; 	//init
	mydev->wvf_lift = 0; 	//init

	// SETUP THE TIMER HERE:
	setup_timer(&mydev->timer, minivosc_timer_function,
	            (unsigned long)mydev);

	mutex_unlock(&mydev->cable_lock);
	return 0;
}

static int minivosc_pcm_close(struct snd_pcm_substream *ss)
{
	struct minivosc_device *mydev = ss->private_data;

	dbg("%s", __func__);

	// copied from aloop-kernel.c:
	// * even though mutexes are retrieved from ss->private_data,
	// * which will be set to null,
	// * lock the mutex here anyway:
	mutex_lock(&mydev->cable_lock);
	// * not much else to do here, but set to null:
	ss->private_data = NULL;
	mutex_unlock(&mydev->cable_lock);

	return 0;
}


static int minivosc_pcm_prepare(struct snd_pcm_substream *ss)
{
	// copied from aloop-kernel.c

	// for one, we could get mydev from ss->private_data...
	// here we try it via ss->runtime->private_data instead.
	// turns out, this type of call via runtime->private_data
	// ends up with mydev as null pointer causing SIGSEGV
	// .. UNLESS runtime->private_data is assigned in _open?
	struct snd_pcm_runtime *runtime = ss->runtime;
	struct minivosc_device *mydev = runtime->private_data;
	unsigned int bps;

	dbg("%s", __func__);

	bps = runtime->rate * runtime->channels; // params requested by user app (arecord, audacity)
	bps *= snd_pcm_format_width(runtime->format);
	bps /= 8;
	if (bps <= 0)
		return -EINVAL;

	mydev->buf_pos = 0;
	mydev->pcm_buffer_size = frames_to_bytes(runtime, runtime->buffer_size);
	dbg2("	bps: %u; runtime->buffer_size: %lu; mydev->pcm_buffer_size: %u", bps, runtime->buffer_size, mydev->pcm_buffer_size);
	if (ss->stream == SNDRV_PCM_STREAM_CAPTURE) {
		/* clear capture buffer */
		mydev->silent_size = mydev->pcm_buffer_size;
		//memset(runtime->dma_area, 0, mydev->pcm_buffer_size);
		// we're in char land here, so let's mark prepare buffer with value 45 (signature)
		// this turns out to set everything permanently throughout - not just first buffer,
		// even though it runs only at start?
		memset(runtime->dma_area, 45, mydev->pcm_buffer_size);
	}

	if (!mydev->running) {
		mydev->irq_pos = 0;
		mydev->period_update_pending = 0;
	}


	mutex_lock(&mydev->cable_lock);
	if (!(mydev->valid & ~(1 << ss->stream))) {
		mydev->pcm_bps = bps;
		mydev->pcm_period_size =
			frames_to_bytes(runtime, runtime->period_size);
		mydev->period_size_frac = frac_pos(mydev->pcm_period_size);

	}
	mydev->valid |= 1 << ss->stream;
	mutex_unlock(&mydev->cable_lock);

	dbg2("	pcm_period_size: %u; period_size_frac: %u", mydev->pcm_period_size, mydev->period_size_frac);

	return 0;
}


static int minivosc_pcm_trigger(struct snd_pcm_substream *ss,
                          int cmd)
{
	int ret = 0;
	//copied from aloop-kernel.c

	//here we do not get mydev from
	// ss->runtime->private_data; but from:
	struct minivosc_device *mydev = ss->private_data;

	dbg("%s - trig %d", __func__, cmd);

	switch (cmd)
	{
		case SNDRV_PCM_TRIGGER_START:
			// Start the hardware capture
			// from aloop-kernel.c:
			if (!mydev->running) {
				mydev->last_jiffies = jiffies;
				// SET OFF THE TIMER HERE:
				minivosc_timer_start(mydev);
			}
			mydev->running |= (1 << ss->stream);
			break;
		case SNDRV_PCM_TRIGGER_STOP:
			// Stop the hardware capture
			// from aloop-kernel.c:
			mydev->running &= ~(1 << ss->stream);
			if (!mydev->running)
				// STOP THE TIMER HERE:
				minivosc_timer_stop(mydev);
			break;
		default:
			ret = -EINVAL;
	}

	return ret;
}


static snd_pcm_uframes_t minivosc_pcm_pointer(struct snd_pcm_substream *ss)
{
	//copied from aloop-kernel.c
	struct snd_pcm_runtime *runtime = ss->runtime;
	struct minivosc_device *mydev= runtime->private_data;

	dbg2("+minivosc_pointer ");
	minivosc_pos_update(mydev);
	dbg2("+	bytes_to_frames(: %lu, mydev->buf_pos: %d", bytes_to_frames(runtime, mydev->buf_pos),mydev->buf_pos);
	return bytes_to_frames(runtime, mydev->buf_pos);

}


/*
 *
 * Timer functions
 *
 */
static void minivosc_timer_start(struct minivosc_device *mydev)
{
	unsigned long tick;
	dbg2("minivosc_timer_start: mydev->period_size_frac: %u; mydev->irq_pos: %u jiffies: %lu pcm_bps %u", mydev->period_size_frac, mydev->irq_pos, jiffies, mydev->pcm_bps);
	tick = mydev->period_size_frac - mydev->irq_pos;
	tick = (tick + mydev->pcm_bps - 1) / mydev->pcm_bps;
	mydev->timer.expires = jiffies + tick;
	add_timer(&mydev->timer);
}

static void minivosc_timer_stop(struct minivosc_device *mydev)
{
	dbg2("minivosc_timer_stop");
	del_timer(&mydev->timer);
}

static void minivosc_pos_update(struct minivosc_device *mydev)
{
	unsigned int last_pos, count;
	unsigned long delta;

	if (!mydev->running)
		return;

	dbg2("*minivosc_pos_update: running ");

	delta = jiffies - mydev->last_jiffies;
	dbg2("*	: jiffies %lu, ->last_jiffies %lu, delta %lu", jiffies, mydev->last_jiffies, delta);

	if (!delta)
		return;

	mydev->last_jiffies += delta;

	last_pos = byte_pos(mydev->irq_pos);
	mydev->irq_pos += delta * mydev->pcm_bps;
	count = byte_pos(mydev->irq_pos) - last_pos;
	dbg2("*	: last_pos %d, c->irq_pos %d, count %d", last_pos, mydev->irq_pos, count);

	if (!count)
		return;

	// FILL BUFFER HERE
	minivosc_xfer_buf(mydev, count);

	if (mydev->irq_pos >= mydev->period_size_frac)
	{
		dbg2("*	: mydev->irq_pos >= mydev->period_size_frac %d", mydev->period_size_frac);
		mydev->irq_pos %= mydev->period_size_frac;
		mydev->period_update_pending = 1;
	}
}

static void minivosc_timer_function(unsigned long data)
{
	struct minivosc_device *mydev = (struct minivosc_device *)data;

	if (!mydev->running)
		return;

	dbg2("minivosc_timer_function: running ");
	minivosc_pos_update(mydev);
	// SET OFF THE TIMER HERE:
	minivosc_timer_start(mydev);

	if (mydev->period_update_pending)
	{
		mydev->period_update_pending = 0;

		if (mydev->running)
		{
			dbg2("	: calling snd_pcm_period_elapsed");
			snd_pcm_period_elapsed(mydev->substream);
		}
	}
}

#define CABLE_PLAYBACK	(1 << SNDRV_PCM_STREAM_PLAYBACK)
#define CABLE_CAPTURE	(1 << SNDRV_PCM_STREAM_CAPTURE)
#define CABLE_BOTH	(CABLE_PLAYBACK | CABLE_CAPTURE)

// choose which  copy (fill) algorithm to use -
// (un)comment as needed, though use only one at a time!
//~ #define COPYALG_V1
//~ #define COPYALG_V2
//~ #define COPYALG_V3
#define BUFFERMARKS // do we want 'buffer mark' samples or not

static void minivosc_xfer_buf(struct minivosc_device *mydev, unsigned int count)
{

	dbg2(">minivosc_xfer_buf: count: %d ", count );

	switch (mydev->running) {
	case CABLE_CAPTURE:
		minivosc_fill_capture_buf(mydev, count);
		break;
	}

		if (mydev->running) {
// activate this buf_pos calculation, either if V3 is defined,
//  or if no COPYALG is defined (which also handles lone BUFFERMARKS)
#if defined(COPYALG_V3) || !(defined(COPYALG_V1) || defined(COPYALG_V2) || defined(COPYALG_V3))
			// here the (auto)increase of buf_pos is handled
			mydev->buf_pos += count;
			mydev->buf_pos %= mydev->pcm_buffer_size;
			dbg2(">	: mydev->buf_pos: %d ", mydev->buf_pos); // */
#endif
		}
}

static void minivosc_fill_capture_buf(struct minivosc_device *mydev, unsigned int bytes)
{
	char *dst = mydev->substream->runtime->dma_area;
	unsigned int dst_off = mydev->buf_pos; // buf_pos is in bytes, not in samples !
	float wrdat; // was char - value to fill silent_size with
	unsigned int dpos = 0; //added
#if defined(COPYALG_V1) || defined(COPYALG_V2)
	int i = 0; //added
	int mylift = 0; //added
#endif
#if defined(COPYALG_V1)
	unsigned int remain = 0; //added
	unsigned int remain2 = 0; //added
	unsigned int wvftocopy = 0; //added
#endif
#if defined(COPYALG_V2)
	int j = 0; //added
#endif
#if defined(COPYALG_V3)
#endif


	dbg2("_ minivosc_fill_capture_buf ss %d bs %d bytes %d buf_pos %d sizeof %ld jiffies %lu", mydev->silent_size, mydev->pcm_buffer_size, bytes, dst_off, sizeof(*dst), jiffies);

#if defined(COPYALG_V1)
	// loop v1.. fill waveform until end of 'bytes'..
	// using memcpy for copying/filling
	//*
	while (dpos < bytes-1)
	{
		mylift = mydev->wvf_lift*10 - 10;
		// create modified - 'lifted' - values of waveform:
		for (i=0; i<wvfsz; i++) {
			wvfdat[i] = wvfdat2[i]+mylift;
		}

		remain = bytes - dpos;
		remain2 = mydev->pcm_buffer_size - mydev->buf_pos;
		if (remain < wvfsz) wvftocopy = remain; //not wvfsz - remain!
		if (remain2 < wvfsz) wvftocopy = remain2; //also see if "big" PCM buffer wraps!
		else wvftocopy = wvfsz;
		if (mydev->wvf_pos > 0) wvftocopy -= mydev->wvf_pos;

		dbg2("::: buf_pos %d; dpos %d; wvf_pos %d; wvftocopy %d; remain %d; remain2 %d; wvfsz %d; wvf_lift %d", mydev->buf_pos, dpos, mydev->wvf_pos, wvftocopy, remain, remain2, wvfsz, mydev->wvf_lift);

		memcpy(dst + mydev->buf_pos, &wvfdat[mydev->wvf_pos], wvftocopy);

		dpos += wvftocopy;
		mydev->buf_pos += wvftocopy; //added if there isn't (auto)increase of buf_pos in xfer_buf
		mydev->wvf_pos += wvftocopy;
		if (mydev->wvf_pos >= wvfsz) { // we should wrap waveform here..
			mydev->wvf_pos -= wvfsz;
			// also handle lift here..
			mydev->wvf_lift++;
			if (mydev->wvf_lift >=4) mydev->wvf_lift = 0;
		}
		//added if there isn't (auto)increase of buf_pos in xfer_buf
		// there may be some misalignments here still, though...
		if (mydev->buf_pos >= mydev->pcm_buffer_size) {
			mydev->buf_pos = 0;
			break; //we don;t really need this.. but maybe here...?
		}
		if (dpos >= bytes-1) break;
	} // end loop v1 */
#endif //defined(COPYALG_V1)

#if defined(COPYALG_V2)
	// hmm... for this loop, v2,  I was getting prepare signature (45), if
	//   mydev->buf_pos autoincrements (wraps) in minivosc_xfer_buf ;
	// however, for more correct, we calculate 'buf_pos' here instead..
	// using direct assignment of elements for copying/filling
	//*
	for (j=0; j<bytes; j++) {
		mylift = mydev->wvf_lift*10 - 10;
		for (i=0; i<sizeof(wvfdat); i++) {
			wvfdat[i] = wvfdat2[i]+mylift;
		}

		dst[mydev->buf_pos] = wvfdat[mydev->wvf_pos];
		dpos++; mydev->buf_pos++;
		mydev->wvf_pos++;

		if (mydev->wvf_pos >= wvfsz) { // we should wrap waveform here..
			mydev->wvf_pos = 0;
			// also handle lift here..
			mydev->wvf_lift++;
			if (mydev->wvf_lift >=4) mydev->wvf_lift = 0;
		}
		if (mydev->buf_pos >= mydev->pcm_buffer_size) {
			mydev->buf_pos = 0;
			//break; //we don;t really need this
		}
		if (dpos >= bytes) break;
	} // end loop v2 */
#endif //defined(COPYALG_V2)

#if defined(COPYALG_V3)
	// as in copy_play_buf in aloop-kernel.c, where we had:
	//~ char *src = play->substream->runtime->dma_area;
	//~ char *dst = capt->substream->runtime->dma_area;
	// 'buf_pos' here is calculated in _xfer_buf, and
	//   the waveform wrapping is not correct
	// using memcpy for copying/filling

	for (;;) {
		unsigned int size = bytes;
		if (mydev->wvf_pos + size > wvfsz)
			size = wvfsz - mydev->wvf_pos;
		if (dst_off + size > mydev->pcm_buffer_size)
			size = mydev->pcm_buffer_size - dst_off;

		memcpy(dst + dst_off, wvfdat + mydev->wvf_pos, size);

		if (size < mydev->silent_size)
			mydev->silent_size -= size;
		else
			mydev->silent_size = 0;
		bytes -= size;
		if (!bytes)
			break;
		mydev->wvf_pos = (mydev->wvf_pos + size) % wvfsz;
		dst_off = (dst_off + size) % mydev->pcm_buffer_size;
	}
#endif //defined(COPYALG_V3)

#if defined(BUFFERMARKS)
	//* //set buffer marks
	//-------------
	//these two shouldn't change in repeated calls of this func:
	memset(dst+1, 160, 1); // mark start of pcm buffer
	memset(dst + mydev->pcm_buffer_size - 2, 140, 1); // mark end of pcm buffer

	memset(dst + dst_off, 120, 1); // mark start of this fill_capture_buf.
	if (dst_off==0) memset(dst + dst_off, 250, 1); // different mark if offset is zero
	// note - if marking end at dst + dst_off + bytes, it gets overwritten by next run
	memset(dst + dst_off + bytes - 2, 90, 1); // mark end fill_capture_buf.
	// end set buffer marks */
#endif //defined(BUFFERMARKS)

	if (mydev->silent_size >= mydev->pcm_buffer_size)
		return;

	// NOTE: usually, the code returns by now -
	// - it doesn't even execute past this point!
	// from here on, apparently silent_size should be handled..

	if (mydev->silent_size + bytes > mydev->pcm_buffer_size)
		bytes = mydev->pcm_buffer_size - mydev->silent_size;

	wrdat = -0.2; // value to copy, instead of 0 for silence (if needed)

	for (;;) {
		unsigned int size = bytes;
		dpos = 0; //added
		dbg2("_ clearrr..	%d", bytes);
		if (dst_off + size > mydev->pcm_buffer_size)
			size = mydev->pcm_buffer_size - dst_off;

		//memset(dst + dst_off, 255, size); //0, size);
		while (dpos < size)
		{
			memcpy(dst + dst_off + dpos, &wrdat, sizeof(wrdat));
			dpos += sizeof(wrdat);
			if (dpos >= size) break;
		}
		mydev->silent_size += size;
		bytes -= size;
		if (!bytes)
			break;
		dst_off = 0;
	}
}





/*
 *
 * snd_device_ops free functions
 *
 */
// these should eventually get called by snd_card_free (via .dev_free)
// however, since we do no special allocations, we need not free anything
static int minivosc_pcm_free(struct minivosc_device *chip)
{
	dbg("%s", __func__);
	return 0;
}

static int minivosc_pcm_dev_free(struct snd_device *device)
{
	dbg("%s", __func__);
	return minivosc_pcm_free(device->device_data);
}



/*
 *
 * functions for driver/kernel module initialization
 * (_init, _exit)
 * copied from aloop-kernel.c (same in dummy.c)
 *
 */
static void minivosc_unregister_all(void)
{
	int i;

	dbg("%s", __func__);

	for (i = 0; i < ARRAY_SIZE(devices); ++i)
		platform_device_unregister(devices[i]);

	platform_driver_unregister(&minivosc_driver);
}

static int __init alsa_card_minivosc_init(void)
{
	int i, err, cards;

	dbg("%s", __func__);
	err = platform_driver_register(&minivosc_driver);

	if (err < 0)
		return err;


	cards = 0;

	for (i = 0; i < SNDRV_CARDS; i++)
	{
		struct platform_device *device;

		if (!enable[i])
			continue;

		device = platform_device_register_simple(SND_MINIVOSC_DRIVER,
		         i, NULL, 0);

		if (IS_ERR(device))
			continue;

		if (!platform_get_drvdata(device))
		{
			platform_device_unregister(device);
			continue;
		}

		devices[i] = device;
		cards++;
	}

	if (!cards)
	{
#ifdef MODULE
		printk(KERN_ERR "minivosc-alsa: No enabled, not found or device busy\n");
#endif
		minivosc_unregister_all();
		return -ENODEV;
	}

	return 0;
}

static void __exit alsa_card_minivosc_exit(void)
{
	dbg("%s", __func__);
	minivosc_unregister_all();
}

module_init(alsa_card_minivosc_init)
module_exit(alsa_card_minivosc_exit)
