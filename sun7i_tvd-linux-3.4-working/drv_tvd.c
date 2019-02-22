#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/freezer.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <media/v4l2-mediabus.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
//#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>
#include <linux/moduleparam.h>
//#include <mach/sys_config.h>
//#include <mach/clock.h>
//#include <mach/irqs.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>


#include "bsp_tvd.h"
#include "drv_tvd.h"

#define DBG_EN 1
#define ERR_EN 1
#define INF_EN 1
#define DIRTY_HACK_RETURN_DMA_ADDR 1
#if(DBG_EN == 0)
	#define __dbg(x, arg...) printk(KERN_DEBUG "[TVD_DBG]"x, ##arg)
#else
	#define __dbg(x, arg...) 
#endif
#if(ERR_EN == 1)
	#define __err(x, arg...) printk(KERN_ERR "[TVD_ERR]"x, ##arg)
#else
	#define __err(x, arg...)
#endif
#if(INF_EN == 1)
	#define __inf(x, arg...) printk(KERN_INFO "[TVD_INF]"x, ##arg)
#else
	#define __inf(x, arg...)
#endif


#define NUM_INPUTS  ARRAY_SIZE(inputs)
#define NUM_FMTS    ARRAY_SIZE(formats)


static struct frmsize frmsizes_1_channel[] = {{720, 480, 1, 1}, {720, 576, 1, 1}, {704, 480, 1, 1}, {704, 576, 1, 1}};
static struct frmsize frmsizes_2_channels[] = {{720, 960, 2, 1}, {1440, 480, 1, 2}, {720, 1152, 2, 1}, {1440, 576, 1, 2}, {704, 960, 2, 1}, {1408, 480, 1, 2}, {704, 1152, 2, 1}, {1408, 576, 1, 2}};
static struct frmsize frmsizes_4_channels[] = {{1440, 960, 2, 2}, {1440, 1152, 2, 2}, {1408, 960, 2, 2}, {1408, 1152, 2, 2}};

static struct fmt formats[] = {
	{
		.name         = "planar YUV420 - NV12",
		.fourcc       = V4L2_PIX_FMT_NV12,
		.output_fmt   = TVD_PL_YUV420,
		.depth        = 12
	},
	{
		.name         = "planar YUV422 - NV16",
		.fourcc       = V4L2_PIX_FMT_NV16,
		.output_fmt   = TVD_PL_YUV422,
		.depth        = 16
	}
};

static struct input_cnf inputs[] = {
	{
		.channels_num = 1,
		.name         = "TV input 1",
		.frmsizes_cnt = ARRAY_SIZE(frmsizes_1_channel),
		.frmsizes     = frmsizes_1_channel,
		.channel_idx  = {1, 0, 0, 0}
	}/*,
	{
		.channels_num = 1,
		.name         = "TV input 2",
		.frmsizes_cnt = ARRAY_SIZE(frmsizes_1_channel),
		.frmsizes     = frmsizes_1_channel,
		.channel_idx  = {0, 1, 0, 0}
	},
	{
		.channels_num = 1,
		.name         = "TV input 3",
		.frmsizes_cnt = ARRAY_SIZE(frmsizes_1_channel),
		.frmsizes     = frmsizes_1_channel,
		.channel_idx  = {0, 0, 1, 0}
	},
	{
		.channels_num = 1,
		.name         = "TV input 4",
		.frmsizes_cnt = ARRAY_SIZE(frmsizes_1_channel),
		.frmsizes     = frmsizes_1_channel,
		.channel_idx  = {0, 0, 0, 1}
	},
	{
		.channels_num = 2,
		.name         = "TV inputs 1 + 2",
		.frmsizes_cnt = ARRAY_SIZE(frmsizes_2_channels),
		.frmsizes     = frmsizes_2_channels,
		.channel_idx  = {1, 2, 0, 0}
	},
	{
		.channels_num = 2,
		.name         = "TV inputs 1 + 3",
		.frmsizes_cnt = ARRAY_SIZE(frmsizes_2_channels),
		.frmsizes     = frmsizes_2_channels,
		.channel_idx  = {1, 0, 2, 0}
	},
	{
		.channels_num = 2,
		.name         = "TV inputs 1 + 4",
		.frmsizes_cnt = ARRAY_SIZE(frmsizes_2_channels),
		.frmsizes     = frmsizes_2_channels,
		.channel_idx  = {1, 0, 0, 2}
	},
	{
		.channels_num = 2,
		.name         = "TV inputs 2 + 3",
		.frmsizes_cnt = ARRAY_SIZE(frmsizes_2_channels),
		.frmsizes     = frmsizes_2_channels,
		.channel_idx  = {0, 1, 2, 0}
	},
	{
		.channels_num = 2,
		.name         = "TV inputs 2 + 4",
		.frmsizes_cnt = ARRAY_SIZE(frmsizes_2_channels),
		.frmsizes     = frmsizes_2_channels,
		.channel_idx  = {0, 1, 0, 2}
	},
	{
		.channels_num = 2,
		.name         = "TV inputs 3 + 4",
		.frmsizes_cnt = ARRAY_SIZE(frmsizes_2_channels),
		.frmsizes     = frmsizes_2_channels,
		.channel_idx  = {0, 0, 1, 2}
	},
	{
		.channels_num = 4,
		.name         = "TV inputs 1 + 2 + 3 + 4",
		.frmsizes_cnt = ARRAY_SIZE(frmsizes_4_channels),
		.frmsizes     = frmsizes_4_channels,
		.channel_idx  = {1, 2, 3, 4}
	} */
};



static int tvd_clk_init(struct tvd_dev *dev,int interface);


static int is_generating(struct tvd_dev *dev)
{
	return test_bit(0, &dev->generating);
}

static int start_generating(struct tvd_dev *dev)
{
	struct dmaqueue *dma_q = &dev->vidq;
	int i;
	
	__dbg("%s\n", __FUNCTION__);
	
	if (is_generating(dev)) {
		__err("stream has been already on\n");
		return 0;
	}
	
	/* Resets frame counters */
	dma_q->frame_jiffies = jiffies;
	dma_q->frames_count = 0;
	
	/* Configure interrupts and set TVD to start capturing */
	for(i = 0; i < 4; i++) {
		if(dev->channel_index[i]){
			dev->channel_irq = i; //FIXME, what frame done irq you should use when more than one channel signal?
			break;
		}
	}
	TVD_irq_status_clear(dev->channel_irq,TVD_FRAME_DONE);	
	TVD_irq_enable(dev->channel_irq,TVD_FRAME_DONE);
	for(i = 0; i < 4; i++) {
		if(dev->channel_index[i])
			TVD_capture_on(i);
	}
	
	/* Set generating flag and return */
	set_bit(0, &dev->generating);
	return 0;
}

static int stop_generating(struct tvd_dev *dev)
{
	struct dmaqueue *dma_q = &dev->vidq;
	int i;

	__dbg("%s\n", __FUNCTION__);
	
	if (!is_generating(dev)) {
		__err("stream has been already off\n");
		return 0;
	}
	
	/* Resets frame counters */
	dma_q->frame_jiffies = jiffies;
	dma_q->frames_count = 0;
	
	/* Disable interrupts and set TVD to stop capturing */
	TVD_irq_disable(dev->channel_irq,TVD_FRAME_DONE);
	TVD_irq_status_clear(dev->channel_irq,TVD_FRAME_DONE);
	for(i = 0; i < 4; i++) {
		if(dev->channel_index[i])
			TVD_capture_off(i);
	}

	/*
	 * TODO:
	 * Typical driver might need to wait here until dma engine stops.
	 * In this case we can abort imiedetly, so it's just a noop.
	 * Is TVD_capture_off enough to stop writing to the buffer?
	 */

	/* Release all active buffers */
	while (!list_empty(&dma_q->list)) {
		struct buffer *buf;
		buf = list_entry(dma_q->list.next, struct buffer, list);
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
		__dbg("Buffer removed from dma queue: %d\n", buf->vb.index);
	}

	/* Clear flags and return */
	first_flag = 0;
	clear_bit(0, &dev->generating);
	return 0;
}

static struct frmsize * get_frmsize (int input, int width, int height)
{
	unsigned int i;
	
	for (i = 0; i < inputs[input].frmsizes_cnt; i++) {
		if (inputs[input].frmsizes[i].width == width && inputs[input].frmsizes[i].height == height)
			return &inputs[input].frmsizes[i];
	}
	return NULL;
}

static struct fmt * get_format(__u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < NUM_FMTS; i++) {
		if (formats[i].fourcc == fourcc)
			return &formats[i];
	}
	return NULL;
}

static inline unsigned long get_frmbytes (struct fmt *fmt, unsigned int width, unsigned int height)
{
	/* 
	 * Sizeimage is usually (width*height*depth)/8 for uncompressed images, but it's different
	 * if bytesperline is used since there could be some padding between lines.
	 * In our case there is no padding (bytesperline = width)
	 */
	return width * height * fmt->depth / 8;
}

static inline void set_addr(struct tvd_dev *dev,struct buffer *buf)
{	
	int i;
	dma_addr_t addr_org;
	
	addr_org = vb2_dma_contig_plane_dma_addr(&buf->vb, 0);
	dev->buf_addr.y = addr_org;
	dev->buf_addr.c = addr_org + dev->width*dev->height;
	for (i = 0; i < 4; i++) {
		if (dev->channel_index[i]) {
			TVD_set_addr_y(i, dev->buf_addr.y + dev->channel_offset_y[i]);
			TVD_set_addr_c(i, dev->buf_addr.c + dev->channel_offset_c[i]);
		}
	}
	
	__dbg("%s: buf_addr_y=%x, buf_addr_cb=%x\n", __FUNCTION__, dev->buf_addr.y, dev->buf_addr.c);
}

static int apply_format (struct tvd_dev *dev)
{
	int i;
	
	__inf("interface=%d\n",dev->interface);
	__inf("system=%d\n",dev->system);
	__inf("format=%d\n",dev->format);
	__inf("row=%d\n",dev->row);
	__inf("column=%d\n",dev->column);
	__inf("channel_index[0]=%d\n",dev->channel_index[0]);
	__inf("channel_index[1]=%d\n",dev->channel_index[1]);
	__inf("channel_index[2]=%d\n",dev->channel_index[2]);
	__inf("channel_index[3]=%d\n",dev->channel_index[3]);
	__inf("width=%d\n",dev->width);
	__inf("height=%d\n",dev->height);
	
	if (dev->width  < MIN_WIDTH || dev->width  > MAX_WIDTH || dev->height < MIN_HEIGHT || dev->height > MAX_HEIGHT) {
		__err("%s: frame size not allowed (%dx%d)\n", __FUNCTION__, dev->width, dev->height);
		return -EINVAL;
	}
	
	TVD_config(dev->interface, dev->system);
	for (i = 0; i < 4; i++) {
		if (dev->channel_index[i]) {
			TVD_set_fmt(i, dev->fmt->output_fmt);
			TVD_set_width(i, (dev->format ? 704 : 720));
			if (dev->interface == 2)
				TVD_set_height(i, (dev->system ? 576 : 480));
			else
				TVD_set_height(i, (dev->system ? 576 : 480) / 2);
			TVD_set_width_jump(i, (dev->format ? 704 : 720) * dev->column);
			dev->channel_offset_y[i] = ((dev->channel_index[i]-1)%dev->column)*(dev->format?704:720) + ((dev->channel_index[i]-1)/dev->column)*dev->column*(dev->format?704:720)*(dev->system?576:480);
			dev->channel_offset_c[i] = ((dev->channel_index[i]-1)%dev->column)*(dev->format?704:720) + ((dev->channel_index[i]-1)/dev->column)*dev->column*(dev->format?704:720)*(dev->system?576:480)/2;
			__inf("channel_offset_y[%d]=%d\n", i, dev->channel_offset_y[i]);
			__inf("channel_offset_c[%d]=%d\n", i, dev->channel_offset_c[i]);
		}
	}
	
	if (tvd_clk_init(dev,dev->interface)) {
		__err("clock init fail!\n");
	}
	
	return 0;
}

static irqreturn_t tvd_irq(int irq, void *priv)
{
	struct buffer *buf;	
	struct tvd_dev *dev = (struct tvd_dev *)priv;
	struct dmaqueue *dma_q = &dev->vidq;
	//struct vb2_v4l2_buffer *vbuf;

	
	spin_lock(&dev->slock);
	if (first_flag == 0) {
		first_flag=1;
		goto set_next_addr;
	}
	
	/* skip frames to complain with framerate configuration */
	if (jiffies - dma_q->frame_jiffies < dev->frameival_jiffies) {
		__dbg("Skipped frame due to selected frame interval\n");
		goto unlock;
	}
	dma_q->frame_jiffies = jiffies;
	
	/* if there are no queued buffers, exit */
	if (list_empty(&dma_q->list)) {
		__err("No active queue to serve\n");
		goto unlock;
	}

	/* get first buffer from dma queue */
	buf = list_entry(dma_q->list.next, struct buffer, list);

	/* remove buffer from dma queue */
	list_del(&buf->list);

	/* fill frame count and timestamp (this is V4L2_FIELD_NONE -not interleaved-, so fields = frames) */
	dma_q->frames_count++;
	//buf->vb.vb2_buf.field = V4L2_FIELD_NONE;
	//buf->vb.vb2_queue->.sequence = dma_q->frames_count;
	buf->vb.timestamp = ktime_get_ns();
	//do_gettimeofday(&buf->vb.vb2_buf.timestamp);

	/* inform buffer is done */
	vb2_buffer_done(&buf->vb, VB2_BUF_STATE_DONE);
	__dbg("done buf = 0x%p (buffer %d), ts %ld\n", buf, buf->vb.index, (long)buf->vb.timestamp);
	/* if (!vb2_has_consumers(&dev->vb_vidq)) { TODO: how to know if there are consumers waiting
		__dbg("Nobody is waiting to dequeue\n");
	}*/
	
	/* judge if the frame queue has been written to the last */
	if (list_empty(&dma_q->list)) {		
		__dbg("No more free frame\n");		
		goto unlock;	
	}
	
	if ((&dma_q->list) == dma_q->list.next->next) {
		__dbg("No more free frame on next time\n");		
		goto unlock;	
	}
		
set_next_addr:
	/* next frame is being writen to first buffer of the queue, set address of second buffer for next time */
	buf = list_entry(dma_q->list.next->next,struct buffer, list);
	set_addr(dev,buf);

unlock:
	spin_unlock(&dev->slock);
	
	TVD_irq_status_clear(dev->channel_irq, TVD_FRAME_DONE);
	
	return IRQ_HANDLED;
}


static int tvd_clk_init(struct tvd_dev *dev,int interface)
{
	int ret =0;
	struct clk    *module1_clk_src;   /* parent clock for module1_clk */
	unsigned long desired_rate;
	unsigned long current_pll_rate;

	// configure module clock source, choosing from clocks video_pll0 and video_pll1. Before seting clock rate, check if any clock has already configured the desired one.
	desired_rate = interface == 2 ? 270000000 : 297000000; // interface 2 = YpbPr_P, else CVBS and YPbPr_I

	// try video_pll1 clock (video_pll1 is chosen first by display driver, so we can reuse if it already has the desired rate, letting video_pll0 free for other usages)
	module1_clk_src= clk_get(&dev->pdev->dev,"pll1");
	if (NULL == module1_clk_src || IS_ERR(module1_clk_src)) {
		__err("get tvd clock source error!\n");	
		goto cnf_clk_try_pll0;
	}
	__inf("pll1 clk_get OK\n");

	current_pll_rate = clk_get_rate(module1_clk_src);
	__inf("pll1 current rate: %dHz\n", current_pll_rate);

	if (current_pll_rate == desired_rate) {
		__inf("pll1 selected as module_clk parent clock\n");
		goto cnf_clk_rate;
	}

cnf_clk_try_pll0:
	// video_pll1 doesn't has desired rate, try video_pll0 clock (set the desired rate if it hasn't it already)
	module1_clk_src= clk_get(&dev->pdev->dev,"pll0");
	if (NULL == module1_clk_src || IS_ERR(module1_clk_src)) {
		__err("get tvd clock source error!\n");
		ret = -1;
		goto cnf_clk_exit;
	}
	__inf("pll1 selected as module_clk parent clock\n");


cnf_clk_rate:
	ret = clk_set_rate(module1_clk_src, desired_rate);	// FIXME: this can break something if video_pll0 is already in use, for a second monitor for example
	if (ret == -1)
	{
		__err("set tvd parent clock error!\n");
		goto cnf_clk_clk_put;
	}

	ret = clk_set_parent(dev->module1_clk, module1_clk_src);
	if (ret == -1)
	{
		__err("set tvd parent clock error!\n");
		goto cnf_clk_clk_put;
	}

	/* add by yaowenjun@allwinnertech.com
	 * spec p77 TVD_CLK_REG
	 * bit[3-0] set TVD_CLK divid ratio(m)
	 * the per-divided clock is divided by(m+1). the divider is from
	 * 1 to 16
	 * 0xb,0x5 from dulianping@allwinnertech.com
	 */
	if (interface == 2) {//YpbPr_P
		ret = clk_set_rate(dev->module1_clk, desired_rate / 0x5);
	} else {//CVBS and YPbPr_I
		ret = clk_set_rate(dev->module1_clk, desired_rate / 0xb);
	}

	if (ret == -1)
	{
		__err("set tvd clk rate error!\n");
	}

cnf_clk_clk_put:
	clk_put(module1_clk_src); //use ok

cnf_clk_exit:
	__inf("%s ret=%d", __FUNCTION__, ret);
	return ret;
}

/*
static int tvd_clk_exit(struct tvd_dev *dev)
{
	if(NULL == dev->ahb_clk || IS_ERR(dev->ahb_clk))
	{
		__err("tvd ahb_clk NULL hdle\n");
		return -1;
	}
	clk_put(dev->ahb_clk);
	dev->ahb_clk = NULL;

	if(NULL == dev->module1_clk || IS_ERR(dev->module1_clk))
	{
		__err("tvd module1_clk NULL hdle\n");
		return -1;
	}
	clk_put(dev->module1_clk);
	dev->module1_clk = NULL;

	if(NULL == dev->module2_clk || IS_ERR(dev->module2_clk))
	{
		__err("tvd module2_clk NULL hdle\n");
		return -1;
	}
	clk_put(dev->module2_clk);
	dev->module2_clk = NULL;

	if(NULL == dev->dram_clk || IS_ERR(dev->dram_clk))
	{
		__err("tvd dram_clk NULL hdle\n");
		return -1;
	}
	clk_put(dev->dram_clk);
	dev->dram_clk = NULL;

	return 0;
}
*/

static int tvd_clk_open(struct tvd_dev *dev)
{
	if(clk_prepare_enable(dev->dram_clk))
	{
		__err("tvd dram_clk enable fail\n");
	}
	if(clk_prepare_enable(dev->module1_clk))
	{
		__err("tvd module1_clk enable fail\n");
	}
	if(clk_prepare_enable(dev->module2_clk))
	{
		__err("tvd module2_clk enable fail\n");
	}
	if(clk_prepare_enable(dev->ahb_clk))
	{
		__err("tvd ahb_clk enable fail\n");
	}
	return 0;
}

static int tvd_clk_close(struct tvd_dev *dev)
{
	int ret=0;

	if (IS_ERR_OR_NULL(dev->ahb_clk))
	{
		__err("tvd ahb_clk NULL hdle\n");
		ret =-1;
	}
	else
		clk_disable_unprepare(dev->ahb_clk);

	if (IS_ERR_OR_NULL(dev->module1_clk))
	{
		__err("tvd module1_clk NULL hdle\n");
		ret =-1;
	}
	else
		clk_disable_unprepare(dev->module1_clk);

	if (IS_ERR_OR_NULL(dev->module2_clk))
	{
		__err("tvd module2_clk NULL hdle\n");
		ret =-1;
	}
	else
		clk_disable_unprepare(dev->module2_clk);

	if (IS_ERR_OR_NULL(dev->dram_clk))
	{
		__err("tvd dram_clk NULL hdle\n");
		ret =-1;
	}
	else
		clk_disable_unprepare(dev->dram_clk);

	return ret;
}



/* ------------------------------------------------------------------
	File operations for the device
   ------------------------------------------------------------------*/

static ssize_t tvd_read(struct file *file, char __user *data, size_t count, loff_t *ppos)
{
	struct tvd_dev *dev = video_drvdata(file);

	__dbg("%s\n", __FUNCTION__);

	return vb2_read(&dev->vb_vidq, data, count, ppos, file->f_flags & O_NONBLOCK);
}

static unsigned int tvd_poll(struct file *file, struct poll_table_struct *wait)
{
	struct tvd_dev *dev = video_drvdata(file);

	__dbg("%s\n", __FUNCTION__);
	
	return vb2_poll(&dev->vb_vidq, file, wait);
}

static int tvd_open(struct file *file)
{
	struct tvd_dev *dev = video_drvdata(file);
	int ret=0;
	
	__inf("%s\n", __FUNCTION__);

	if (mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;

	if (dev->opened == 1) {
		__err("device open busy\n");
		ret = -EBUSY;
		goto tvd_open_unlock;
	}

	tvd_clk_open(dev);
	TVD_init(dev->regs);
	
	dev->opened=1;

tvd_open_unlock:
	mutex_unlock(&dev->mutex);

	__inf("%s end\n", __FUNCTION__);
	return ret;
}

static int tvd_close(struct file *file)
{
	struct tvd_dev *dev = video_drvdata(file);

	__inf("%s\n", __FUNCTION__);
	mutex_lock(&dev->mutex);

	tvd_clk_close(dev);
	vb2_queue_release(&dev->vb_vidq);
	v4l2_fh_release(file);
	dev->opened=0;

	mutex_unlock(&dev->mutex);
	__inf("%s end\n", __FUNCTION__);
	return 0;
}

static int tvd_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct tvd_dev *dev = video_drvdata(file);
	int ret;

	__dbg("%s\n", __FUNCTION__);
	__dbg("mmap called, vma=0x%08lx\n", (unsigned long)vma);

	ret = vb2_mmap(&dev->vb_vidq, vma);

	__dbg("vma start=0x%08lx, size=%ld, ret=%d\n",
		(unsigned long)vma->vm_start,
		(unsigned long)vma->vm_end-(unsigned long)vma->vm_start,ret);
	
	return ret;
}

static const struct v4l2_file_operations fops = {
	.owner	  = THIS_MODULE,
	.open	  = tvd_open,
	.release  = tvd_close,
	.read     = tvd_read,
	.poll	  = tvd_poll,
	.unlocked_ioctl    = video_ioctl2,
	.mmap     = tvd_mmap,
};



/* ------------------------------------------------------------------
	IOCTL vidioc handling
   ------------------------------------------------------------------*/

static int vidioc_querycap(struct file *file, void  *priv,
		struct v4l2_capability *cap)
{
	struct tvd_dev *dev = video_drvdata(file);

	__dbg("%s\n", __FUNCTION__);

	strscpy(cap->driver, "tvd", sizeof(cap->driver));
	strscpy(cap->card, "tvd", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
			dev->pdev->dev.of_node->name);

	cap->version = TVD_VERSION;

	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING|V4L2_CAP_READWRITE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void  *priv,struct v4l2_fmtdesc *format)
{
	struct fmt *fmt;

	__dbg("%s\n", __FUNCTION__);

	if (format->index > NUM_FMTS - 1)
		return -EINVAL;
	
	fmt = &formats[format->index];

	strlcpy(format->description, fmt->name, sizeof(format->description));
	format->pixelformat = fmt->fourcc;
	
	return 0;
}

static int vidioc_enum_framesizes (struct file *file, void *priv, struct v4l2_frmsizeenum *frmsize)
{
	struct fmt *fmt;
	struct frmsize *fsz;
	struct tvd_dev *dev = video_drvdata(file);
	
	__dbg("%s: idx=%d, format=%d\n", __FUNCTION__, frmsize->index, frmsize->pixel_format);
	
	if (frmsize->index > inputs[dev->input].frmsizes_cnt - 1)
		return -EINVAL;
	
	fmt = get_format(frmsize->pixel_format);
	if (fmt == NULL)
		return -EINVAL;
	
	fsz = &inputs[dev->input].frmsizes[frmsize->index];
	
	frmsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frmsize->discrete.width = fsz->width;
	frmsize->discrete.height = fsz->height;
	
	return 0;
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,struct v4l2_format *format)
{
	struct tvd_dev *dev = video_drvdata(file);
	struct fmt *fmt;
	struct frmsize *frmsize;
	
	if (format->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		__err("%s: wrong buffer type (%d), only V4L2_BUF_TYPE_VIDEO_CAPTURE is supported\n", __FUNCTION__, format->type);
		return -EINVAL;
	}
	
	fmt = get_format(format->fmt.pix.pixelformat);
	if (fmt == NULL) { /* invalid format, pick a valid one instead */
		fmt = &formats[0];
		format->fmt.pix.pixelformat = fmt->fourcc;
	}
	
	frmsize = get_frmsize(dev->input, format->fmt.pix.width, format->fmt.pix.height);
	if (frmsize == NULL) { /* invalid framesize, pick a valid one instead */
		frmsize = &inputs[dev->input].frmsizes[0];
		format->fmt.pix.width = frmsize->width;
		format->fmt.pix.height = frmsize->height;
	}

	format->fmt.pix.field        = V4L2_FIELD_NONE;
	format->fmt.pix.sizeimage    = get_frmbytes(fmt, frmsize->width, frmsize->height); 
	format->fmt.pix.bytesperline = frmsize->width; /* planar fmts: width of first plane (Y) + padding (we use padding = 0) */
	format->fmt.pix.colorspace   = V4L2_COLORSPACE_SMPTE170M; /* correct? standard colorspace for PAL & NTSC */
	format->fmt.pix.priv         = 0;
	
	return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,struct v4l2_format *format)
{
	struct tvd_dev *dev = video_drvdata(file);

	__dbg("%s\n", __FUNCTION__);

	format->fmt.pix.width        = dev->width;
	format->fmt.pix.height       = dev->height;
	format->fmt.pix.field        = V4L2_FIELD_NONE;
	format->fmt.pix.pixelformat  = dev->fmt->fourcc;
	format->fmt.pix.sizeimage    = get_frmbytes(dev->fmt, dev->width, dev->height); 
	format->fmt.pix.bytesperline = dev->width; /* planar fmts: width of first plane (Y) + padding (we use padding = 0) */
	format->fmt.pix.colorspace   = V4L2_COLORSPACE_SMPTE170M; /* correct? standard colorspace for PAL & NTSC */
	format->fmt.pix.priv         = 0;
	
	__dbg("CALCULATIONS: %i, %i\n", format->fmt.pix.bytesperline, format->fmt.pix.sizeimage);

	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,struct v4l2_format *format)
{
	__dbg("%s\n", __FUNCTION__);
	
	struct tvd_dev *dev = video_drvdata(file);
	struct frmsize *frmsize;
	
	/* check errors */
	if (format->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		__err("%s: wrong buffer type (%d), only V4L2_BUF_TYPE_VIDEO_CAPTURE is supported\n", __FUNCTION__, format->type);
		return -EINVAL;
	}

	if (is_generating(dev)) {
		__err("%s: device busy\n", __func__);
		return -EBUSY;
	}

	
	/* check if format is valid, correct it if not */
	vidioc_try_fmt_vid_cap(file, priv, format);
	
	/* save and apply the new format */
	frmsize = get_frmsize(dev->input, format->fmt.pix.width, format->fmt.pix.height);

	dev->fmt              = get_format(format->fmt.pix.pixelformat);;
	dev->interface        = 0;
	dev->width            = frmsize->width;
	dev->height           = frmsize->height;
	dev->row              = frmsize->rows;
	dev->column           = frmsize->cols;
	dev->system           = frmsize->height / frmsize->rows == 480 ? 0 : 1; /* 0 = ntsc, 1 = pal */
	dev->format           = frmsize->width  / frmsize->cols == 720 ? 0 : 1; /* 0 = non mb, 1 = mb */
	dev->channel_index[0] = inputs[dev->input].channel_idx[0];
	dev->channel_index[1] = inputs[dev->input].channel_idx[1];
	dev->channel_index[2] = inputs[dev->input].channel_idx[2];
	dev->channel_index[3] = inputs[dev->input].channel_idx[3];
	
	return apply_format(dev);
}

#if 0
static int vidioc_g_fmt_type_private(struct file *file, void *priv,struct v4l2_format *format)
{
	struct tvd_dev *dev = video_drvdata(file);
	int i;

	__dbg("%s\n", __FUNCTION__);
	
	format->fmt.raw_data[0]  = dev->interface       ;
	format->fmt.raw_data[1]  = dev->system          ;
	format->fmt.raw_data[2]  = dev->format          ; //for test only
	format->fmt.raw_data[8]  = dev->row             ;
	format->fmt.raw_data[9]  = dev->column          ;
	format->fmt.raw_data[10] = dev->channel_index[0];
	format->fmt.raw_data[11] = dev->channel_index[1];
	format->fmt.raw_data[12] = dev->channel_index[2];
	format->fmt.raw_data[13] = dev->channel_index[3];
	
	for(i=0;i<4;i++){
		format->fmt.raw_data[16 + i] = TVD_get_status(i);
	}
	
	return 0;
}

static int vidioc_s_fmt_type_private(struct file *file, void *priv,struct v4l2_format *format)
{
	struct tvd_dev *dev = video_drvdata(file);

	__dbg("%s\n", __FUNCTION__);
	
	if (is_generating(dev)) {
		__err("%s device busy\n", __func__);
		return -EBUSY;
	}
	
	dev->interface          = format->fmt.raw_data[0];   //cvbs or yuv
	dev->system             = format->fmt.raw_data[1];   //ntsc or pal
	dev->format             = format->fmt.raw_data[2];   //mb or non-mb
	dev->row                = format->fmt.raw_data[8];
	dev->column             = format->fmt.raw_data[9];
	dev->channel_index[0]   = format->fmt.raw_data[10];
	dev->channel_index[1]   = format->fmt.raw_data[11];
	dev->channel_index[2]   = format->fmt.raw_data[12];
	dev->channel_index[3]   = format->fmt.raw_data[13];
	dev->width              = dev->column * (dev->format ? 704 : 720);
	dev->height             = dev->row * (dev->system ? 576 : 480);
	dev->fmt                = &formats[0]; // NV12

	return apply_format(dev);
}
#endif



static int vidioc_reqbufs(struct file *file, void *priv,struct v4l2_requestbuffers *p)
{
	struct tvd_dev *dev = video_drvdata(file);
	__dbg("%s: buffs requested: count=%d, type=%d, mem=%d\n", __FUNCTION__, p->count, p->type, p->memory);
	
	return vb2_reqbufs(&dev->vb_vidq, p);
}

static int vidioc_querybuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct tvd_dev *dev = video_drvdata(file);
	__dbg("%s: buffer %d\n", __FUNCTION__, p->index);
	
	return vb2_querybuf(&dev->vb_vidq, p);
}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct tvd_dev *dev = video_drvdata(file);
	__dbg("%s: buffer %d\n", __FUNCTION__, p->index);
	
	return vb2_qbuf(&dev->vb_vidq, dev->v4l2_dev.mdev, p);
}

static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	int ret;
	int i;
	struct tvd_dev *dev = video_drvdata(file);
	__dbg("%s: buffer dequeue requested\n", __FUNCTION__);

	ret = vb2_dqbuf(&dev->vb_vidq, p, file->f_flags & O_NONBLOCK);
	
	if (ret == 0)            {__dbg("%s: buffer dequeued %d, flags %d\n", __FUNCTION__, p->index, p->flags);}
	else if (ret == -EAGAIN) {__dbg("%s: buffer not ready yet (returned EAGAIN with O_NONBLOCK == 1)\n", __FUNCTION__);}
	else                     {__err("%s: error dequeueing, error %d\n", __FUNCTION__, -ret);}
	
	#if DIRTY_HACK_RETURN_DMA_ADDR
	for (i = 0; i < dev->vb_vidq.num_buffers; i++) {
		if (dev->vb_vidq.bufs[i]->index == p->index)
			p->reserved = vb2_dma_contig_plane_dma_addr(dev->vb_vidq.bufs[i], 0);
	}
	#endif
	
	return ret;
}

static int vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct tvd_dev *dev = video_drvdata(file);
	__dbg("%s\n", __FUNCTION__);
	
	return vb2_streamon(&dev->vb_vidq, i);
}

static int vidioc_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct tvd_dev *dev = video_drvdata(file);
	__dbg("%s\n", __FUNCTION__);
	
	return vb2_streamoff(&dev->vb_vidq, i);
}


static int vidioc_enum_input(struct file *file, void *priv,struct v4l2_input *inp)
{
	__dbg("%s\n", __FUNCTION__);
	
	if (inp->index > NUM_INPUTS-1) {
		__err("input index invalid! idx: %d\n", inp->index);
		return -EINVAL;
	}

	inp->type = V4L2_INPUT_TYPE_CAMERA;
	strlcpy(inp->name, inputs[inp->index].name, sizeof(inp->name));
	// TODO: fill supported standards (PAL, NTSC...)
	
	return 0;
}

static int vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
	struct tvd_dev *dev = video_drvdata(file);
	__dbg("%s\n", __FUNCTION__);

	*i = dev->input; 
	return 0;
}

static int vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
	struct frmsize *frmsize;
	struct tvd_dev *dev = video_drvdata(file);
	
	__dbg("%s\n", __FUNCTION__);

	if (i > NUM_INPUTS-1) {
		__err("set input error!\n");
		return -EINVAL;
	}
	
	if (is_generating(dev)) {
		__err("%s device busy\n", __func__);
		return -EBUSY;
	}
	
	dev->input = i;
	
	// if previous framesize is not available for the new input pick another one
	frmsize = get_frmsize(i, dev->width, dev->height);
	if (!frmsize)
		frmsize = &inputs[i].frmsizes[0];
	
	// edit values that may have changed
	dev->width            = frmsize->width;
	dev->height           = frmsize->height;
	dev->row              = frmsize->rows;
	dev->column           = frmsize->cols;
	dev->system           = frmsize->height / frmsize->rows == 480 ? 0 : 1; /* 0 = ntsc, 1 = pal */
	dev->format           = frmsize->width  / frmsize->cols == 720 ? 0 : 1; /* 0 = non mb, 1 = mb */
	dev->channel_index[0] = inputs[dev->input].channel_idx[0];
	dev->channel_index[1] = inputs[dev->input].channel_idx[1];
	dev->channel_index[2] = inputs[dev->input].channel_idx[2];
	dev->channel_index[3] = inputs[dev->input].channel_idx[3];
	
	return apply_format(dev);
}

static int vidioc_queryctrl(struct file *file, void *priv,struct v4l2_queryctrl *qc)
{
	bool next;
	__u32 id;
	
	//struct tvd_dev *dev = video_drvdata(file);	
	__dbg("%s\n", __FUNCTION__);

	next = qc->id & V4L2_CTRL_FLAG_NEXT_CTRL;
	id = qc->id & (~V4L2_CTRL_FLAG_NEXT_CTRL);
	if (next) {
		if (id < V4L2_CID_MIN_BUFFERS_FOR_CAPTURE)
			qc->id = V4L2_CID_MIN_BUFFERS_FOR_CAPTURE;
		else
			return -EINVAL;
	}
	
	if (qc->id == V4L2_CID_MIN_BUFFERS_FOR_CAPTURE) {
		qc->type = V4L2_CTRL_TYPE_INTEGER;
		qc->minimum = 3;
		qc->maximum = 3;
		qc->default_value = 3;
		qc->flags = V4L2_CTRL_FLAG_READ_ONLY;
		return 0;
	}

	return -EINVAL;
}

static int vidioc_g_ctrl(struct file *file, void *priv,struct v4l2_control *ctrl)
{
	//struct tvd_dev *dev = video_drvdata(file);	
	__dbg("%s\n", __FUNCTION__);
	
	if (ctrl->id == V4L2_CID_MIN_BUFFERS_FOR_CAPTURE) {
		ctrl->value = 3;
		return 0;
	}
	
	return -EINVAL;
}


static int vidioc_s_ctrl(struct file *file, void *priv,struct v4l2_control *ctrl)
{
	struct tvd_dev *dev = video_drvdata(file);	
	__dbg("%s\n", __FUNCTION__);
	
	return -EINVAL;
}

static int vidioc_g_parm(struct file *file, void *priv,struct v4l2_streamparm *parms) 
{
	int ret = 0;       
	struct tvd_dev *dev = video_drvdata(file);
	
	__dbg("%s: type=%d\n", __FUNCTION__, parms->type);
	
	if (parms->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
	{
		parms->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
		parms->parm.capture.capturemode = 0;
		parms->parm.capture.timeperframe = dev->frameival_secs;
		parms->parm.capture.extendedmode = 0;
		// TODO: parms->parm.capture.readbuffers = ?; see https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/vidioc-g-parm.html
	}
	return ret;
}

static int vidioc_s_parm(struct file *file, void *priv,struct v4l2_streamparm *parms)
{
	struct tvd_dev *dev = video_drvdata(file);
	int ret=0;
	
	__dbg("%s: type=%d\n", __FUNCTION__, parms->type);
	
	if(parms->type==V4L2_BUF_TYPE_PRIVATE)
	{
		if(parms->parm.raw_data[0] == TVD_COLOR_SET)
		{
			dev->luma       = parms->parm.raw_data[1];
			dev->contrast   = parms->parm.raw_data[2];
			dev->saturation = parms->parm.raw_data[3];
			dev->hue        = parms->parm.raw_data[4];
			TVD_set_color(0,dev->luma,dev->contrast,dev->saturation,dev->hue);
		}
		else if(parms->parm.raw_data[0] == TVD_UV_SWAP)
		{
			dev->uv_swap    = parms->parm.raw_data[1];
			TVD_uv_swap(dev->uv_swap);
		}
	}
	else if(parms->type==V4L2_BUF_TYPE_VIDEO_CAPTURE)
	{
		if (!parms->parm.capture.timeperframe.denominator || !parms->parm.capture.timeperframe.numerator) {
			dev->frameival_jiffies = 0; // 0 = don't drop frames, pass to applicaton as soon as posible
			dev->frameival_secs.numerator = 0;
			dev->frameival_secs.denominator = 0;
		}
		else {
			dev->frameival_secs = parms->parm.capture.timeperframe;
			dev->frameival_jiffies = msecs_to_jiffies(1000 * parms->parm.capture.timeperframe.numerator /
				parms->parm.capture.timeperframe.denominator);
		}
		// TODO: ? = parms->parm.capture.readbuffers; see https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/vidioc-g-parm.html
		__dbg("frame interval set to %u/%u (interval (jiffies)=%lu)\n", dev->frameival_secs.numerator, dev->frameival_secs.denominator, dev->frameival_jiffies);
	}
	return ret;
}

static int vidioc_enum_frameintervals(struct file *file, void *priv, struct v4l2_frmivalenum *frmivalenum)
{
	struct tvd_dev *dev = video_drvdata(file);
	struct fmt *fmt;
	struct frmsize *frmsize;

	__dbg("%s\n", __FUNCTION__);
	
	/* validate index, format and framesize */
	if (frmivalenum->index > 0) {
		__err("%s: invalid idx: idx=%d\n", __FUNCTION__, frmivalenum->index);
		return -EINVAL;
	}
	fmt = get_format(frmivalenum->pixel_format);
	frmsize = get_frmsize(dev->input, frmivalenum->width, frmivalenum->height);
	if (!fmt || !frmsize) {
		__err("%s: invalid format: fmt=%d, width=%d, height=%d\n", __FUNCTION__, frmivalenum->pixel_format, frmivalenum->width, frmivalenum->height);
		return -EINVAL;
	}
	
	/* Note: framerate is the same for all valid formats and framesizes */
	/* TODO: consider changing to stepwise, not continuous, depending on selected video standard (PAL or NTSC, which can be obtained from framesize in frmivalenum variable) */
	frmivalenum->type = V4L2_FRMIVAL_TYPE_CONTINUOUS;
	frmivalenum->stepwise.min.numerator = 1;    /* min = NTSC time per frame */
	frmivalenum->stepwise.min.denominator = 30;
	frmivalenum->stepwise.max.numerator = 600;  /* max = arbitrary big value */
	frmivalenum->stepwise.max.denominator = 1;
	frmivalenum->stepwise.step.numerator = 1;   /* step = 1 for continuous intervals */
	frmivalenum->stepwise.step.denominator = 1;
	
	return 0;
}

static const struct v4l2_ioctl_ops ioctl_ops = {
	.vidioc_querycap                = vidioc_querycap,
	.vidioc_enum_fmt_vid_cap        = vidioc_enum_fmt_vid_cap,
	.vidioc_enum_framesizes         = vidioc_enum_framesizes,
	.vidioc_g_fmt_vid_cap           = vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap         = vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap           = vidioc_s_fmt_vid_cap, 
	//.vidioc_s_fmt_type_private      = vidioc_s_fmt_type_private,
	//.vidioc_g_fmt_type_private      = vidioc_g_fmt_type_private,
	.vidioc_reqbufs                 = vidioc_reqbufs,
	.vidioc_querybuf                = vidioc_querybuf,
	.vidioc_qbuf                    = vidioc_qbuf,
	.vidioc_dqbuf                   = vidioc_dqbuf,
	.vidioc_enum_input              = vidioc_enum_input,
	.vidioc_g_input                 = vidioc_g_input,
	.vidioc_s_input                 = vidioc_s_input,
	.vidioc_streamon                = vidioc_streamon,
	.vidioc_streamoff               = vidioc_streamoff,
	.vidioc_queryctrl               = vidioc_queryctrl,
	.vidioc_g_ctrl                  = vidioc_g_ctrl,
	.vidioc_s_ctrl                  = vidioc_s_ctrl,
	.vidioc_g_parm                  = vidioc_g_parm,
	.vidioc_s_parm                  = vidioc_s_parm,
	.vidioc_enum_frameintervals     = vidioc_enum_frameintervals,
};


static struct video_device device = {
	.name		= "tvd",
	.fops           = &fops,
	.ioctl_ops 	= &ioctl_ops,
	.release	= video_device_release,

};



/* ------------------------------------------------------------------
	Videobuf operations
   ----------------------------------*/
static int queue_setup(struct vb2_queue *vq, /*const struct v4l2_format *fmt,*/
				unsigned int *nbuffers, unsigned int *nplanes,
				unsigned int sizes[], /*void*/ struct device *alloc_ctxs[])
{
	struct tvd_dev *dev = vb2_get_drv_priv(vq);
	
	__dbg("%s\n", __FUNCTION__);
	
	if (*nplanes == 0) /* called from VIDIOC_REQBUFS */
	{
		if (*nbuffers < 3) // TODO: remove in new version of vb2, setting vq->min_buffers is enough
			*nbuffers = 3;
		*nplanes = 1;
		sizes[0] = get_frmbytes(dev->fmt, dev->width, dev->height);
	//	alloc_ctxs[0] = dev->alloc_ctx; // TODO: remove in new version of vb2, alloc_ctx was removed
		return 0;
	}
	else /* called from VIDIOC_CREATE_BUFS */
	{
		if (*nbuffers + vq->num_buffers < 3) // TODO: remove in new version of vb2, setting vq->min_buffers is enough
			*nbuffers = 3 - vq->num_buffers;
		return (sizes[0] < get_frmbytes(dev->fmt, dev->width, dev->height)) ? -EINVAL : 0;
	}
}


static int buffer_init(struct vb2_buffer *vb)
{
	struct tvd_dev *dev = vb2_get_drv_priv(vb->vb2_queue);
	struct buffer *buf = container_of(vb, struct buffer, vb);

	__dbg("%s: buffer %d\n", __FUNCTION__, vb->index);

	BUG_ON(NULL == dev->fmt);

	INIT_LIST_HEAD(&buf->list);
	
	/*
	 * This callback is called once per buffer, after its allocation.
	 *
	 * This driver does not allow changing format during streaming, but it is
	 * possible to do so when streaming is paused (i.e. in streamoff state).
	 * Buffers however are not freed when going into streamoff and so
	 * buffer size verification has to be done in buffer_prepare, on each
	 * qbuf.
	 * It would be best to move verification code here to buf_init and
	 * s_fmt though.
	 */

	return 0;
}

static int buffer_prepare(struct vb2_buffer *vb)
{
	struct tvd_dev *dev = vb2_get_drv_priv(vb->vb2_queue);
	struct buffer *buf = container_of(vb, struct buffer, vb);
	unsigned long size;

	__dbg("%s: buffer %d\n", __FUNCTION__, vb->index);

	BUG_ON(NULL == dev->fmt);

	/* 
	 * format may have changed since buffers were allocated. This is because
	 * when doing streamoff they're not deallocated and they can be reused.
	 * Check if they're still big enough and set them to current size and format.
	 */
	size = get_frmbytes(dev->fmt, dev->width, dev->height);
	
	if (vb2_plane_size(vb, 0) < size) {
		__err("%s: buffer size is not enough\n", __FUNCTION__);
		return -EINVAL;
	}
	
	vb2_set_plane_payload(vb, 0, size); /* set used size in buffer (buffer may be bigger) */
	buf->fmt = dev->fmt;

	return 0;
}

static void buffer_queue(struct vb2_buffer *vb)
{
	struct tvd_dev *dev = vb2_get_drv_priv(vb->vb2_queue);
	struct buffer *buf = container_of(vb, struct buffer, vb);
	struct dmaqueue *vidq = &dev->vidq;
	unsigned long flags = 0;

	__dbg("%s: buffer %d\n", __FUNCTION__, vb->index);

	/* buffer is queued and we have now the control of it */
	spin_lock_irqsave(&dev->slock, flags);
	if (list_empty(&vidq->list) || (&vidq->list) == vidq->list.next->next)
		set_addr(dev, buf); /* if there was no next frame in dma queue, set it address */
	list_add_tail(&buf->list, &vidq->list); /* add buffer to dma queue */
	spin_unlock_irqrestore(&dev->slock, flags);
}

static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct tvd_dev *dev = vb2_get_drv_priv(vq);
	__dbg("%s\n", __FUNCTION__);
	return start_generating(dev);
}

static void stop_streaming(struct vb2_queue *vq)
{
	struct tvd_dev *dev = vb2_get_drv_priv(vq);
	__dbg("%s\n", __FUNCTION__);
	stop_generating(dev);
}

static void wait_finish(struct vb2_queue *vq)
{
	struct tvd_dev *dev = vb2_get_drv_priv(vq);
	mutex_lock(&dev->mutex);
}

static void wait_prepare(struct vb2_queue *vq)
{
	struct tvd_dev *dev = vb2_get_drv_priv(vq);
	mutex_unlock(&dev->mutex);
}

static struct vb2_ops buffers_ops = {
	.queue_setup		= queue_setup,
	.buf_init		= buffer_init,
	.buf_prepare		= buffer_prepare,
	.buf_queue		= buffer_queue,
	.start_streaming	= start_streaming,
	.stop_streaming		= stop_streaming,
	.wait_prepare		= wait_prepare,
	.wait_finish		= wait_finish,
};



static int tvd_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct tvd_dev *dev;
	struct resource *res;
	struct video_device *vfd;
	int ret = 0;
	__dbg("%s start\n", __FUNCTION__);


	/*request mem for dev*/	
	//dev = kzalloc(sizeof(struct tvd_dev), GFP_KERNEL);
	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		__err("request dev mem failed!\n");
		return -ENOMEM;
	}
	dev->id = pdev->id;
	dev->pdev = pdev;
	
	spin_lock_init(&dev->slock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		__err("failed to find the registers\n");
		ret = -ENOENT;
		goto err_info;
	}

	dev->regs_res = request_mem_region(res->start, resource_size(res),
			dev_name(&pdev->dev));
	if (!dev->regs_res) {
		__err("failed to obtain register region\n");
		ret = -ENOENT;
		goto err_info;
	}
	
	dev->regs = ioremap(res->start, resource_size(res));
	if (!dev->regs) {
		__err("failed to map registers\n");
		ret = -ENXIO;
		goto err_req_region;
	}
 
    /*get irq resource*/	
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		__err("failed to get IRQ resource\n");
		ret = -ENXIO;
		goto err_regs_unmap;
	}
	dev->irq = res->start;
	
	ret = request_irq(dev->irq, tvd_irq, 0, pdev->name, dev);
	if (ret) {
		__err("failed to install irq (%d)\n", ret);
		goto err_regs_unmap;
	}
	
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	/* v4l2 device register */
	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);	
	if (ret) {
		__err("Error registering v4l2 device\n");
		goto err_regs_unmap;
		
	}

	dev_set_drvdata(&(pdev)->dev, (dev));

	ret = -ENXIO;
	dev->ahb_clk=devm_clk_get(&dev->pdev->dev, "ahb");
	if (NULL == dev->ahb_clk || IS_ERR(dev->ahb_clk))
	{
		__err("get tvd ahb clk error!\n");
		goto unreg_dev;
	}

	dev->module1_clk=devm_clk_get(&dev->pdev->dev,"sclk1");
	if(NULL == dev->module1_clk || IS_ERR(dev->module1_clk))
	{
		__err("get tvd module1 clock error!\n");
		goto unreg_dev;
	}

	dev->module2_clk=devm_clk_get(&dev->pdev->dev, "sclk2");
	if(NULL == dev->module2_clk || IS_ERR(dev->module2_clk))
	{
		__err("get tvd module2 clock error!\n");
		goto unreg_dev;
	}

	dev->dram_clk = devm_clk_get(&dev->pdev->dev, "ram");
	if (NULL == dev->dram_clk || IS_ERR(dev->dram_clk))
	{
		__err("get tvd dram clock error!\n");
		goto unreg_dev;
	}

	if (tvd_clk_init(dev,0)) {
		__err("clock init fail!\n");
		goto unreg_dev;
	}

	/*video device register	*/
	ret = -ENOMEM;
	vfd = video_device_alloc();
	if (!vfd) {
		goto err_clk;
	}	

	*vfd = device;
	vfd->v4l2_dev = &dev->v4l2_dev;

	dev_set_name(&vfd->dev, "tvd");
	ret = video_register_device(vfd, VFL_TYPE_GRABBER, video_nr);
	if (ret < 0) {
		goto rel_vdev;
	}	
	video_set_drvdata(vfd, dev);
	
	/*add device list*/
	/* Now that everything is fine, let's add it to device list */
	list_add_tail(&dev->devlist, &devlist);

	if (video_nr != -1) {
		video_nr++;
	}
	dev->vfd = vfd;

	__inf("V4L2 device registered as %s\n",video_device_node_name(vfd));
	
	/* init videobuf2 allocation */
	/*
	dev->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(dev->alloc_ctx)) {
		__err("DMA contig init error\n");
		goto unreg_vdev;
	}
	*/
	dev->alloc_ctx = &pdev->dev;

	/* init video buffer queue */
	memset(&dev->vb_vidq, 0, sizeof(dev->vb_vidq));
	dev->vb_vidq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dev->vb_vidq.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	dev->vb_vidq.io_modes = VB2_MMAP | VB2_DMABUF;
	dev->vb_vidq.dev = &pdev->dev;
	dev->vb_vidq.drv_priv = dev;
	dev->vb_vidq.buf_struct_size = sizeof(struct buffer);
	dev->vb_vidq.ops = &buffers_ops;
	dev->vb_vidq.mem_ops = &vb2_dma_contig_memops;
	ret = vb2_queue_init(&dev->vb_vidq);
	if (ret < 0) {
		__err("vb2 queue init error\n");
		goto /*clean_alloc_ctx*/unreg_vdev;
	}
	
	/* Provide a mutex to v4l2 core. It will be used to protect all fops and v4l2 ioctls. */
	mutex_init(&dev->mutex);
	vfd->lock = &dev->mutex;

	
	/* init video dma queues */
	INIT_LIST_HEAD(&dev->vidq.list);
	//init_waitqueue_head(&dev->vidq.wq);
	
	/* set initial config */
	dev->input            = 0;
	dev->fmt              = &formats[0];
	dev->interface        = 0;
	dev->width            = inputs[dev->input].frmsizes[0].width;
	dev->height           = inputs[dev->input].frmsizes[0].height;
	dev->row              = inputs[dev->input].frmsizes[0].rows;
	dev->column           = inputs[dev->input].frmsizes[0].cols;
	dev->system           = dev->height / dev->row == 480 ? 0 : 1; // 0 = ntsc, 1 = pal
	dev->format           = dev->width / dev->column == 720 ? 0 : 1; // 0 = non mb, 1 = mb
	dev->channel_index[0] = inputs[dev->input].channel_idx[0];
	dev->channel_index[1] = inputs[dev->input].channel_idx[1];
	dev->channel_index[2] = inputs[dev->input].channel_idx[2];
	dev->channel_index[3] = inputs[dev->input].channel_idx[3];
	dev->frameival_secs.numerator   = 0;
	dev->frameival_secs.denominator = 0;
	dev->frameival_jiffies          = 0;
	dev->opened = 0;

	TVD_init(dev->regs);
	ret = apply_format(dev);

	__dbg("%s start ret=0\n", __FUNCTION__);
	return 0;
/*
clean_alloc_ctx:
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
*/
unreg_vdev:
	video_unregister_device(dev->vfd);
rel_vdev:
	video_device_release(vfd);
err_clk:
	//tvd_clk_exit(dev);
unreg_dev:
	v4l2_device_unregister(&dev->v4l2_dev);
err_irq:
	free_irq(dev->irq, dev);
err_regs_unmap:
	iounmap(dev->regs);
err_req_region:
	release_resource(dev->regs_res);
err_info:
	//kfree(dev);
	__err("%s failed to install\n", __FUNCTION__);
	
	return ret;
}

static int tvd_release(void)
{
	struct list_head *list;
	struct tvd_dev *dev;

	__dbg("%s\n", __FUNCTION__);
	
	while (!list_empty(&devlist)) 
	{
		list = devlist.next;
		list_del(list);
		dev = list_entry(list, struct tvd_dev, devlist);

		v4l2_info(&dev->v4l2_dev, "unregistering %s\n", video_device_node_name(dev->vfd));
		video_unregister_device(dev->vfd);
		v4l2_device_unregister(&dev->v4l2_dev);
		//kfree(dev);
	}

	return 0;
}

static int tvd_remove(struct platform_device *pdev)
{
	struct tvd_dev *dev=(struct tvd_dev *)platform_get_drvdata(pdev);
	__inf("%s start\n", __FUNCTION__);
	/*vb2_dma_contig_cleanup_ctx(dev->alloc_ctx); */
	free_irq(dev->irq, dev);//	
	//tvd_clk_exit(dev);
	iounmap(dev->regs);
	release_resource(dev->regs_res);
	//kfree(dev->regs_res);
	//kfree(dev);
	tvd_release();
	__inf("%s end\n", __FUNCTION__);
	return 0;
}

static int tvd_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tvd_dev *dev=(struct tvd_dev *)dev_get_drvdata(&(pdev)->dev);
	int ret=0;
	
	__dbg("%s\n", __FUNCTION__);
	
	if (dev->opened==1) {
		tvd_clk_close(dev);		
	}
	return ret;
}

static int tvd_resume(struct platform_device *pdev)
{
	int ret=0;
	struct tvd_dev *dev=(struct tvd_dev *)dev_get_drvdata(&(pdev)->dev);
	
	__dbg("%s\n", __FUNCTION__);

	if (dev->opened==1) {
		tvd_clk_open(dev);
	} 
	
	return ret;
}

static const struct of_device_id drvr_of_match[] = {
		{
				.compatible ="allwinner,sun4i-a10-tv-decoder"
		},
		{},
};

static struct platform_driver tvd_driver = {
	.probe		= tvd_probe,
	.remove		= tvd_remove,
	.suspend	= tvd_suspend,
	.resume		= tvd_resume,
	.driver = {
		.name	= "tvd",
		.owner	= THIS_MODULE,
		.of_match_table = drvr_of_match,
	}
};

#if 0
/*
#define AW_IRQ_GIC_START        32
#define AW_IRQ_TVD       	(AW_IRQ_GIC_START + 61)    /* TVD   */

static struct resource tvd_resource[2] = {
	[0] = {
		.start	= TVD_REGS_BASE,
		.end	= (TVD_REGS_BASE + TVD_REG_SIZE - 1),
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AW_IRQ_TVD,
		.end	= AW_IRQ_TVD,
		.flags	= IORESOURCE_IRQ,
	},
}; */

static struct platform_device tvd_device = {
	.name           	= "tvd",
	.id             	= -1, // FIXME
	//.num_resources		= ARRAY_SIZE(tvd_resource),
	//.resource       	= tvd_resource,
	.dev            	= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
	}
};
#endif

#if 0
static int __init tvd_init(void)
{
	__u32 ret=0;
	
	ret = platform_device_register(&tvd_device);
	if (ret) {
		__err("platform device register failed!\n");
		return -1;
	}
	
	ret = platform_driver_register(&tvd_driver);
	
	if (ret) {
		__err("platform driver register failed!\n");
		return -1;
	}
	return ret;
}

static void __exit tvd_exit(void)
{
	__dbg("%s\n", __FUNCTION__);
	tvd_release();
	platform_driver_unregister(&tvd_driver);
}
#endif

//module_init(tvd_init);
//module_exit(tvd_exit);
module_platform_driver(tvd_driver);

MODULE_AUTHOR("jshwang");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TV decoder driver");
