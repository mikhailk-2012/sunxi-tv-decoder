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
#include <media/videobuf-core.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf-dma-contig.h>
#include <linux/moduleparam.h>
//#include <mach/sys_config.h>
//#include <mach/clock.h>
//#include <mach/irqs.h>
#include <linux/regulator/consumer.h>

#include <linux/types.h>

#define TVD_REGS_BASE   0x01c08000
#define TVD_REG_SIZE    0x1000

#define TVD_MAJOR_VERSION 1
#define TVD_MINOR_VERSION 0
#define TVD_RELEASE 0
#define TVD_VERSION KERNEL_VERSION(TVD_MAJOR_VERSION, TVD_MINOR_VERSION, TVD_RELEASE)
#define TVD_MODULE_NAME "tvd"

#define MIN_WIDTH  (32)
#define MIN_HEIGHT (32)
#define MAX_WIDTH  (4096)
#define MAX_HEIGHT (4096)
#define MAX_BUFFER (32*1024*1024)


#define BLINK_DELAY   HZ/2

static unsigned video_nr = 1;
static unsigned first_flag = 0;
//static struct timer_list timer;

typedef enum
{
        TVD_UV_SWAP,
        TVD_COLOR_SET,
}tvd_param_t;


struct frmsize {
	__u32 width;
	__u32 height;
	int rows;
	int cols;
};

struct fmt {
        u8              name[32];
        __u32           fourcc;          /* v4l2 format id */
        tvd_fmt_t       output_fmt;	
        int   	        depth;
};

struct input_cnf {
	int             channels_num;    /* number of input channels used: 1, 2 or 4 */
	char            *name;           /* string with the input name */
	int             frmsizes_cnt;    /* number of framesizes available for this input */
	struct frmsize  *frmsizes;       /* framesizes available for this input */
	int             channel_idx[4];  /* indexes of channels: enabled/disabled channels and order */
};

/* buffer for one video frame */
struct buffer {
	struct /*vb2_v4l2_buffer*/vb2_buffer vb;
	struct list_head  list;
	struct fmt        *fmt;
};

struct dmaqueue {
	struct list_head list;
	unsigned long    frame_jiffies;
	unsigned int     frames_count;
};

struct buf_addr {
	dma_addr_t	y;
	dma_addr_t	c;
};

static LIST_HEAD(devlist);

struct tvd_dev {
	struct list_head       	devlist;
	struct v4l2_device      v4l2_dev;
	struct v4l2_subdev	*sd;
	struct platform_device	*pdev;

	int			id;
	
	spinlock_t              slock;
	struct mutex            mutex;
	
	//struct vb2_alloc_ctx    *alloc_ctx; // TODO: remove for new version of vb2
	struct device *alloc_ctx;

	/* various device info */
	struct video_device     *vfd;

	/* queues */
	struct dmaqueue         vidq;     /* queue for driver */
	struct vb2_queue        vb_vidq;  /* queue for videobuf2 */

	/* Input Number */
	int	                input;

	/* video capture */
	struct fmt              *fmt;
	unsigned int            width;
	unsigned int            height;
	unsigned int            interface;
	unsigned int            system;
	unsigned int            format;
	unsigned int            row;
	unsigned int            column;
	unsigned int            channel_index[4];	
	unsigned int            channel_offset_y[4];
	unsigned int            channel_offset_c[4];
	unsigned int            channel_irq;

	/*working state*/
	unsigned long 	        generating;
	int			opened;

	//clock
	struct clk		*ahb_clk;
	struct clk		*module1_clk;
    struct clk		*module2_clk;
	struct clk		*dram_clk;

	int			irq;
	void __iomem		*regs;
	struct resource		*regs_res; //??
	
	struct buf_addr		buf_addr;

        //luma contrast saturation hue 
        unsigned int            luma;
        unsigned int            contrast;
        unsigned int            saturation;
        unsigned int            hue;

        unsigned int            uv_swap;
	
        // frame interval (to control fps)
	struct v4l2_fract       frameival_secs;    // selected by user, in seconds (set to 0/X or X/0 to avoid skipping frames)
	unsigned long           frameival_jiffies; // frame interval converted to jiffies
        
};

