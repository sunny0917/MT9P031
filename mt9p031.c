/*
 * A V4L2 driver for Micron mt9d112 cameras.
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-mediabus.h>//linux-3.0
#include <linux/io.h>
#include <linux/gpio.h>
#include <mach/sys_config.h>
#include <linux/regulator/consumer.h>
#include <mach/system.h>
#include "../include/sunxi_csi_core.h"
#include "../include/sunxi_dev_csi.h"

MODULE_AUTHOR("raymonxiu");
MODULE_DESCRIPTION("A low-level driver for Micron MT9P031 sensors");
MODULE_LICENSE("GPL");

//for internel driver debug
#define DEV_DBG_EN   		1
#if(DEV_DBG_EN == 1)		
#define csi_dev_dbg(x,arg...) printk(KERN_INFO"[CSI_DEBUG][MT9P031]"x,##arg)
#else
#define csi_dev_dbg(x,arg...) 
#endif
#define csi_dev_err(x,arg...) printk(KERN_INFO"[CSI_ERR][MT9P031]"x,##arg)
#define csi_dev_print(x,arg...) printk(KERN_INFO"[CSI][MT9P031]"x,##arg)

#define MCLK (24*1000*1000)

#define VREF_POL	CSI_HIGH
#define HREF_POL	CSI_HIGH
#define CLK_POL		CSI_RISING
#define IO_CFG		0						//0 for csi0

//define the voltage level of control signal
#define CSI_STBY_ON			1
#define CSI_STBY_OFF 		0
#define CSI_RST_ON			0
#define CSI_RST_OFF			1
#define CSI_PWR_ON			1
#define CSI_PWR_OFF			0

#define MIRROR 0	//0 = normal, 1 = mirror
#define FLIP   0	//0 = normal, 1 = flip

#define V4L2_IDENT_SENSOR 0x0330

#define REG_TERM 0xff
#define VAL_TERM 0xff


#define REG_ADDR_STEP 2
#define REG_DATA_STEP 2
#define REG_STEP 			(REG_ADDR_STEP+REG_DATA_STEP)


/*
 * Basic window sizes.  These probably belong somewhere more globally
 * useful.
 */

#define HD_WIDTH	2280//1920	//2304  //UXGA_WIDTH
#define HD_HEIGHT	1080  	//UXGA_HEIGHT

/************************************************************************
		Register Address
************************************************************************/
#define REG_MT9P031_CHIP_VERSION		0x00
#define REG_MT9P031_ROWSTART			0x01
#define REG_MT9P031_COLSTART			0x02
#define REG_MT9P031_HEIGHT			0x03
#define REG_MT9P031_WIDTH			0x04
#define REG_MT9P031_HBLANK			0x05
#define REG_MT9P031_VBLANK			0x06
#define REG_MT9P031_OUT_CTRL			0x07
#define REG_MT9P031_SHUTTER_WIDTH_U		0x08
#define REG_MT9P031_SHUTTER_WIDTH_L		0x09
#define REG_MT9P031_PCLK_CTRL			0x0a
#define REG_MT9P031_RESTART			0x0b
#define REG_MT9P031_SHUTTER_DELAY		0x0c
#define REG_MT9P031_RESET			0x0d
#define REG_MT9P031_PLL_CTRL			0x10
#define REG_MT9P031_PLL_CONF1			0x11
#define REG_MT9P031_PLL_CONF2			0x12
#define REG_MT9P031_READ_MODE1			0x1e
#define REG_MT9P031_READ_MODE2			0x20
#define REG_MT9P031_ROW_ADDR_MODE		0x22
#define REG_MT9P031_COL_ADDR_MODE		0x23
#define REG_MT9P031_GREEN_1_GAIN		0x2b
#define REG_MT9P031_BLUE_GAIN			0x2c
#define REG_MT9P031_RED_GAIN			0x2d
#define REG_MT9P031_GREEN_2_GAIN		0x2e
#define REG_MT9P031_GLOBAL_GAIN			0x35
#define REG_MT9P031_CHIP_VERSION_ALT	        0x0FF


/*
 * Our nominal (default) frame rate.
 */
#define SENSOR_FRAME_RATE 30 

/*
 * The ar0330 i2c address
 */
#define I2C_ADDR 0x20 //(0x78 for write,0x79 for read)

/* Registers */


/*
 * Information we maintain about a known sensor.
 */
struct sensor_format_struct;  /* coming later */
__csi_subdev_info_t ccm_info_con = 
{
	.mclk 	= MCLK,
	.vref 	= VREF_POL,
	.href 	= HREF_POL,
	.clock	= CLK_POL,
	.iocfg	= IO_CFG,
};

struct sensor_info {
	struct v4l2_subdev sd;
	struct sensor_format_struct *fmt;  /* Current format */
	__csi_subdev_info_t *ccm_info;
	int	width;
	int	height;
	int brightness;
	int	contrast;
	int saturation;
	int hue;
	int hflip;
	int vflip;
	int gain;
	int autogain;
	int exp;
	enum v4l2_exposure_auto_type autoexp;
	int autowb;
	enum v4l2_whiteblance wb;
	enum v4l2_colorfx clrfx;
	enum v4l2_flash_mode flash_mode;
	u8 clkrc;			/* Clock divider value */
};

static inline struct sensor_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sensor_info, sd);
}

struct regval_list {
	unsigned char reg_num[REG_ADDR_STEP];
	unsigned char value[REG_DATA_STEP];
};

struct regval {
	u16 reg_num;
	u16 value;
};

struct mt9p031_format_params {
	int width;
	int height;
	int row_start;
	int col_start;
	int row_size;
	int col_size;
	int hblank;
	int vblank;
	int integ_time;
	int row_addr_mode;
	int col_addr_mode;
	int read_mode_2_config;
	int shutter_width_hi;
	int shutter_delay;
	int row_bin;
	int col_bin;
};

enum mt9p031_image_size {
	VGA_BIN_30FPS,
	HDV_720P_30FPS,
	//HDV_720P_60FPS,
	//HDV_720P_60FPS_LVB,
	HDV_1080P_30FPS,
	MT9P031_THREE_MP,
	MT9P031_2M7P,
	MT9P031_FIVE_MP,
};

const struct mt9p031_format_params mt9p031_supported_formats[] = {
	{ 640, 480, 64, 24, 1919, 2559, 0, 0, 0x0296,  0x0033, 0x0033, 0x0060, 0, 0, 3, 3 },  // VGA_BIN_30FPS
	{ 1280, 720, 64, 24, 1439, 2559, 0, 0, 0x0296, 0x0011, 0x0011, 0x0060, 0, 0, 1, 1 },  // 720P_HD_30FPS
	//	{ 1280, 720, 0x0040, 0x0018, 0x059F, 0x09FF, 0, 0, 0x0296, 0x0011, 0x0011, 0x0060, 0, 0, 1, 1 },  // 720P_HD_60FPS
	//	{ 1280, 720, 0x0040, 0x0018, 0x059F, 0x09FF, 0, 0x02D0, 0x0296, 0x0011, 0x0011, 0x0060, 0, 0, 1, 1 },  // 720P_HD_60FPS_LVB
	{ 1920, 1080, 431, 335, 1079, 1919, 0, 0x0037, 0x01AC, 0, 0, 0x0040, 0, 0, 0, 0 },	// 1080P_30FPS
	{ 2048, 1536, 431, 335, 1535, 2047, 0, 0x0037, 0x01AC, 0, 0, 0x0040, 0, 0, 0, 0 },	// 3MP CAPTURE
	//	{ 2560, 1080, 486, 32, 1079, 2559, 0, 0x0008, 0x03C0, 0, 0, 0x0040, 0, 0, 0, 0 },	// 2M7P CAPTURE
	{ 2280, 1080, 431, 15, 1079, 2279, 0, 0x0008, 720, 0, 0, 0x0040, 0, 0, 0, 0},
	{ 2560, 1080, 431, 15, 1079, 2559, 0, 0x0008, 720, 0, 0, 0x0040, 0, 0, 0, 0 },	// 2M7P CAPTURE
	{ 2592, 1944, 431, 335, 1943, 2591, 0, 0x0037, 0x01AC, 0, 0, 0x0040, 0, 0, 0, 0 },	// 5MP CAPTURE
};

static struct regval sensor_default_regs[] = {

//[Demo initialization]
			  
{{0x000D}, {0x0001}}, 	// RESET_REG
{{0xffff}, {0x0032}},	//delay=50		  
{{0x000D}, {0x0000}}, 	// RESET_REG

{{0x0010}, {0x0051}},		//PLL Control1 = 81
{{0x0011}, {0x2503}},		//PLL Config1 = 9475
{{0x0012}, {0x0002}},		//PLL Config2 = 2
{{0xffff}, {0x0001}},	//DELAY=1
{{0x0010}, {0x0053}},		//PLL Control2 = 83
{{0x0007}, {0x1F8E}},		//Control Mode = 8078
{{0xffff}, {0x00C8}},	//DELAY=200


//[Timing_settings]
{{0x002B}, {0x0008}},		//(3) GREEN1_GAIN_REG = 8
{{0x002C}, {0x0008}},		//(3) BLUE_GAIN_REG = 8
{{0x002D}, {0x0008}},		//(3) RED_GAIN_REG = 8
{{0x002E}, {0x0008}},		//(3) GREEN2_GAIN_REG = 8
{{0x0001}, {0x01B0}},		//Row Start = 432§Ô§Ú1944-1080)/2
{{0x0002}, {0x0010}},		//Column Start = 16(2592-2560)/2
{{0x0003}, {0x0437}},		//Row Size = 1079
{{0x0004}, {0x077F}},		//Column Size = 2559
{{0x0005}, {0x0030}},		//Horz. Blank = 289
{{0x0006}, {0x0030}},		//Vert. Blank = 104
{{0x001E}, {0x0006}},		//Read Mode 1 = 4006
{{0x0020}, {0x0040}},		//Read Mode 2 = 64
{{0x0022}, {0x0000}},		//Row Mode = 0
{{0x0023}, {0x0000}},		//Column Mode = 0
{{0x0008}, {0x0000}},		//Shut. Wid. Upper = 0
{{0x0009}, {0x0300}},		//Integration Time = 1183
{{0x000C}, {0x0000}},		//Shutter Delay = 0

//REG = 0x0035, 0x000A // Recommended minimum gain is 1.25
//REG = 0x0049, 0x0000 	// 0x00A8 adding BLC
//REG = 0x005B, 0x0007 	// 0x0007 sampling size BLC
 
{{0x004F}, {0x0011}}, 	// RESERVED_CORE_4F
{{0x0029}, {0x0481}}, 	// RESERVED_CORE_29
{{0x003E}, {0x0007}}, 	// RESERVED_CORE_3E
{{0x003F}, {0x0007}}, 	// RESERVED_CORE_3F
{{0x0041}, {0x0003}}, 	// RESERVED_CORE_41
{{0x0048}, {0x0018}}, 	// RESERVED_CORE_48
{{0x0057}, {0x0002}}, 	// RESERVED_CORE_57
{{0x002A}, {0x7F72}}, 	// RESERVED_CORE_2A
{{0x001E}, {0x0006}}, 	// RESERVED_CORE_1E

{{0x0070}, {0x0079}}, 	// RESERVED_CORE_70
{{0x0071}, {0x7800}}, 	// RESERVED_CORE_71
{{0x0072}, {0x7800}}, 	// RESERVED_CORE_72
{{0x0073}, {0x0300}}, 	// RESERVED_CORE_73
{{0x0074}, {0x0300}}, 	// RESERVED_CORE_74
{{0x0075}, {0x3C00}}, 	// RESERVED_CORE_75
{{0x0076}, {0x4E3D}}, 	// RESERVED_CORE_76
{{0x0077}, {0x4E3D}}, 	// RESERVED_CORE_77
{{0x0078}, {0x774F}}, 	// RESERVED_CORE_78
{{0x0079}, {0x7900}}, 	// RESERVED_CORE_79
{{0x007A}, {0x7900}}, 	// RESERVED_CORE_7A
{{0x007B}, {0x7800}}, 	// RESERVED_CORE_7B
{{0x007C}, {0x7800}}, 	// RESERVED_CORE_7C
{{0x007E}, {0x7800}}, 	// RESERVED_CORE_7E
{{0x007F}, {0x7800}}, 	// RESERVED_CORE_7F
{{0x0006}, {0x0000}},		//Reserved
{{0x0029}, {0x0481}},		//Reserved
{{0x003e}, {0x0087}},		//Reserved
{{0x003f}, {0x0007}},		//Reserved
{{0x0041}, {0x0003}},		//Reserved
{{0x0048}, {0x0018}},		//Reserved
{{0x005f}, {0x1c16}},		//Reserved
{{0x0057}, {0x0002}},		//Reserved
{{0x004F}, {0x0011}},		//Reserved

};

static struct regval_list sensor_uxga_regs[] = {
  //NULL
};

static struct regval_list sensor_vga_regs[] = {
//NULL
};

/*
 * The white balance settings
 * Here only tune the R G B channel gain. 
 * The white balance enalbe bit is modified in sensor_s_autowb and sensor_s_wb
 */
static struct regval_list sensor_wb_auto_regs[] = {
//NULL
};

static struct regval_list sensor_wb_cloud_regs[] = {
//NULL
};

static struct regval_list sensor_wb_daylight_regs[] = {
	//tai yang guang
//NULL
};

static struct regval_list sensor_wb_incandescence_regs[] = {
	//bai re guang
//NULL
};

static struct regval_list sensor_wb_fluorescent_regs[] = {
	//ri guang deng
//NULL
};

static struct regval_list sensor_wb_tungsten_regs[] = {
	//wu si deng
//NULL
};

/*
 * The color effect settings
 */
static struct regval_list sensor_colorfx_none_regs[] = {
//NULL
};

static struct regval_list sensor_colorfx_bw_regs[] = {
//NULL
};

static struct regval_list sensor_colorfx_sepia_regs[] = {
//NULL
};

static struct regval_list sensor_colorfx_negative_regs[] = {
//NULL
};

static struct regval_list sensor_colorfx_emboss_regs[] = {
	//NULL
};

static struct regval_list sensor_colorfx_sketch_regs[] = {
	//NULL
};

static struct regval_list sensor_colorfx_sky_blue_regs[] = {
	//NULL
};

static struct regval_list sensor_colorfx_grass_green_regs[] = {
	//NULL
};

static struct regval_list sensor_colorfx_skin_whiten_regs[] = {
	//NULL
};

static struct regval_list sensor_colorfx_vivid_regs[] = {
	//NULL
};

/*
 * The brightness setttings
 */
static struct regval_list sensor_brightness_neg4_regs[] = {
	//NULL
};

static struct regval_list sensor_brightness_neg3_regs[] = {
	//NULL
};

static struct regval_list sensor_brightness_neg2_regs[] = {
	//NULL
};

static struct regval_list sensor_brightness_neg1_regs[] = {
	//NULL
};

static struct regval_list sensor_brightness_zero_regs[] = {
	//NULL
};

static struct regval_list sensor_brightness_pos1_regs[] = {
	//NULL
};

static struct regval_list sensor_brightness_pos2_regs[] = {
	//NULL
};

static struct regval_list sensor_brightness_pos3_regs[] = {
	//NULL
};

static struct regval_list sensor_brightness_pos4_regs[] = {
	//NULL
};

/*
 * The contrast setttings
 */
static struct regval_list sensor_contrast_neg4_regs[] = {
	//NULL
};

static struct regval_list sensor_contrast_neg3_regs[] = {
	//NULL
};

static struct regval_list sensor_contrast_neg2_regs[] = {
	//NULL
};

static struct regval_list sensor_contrast_neg1_regs[] = {
	//NULL
};

static struct regval_list sensor_contrast_zero_regs[] = {
	//NULL
};

static struct regval_list sensor_contrast_pos1_regs[] = {
	//NULL
};

static struct regval_list sensor_contrast_pos2_regs[] = {
	//NULL
};

static struct regval_list sensor_contrast_pos3_regs[] = {
	//NULL
};

static struct regval_list sensor_contrast_pos4_regs[] = {
	//NULL
};

/*
 * The saturation setttings
 */
static struct regval_list sensor_saturation_neg4_regs[] = {
	//NULL
};

static struct regval_list sensor_saturation_neg3_regs[] = {
	//NULL
};

static struct regval_list sensor_saturation_neg2_regs[] = {
	//NULL
};

static struct regval_list sensor_saturation_neg1_regs[] = {
	//NULL
};

static struct regval_list sensor_saturation_zero_regs[] = {
	//NULL
};

static struct regval_list sensor_saturation_pos1_regs[] = {
	//NULL
};

static struct regval_list sensor_saturation_pos2_regs[] = {
	//NULL
};

static struct regval_list sensor_saturation_pos3_regs[] = {
	//NULL
};

static struct regval_list sensor_saturation_pos4_regs[] = {
	//NULL
};
#if 0
/*
 * The exposure target setttings
 */
static struct regval_list sensor_ev_pos10_regs[] = {
	{{0x30,0x12},{0x06,0x00}},
};

static struct regval_list sensor_ev_pos9_regs[] = {
	{{0x30,0x12},{0x05,0xc0}},
};

static struct regval_list sensor_ev_pos8_regs[] = {
	{{0x30,0x12},{0x05,0x80}},
};

static struct regval_list sensor_ev_pos7_regs[] = {
	{{0x30,0x12},{0x05,0x40}},
};

static struct regval_list sensor_ev_pos6_regs[] = {
	{{0x30,0x12},{0x05,0x00}},
};

static struct regval_list sensor_ev_pos5_regs[] = {
	{{0x30,0x12},{0x04,0xc0}},
};

static struct regval_list sensor_ev_pos4_regs[] = {
	{{0x30,0x12},{0x04,0x80}},
};

static struct regval_list sensor_ev_pos3_regs[] = {
	{{0x30,0x12},{0x04,0x40}},
};

static struct regval_list sensor_ev_pos2_regs[] = {
	{{0x30,0x12},{0x04,0x00}},
};

static struct regval_list sensor_ev_pos1_regs[] = {
	{{0x30,0x12},{0x03,0xc0}},
};

static struct regval_list sensor_ev_zero_regs[] = {
	{{0x30,0x12},{0x03,0x80}},
};

static struct regval_list sensor_ev_neg1_regs[] = {
	{{0x30,0x12},{0x03,0x40}},
};

static struct regval_list sensor_ev_neg2_regs[] = {
	{{0x30,0x12},{0x03,0x00}},
};

static struct regval_list sensor_ev_neg3_regs[] = {
	{{0x30,0x12},{0x02,0xc0}},
};

static struct regval_list sensor_ev_neg4_regs[] = {
	{{0x30,0x12},{0x02,0x80}},
};

static struct regval_list sensor_ev_neg5_regs[] = {
	{{0x30,0x12},{0x02,0x40}},
};

static struct regval_list sensor_ev_neg6_regs[] = {
	{{0x30,0x12},{0x02,0x00}},
};

static struct regval_list sensor_ev_neg7_regs[] = {
	{{0x30,0x12},{0x02,0xc0}},
};

static struct regval_list sensor_ev_neg8_regs[] = {
	{{0x30,0x12},{0x01,0x80}},
};

static struct regval_list sensor_ev_neg9_regs[] = {
	{{0x30,0x12},{0x01,0x40}},
};

static struct regval_list sensor_ev_neg10_regs[] = {
	{{0x30,0x12},{0x01,0x00}},
};
#else
/*
 * The exposure target setttings
 */
static struct regval_list sensor_ev_pos5_regs[] = {
	{{0x30,0x12},{0x05,0x00}},
};

static struct regval_list sensor_ev_pos4_regs[] = {
	{{0x30,0x12},{0x04,0xA0}},
};

static struct regval_list sensor_ev_pos3_regs[] = {
	{{0x30,0x12},{0x04,0x40}},
};

static struct regval_list sensor_ev_pos2_regs[] = {
	{{0x30,0x12},{0x03,0xE0}},
};

static struct regval_list sensor_ev_pos1_regs[] = {
	{{0x30,0x12},{0x03,0x80}},
};

static struct regval_list sensor_ev_zero_regs[] = {
	{{0x30,0x12},{0x03,0x20}},
};

static struct regval_list sensor_ev_neg1_regs[] = {
	{{0x30,0x12},{0x02,0xC0}},
};

static struct regval_list sensor_ev_neg2_regs[] = {
	{{0x30,0x12},{0x02,0x60}},
};

static struct regval_list sensor_ev_neg3_regs[] = {
	{{0x30,0x12},{0x02,0x00}},
};

static struct regval_list sensor_ev_neg4_regs[] = {
	{{0x30,0x12},{0x01,0xA0}},
};

static struct regval_list sensor_ev_neg5_regs[] = {
	{{0x30,0x12},{0x01,0x40}},
};

#endif
/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 * 
 */


static struct regval_list sensor_fmt_yuv422_yuyv[] = {
//NULL
	
};


static struct regval_list sensor_fmt_yuv422_yvyu[] = {
//NULL
};

static struct regval_list sensor_fmt_yuv422_vyuy[] = {
//NULL
};

static struct regval_list sensor_fmt_yuv422_uyvy[] = {
//NULL
};

static struct regval_list sensor_fmt_raw[] = {
	//NULL
};



/*
 * Low-level register I/O.
 *
 */


/*
 * On most platforms, we'd rather do straight i2c I/O.
 */
static int sensor_read(struct v4l2_subdev *sd, unsigned char *reg,
		unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 data[REG_STEP];
	struct i2c_msg msg;
	int ret,i;
	
	for(i = 0; i < REG_ADDR_STEP; i++)
		data[i] = reg[i];
	
	data[REG_ADDR_STEP] = 0xff;
	/*
	 * Send out the register address...
	 */
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = REG_ADDR_STEP;
	msg.buf = data;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		csi_dev_err("Error %d on register write\n", ret);
		return ret;
	}
	/*
	 * ...then read back the result.
	 */
	
	msg.flags = I2C_M_RD;
	msg.len = REG_DATA_STEP;
	msg.buf = &data[REG_ADDR_STEP];
	
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0) {
		for(i = 0; i < REG_DATA_STEP; i++)
			value[i] = data[i+REG_ADDR_STEP];
		ret = 0;
	}
	else {
		csi_dev_err("Error %d on register read\n", ret);
	}
	return ret;
}


static int sensor_write(struct v4l2_subdev *sd, unsigned char *reg,
		unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg;
	unsigned char data[REG_STEP];
	int ret,i;
	
	for(i = 0; i < REG_ADDR_STEP; i++)
			data[i] = reg[i];
	for(i = REG_ADDR_STEP; i < REG_STEP; i++)
			data[i] = value[i-REG_ADDR_STEP];
		
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = REG_STEP;
	msg.buf = data;

	
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0) {
		ret = 0;
	}
	else if (ret < 0) {
		csi_dev_err("sensor_write error!\n");
	}
	return ret;
}

static int mt9p031_reg_write(const struct i2c_client *client,u16 command, u16 data)
{	
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	// 8-bit/ byte addressable register
	buf[0] = command & 0xff;
	data = swab16(data);
	memcpy(buf + 1, &data,    2);
	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = 3;
	msg.buf   = buf;
	/*	 
	* i2c_transfer return message length,
	* but this function should return 0 if correct case
	*/	
	ret = i2c_transfer(client->adapter, &msg, 1);
	printk(KERN_ERR "%s:ret = %d\n",__func__,ret);
	if (ret >= 0)
		ret = 0;
	
	return ret;
}

static int mt9p031_reg_read(const struct i2c_client *client, u16 command, u16 *val)
{	
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	// 8-bit/ byte addressable register
	buf[0] = command & 0xff;
	msg[0].addr  = client->addr;
	msg[0].flags = 0;
	msg[0].len   = 1;
	msg[0].buf   = buf ;
	ret = i2c_transfer(client->adapter, &msg[0], 1);
	if(ret >= 0) {
		msg[1].addr  = client->addr;
		msg[1].flags = I2C_M_RD;//1
		msg[1].len   = 2;
		msg[1].buf   = buf;
		ret = i2c_transfer(client->adapter, &msg[1], 1);
	}	

	/*
	* if return value of this function is < 0,
	* it mean error.
	* else, under 16bit is valid data.
	*/
	if(ret >= 0) {
		*val = 0;
		*val = buf[1] + (buf[0] << 8);
		return 0;
	}
	printk(KERN_ERR "read from offset 0x%x error %d", command, ret);
	return ret;
}

/*
 * Write a list of register settings;
 */
static int sensor_write_array(struct v4l2_subdev *sd, struct regval_list *vals , uint size)
{
	int i,ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	
	if (size == 0)
		return -EINVAL;
	
	for(i = 0; i < size ; i++)
	{
		if(vals->reg_num[0] == 0xff)
			mdelay(vals->value[1] * 256 + vals->value[0]);
		else {	
			ret = sensor_write(sd, vals->reg_num, vals->value);
			//ret = mt9p031_reg_write(client,vals->reg_num,vals->value);
			if (ret < 0)
				{
					csi_dev_err("sensor_write_err!\n");
					return ret;
			}
		}
		vals++;
	}

	return 0;
}

static int mt9p031_write_array(struct v4l2_subdev *sd, struct regval *vals , uint size)
{
	int i,ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	
	if (size == 0)
		return -EINVAL;
	
	for(i = 0; i < size ; i++)
	{
		if(vals->reg_num == 0xffff)
			mdelay(vals->value);
		else {	
			ret = mt9p031_reg_write(client,vals->reg_num,vals->value);
			if (ret < 0)
				{
					csi_dev_err("sensor_write_err!\n");
					return ret;
			}
		}
		vals++;
	}

	return 0;
}

/*
 * CSI GPIO control
 */
static void csi_gpio_write(struct v4l2_subdev *sd, struct gpio_config *gpio, int level)
{
//	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
  if(gpio->gpio==GPIO_INDEX_INVALID)
  {
    csi_dev_dbg("invalid gpio\n");
    return;
  }
  
	if(gpio->mul_sel==1)
	{
	  gpio_direction_output(gpio->gpio, level);
	  gpio->data=level;
	} else {
	  csi_dev_dbg("gpio is not in output function\n");
	}
}

static void csi_gpio_set_status(struct v4l2_subdev *sd, struct gpio_config *gpio, int status)
{
//	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
	
	if(1 == status) {  /* output */
		if(0 != gpio_direction_output(gpio->gpio, gpio->data))
			csi_dev_dbg("gpio_direction_output failed\n");
	} else if(0 == status) {  /* input */
	  if(0 != gpio_direction_input(gpio->gpio) )
	    csi_dev_dbg("gpio_direction_input failed\n");
	}
	gpio->mul_sel=status;
}


/*
 * Stuff that knows about the sensor.
 */
 
static int sensor_power(struct v4l2_subdev *sd, int on)
{
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	
	csi_dev_dbg("sensor_power on=0x%02x\n",on);
  //make sure that no device can access i2c bus during sensor initial or power down
  //when using i2c_lock_adpater function, the following codes must not access i2c bus before calling i2c_unlock_adapter
  i2c_lock_adapter(client->adapter);

  //insure that clk_disable() and clk_enable() are called in pair 
  //when calling CSI_SUBDEV_STBY_ON/OFF and CSI_SUBDEV_PWR_ON/OFF  
  switch(on)
	{
		case CSI_SUBDEV_STBY_ON:
			csi_dev_dbg("CSI_SUBDEV_STBY_ON\n");
			//break;
			//reset off io
			//csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			//standby on io
			//csi_gpio_write(sd,&dev->standby_io,CSI_STBY_ON);
			mdelay(20);
			csi_gpio_write(sd,&dev->standby_io,CSI_STBY_OFF);
			mdelay(20);
			//csi_gpio_write(sd,&dev->standby_io,CSI_STBY_ON);
			mdelay(20);
			//inactive mclk after stadby in
			clk_disable(dev->csi_module_clk);
			//reset on io
			//csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(10);
			break;
		case CSI_SUBDEV_STBY_OFF:
			csi_dev_dbg("CSI_SUBDEV_STBY_OFF\n");
			//break;
			//active mclk before stadby out
			clk_enable(dev->csi_module_clk);
			mdelay(10);
			//standby off io
			csi_gpio_write(sd,&dev->standby_io,CSI_STBY_ON);
			mdelay(10);
			//reset off io
			//csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			//csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(20);
			//csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(20);
			break;
		case CSI_SUBDEV_PWR_ON:
			csi_dev_dbg("CSI_SUBDEV_PWR_ON\n");
			//power on reset
			csi_gpio_set_status(sd,&dev->standby_io,1);//set the gpio to output
			csi_gpio_set_status(sd,&dev->reset_io,1);//set the gpio to output
			//csi_gpio_write(sd,&dev->standby_io,CSI_STBY_ON);
			//reset on io
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(10);
			//active mclk before power on
			clk_enable(dev->csi_module_clk);
			mdelay(10);
			//power supply
			csi_gpio_write(sd,&dev->power_io,CSI_PWR_ON);
			mdelay(10);
			if(dev->dvdd) {
				regulator_enable(dev->dvdd);
				mdelay(10);
			}
			if(dev->avdd) {
				regulator_enable(dev->avdd);
				mdelay(10);
			}
			if(dev->iovdd) {
				regulator_enable(dev->iovdd);
				mdelay(10);
			}
			//standby off io
			//csi_gpio_write(sd,&dev->standby_io,CSI_STBY_OFF);
			mdelay(10);
			//reset after power on
			//csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			//csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(20);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(20);
			break;
		case CSI_SUBDEV_PWR_OFF:
			csi_dev_dbg("CSI_SUBDEV_PWR_OFF\n");
			//standby and reset io
			//csi_gpio_write(sd,&dev->standby_io,CSI_STBY_ON);
			mdelay(20);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(20);
			//power supply off
			if(dev->iovdd) {
				regulator_disable(dev->iovdd);
				mdelay(10);
			}
			if(dev->avdd) {
				regulator_disable(dev->avdd);
				mdelay(10);
			}
			if(dev->dvdd) {
				regulator_disable(dev->dvdd);
				mdelay(10);	
			}
			csi_gpio_write(sd,&dev->power_io,CSI_PWR_OFF);
			mdelay(10);
			//inactive mclk after power off
			clk_disable(dev->csi_module_clk);
			//set the io to hi-z
			csi_gpio_set_status(sd,&dev->reset_io,0);//set the gpio to input
			csi_gpio_set_status(sd,&dev->standby_io,0);//set the gpio to input
			break;
		default:
			return -EINVAL;
	}		

	//remember to unlock i2c adapter, so the device can access the i2c bus again
	i2c_unlock_adapter(client->adapter);	
	return 0;
}
 
static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
	
	csi_dev_dbg("sensor_reset val =0x%02x \n",val);

	switch(val)
	{
		case CSI_SUBDEV_RST_OFF:
			csi_dev_dbg("CSI_SUBDEV_RST_OFF\n");
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			break;
		case CSI_SUBDEV_RST_ON:
			csi_dev_dbg("CSI_SUBDEV_RST_ON\n");
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(10);
			break;
		case CSI_SUBDEV_RST_PUL:
			csi_dev_dbg("CSI_SUBDEV_RST_PUL\n");
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(20);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(20);
			break;
		default:
			return -EINVAL;
	}
		
	return 0;
}

static int sensor_detect(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	s32 data = 0x0;


	int ret;
	/*struct regval_list regs;
	
	regs.value[0] = 0x00; 
	regs.value[1] = 0x00;
	
	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x00;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_detect!\n");
		return ret;
	}

	if( (regs.value[0] != 0x26) || (regs.value[1] != 0x04)  )//ar0330 sensor id=0x2604
		return -ENODEV;*/

	/* Read out the chip version register */
	ret = mt9p031_reg_read(client, 0x00,&data);
	
	dev_err(&client->dev, "MT9P031 is found, version 0x%04x\n", data);
	if (data != 0x1801) {
		dev_err(&client->dev, "MT9P031 not detected, wrong version "
			"0x%04x\n", data);
		return -ENODEV;
	}

	
	return 0;
}

// PCLK = (CLOCK_IN / N) * M / P1
#if 0  // 96M
#define MT9P031_PLL_M	24
#define MT9P031_PLL_N	2
#define MT9P031_PLL_P1	3
#elif 0  // 66M
#define MT9P031_PLL_M	88
#define MT9P031_PLL_N	4
#define MT9P031_PLL_P1	8
#else  // 48M
#define MT9P031_PLL_M	24
#define MT9P031_PLL_N	2
#define MT9P031_PLL_P1	6
#endif

static enum mt9p031_image_size mt9p031_calc_size(unsigned int width)
{
	enum mt9p031_image_size isize;
	for (isize = VGA_BIN_30FPS; isize <= MT9P031_FIVE_MP; isize++)
	{
		if (mt9p031_supported_formats[isize].width >= width)
		{
			return isize;
		}
	}
	return MT9P031_FIVE_MP;
}


/** * mt9p031_set_params - sets register settings according to resolution
* @client: pointer to standard i2c client
* @width: width as queried by ioctl
* @height: height as queried by ioctl
*/
static int mt9p031_set_params(struct i2c_client *client, u32 width, u32 height)
{	
	//struct mt9p031_priv *priv = i2c_get_clientdata(client);
	//struct v4l2_pix_format *pix = &priv->pix;
	int ret;
	enum mt9p031_image_size i;
	i = mt9p031_calc_size(2280);
	//priv->pix.width = mt9p031_supported_formats[i].width;
	//priv->pix.height = mt9p031_supported_formats[i].height;
	ret = mt9p031_reg_write(client, REG_MT9P031_ROWSTART,mt9p031_supported_formats[i].row_start);// ROW_WINDOW_START_REG
	ret |= mt9p031_reg_write(client, REG_MT9P031_COLSTART,mt9p031_supported_formats[i].col_start);// COL_WINDOW_START_REG
	ret |= mt9p031_reg_write(client, REG_MT9P031_HEIGHT,mt9p031_supported_formats[i].row_size);// ROW_WINDOW_SIZE_REG=1439
	ret |= mt9p031_reg_write(client, REG_MT9P031_WIDTH,mt9p031_supported_formats[i].col_size);// COL_WINDOW_SIZE_REG=2559
	ret |= mt9p031_reg_write(client, REG_MT9P031_HBLANK,mt9p031_supported_formats[i].hblank);// HORZ_BLANK=0
	ret |= mt9p031_reg_write(client, REG_MT9P031_VBLANK,mt9p031_supported_formats[i].vblank);// VERT_BLANK_REG=720
	ret |= mt9p031_reg_write(client, REG_MT9P031_SHUTTER_WIDTH_L,0x0400);// SHUTTER_WIDTH_LOW (INTEG_TIME_REG = 1024)
	ret |= mt9p031_reg_write(client, REG_MT9P031_ROW_ADDR_MODE,mt9p031_supported_formats[i].row_addr_mode);// ROW_MODE, ROW_SKIP=1, ROW_BIN=1
	ret |= mt9p031_reg_write(client, REG_MT9P031_COL_ADDR_MODE,mt9p031_supported_formats[i].col_addr_mode);// COL_MODE, COL_SKIP=1, COL_BIN=1
	ret |= mt9p031_reg_write(client, REG_MT9P031_READ_MODE2,mt9p031_supported_formats[i].read_mode_2_config);// READ_MODE_2, COL_SUM
	ret |= mt9p031_reg_write(client, REG_MT9P031_SHUTTER_WIDTH_U,mt9p031_supported_formats[i].shutter_width_hi);// SHUTTER_WIDTH_HI
	ret |= mt9p031_reg_write(client, REG_MT9P031_SHUTTER_WIDTH_L,mt9p031_supported_formats[i].integ_time);// SHUTTER_WIDTH_LOW (INTEG_TIME_REG)
	ret |= mt9p031_reg_write(client, REG_MT9P031_SHUTTER_DELAY,mt9p031_supported_formats[i].shutter_delay);// SHUTTER_DELAY_REG
	return ret;
}

static u16 output_control = 0x1F82;

static int mt9p031_set_output_control(struct v4l2_subdev *sd, u16 clear,u16 set)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 value = (output_control & ~clear) | set;
	int ret;
	ret = mt9p031_reg_write(client, 0x07, value);
	if (ret < 0)
		return ret;
	output_control = value;
	return 0;
}


static int mt9p031_init_camera(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	
	ret = mt9p031_reg_write(client, 0x0d, 0x0001);		// High
	//ret = mt9p031_set_output_control(sd, 0,2);
	mdelay(50);
	ret |= mt9p031_reg_write(client, 0x0d, 0x0000);	// Low
	mdelay(50);

	ret |= mt9p031_reg_write(client, 0x10, 0x0051);		// PLL_CTRL; power up pll
//	ret |= mt9p031_reg_write(client, REG_MT9P031_PLL_CONF1, 0x1801);	// PLL_CONFIG_1: m=24, n=1
//	ret |= mt9p031_reg_write(client, REG_MT9P031_PLL_CONF2, 0x0002);	// PLL_CONFIG_2: p1=2, p2=0
	ret |= mt9p031_reg_write(client, 0x11, 0x2503);		// PLL_CONFIG_1: m=25, n=4
	ret |= mt9p031_reg_write(client, 0x12, 0x0002);		// PLL_CONFIG_2: p1=3
	mdelay(10);  //wait 10 ms for VCO to lock
	ret |= mt9p031_reg_write(client, 0x10, 0x0053);		// PLL_CONTROL; use PLL
	ret |= mt9p031_reg_write(client, 0x07, 0x1F8E);
	mdelay(200);

	ret |= mt9p031_reg_write(client, 0x002B, 0x0008);		// RESERVED_CORE_70
	ret |= mt9p031_reg_write(client, 0x002C, 0x0008);		// RESERVED_CORE_71
	ret |= mt9p031_reg_write(client, 0x002D, 0x0008);		// RESERVED_CORE_72
	ret |= mt9p031_reg_write(client, 0x002E, 0x0008);		// RESERVED_CORE_73
	ret |= mt9p031_reg_write(client, 0x0001, 0x01B0);		// RESERVED_CORE_74
	ret |= mt9p031_reg_write(client, 0x0002, 0x0010);		// RESERVED_CORE_75
	ret |= mt9p031_reg_write(client, 0x0003, 0x0437);		// RESERVED_CORE_76
	ret |= mt9p031_reg_write(client, 0x0004, 0x077F);		// RESERVED_CORE_77
	ret |= mt9p031_reg_write(client, 0x0005, 0x0030);		// RESERVED_CORE_78
	ret |= mt9p031_reg_write(client, 0x0006, 0x0030);		// RESERVED_CORE_79
	ret |= mt9p031_reg_write(client, 0x001E, 0x0006);		// RESERVED_CORE_7A
	ret |= mt9p031_reg_write(client, 0x0020, 0x0040);		// RESERVED_CORE_7B
	ret |= mt9p031_reg_write(client, 0x0022, 0x0000);		// RESERVED_CORE_7C
	ret |= mt9p031_reg_write(client, 0x0023, 0x0000);		// RESERVED_CORE_7E
	ret |= mt9p031_reg_write(client, 0x0008, 0x0000);		// RESERVED_CORE_7F
	ret |= mt9p031_reg_write(client, 0x0009, 0x0300);		// RESERVED_CORE_29
	ret |= mt9p031_reg_write(client, 0x000C, 0x0000);		// RESERVED_CORE_2A
	ret |= mt9p031_reg_write(client, 0x004F, 0x0011);		// RESERVED_CORE_3E
	ret |= mt9p031_reg_write(client, 0x0029, 0x0481);		// RESERVED_CORE_3F
	ret |= mt9p031_reg_write(client, 0x003E, 0x0007);		// RESERVED_CORE_41
	ret |= mt9p031_reg_write(client, 0x003F, 0x0007);		// RESERVED_CORE_48
	ret |= mt9p031_reg_write(client, 0x0041, 0x0003);		// CAL_THRESHOLD
	ret |= mt9p031_reg_write(client, 0x0048, 0x0018);		// RESERVED_CORE_57

	ret |= mt9p031_set_params(client,2280,1080);
	
	ret |= mt9p031_reg_write(client, 0x0057, 0x0002);		// RESERVED_CORE_78
	ret |= mt9p031_reg_write(client, 0x002A, 0x7F72);		// RESERVED_CORE_79
	ret |= mt9p031_reg_write(client, 0x001E, 0x0006);		// RESERVED_CORE_7A
	
	ret |= mt9p031_reg_write(client, 0x0070, 0x0079);		// RESERVED_CORE_70	
	ret |= mt9p031_reg_write(client, 0x0071, 0x7800);		// RESERVED_CORE_71
	ret |= mt9p031_reg_write(client, 0x0072, 0x7800);		// RESERVED_CORE_72
	ret |= mt9p031_reg_write(client, 0x0073, 0x0300);		// RESERVED_CORE_73
	ret |= mt9p031_reg_write(client, 0x0074, 0x0300);		// RESERVED_CORE_74
	ret |= mt9p031_reg_write(client, 0x0075, 0x3C00);		// RESERVED_CORE_75
	ret |= mt9p031_reg_write(client, 0x0076, 0x4E3D);		// RESERVED_CORE_76
	ret |= mt9p031_reg_write(client, 0x0077, 0x4E3D);		// RESERVED_CORE_77
	ret |= mt9p031_reg_write(client, 0x0078, 0x774F);		// RESERVED_CORE_78
	ret |= mt9p031_reg_write(client, 0x0079, 0x7900);		// RESERVED_CORE_79
	ret |= mt9p031_reg_write(client, 0x007A, 0x7900);		// RESERVED_CORE_7A
	ret |= mt9p031_reg_write(client, 0x007B, 0x7800);		// RESERVED_CORE_7B
	ret |= mt9p031_reg_write(client, 0x007C, 0x7800);		// RESERVED_CORE_7C
	ret |= mt9p031_reg_write(client, 0x007E, 0x7800);		// RESERVED_CORE_7E
	ret |= mt9p031_reg_write(client, 0x007F, 0x7800);		// RESERVED_CORE_7F
	
	ret |= mt9p031_reg_write(client, 0x0006, 0x0000);
	ret |= mt9p031_reg_write(client, 0x0029, 0x0481);		// RESERVED_CORE_29
	ret |= mt9p031_reg_write(client, 0x003E, 0x0087);		// RESERVED_CORE_3E
	ret |= mt9p031_reg_write(client, 0x003F, 0x0007);		// RESERVED_CORE_3F
	ret |= mt9p031_reg_write(client, 0x0041, 0x0003);		// RESERVED_CORE_41
	ret |= mt9p031_reg_write(client, 0x0048, 0x0018);		// RESERVED_CORE_48
	ret |= mt9p031_reg_write(client, 0x005F, 0x1C16);		// CAL_THRESHOLD
	ret |= mt9p031_reg_write(client, 0x0057, 0x0002);		// RESERVED_CORE_57
	
	ret |= mt9p031_reg_write(client, 0x004F, 0x0011);		// RESERVED_CORE_57

	return ret>= 0 ? 0 : -EIO;
}


static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	printk(KERN_ERR "sensor_init\n");
	/*Make sure it is a target sensor*/
	ret = sensor_detect(sd);
	if (ret) {
		csi_dev_err("chip found is not an target chip.\n");
		return ret;
	}
#if 0
	ret = mt9p031_init_camera(sd);
	if(ret!=0)
	{
		csi_dev_err("sensor_write_array fail\n");
	}
#else
	ret = mt9p031_write_array(sd, sensor_default_regs , ARRAY_SIZE(sensor_default_regs));
	if(ret!=0)
	{
		csi_dev_err("sensor_write_array fail\n");
	}
	
	ret |= mt9p031_set_params(client,2280,1080);
	if(ret!=0)
	{
		csi_dev_err("mt9p031_set_params fail\n");
	}
#endif
	return ret;
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret=0;
	
	switch(cmd){
		case CSI_SUBDEV_CMD_GET_INFO: 
		{
			struct sensor_info *info = to_state(sd);
			__csi_subdev_info_t *ccm_info = arg;
			
			csi_dev_dbg("CSI_SUBDEV_CMD_GET_INFO\n");
			ccm_info->mclk 	=	info->ccm_info->mclk ;
			ccm_info->vref 	=	info->ccm_info->vref ;
			ccm_info->href 	=	info->ccm_info->href ;
			ccm_info->clock	=	info->ccm_info->clock;
			ccm_info->iocfg	=	info->ccm_info->iocfg;
			csi_dev_dbg("ccm_info.mclk=%x\n ",info->ccm_info->mclk);
			csi_dev_dbg("ccm_info.vref=%x\n ",info->ccm_info->vref);
			csi_dev_dbg("ccm_info.href=%x\n ",info->ccm_info->href);
			csi_dev_dbg("ccm_info.clock=%x\n ",info->ccm_info->clock);
			csi_dev_dbg("ccm_info.iocfg=%x\n ",info->ccm_info->iocfg);
			break;
		}
		case CSI_SUBDEV_CMD_SET_INFO:
		{
			struct sensor_info *info = to_state(sd);
			__csi_subdev_info_t *ccm_info = arg;
			
			csi_dev_dbg("CSI_SUBDEV_CMD_SET_INFO\n");
			info->ccm_info->mclk 	=	ccm_info->mclk 	;
			info->ccm_info->vref 	=	ccm_info->vref 	;
			info->ccm_info->href 	=	ccm_info->href 	;
			info->ccm_info->clock	=	ccm_info->clock	;
			info->ccm_info->iocfg	=	ccm_info->iocfg	;
			csi_dev_dbg("ccm_info.mclk=%x\n ",info->ccm_info->mclk);
			csi_dev_dbg("ccm_info.vref=%x\n ",info->ccm_info->vref);
			csi_dev_dbg("ccm_info.href=%x\n ",info->ccm_info->href);
			csi_dev_dbg("ccm_info.clock=%x\n ",info->ccm_info->clock);
			csi_dev_dbg("ccm_info.iocfg=%x\n ",info->ccm_info->iocfg);
			break;
		}
		default:
			return -EINVAL;
	}		
		return ret;
}


/*
 * Store information about the video data format. 
 */
static struct sensor_format_struct {
	__u8 *desc;
	//__u32 pixelformat;
	enum v4l2_mbus_pixelcode mbus_code;//linux-3.0
	struct regval_list *regs;
	int	regs_size;
	int bpp;   /* Bytes per pixel */
} sensor_formats[] = {
	{
		.desc		= "Raw RGB Bayer",
		.mbus_code	= V4L2_MBUS_FMT_SBGGR8_1X8,//linux-3.0
		.regs 		= sensor_fmt_raw,
		.regs_size = ARRAY_SIZE(sensor_fmt_raw),
		.bpp		= 1
	},
	{
		.desc		= "YUYV 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_YUYV8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_yuyv,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_yuyv),
		.bpp		= 2,
	},
	{
		.desc		= "YVYU 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_YVYU8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_yvyu,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_yvyu),
		.bpp		= 2,
	},
	{
		.desc		= "UYVY 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_UYVY8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_uyvy,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_uyvy),
		.bpp		= 2,
	},
	{
		.desc		= "VYUY 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_VYUY8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_vyuy,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_vyuy),
		.bpp		= 2,
	},

};
#define N_FMTS ARRAY_SIZE(sensor_formats)


/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */


static struct sensor_win_size {
	int	width;
	int	height;
	int	hstart;		/* Start/stop values for the camera.  Note */
	int	hstop;		/* that they do not always make complete */
	int	vstart;		/* sense to humans, but evidently the sensor */
	int	vstop;		/* will do the right thing... */
	struct regval_list *regs; /* Regs to tweak */
	int regs_size;
	int (*set_size) (struct v4l2_subdev *sd);
/* h/vref stuff */
} sensor_win_sizes[] = {
	/* SXGA */
	{
		.width		= HD_WIDTH,
		.height		= HD_HEIGHT,
		.regs 		= sensor_uxga_regs,
		.regs_size	= ARRAY_SIZE(sensor_uxga_regs),
		.set_size		= NULL,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))

 
 
 
static int sensor_enum_fmt(struct v4l2_subdev *sd, unsigned index,
                 enum v4l2_mbus_pixelcode *code)//linux-3.0
{
//	struct sensor_format_struct *ofmt;

	if (index >= N_FMTS)//linux-3.0
		return -EINVAL;

	*code = sensor_formats[index].mbus_code;//linux-3.0
//	ofmt = sensor_formats + fmt->index;
//	fmt->flags = 0;
//	strcpy(fmt->description, ofmt->desc);
//	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}


static int sensor_try_fmt_internal(struct v4l2_subdev *sd,
		//struct v4l2_format *fmt,
		struct v4l2_mbus_framefmt *fmt,//linux-3.0
		struct sensor_format_struct **ret_fmt,
		struct sensor_win_size **ret_wsize)
{
	int index;
	struct sensor_win_size *wsize;
	enum mt9p031_image_size isize;
//	struct v4l2_pix_format *pix = &fmt->fmt.pix;//linux-3.0

	isize = mt9p031_calc_size(2280);

	csi_dev_dbg("sensor_try_fmt_internal,fmt->code:0x%x\n",fmt->code);
	for (index = 0; index < N_FMTS; index++)
		if (sensor_formats[index].mbus_code == fmt->code)//linux-3.0
			break;
	
	if (index >= N_FMTS) {
		/* default to first format */
		index = 0;
		fmt->code = sensor_formats[0].mbus_code;//linux-3.0
	}
	
	if (ret_fmt != NULL)
		*ret_fmt = sensor_formats + index;
		
	/*
	 * Fields: the sensor devices claim to be progressive.
	 */
	fmt->field = V4L2_FIELD_NONE;//linux-3.0
	
	
	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	for (wsize = sensor_win_sizes; wsize < sensor_win_sizes + N_WIN_SIZES;
	     wsize++)
		if (fmt->width >= wsize->width && fmt->height >= wsize->height)//linux-3.0
			break;
	
	if (wsize >= sensor_win_sizes + N_WIN_SIZES)
		wsize--;   /* Take the smallest one */
		
	if (ret_wsize != NULL)
	{
		csi_dev_dbg("ret_wsize is not null\n");
		*ret_wsize = wsize;
	}
	/*
	 * Note the size we'll actually handle.
	 */
	//fmt->width = wsize->width;//linux-3.0
	//fmt->height = wsize->height;//linux-3.0
	fmt->width = mt9p031_supported_formats[isize].width;
	fmt->height = mt9p031_supported_formats[isize].height;
	csi_dev_dbg("fmt->width:%d ,fmt->height:%d,size:%d\n",fmt->width,fmt->height,wsize->regs_size);
	//pix->bytesperline = pix->width*sensor_formats[index].bpp;//linux-3.0
	//pix->sizeimage = pix->height*pix->bytesperline;//linux-3.0
	
	return 0;
}

static int sensor_try_fmt(struct v4l2_subdev *sd, 
             struct v4l2_mbus_framefmt *fmt)//linux-3.0
{
	return sensor_try_fmt_internal(sd, fmt, NULL, NULL);
}

/*
 * Set a format.
 */
static int sensor_s_fmt(struct v4l2_subdev *sd, 
             struct v4l2_mbus_framefmt *fmt)//linux-3.0
{
	int ret;
	struct sensor_format_struct *sensor_fmt;
	struct sensor_win_size *wsize;
	struct sensor_info *info = to_state(sd);
	csi_dev_err("sensor_s_fmt\n");
	ret = sensor_try_fmt_internal(sd, fmt, &sensor_fmt, &wsize);
	if (ret)
		return ret;
	
	csi_dev_err("sensor_try_fmt_internal\n");
	//sensor_write_array(sd, sensor_fmt->regs , sensor_fmt->regs_size);
	csi_dev_err("sensor_write_array\n");
	ret = 0;
	//if (wsize->regs)
	//{
	//	ret = sensor_write_array(sd, wsize->regs , wsize->regs_size);
	//	if (ret < 0)
	//	{
	//		csi_dev_err("sensor_write_array fail wsize->regs_size is %d\n",wsize->regs_size);
	//		return ret;
	//	}
	//}
	//csi_dev_err("sensor_write_array2\n");
	//if (wsize->set_size)
	//{
	//	ret = wsize->set_size(sd);
	//	if (ret < 0)
	//		return ret;
	//}
	
	info->fmt = sensor_fmt;
	info->width = wsize->width;
	info->height = wsize->height;
	
	return 0;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int sensor_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	//struct sensor_info *info = to_state(sd);

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;
	cp->timeperframe.denominator = SENSOR_FRAME_RATE;
	
	return 0;
}

static int sensor_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
//	struct v4l2_captureparm *cp = &parms->parm.capture;
	//struct v4l2_fract *tpf = &cp->timeperframe;
	//struct sensor_info *info = to_state(sd);
	//int div;

//	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
//		return -EINVAL;
//	if (cp->extendedmode != 0)
//		return -EINVAL;

//	if (tpf->numerator == 0 || tpf->denominator == 0)
//		div = 1;  /* Reset to full rate */
//	else
//		div = (tpf->numerator*SENSOR_FRAME_RATE)/tpf->denominator;
//		
//	if (div == 0)
//		div = 1;
//	else if (div > CLK_SCALE)
//		div = CLK_SCALE;
//	info->clkrc = (info->clkrc & 0x80) | div;
//	tpf->numerator = 1;
//	tpf->denominator = sensor_FRAME_RATE/div;
//sensor_write(sd, REG_CLKRC, info->clkrc);
	return 0;
}


/* 
 * Code for dealing with controls.
 * fill with different sensor module
 * different sensor module has different settings here
 * if not support the follow function ,retrun -EINVAL
 */

/* *********************************************begin of ******************************************** */
static int sensor_queryctrl(struct v4l2_subdev *sd,
		struct v4l2_queryctrl *qc)
{
	/* Fill in min, max, step and default value for these controls. */
	/* see include/linux/videodev2.h for details */
	/* see sensor_s_parm and sensor_g_parm for the meaning of value */
	
	switch (qc->id) {
//	case V4L2_CID_BRIGHTNESS:
//		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 1);
//	case V4L2_CID_CONTRAST:
//		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 1);
//	case V4L2_CID_SATURATION:
//		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 1);
//	case V4L2_CID_HUE:
//		return v4l2_ctrl_query_fill(qc, -180, 180, 5, 0);
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	case V4L2_CID_GAIN:
		return v4l2_ctrl_query_fill(qc, 1, 8, 1, 2);
//	case V4L2_CID_AUTOGAIN:
//		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 1);
	case V4L2_CID_EXPOSURE:
		return v4l2_ctrl_query_fill(qc, 32, 1920, 8, 800);
//	case V4L2_CID_EXPOSURE_AUTO:
//		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
//	case V4L2_CID_DO_WHITE_BALANCE:
//		return v4l2_ctrl_query_fill(qc, 0, 5, 1, 0);
//	case V4L2_CID_AUTO_WHITE_BALANCE:
//		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 1);
//	case V4L2_CID_COLORFX:
//		return v4l2_ctrl_query_fill(qc, 0, 9, 1, 0);
	case V4L2_CID_CAMERA_FLASH_MODE:
	  return v4l2_ctrl_query_fill(qc, 0, 4, 1, 0);	
	}
	return -EINVAL;
}

static int sensor_g_hflip(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x40;
	
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_g_hflip!\n");
		return ret;
	}

	*value = (regs.value[0]>>6)&1; //bit15 is hflip
	return 0;
}

static int sensor_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x40;
	
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_g_hflip!\n");
		return ret;
	}

	switch(value) {
	case 0:
		regs.value[0] &= 0xBf;//bit14 is hflip disable
		break;
	case 1:
		regs.value[0] |= (1<<6); //bit14 is hflip enable
		break;
	default:
		break;
	}
	
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_hflip!\n");
		return ret;
	}
	return 0;
}

static int sensor_g_vflip(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x40;
	
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_g_hflip!\n");
		return ret;
	}

	*value = (regs.value[0]>>7)&1; //bit15 is hflip
	return 0;
}

static int sensor_s_vflip(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x40;
	
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_g_hflip!\n");
		return ret;
	}

	switch(value) {
	case 0:
		regs.value[0] &= 0x7f;//bit15 is hflip disable
		break;
	case 1:
		regs.value[0] |= (1<<7); //bit15 is hflip enable
		break;
	default:
		break;
	}
	
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_hflip!\n");
		return ret;
	}
	return 0;
}

static int sensor_g_autogain(struct v4l2_subdev *sd, __s32 *value)
{
	return -EINVAL;
}

static int sensor_s_autogain(struct v4l2_subdev *sd, int value)
{
	return -EINVAL;
}

static int sensor_g_autoexp(struct v4l2_subdev *sd, __s32 *value)
{
	return -EINVAL;
}

static int sensor_s_autoexp(struct v4l2_subdev *sd,
		enum v4l2_exposure_auto_type value)
{
	return -EINVAL;
}

static int sensor_g_autowb(struct v4l2_subdev *sd, int *value)
{
	struct sensor_info *info = to_state(sd);
	
	*value = info->autowb ;
	return 0;
}

static int sensor_s_autowb(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	
	ret = sensor_write_array(sd, sensor_wb_auto_regs, ARRAY_SIZE(sensor_wb_auto_regs));
	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_autowb!\n");
		return ret;
	}
	
	mdelay(10);
	
	info->autowb = value;
	return 0;
}

static int sensor_g_hue(struct v4l2_subdev *sd, __s32 *value)
{
	return -EINVAL;
}

static int sensor_s_hue(struct v4l2_subdev *sd, int value)
{
	return -EINVAL;
}

static int sensor_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	return -EINVAL;
}

static int sensor_s_gain(struct v4l2_subdev *sd, int value)
{
	//8X:0x30 7X:0X2C 6X 0X28 5X 0x24 4X:0x20 3X 0x1A 2X:0x10 1X:0x00
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x60;
	regs.value[0] 	= 0x00; 
	switch(value)
	{
		case 1://X1
			regs.value[1] = 0x00;
			break;
		case 2://X2
			regs.value[1] = 0x10;
			break;
		case 3://X3
			regs.value[1] = 0x1A;
			break;
		case 4://X4
			regs.value[1] = 0x20;
			break;
		case 5://X5
			regs.value[1] = 0x24;
			break;
		case 6://X6
			regs.value[1] = 0X28;
			break;
		case 7://X7
			regs.value[1] = 0X2C;
			break;
		case 8://X8
			regs.value[1] = 0x30;
			break;
		default:
			break;
	}
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_gain!\n");
		return ret;
	}
	return 0;
}
/* *********************************************end of ******************************************** */

static int sensor_g_brightness(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	
	*value = info->brightness;
	return 0;
}

static int sensor_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	
	switch (value) {
		case -4:
		  ret = sensor_write_array(sd, sensor_brightness_neg4_regs, ARRAY_SIZE(sensor_brightness_neg4_regs));
			break;
		case -3:
			ret = sensor_write_array(sd, sensor_brightness_neg3_regs, ARRAY_SIZE(sensor_brightness_neg3_regs));
			break;
		case -2:
			ret = sensor_write_array(sd, sensor_brightness_neg2_regs, ARRAY_SIZE(sensor_brightness_neg2_regs));
			break;   
		case -1:
			ret = sensor_write_array(sd, sensor_brightness_neg1_regs, ARRAY_SIZE(sensor_brightness_neg1_regs));
			break;
		case 0:   
			ret = sensor_write_array(sd, sensor_brightness_zero_regs, ARRAY_SIZE(sensor_brightness_zero_regs));
			break;
		case 1:
			ret = sensor_write_array(sd, sensor_brightness_pos1_regs, ARRAY_SIZE(sensor_brightness_pos1_regs));
			break;
		case 2:
			ret = sensor_write_array(sd, sensor_brightness_pos2_regs, ARRAY_SIZE(sensor_brightness_pos2_regs));
			break;	
		case 3:
			ret = sensor_write_array(sd, sensor_brightness_pos3_regs, ARRAY_SIZE(sensor_brightness_pos3_regs));
			break;
		case 4:
			ret = sensor_write_array(sd, sensor_brightness_pos4_regs, ARRAY_SIZE(sensor_brightness_pos4_regs));
			break;
		default:
			return -EINVAL;
	}
	
	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_brightness!\n");
		return ret;
	}
	mdelay(10);
	info->brightness = value;
	return 0;
}

static int sensor_g_contrast(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	
	*value = info->contrast;
	return 0;
}

static int sensor_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	
	switch (value) {
		case -4:
		  ret = sensor_write_array(sd, sensor_contrast_neg4_regs, ARRAY_SIZE(sensor_contrast_neg4_regs));
			break;
		case -3:
			ret = sensor_write_array(sd, sensor_contrast_neg3_regs, ARRAY_SIZE(sensor_contrast_neg3_regs));
			break;
		case -2:
			ret = sensor_write_array(sd, sensor_contrast_neg2_regs, ARRAY_SIZE(sensor_contrast_neg2_regs));
			break;   
		case -1:
			ret = sensor_write_array(sd, sensor_contrast_neg1_regs, ARRAY_SIZE(sensor_contrast_neg1_regs));
			break;
		case 0:   
			ret = sensor_write_array(sd, sensor_contrast_zero_regs, ARRAY_SIZE(sensor_contrast_zero_regs));
			break;
		case 1:
			ret = sensor_write_array(sd, sensor_contrast_pos1_regs, ARRAY_SIZE(sensor_contrast_pos1_regs));
			break;
		case 2:
			ret = sensor_write_array(sd, sensor_contrast_pos2_regs, ARRAY_SIZE(sensor_contrast_pos2_regs));
			break;	
		case 3:
			ret = sensor_write_array(sd, sensor_contrast_pos3_regs, ARRAY_SIZE(sensor_contrast_pos3_regs));
			break;
		case 4:
			ret = sensor_write_array(sd, sensor_contrast_pos4_regs, ARRAY_SIZE(sensor_contrast_pos4_regs));
			break;
		default:
			return -EINVAL;
	}
	
	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_contrast!\n");
		return ret;
	}
	mdelay(10);
	info->contrast = value;
	return 0;
}

static int sensor_g_saturation(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	
	*value = info->saturation;
	return 0;
}

static int sensor_s_saturation(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	
	switch (value) {
		case -4:
		  ret = sensor_write_array(sd, sensor_saturation_neg4_regs, ARRAY_SIZE(sensor_saturation_neg4_regs));
			break;
		case -3:
			ret = sensor_write_array(sd, sensor_saturation_neg3_regs, ARRAY_SIZE(sensor_saturation_neg3_regs));
			break;
		case -2:
			ret = sensor_write_array(sd, sensor_saturation_neg2_regs, ARRAY_SIZE(sensor_saturation_neg2_regs));
			break;   
		case -1:
			ret = sensor_write_array(sd, sensor_saturation_neg1_regs, ARRAY_SIZE(sensor_saturation_neg1_regs));
			break;
		case 0:   
			ret = sensor_write_array(sd, sensor_saturation_zero_regs, ARRAY_SIZE(sensor_saturation_zero_regs));
			break;
		case 1:
			ret = sensor_write_array(sd, sensor_saturation_pos1_regs, ARRAY_SIZE(sensor_saturation_pos1_regs));
			break;
		case 2:
			ret = sensor_write_array(sd, sensor_saturation_pos2_regs, ARRAY_SIZE(sensor_saturation_pos2_regs));
			break;	
		case 3:
			ret = sensor_write_array(sd, sensor_saturation_pos3_regs, ARRAY_SIZE(sensor_saturation_pos3_regs));
			break;
		case 4:
			ret = sensor_write_array(sd, sensor_saturation_pos4_regs, ARRAY_SIZE(sensor_saturation_pos4_regs));
			break;
		default:
			return -EINVAL;
	}
	
	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_saturation!\n");
		return ret;
	}
	mdelay(10);
	info->saturation = value;
	return 0;
}

static int sensor_g_exp(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	
	*value = info->exp;
	return 0;
}

static int sensor_s_exp(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list exp[1] = {0};
	exp[0].reg_num[0] = 0x30;
	exp[0].reg_num[1] = 0x12;
	exp[0].value[0] = (value>>8)&0xFF;
	exp[0].value[1] = value&0xFF;
	//csi_dev_err("sensor_s_exp...set shutter %d\n",value);
	ret = sensor_write_array(sd, exp, 1);
	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_exp!\n");
		return ret;
	}
	mdelay(10);
	info->exp = value;
	return 0;
}

static int sensor_g_wb(struct v4l2_subdev *sd, int *value)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_whiteblance *wb_type = (enum v4l2_whiteblance*)value;
	
	*wb_type = info->wb;
	
	return 0;
}

static int sensor_s_wb(struct v4l2_subdev *sd,
		enum v4l2_whiteblance value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	
	if (value == V4L2_WB_AUTO) {
		ret = sensor_s_autowb(sd, 1);
		return ret;
	} 
	else {
		ret = sensor_s_autowb(sd, 0);
		if(ret < 0) {
			csi_dev_err("sensor_s_autowb error, return %x!\n",ret);
			return ret;
		}
		
		switch (value) {
			case V4L2_WB_CLOUD:
			  ret = sensor_write_array(sd, sensor_wb_cloud_regs, ARRAY_SIZE(sensor_wb_cloud_regs));
				break;
			case V4L2_WB_DAYLIGHT:
				ret = sensor_write_array(sd, sensor_wb_daylight_regs, ARRAY_SIZE(sensor_wb_daylight_regs));
				break;
			case V4L2_WB_INCANDESCENCE:
				ret = sensor_write_array(sd, sensor_wb_incandescence_regs, ARRAY_SIZE(sensor_wb_incandescence_regs));
				break;    
			case V4L2_WB_FLUORESCENT:
				ret = sensor_write_array(sd, sensor_wb_fluorescent_regs, ARRAY_SIZE(sensor_wb_fluorescent_regs));
				break;
			case V4L2_WB_TUNGSTEN:   
				ret = sensor_write_array(sd, sensor_wb_tungsten_regs, ARRAY_SIZE(sensor_wb_tungsten_regs));
				break;
			default:
				return -EINVAL;
		} 
	}
	
	if (ret < 0) {
		csi_dev_err("sensor_s_wb error, return %x!\n",ret);
		return ret;
	}
	mdelay(10);
	info->wb = value;
	return 0;
}

static int sensor_g_colorfx(struct v4l2_subdev *sd,
		__s32 *value)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_colorfx *clrfx_type = (enum v4l2_colorfx*)value;
	
	*clrfx_type = info->clrfx;
	return 0;
}

static int sensor_s_colorfx(struct v4l2_subdev *sd,
		enum v4l2_colorfx value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	
	switch (value) {
	case V4L2_COLORFX_NONE:
	  ret = sensor_write_array(sd, sensor_colorfx_none_regs, ARRAY_SIZE(sensor_colorfx_none_regs));
		break;
	case V4L2_COLORFX_BW:
		ret = sensor_write_array(sd, sensor_colorfx_bw_regs, ARRAY_SIZE(sensor_colorfx_bw_regs));
		break;  
	case V4L2_COLORFX_SEPIA:
		ret = sensor_write_array(sd, sensor_colorfx_sepia_regs, ARRAY_SIZE(sensor_colorfx_sepia_regs));
		break;   
	case V4L2_COLORFX_NEGATIVE:
		ret = sensor_write_array(sd, sensor_colorfx_negative_regs, ARRAY_SIZE(sensor_colorfx_negative_regs));
		break;
	case V4L2_COLORFX_EMBOSS:   
		ret = sensor_write_array(sd, sensor_colorfx_emboss_regs, ARRAY_SIZE(sensor_colorfx_emboss_regs));
		break;
	case V4L2_COLORFX_SKETCH:     
		ret = sensor_write_array(sd, sensor_colorfx_sketch_regs, ARRAY_SIZE(sensor_colorfx_sketch_regs));
		break;
	case V4L2_COLORFX_SKY_BLUE:
		ret = sensor_write_array(sd, sensor_colorfx_sky_blue_regs, ARRAY_SIZE(sensor_colorfx_sky_blue_regs));
		break;
	case V4L2_COLORFX_GRASS_GREEN:
		ret = sensor_write_array(sd, sensor_colorfx_grass_green_regs, ARRAY_SIZE(sensor_colorfx_grass_green_regs));
		break;
	case V4L2_COLORFX_SKIN_WHITEN:
		ret = sensor_write_array(sd, sensor_colorfx_skin_whiten_regs, ARRAY_SIZE(sensor_colorfx_skin_whiten_regs));
		break;
	case V4L2_COLORFX_VIVID:
		ret = sensor_write_array(sd, sensor_colorfx_vivid_regs, ARRAY_SIZE(sensor_colorfx_vivid_regs));
		break;
	default:
		return -EINVAL;
	}
	
	if (ret < 0) {
		csi_dev_err("sensor_s_colorfx error, return %x!\n",ret);
		return ret;
	}
	mdelay(10);
	info->clrfx = value;
	return 0;
}

static int sensor_g_flash_mode(struct v4l2_subdev *sd,
    __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_flash_mode *flash_mode = (enum v4l2_flash_mode*)value;
	
	*flash_mode = info->flash_mode;
	return 0;
}

static int sensor_s_flash_mode(struct v4l2_subdev *sd,
    enum v4l2_flash_mode value)
{
	struct sensor_info *info = to_state(sd);
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
	int flash_on,flash_off;
	
	flash_on = (dev->flash_pol!=0)?1:0;
	flash_off = (flash_on==1)?0:1;
	
	switch (value) {
	case V4L2_FLASH_MODE_OFF:
		csi_gpio_write(sd,&dev->flash_io,flash_off);
		break;
	case V4L2_FLASH_MODE_AUTO:
		return -EINVAL;
		break;  
	case V4L2_FLASH_MODE_ON:
		csi_gpio_write(sd,&dev->flash_io,flash_on);
		break;   
	case V4L2_FLASH_MODE_TORCH:
		return -EINVAL;
		break;
	case V4L2_FLASH_MODE_RED_EYE:   
		return -EINVAL;
		break;
	default:
		return -EINVAL;
	}
	
	info->flash_mode = value;
	return 0;
}

static int sensor_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
#if 0
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return sensor_g_brightness(sd, &ctrl->value);
	case V4L2_CID_CONTRAST:
		return sensor_g_contrast(sd, &ctrl->value);
	case V4L2_CID_SATURATION:
		return sensor_g_saturation(sd, &ctrl->value);
	case V4L2_CID_HUE:
		return sensor_g_hue(sd, &ctrl->value);	
	case V4L2_CID_VFLIP:
		return sensor_g_vflip(sd, &ctrl->value);
	case V4L2_CID_HFLIP:
		return sensor_g_hflip(sd, &ctrl->value);
	case V4L2_CID_GAIN:
		return sensor_g_gain(sd, &ctrl->value);
	case V4L2_CID_AUTOGAIN:
		return sensor_g_autogain(sd, &ctrl->value);
	case V4L2_CID_EXPOSURE:
		return sensor_g_exp(sd, &ctrl->value);
	case V4L2_CID_EXPOSURE_AUTO:
		return sensor_g_autoexp(sd, &ctrl->value);
	case V4L2_CID_DO_WHITE_BALANCE:
		return sensor_g_wb(sd, &ctrl->value);
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return sensor_g_autowb(sd, &ctrl->value);
	case V4L2_CID_COLORFX:
		return sensor_g_colorfx(sd,	&ctrl->value);
	case V4L2_CID_CAMERA_FLASH_MODE:
		return sensor_g_flash_mode(sd, &ctrl->value);
	}
	#endif
	return -EINVAL;
}

static int sensor_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	//csi_dev_err("sensor_s_ctrl test 0x%x-->0x%x\n",ctrl->id,ctrl->value);
	switch(ctrl->id)
	{
		case V4L2_CID_GAIN:
			printk("gain will be set %d\n",ctrl->value);
			return sensor_s_gain(sd, ctrl->value);
		case V4L2_CID_EXPOSURE:
			return sensor_s_exp(sd, ctrl->value);
		case V4L2_CID_VFLIP:
			return sensor_s_vflip(sd, ctrl->value);
		case V4L2_CID_HFLIP:
			return sensor_s_hflip(sd, ctrl->value);
	}
#if 0
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return sensor_s_brightness(sd, ctrl->value);
	case V4L2_CID_CONTRAST:
		return sensor_s_contrast(sd, ctrl->value);
	case V4L2_CID_SATURATION:
		return sensor_s_saturation(sd, ctrl->value);
	case V4L2_CID_HUE:
		return sensor_s_hue(sd, ctrl->value);		
	case V4L2_CID_VFLIP:
		return sensor_s_vflip(sd, ctrl->value);
	case V4L2_CID_HFLIP:
		return sensor_s_hflip(sd, ctrl->value);
	case V4L2_CID_GAIN:
		return sensor_s_gain(sd, ctrl->value);
	case V4L2_CID_AUTOGAIN:
		return sensor_s_autogain(sd, ctrl->value);
	case V4L2_CID_EXPOSURE:
		return sensor_s_exp(sd, ctrl->value);
	case V4L2_CID_EXPOSURE_AUTO:
		return sensor_s_autoexp(sd,
				(enum v4l2_exposure_auto_type) ctrl->value);
	case V4L2_CID_DO_WHITE_BALANCE:
		return sensor_s_wb(sd,
				(enum v4l2_whiteblance) ctrl->value);	
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return sensor_s_autowb(sd, ctrl->value);
	case V4L2_CID_COLORFX:
		return sensor_s_colorfx(sd,
				(enum v4l2_colorfx) ctrl->value);
	case V4L2_CID_CAMERA_FLASH_MODE:
	  return sensor_s_flash_mode(sd,
	      (enum v4l2_flash_mode) ctrl->value);
	}
	#endif
	return -EINVAL;
}

static int sensor_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_SENSOR, 0);
}


/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.g_chip_ident = sensor_g_chip_ident,
	.g_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
	.queryctrl = sensor_queryctrl,
	.reset = sensor_reset,
	.init = sensor_init,
	.s_power = sensor_power,
	.ioctl = sensor_ioctl,
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.enum_mbus_fmt = sensor_enum_fmt,//linux-3.0
	.try_mbus_fmt = sensor_try_fmt,//linux-3.0
	.s_mbus_fmt = sensor_s_fmt,//linux-3.0
	.s_parm = sensor_s_parm,//linux-3.0
	.g_parm = sensor_g_parm,//linux-3.0
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
};

/* ----------------------------------------------------------------------- */

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct sensor_info *info;
//	int ret;
	printk(KERN_ERR "sensor_probe start\n");
	info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;
	v4l2_i2c_subdev_init(sd, client, &sensor_ops);

	info->fmt = &sensor_formats[0];
	info->ccm_info = &ccm_info_con;
	
	info->brightness = 0;
	info->contrast = 0;
	info->saturation = 0;
	info->hue = 0;
	info->hflip = 0;
	info->vflip = 0;
	info->gain = 0;
	info->autogain = 1;
	info->exp = 0;
	info->autoexp = 0;
	info->autowb = 1;
	info->wb = 0;
	info->clrfx = 0;
	
//	info->clkrc = 1;	/* 30fps */

	printk(KERN_ERR "sensor_probe end\n");

	return 0;
}


static int sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ "mt9p031", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

//linux-3.0
static struct i2c_driver sensor_driver =
{
	.driver = 
	{
	.owner = THIS_MODULE,
	.name = "mt9p031",
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};
static __init int init_sensor(void)
{
	return i2c_add_driver(&sensor_driver);
}

static __exit void exit_sensor(void)
{
  i2c_del_driver(&sensor_driver);
}

module_init(init_sensor);
module_exit(exit_sensor);

