#ifndef _CTRL_H
#define _CTRL_H

#include <linux/bitops.h>
#include <drm/drm_os_linux.h>

#define VGA_CONFIGURATION                             0x000088
#define VGA_CONFIGURATION_USER_DEFINE_MASK            (0x3<<4)
#define VGA_CONFIGURATION_PLL                         BIT(2)
#define VGA_CONFIGURATION_MODE                        BIT(1)

#define MXCLK_PLL_CTRL                                0x000070
#define MXCLK_PLL_CTRL_BYPASS                         BIT(18)
#define MXCLK_PLL_CTRL_POWER                          BIT(17)
#define MXCLK_PLL_CTRL_INPUT                          BIT(16)

#define PANEL_PLL_CTRL                                0x00005C
#define PANEL_PLL_CTRL_BYPASS                         BIT(18)
#define PANEL_PLL_CTRL_POWER                          BIT(17)
#define PANEL_PLL_CTRL_INPUT                          BIT(16)
/*#ifdef VALIDATION_CHIP
    #define PLL_CTRL_OD_SHIFT                         14
    #define PLL_CTRL_OD_MASK                          (0x3<<14)
#else
    #define PLL_CTRL_POD_SHIFT                        14
    #define PLL_CTRL_POD_MASK                         (0x3<<14)
    #define PLL_CTRL_OD_SHIFT                         12
    #define PLL_CTRL_OD_MASK                          (0x3<<12)
#endif
#define PLL_CTRL_N_SHIFT                              8
#define PLL_CTRL_N_MASK                               (0xf<<8)
#define PLL_CTRL_M_SHIFT                              0
#define PLL_CTRL_M_MASK                               0xff*/


#define MODE0_GATE                                    0x000044
#define MODE0_GATE_MCLK_MASK                          (0x3<<14)
#define MODE0_GATE_MCLK_112MHZ                        (0x0<<14)
#define MODE0_GATE_MCLK_84MHZ                         (0x1<<14)
#define MODE0_GATE_MCLK_56MHZ                         (0x2<<14)
#define MODE0_GATE_MCLK_42MHZ                         (0x3<<14)
#define MODE0_GATE_M2XCLK_MASK                        (0x3<<12)
#define MODE0_GATE_M2XCLK_336MHZ                      (0x0<<12)
#define MODE0_GATE_M2XCLK_168MHZ                      (0x1<<12)
#define MODE0_GATE_M2XCLK_112MHZ                      (0x2<<12)
#define MODE0_GATE_M2XCLK_84MHZ                       (0x3<<12)
#define MODE0_GATE_VGA                                BIT(10)
#define MODE0_GATE_PWM                                BIT(9)
#define MODE0_GATE_I2C                                BIT(8)
#define MODE0_GATE_SSP                                BIT(7)
#define MODE0_GATE_GPIO                               BIT(6)
#define MODE0_GATE_ZVPORT                             BIT(5)
#define MODE0_GATE_CSC                                BIT(4)
#define MODE0_GATE_DE                                 BIT(3)
#define MODE0_GATE_DISPLAY                            BIT(2)
#define MODE0_GATE_LOCALMEM                           BIT(1)
#define MODE0_GATE_DMA                                BIT(0)

#define MISC_CTRL                                     0x000004
#define MISC_CTRL_DRAM_RERESH_COUNT                   BIT(27)
#define MISC_CTRL_DRAM_REFRESH_TIME_MASK              (0x3<<25)
#define MISC_CTRL_DRAM_REFRESH_TIME_8                 (0x0<<25)
#define MISC_CTRL_DRAM_REFRESH_TIME_16                (0x1<<25)
#define MISC_CTRL_DRAM_REFRESH_TIME_32                (0x2<<25)
#define MISC_CTRL_DRAM_REFRESH_TIME_64                (0x3<<25)
#define MISC_CTRL_INT_OUTPUT_INVERT                   BIT(24)
#define MISC_CTRL_PLL_CLK_COUNT                       BIT(23)
#define MISC_CTRL_DAC_POWER_OFF                       BIT(20)
#define MISC_CTRL_CLK_SELECT_TESTCLK                  BIT(16)
#define MISC_CTRL_DRAM_COLUMN_SIZE_MASK               (0x3<<14)
#define MISC_CTRL_DRAM_COLUMN_SIZE_256                (0x0<<14)
#define MISC_CTRL_DRAM_COLUMN_SIZE_512                (0x1<<14)
#define MISC_CTRL_DRAM_COLUMN_SIZE_1024               (0x2<<14)
#define MISC_CTRL_LOCALMEM_SIZE_MASK                  (0x3<<12)
#define MISC_CTRL_LOCALMEM_SIZE_8M                    (0x3<<12)
#define MISC_CTRL_LOCALMEM_SIZE_16M                   (0x0<<12)
#define MISC_CTRL_LOCALMEM_SIZE_32M                   (0x1<<12)
#define MISC_CTRL_LOCALMEM_SIZE_64M                   (0x2<<12)
#define MISC_CTRL_DRAM_TWTR                           BIT(11)
#define MISC_CTRL_DRAM_TWR                            BIT(10)
#define MISC_CTRL_DRAM_TRP                            BIT(9)
#define MISC_CTRL_DRAM_TRFC                           BIT(8)
#define MISC_CTRL_DRAM_TRAS                           BIT(7)
#define MISC_CTRL_LOCALMEM_RESET                      BIT(6)
#define MISC_CTRL_LOCALMEM_STATE_INACTIVE             BIT(5)
#define MISC_CTRL_CPU_CAS_LATENCY                     BIT(4)
#define MISC_CTRL_DLL_OFF                             BIT(3)
#define MISC_CTRL_DRAM_OUTPUT_HIGH                    BIT(2)
#define MISC_CTRL_LOCALMEM_BUS_SIZE                   BIT(1)
#define MISC_CTRL_EMBEDDED_LOCALMEM_OFF               BIT(0)

#define SYSTEM_CTRL                                   0x000000
#define SYSTEM_CTRL_DPMS_MASK                         (0x3<<30)
#define SYSTEM_CTRL_DPMS_VPHP                         (0x0<<30)
#define SYSTEM_CTRL_DPMS_VPHN                         (0x1<<30)
#define SYSTEM_CTRL_DPMS_VNHP                         (0x2<<30)
#define SYSTEM_CTRL_DPMS_VNHN                         (0x3<<30)
#define SYSTEM_CTRL_PCI_BURST                         BIT(29)
#define SYSTEM_CTRL_PCI_MASTER                        BIT(25)
#define SYSTEM_CTRL_LATENCY_TIMER_OFF                 BIT(24)
#define SYSTEM_CTRL_DE_FIFO_EMPTY                     BIT(23)
#define SYSTEM_CTRL_DE_STATUS_BUSY                    BIT(22)
#define SYSTEM_CTRL_DE_MEM_FIFO_EMPTY                 BIT(21)
#define SYSTEM_CTRL_CSC_STATUS_BUSY                   BIT(20)
#define SYSTEM_CTRL_CRT_VSYNC_ACTIVE                  BIT(19)
#define SYSTEM_CTRL_PANEL_VSYNC_ACTIVE                BIT(18)
#define SYSTEM_CTRL_CURRENT_BUFFER_FLIP_PENDING       BIT(17)
#define SYSTEM_CTRL_DMA_STATUS_BUSY                   BIT(16)
#define SYSTEM_CTRL_PCI_BURST_READ                    BIT(15)
#define SYSTEM_CTRL_DE_ABORT                          BIT(13)
#define SYSTEM_CTRL_PCI_SUBSYS_ID_LOCK                BIT(11)
#define SYSTEM_CTRL_PCI_RETRY_OFF                     BIT(7)
#define SYSTEM_CTRL_PCI_SLAVE_BURST_READ_SIZE_MASK    (0x3<<4)
#define SYSTEM_CTRL_PCI_SLAVE_BURST_READ_SIZE_1       (0x0<<4)
#define SYSTEM_CTRL_PCI_SLAVE_BURST_READ_SIZE_2       (0x1<<4)
#define SYSTEM_CTRL_PCI_SLAVE_BURST_READ_SIZE_4       (0x2<<4)
#define SYSTEM_CTRL_PCI_SLAVE_BURST_READ_SIZE_8       (0x3<<4)
#define SYSTEM_CTRL_CRT_TRISTATE                      BIT(3)
#define SYSTEM_CTRL_PCIMEM_TRISTATE                   BIT(2)
#define SYSTEM_CTRL_LOCALMEM_TRISTATE                 BIT(1)
#define SYSTEM_CTRL_PANEL_TRISTATE                    BIT(0)

#define PANEL_HORIZONTAL_TOTAL                        0x080024
#define PANEL_HORIZONTAL_TOTAL_TOTAL_SHIFT            16
#define PANEL_HORIZONTAL_TOTAL_TOTAL_MASK             (0xfff<<16)
#define PANEL_HORIZONTAL_TOTAL_DISPLAY_END_MASK       0xfff

#define PANEL_HORIZONTAL_SYNC                         0x080028
#define PANEL_HORIZONTAL_SYNC_WIDTH_SHIFT             16
#define PANEL_HORIZONTAL_SYNC_WIDTH_MASK              (0xff<<16)
#define PANEL_HORIZONTAL_SYNC_START_MASK              0xfff

#define PANEL_VERTICAL_TOTAL                          0x08002C
#define PANEL_VERTICAL_TOTAL_TOTAL_SHIFT              16
#define PANEL_VERTICAL_TOTAL_TOTAL_MASK               (0x7ff<<16)
#define PANEL_VERTICAL_TOTAL_DISPLAY_END_MASK         0x7ff

#define PANEL_VERTICAL_SYNC                           0x080030
#define PANEL_VERTICAL_SYNC_HEIGHT_SHIFT              16
#define PANEL_VERTICAL_SYNC_HEIGHT_MASK               (0x3f<<16)
#define PANEL_VERTICAL_SYNC_START_MASK                0x7ff

#define PANEL_DISPLAY_CTRL                            0x080000
#define PANEL_DISPLAY_CTRL_RESERVED_MASK              0xc0f08000
#define PANEL_DISPLAY_CTRL_SELECT_SHIFT               28
#define PANEL_DISPLAY_CTRL_SELECT_MASK                (0x3<<28)
#define PANEL_DISPLAY_CTRL_SELECT_PANEL               (0x0<<28)
#define PANEL_DISPLAY_CTRL_SELECT_VGA                 (0x1<<28)
#define PANEL_DISPLAY_CTRL_SELECT_CRT                 (0x2<<28)
#define PANEL_DISPLAY_CTRL_FPEN                       BIT(27)
#define PANEL_DISPLAY_CTRL_VBIASEN                    BIT(26)
#define PANEL_DISPLAY_CTRL_DATA                       BIT(25)
#define PANEL_DISPLAY_CTRL_FPVDDEN                    BIT(24)
#define PANEL_DISPLAY_CTRL_DUAL_DISPLAY               BIT(19)
#define PANEL_DISPLAY_CTRL_DOUBLE_PIXEL               BIT(18)
#define PANEL_DISPLAY_CTRL_FIFO                       (0x3<<16)
#define PANEL_DISPLAY_CTRL_FIFO_1                     (0x0<<16)
#define PANEL_DISPLAY_CTRL_FIFO_3                     (0x1<<16)
#define PANEL_DISPLAY_CTRL_FIFO_7                     (0x2<<16)
#define PANEL_DISPLAY_CTRL_FIFO_11                    (0x3<<16)
#define DISPLAY_CTRL_CLOCK_PHASE                      BIT(14)
#define DISPLAY_CTRL_VSYNC_PHASE                      BIT(13)
#define DISPLAY_CTRL_HSYNC_PHASE                      BIT(12)
#define PANEL_DISPLAY_CTRL_VSYNC                      BIT(11)
#define PANEL_DISPLAY_CTRL_CAPTURE_TIMING             BIT(10)
#define PANEL_DISPLAY_CTRL_COLOR_KEY                  BIT(9)
#define DISPLAY_CTRL_TIMING                           BIT(8)
#define PANEL_DISPLAY_CTRL_VERTICAL_PAN_DIR           BIT(7)
#define PANEL_DISPLAY_CTRL_VERTICAL_PAN               BIT(6)
#define PANEL_DISPLAY_CTRL_HORIZONTAL_PAN_DIR         BIT(5)
#define PANEL_DISPLAY_CTRL_HORIZONTAL_PAN             BIT(4)
#define DISPLAY_CTRL_GAMMA                            BIT(3)
#define DISPLAY_CTRL_PLANE                            BIT(2)
#define PANEL_DISPLAY_CTRL_FORMAT_MASK                (0x3<<0)
#define PANEL_DISPLAY_CTRL_FORMAT_8                   (0x0<<0)
#define PANEL_DISPLAY_CTRL_FORMAT_16                  (0x1<<0)
#define PANEL_DISPLAY_CTRL_FORMAT_32                  (0x2<<0)

#define PANEL_FB_ADDRESS                              0x08000C
#define PANEL_FB_ADDRESS_STATUS                       BIT(31)
#define PANEL_FB_ADDRESS_EXT                          BIT(27)
#define PANEL_FB_ADDRESS_ADDRESS_MASK                 0x1ffffff

#define PANEL_FB_WIDTH                                0x080010
#define PANEL_FB_WIDTH_WIDTH_SHIFT                    16
#define PANEL_FB_WIDTH_WIDTH_MASK                     (0x3fff<<16)
#define PANEL_FB_WIDTH_OFFSET_MASK                    0x3fff

#define PANEL_WINDOW_WIDTH                            0x080014
#define PANEL_WINDOW_WIDTH_WIDTH_SHIFT                16
#define PANEL_WINDOW_WIDTH_WIDTH_MASK                 (0xfff<<16)
#define PANEL_WINDOW_WIDTH_X_MASK                     0xfff

#define PANEL_WINDOW_HEIGHT                           0x080018
#define PANEL_WINDOW_HEIGHT_HEIGHT_SHIFT              16
#define PANEL_WINDOW_HEIGHT_HEIGHT_MASK               (0xfff<<16)
#define PANEL_WINDOW_HEIGHT_Y_MASK                    0xfff

#define PANEL_PLANE_TL                                0x08001C
#define PANEL_PLANE_TL_TOP_SHIFT                      16
#define PANEL_PLANE_TL_TOP_MASK                       (0xeff<<16)
#define PANEL_PLANE_TL_LEFT_MASK                      0xeff

#define PANEL_PLANE_BR                                0x080020
#define PANEL_PLANE_BR_BOTTOM_SHIFT                   16
#define PANEL_PLANE_BR_BOTTOM_MASK                    (0xeff<<16)
#define PANEL_PLANE_BR_RIGHT_MASK                     0xeff



#define CRT_DISPLAY_CTRL                              0x080200
#define CRT_DISPLAY_CTRL_RESERVED_MASK                0xfb008200

/* SM750LE definition */
#define CRT_DISPLAY_CTRL_DPMS_SHIFT                   30
#define CRT_DISPLAY_CTRL_DPMS_MASK                    (0x3<<30)
#define CRT_DISPLAY_CTRL_DPMS_0                       (0x0<<30)
#define CRT_DISPLAY_CTRL_DPMS_1                       (0x1<<30)
#define CRT_DISPLAY_CTRL_DPMS_2                       (0x2<<30)
#define CRT_DISPLAY_CTRL_DPMS_3                       (0x3<<30)
#define CRT_DISPLAY_CTRL_CLK_MASK                     (0x7<<27)
#define CRT_DISPLAY_CTRL_CLK_PLL25                    (0x0<<27)
#define CRT_DISPLAY_CTRL_CLK_PLL41                    (0x1<<27)
#define CRT_DISPLAY_CTRL_CLK_PLL62                    (0x2<<27)
#define CRT_DISPLAY_CTRL_CLK_PLL65                    (0x3<<27)
#define CRT_DISPLAY_CTRL_CLK_PLL74                    (0x4<<27)
#define CRT_DISPLAY_CTRL_CLK_PLL80                    (0x5<<27)
#define CRT_DISPLAY_CTRL_CLK_PLL108                   (0x6<<27)
#define CRT_DISPLAY_CTRL_CLK_RESERVED                 (0x7<<27)
#define CRT_DISPLAY_CTRL_SHIFT_VGA_DAC                BIT(26)

/* SM750LE definition */
#define CRT_DISPLAY_CTRL_CRTSELECT                    BIT(25)
#define CRT_DISPLAY_CTRL_RGBBIT                       BIT(24)

#ifndef VALIDATION_CHIP
    #define CRT_DISPLAY_CTRL_CENTERING                BIT(24)
#endif
#define CRT_DISPLAY_CTRL_LOCK_TIMING                  BIT(23)
#define CRT_DISPLAY_CTRL_EXPANSION                    BIT(22)
#define CRT_DISPLAY_CTRL_VERTICAL_MODE                BIT(21)
#define CRT_DISPLAY_CTRL_HORIZONTAL_MODE              BIT(20)
#define CRT_DISPLAY_CTRL_SELECT_SHIFT                 18
#define CRT_DISPLAY_CTRL_SELECT_MASK                  (0x3<<18)
#define CRT_DISPLAY_CTRL_SELECT_PANEL                 (0x0<<18)
#define CRT_DISPLAY_CTRL_SELECT_VGA                   (0x1<<18)
#define CRT_DISPLAY_CTRL_SELECT_CRT                   (0x2<<18)
#define CRT_DISPLAY_CTRL_FIFO_MASK                    (0x3<<16)
#define CRT_DISPLAY_CTRL_FIFO_1                       (0x0<<16)
#define CRT_DISPLAY_CTRL_FIFO_3                       (0x1<<16)
#define CRT_DISPLAY_CTRL_FIFO_7                       (0x2<<16)
#define CRT_DISPLAY_CTRL_FIFO_11                      (0x3<<16)
#define CRT_DISPLAY_CTRL_BLANK                        BIT(10)
#define CRT_DISPLAY_CTRL_PIXEL_MASK                   (0xf<<4)
#define CRT_DISPLAY_CTRL_FORMAT_MASK                  (0x3<<0)
#define CRT_DISPLAY_CTRL_FORMAT_8                     (0x0<<0)
#define CRT_DISPLAY_CTRL_FORMAT_16                    (0x1<<0)
#define CRT_DISPLAY_CTRL_FORMAT_32                    (0x2<<0)


static inline void ctrl_update(struct drm_device *dev, u32 reg, u32 val) {
    struct _device *dev_priv = dev->dev_private;
    //pr_info(">>%s:%d %08x:%08x\n", __func__, __LINE__, DRM_READ32(dev_priv->mmio, reg), val);
    DRM_WRITE32(dev_priv->mmio, reg, DRM_READ32(dev_priv->mmio, reg)|val);
}
static inline void ctrl_clear(struct drm_device *dev, u32 reg, u32 val) {
    struct _device *dev_priv = dev->dev_private;
    //pr_info(">>%s:%d\n", __func__, __LINE__);
    DRM_WRITE32(dev_priv->mmio, reg, DRM_READ32(dev_priv->mmio, reg)&~val);
}
/*static inline void ctrl_flip(struct drm_device *dev, u32 reg, u32 val) {
    struct _device *dev_priv = dev->dev_private;
    pr_info(">>%s:%d\n", __func__, __LINE__);
    DRM_WRITE32(dev_priv->mmio, reg, DRM_READ32(dev_priv->mmio, reg)|val);
}*/

static inline void ctrl_write(struct drm_device *dev, u32 reg, u32 val) {
    struct _device *dev_priv = dev->dev_private;
    //pr_info(">>%s:%d %08x:%08x\n", __func__, __LINE__, val, DRM_READ32(dev_priv->mmio, reg));
    DRM_WRITE32(dev_priv->mmio, reg, val);
}
/*static inline void ctrl_write_hl(struct drm_device *dev, u32 reg, u32 hight, u32 low) {
    struct _device *dev_priv = dev->dev_private;
    DRM_WRITE32(dev_priv->mmio, reg, hight<<16|low);
}*/
// TODO: rewrite this function with exta parameter "shift/offset" = 20
static inline void ctrl_wrup(struct drm_device *dev, u32 reg, u32 val) {	// TODO: weakly realization, u should tinlk about algo a bit.
    struct _device *dev_priv = dev->dev_private;
    //pr_info(">>%s:%d\n", __func__, __LINE__);
    DRM_WRITE32(dev_priv->mmio, reg, val|((DRM_READ32(dev_priv->mmio, reg)<<20)>>20));
}

static inline u32 ctrl_read(struct drm_device *dev, u32 reg) {
    struct _device *dev_priv = dev->dev_private;
    //pr_info(">>%s:%d\n", __func__, __LINE__);
    return DRM_READ32(dev_priv->mmio, reg);
}
static inline void pr_ctrl(struct drm_device *dev) {
    pr_info(">>%s:%d vga:%08x mxclk:%08x mode0:%08x misc:%08x sys:%08x pll:%08x htotal:%08x hsync:%08x vtotal:%08x vsync:%08x disp:%08x fbadrr:%08x fbw:%08x ww:%08x wh:%08x pbr:%08x crt:%08x\n",
	__func__, __LINE__, 
	ctrl_read(dev, VGA_CONFIGURATION),
	ctrl_read(dev, MXCLK_PLL_CTRL),
	ctrl_read(dev, MODE0_GATE),
	ctrl_read(dev, MISC_CTRL),
	ctrl_read(dev, SYSTEM_CTRL),
	ctrl_read(dev, PANEL_PLL_CTRL),
	ctrl_read(dev, PANEL_HORIZONTAL_TOTAL),
	ctrl_read(dev, PANEL_HORIZONTAL_SYNC),
	ctrl_read(dev, PANEL_VERTICAL_TOTAL),
	ctrl_read(dev, PANEL_VERTICAL_SYNC),
	ctrl_read(dev, PANEL_DISPLAY_CTRL),
	ctrl_read(dev, PANEL_FB_ADDRESS),
	ctrl_read(dev, PANEL_FB_WIDTH),
	ctrl_read(dev, PANEL_WINDOW_WIDTH),
	ctrl_read(dev, PANEL_WINDOW_HEIGHT),
	ctrl_read(dev, PANEL_PLANE_BR),
	ctrl_read(dev, CRT_DISPLAY_CTRL)
    );
}

#endif /* _CTRL_H */