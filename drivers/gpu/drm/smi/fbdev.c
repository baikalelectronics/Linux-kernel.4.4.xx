#include <drm/drm_crtc.h>
#include <drm/drm_fb_helper.h>
#include <drm/drmP.h>

#include "smi.h"
#include "fbdev.h"


static void _fb_destroy(struct drm_framebuffer *fb) {
    pr_info(">>%s:%d FB ID: %d (%p)\n", __func__, __LINE__, fb->base.id, fb);

    drm_framebuffer_cleanup(fb);
}

static int _fb_create_handle(struct drm_framebuffer *fb, struct drm_file *file_priv, unsigned int *handle) {
    pr_info(">>%s:%d\n", __func__, __LINE__);
    return 0;
}
static int _fb_dirty(struct drm_framebuffer *fb, struct drm_file *file, unsigned flags, unsigned color, struct drm_clip_rect *clips, unsigned num_clips) {
    pr_info(">>%s:%d x[%d, %d], y[%d, %d]\n", __func__, __LINE__, clips->x1, clips->x2, clips->y1, clips->y2);
    return 0;
}

static const struct drm_framebuffer_funcs _fb_funcs = {
    .destroy = _fb_destroy,
    .create_handle = _fb_create_handle,
    .dirty = _fb_dirty,
};


static struct fb_ops _fb_ops = {
    .owner		= THIS_MODULE,
    .fb_fillrect	= drm_fb_helper_sys_fillrect,
    .fb_copyarea	= drm_fb_helper_sys_copyarea,
    .fb_imageblit	= drm_fb_helper_sys_imageblit,
    .fb_check_var	= drm_fb_helper_check_var,
    .fb_set_par		= drm_fb_helper_set_par,
    .fb_blank		= drm_fb_helper_blank,
    .fb_pan_display	= drm_fb_helper_pan_display,
    .fb_setcmap		= drm_fb_helper_setcmap,
};


static int _fb_create(struct drm_fb_helper *helper, struct drm_fb_helper_surface_size *sizes) {
    struct drm_device *dev = helper->dev;
    struct fb_info *info;
    struct drm_framebuffer *fb = helper->fb;
    struct drm_mode_fb_cmd2 mode_cmd;
    int ret;

    pr_info(">>%s:%d surface width(%d), height(%d) and bpp(%d)\n", __func__, __LINE__, sizes->surface_width, sizes->surface_height, sizes->surface_bpp);

    mode_cmd.width = sizes->surface_width;
    mode_cmd.height = sizes->surface_height;
    mode_cmd.pitches[0] = mode_cmd.width * ((sizes->surface_bpp + 7) / 8);
    mode_cmd.pixel_format = drm_mode_legacy_fb_format(sizes->surface_bpp, sizes->surface_depth);

    info = drm_fb_helper_alloc_fbi(helper);
    if (IS_ERR(info)) {
	    return PTR_ERR(info);
    }

    pr_info(">>%s:%d\n", __func__, __LINE__);
    drm_helper_mode_fill_fb_struct(fb, &mode_cmd);

    ret = drm_framebuffer_init(dev, fb, &_fb_funcs);
    if (ret) {
	dev_err(dev->dev, "Failed to initialize framebuffer: %d\n", ret);
	return ret;
    }

    info->par = helper;
    info->flags = FBINFO_DEFAULT;
    info->fbops = &_fb_ops;

    //pr_info(">>%s:%d %d:%d:%d:%d\n", __func__, __LINE__, DIV_ROUND_UP(sizes->surface_bpp, 8), info->var.xoffset, info->var.yoffset, fb->pitches[0]);
    //4:0:0:7680
    //unsigned bytes_per_pixel = DIV_ROUND_UP(sizes->surface_bpp, 8);
    /*unsigned long offset;
    offset = fbi->var.xoffset * bytes_per_pixel;
    offset += fbi->var.yoffset * fb->pitches[0];*/


    info->fix.smem_start = pci_resource_start(dev->pdev, 0);
    info->fix.smem_len = pci_resource_len(dev->pdev, 0);

    info->screen_base = ioremap_wc(info->fix.smem_start, info->fix.smem_len);
    info->screen_size = info->fix.smem_len;

    strcpy(info->fix.id, "smidrmfb");

    drm_fb_helper_fill_fix(info, fb->pitches[0], fb->depth);
    drm_fb_helper_fill_var(info, helper, sizes->fb_width, sizes->fb_height);

    return 0;
}


static struct drm_fb_helper_funcs _helper_funcs = {
    .fb_probe = _fb_create,
};

int _fbdev_init(struct drm_device *dev) {
    struct _device *priv = dev->dev_private;
    struct drm_fb_helper *helper = &priv->helper;
    int ret;

    pr_info(">>%s:%d\n", __func__, __LINE__);
    drm_fb_helper_prepare(dev, helper, &_helper_funcs);
    ret = drm_fb_helper_init(dev, helper, 1, 1);
    if (ret) {
	return ret;
    }
    helper->fb = &priv->fb;
    drm_fb_helper_single_add_all_connectors(helper);

    return drm_fb_helper_initial_config(helper, dev->mode_config.preferred_depth);
}

//static 
void _fbdev_fini(struct drm_device *dev) {
    struct _device *private = dev->dev_private;
    struct drm_fb_helper *helper = &private->helper;

    drm_fb_helper_unregister_fbi(helper);
    drm_fb_helper_release_fbi(helper);

    if (helper->fb)
	drm_framebuffer_unreference(helper->fb);

    drm_fb_helper_fini(helper);
}
