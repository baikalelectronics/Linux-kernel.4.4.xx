#include "smi.h"
#include "ctrl.h"

/* These provide the minimum set of functions required to handle a CRTC */
static int _crtc_page_flip(struct drm_crtc *crtc, struct drm_framebuffer *fb, struct drm_pending_vblank_event *event, uint32_t page_flip_flags) {
    pr_info(">>%s:%d\n", __func__, __LINE__);
    return 0;
}

static const struct drm_crtc_funcs _crtc_funcs = {
    .set_config = drm_crtc_helper_set_config,
    .destroy = drm_crtc_cleanup,
    .page_flip = _crtc_page_flip,
};

static void _crtc_dpms(struct drm_crtc *crtc, int mode) {
    struct drm_device *dev = crtc->dev;

    pr_info(">>%s:%d %d\n", __func__, __LINE__, mode);
    if(mode == DRM_MODE_DPMS_STANDBY) {
	ctrl_clear(dev, PANEL_DISPLAY_CTRL, PANEL_DISPLAY_CTRL_DATA);
	ctrl_update(dev, CRT_DISPLAY_CTRL, CRT_DISPLAY_CTRL_BLANK);
    } else {
	// 0,3
	pr_info(">>%s:%d %d <- UNRECOGNIZED POWER MODE\n", __func__, __LINE__, mode);
    }
    /*unsigned pps = 0;
    switch (mode) {
    case DRM_MODE_DPMS_ON:
	// 0: enable video ports
	pps = PANEL_DISPLAY_CTRL_DATA;
	break;
    case DRM_MODE_DPMS_STANDBY:
	// 1: ?
	break;
    case DRM_MODE_DPMS_SUSPEND:
	// 2: ?
	break;
    case DRM_MODE_DPMS_OFF:
	// 3: disable video ports
	//reg_write(priv, REG_ENA_VP_0, 0x00);
	break;
    }*/

    //pdc_write(crtc->dev, (pdc_read(crtc->dev)&~PANEL_DISPLAY_CTRL_DATA)|mode?0:PANEL_DISPLAY_CTRL_DATA);
}
static inline u32 calc_pll(unsigned request_freq) {
    // as sm750 register definition, N located in 2,15 and M located in 1,255
    static unsigned od, n, m, pod;
    int N, M, X, d;
    int mini_diff = ~0;
    unsigned RN, quo, rem, fl_quo;
    unsigned input, request;
    unsigned tmpClock, adjusted_clock = 0;
    const int max_OD = 3;
    const unsigned input_freq = 14318181;

    request = request_freq/1000;
    input = input_freq/1000;
    pr_info(">>%s:%d request:%d input:%d\n", __func__, __LINE__, request, input);

    for (N = 15; N > 1; N--) {
	// RN will not exceed maximum long if @request <= 285 MHZ (for 32bit cpu)
	RN = N * request;
	quo = RN/input;
	rem = RN % input;// rem always small than 14318181
	fl_quo = (rem * 10000/input);

	for (d = 6; d >= 0; d--) {
	    X = (1<<d);
	    M = quo * X;
	    M += fl_quo * X/10000;
	    // round step
	    M += (fl_quo * X % 10000) > 5000?1:0;
	    if (M < 256 && M > 0) {
		unsigned int diff;

		tmpClock = input_freq * M/N/X;
		diff = abs(tmpClock - request_freq);
		if (diff < mini_diff) {
		    m = M;
		    n = N;
		    pod = 0;
		    if (d > max_OD)
			pod = d - max_OD;
		    od = d - pod;
		    mini_diff = diff;
		    adjusted_clock = tmpClock;
		}
	    }
	}
    }
    pr_info(">>%s:%d clk:: request:%d adjusted:%d pll:: pod:%d od:%d n:%d m:%d\n", __func__, __LINE__, request_freq, adjusted_clock, pod, od, n, m);
    return (((od<<4)|n)<<8)|m;
}

static bool _crtc_mode_fixup(struct drm_crtc *crtc, const struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode) {
    struct drm_device *dev = crtc->dev;

    pr_info(">>%s:%d (%d,%d) => (%d,%d)\n", __func__, __LINE__, mode->hdisplay, mode->vdisplay, adjusted_mode->hdisplay, adjusted_mode->vdisplay);
    ctrl_write(dev, PANEL_PLL_CTRL, PANEL_PLL_CTRL_POWER|calc_pll(1000*mode->clock));
    //ctrl_write(dev, PANEL_PLL_CTRL, PANEL_PLL_CTRL_POWER|calc_pll(148500000));
    return true;
}

static void _crtc_prepare(struct drm_crtc *crtc) {
    pr_info(">>%s:%d\n", __func__, __LINE__);
}
static void _crtc_commit(struct drm_crtc *crtc) {
    pr_info(">>%s:%d\n", __func__, __LINE__);
}

static inline void pr_display_mode(struct drm_display_mode *mode) {
    pr_info(">>%s:%d name:%s clock:%d hdisplay:%d hsync_start:%d hsync_end:%d htotal:%d hskew:%d vdisplay:%d vsync_start:%d vsync_end:%d vtotal:%d vscan:%d flags:%08x vrefresh:%d hsync:%d\n", __func__, __LINE__,
	mode->name,
	mode->clock,
	mode->hdisplay,
	mode->hsync_start,
	mode->hsync_end,
	mode->htotal,
	mode->hskew,
	mode->vdisplay,
	mode->vsync_start,
	mode->vsync_end,
	mode->vtotal,
	mode->vscan,
	mode->flags,
	mode->vrefresh,
	mode->hsync
    );
}

static void _crtc_mode_set_nofb(struct drm_crtc *crtc)  {
    struct drm_display_mode *mode;
    struct drm_device *dev = crtc->dev;
    uint32_t bpp = dev->mode_config.preferred_depth;

    pr_info(">>%s:%d\n", __func__, __LINE__);

    /*pr_display_mode(&crtc->mode);
    pr_display_mode(&crtc->hwmode);*/
    mode = &crtc->mode;

    ctrl_write(dev, PANEL_HORIZONTAL_TOTAL, (mode->htotal - 1)<<16|(mode->hdisplay - 1));
    ctrl_write(dev, PANEL_HORIZONTAL_SYNC, (mode->hsync_end - mode->hsync_start)<<16|(mode->hsync_start - 1));	//?
    ctrl_write(dev, PANEL_VERTICAL_TOTAL, (mode->vtotal-1)<<16|(mode->vdisplay - 1));
    ctrl_write(dev, PANEL_VERTICAL_SYNC, (mode->vsync_end - mode->vsync_start)<<16|(mode->vsync_start - 1));	//?

    ctrl_update(dev, PANEL_DISPLAY_CTRL, DISPLAY_CTRL_CLOCK_PHASE|DISPLAY_CTRL_TIMING|DISPLAY_CTRL_PLANE);	// white screen begins here
    ctrl_write(dev, PANEL_FB_ADDRESS, 0);
    ctrl_write(dev, PANEL_FB_WIDTH, (mode->hdisplay*(bpp>3))<<16|(mode->hdisplay*(bpp>>3)));	// TODO: u should use smth<<16|fb->pitches[0]
    ctrl_write(dev, PANEL_WINDOW_WIDTH, (mode->hdisplay-1)<<16|0);				// TODO: add hoffset
    ctrl_write(dev, PANEL_WINDOW_HEIGHT, (mode->vdisplay-1)<<16|0);				// TODO: add voffset
    ctrl_write(dev, PANEL_PLANE_BR, (mode->vdisplay-1)<<16|((mode->hdisplay-1)&PANEL_PLANE_BR_RIGHT_MASK));
    ctrl_update(dev, PANEL_DISPLAY_CTRL, (bpp>>4)&PANEL_DISPLAY_CTRL_FORMAT_MASK);
    //ctrl_update(dev, PANEL_DISPLAY_CTRL, PANEL_DISPLAY_CTRL_FORMAT_16|PANEL_DISPLAY_CTRL_FPEN|PANEL_DISPLAY_CTRL_DATA|PANEL_DISPLAY_CTRL_VBIASEN|PANEL_DISPLAY_CTRL_FPVDDEN);
    ctrl_update(dev, PANEL_DISPLAY_CTRL, PANEL_DISPLAY_CTRL_FPEN|PANEL_DISPLAY_CTRL_DATA|PANEL_DISPLAY_CTRL_VBIASEN|PANEL_DISPLAY_CTRL_FPVDDEN);
    pr_ctrl(dev);
}


static const struct drm_crtc_helper_funcs _crtc_helper_funcs = {
    .dpms		= _crtc_dpms,
    .prepare		= _crtc_prepare,
    .commit		= _crtc_commit,
    .mode_fixup		= _crtc_mode_fixup,
    .mode_set		= drm_helper_crtc_mode_set,
    .mode_set_nofb	= _crtc_mode_set_nofb,
    .mode_set_base	= drm_helper_crtc_mode_set_base,
};

//int drm_crtc_init(struct drm_device *dev, struct drm_crtc *crtc, const struct drm_crtc_funcs *funcs);

static const u32 _formats[] = {
    DRM_FORMAT_RGB565,
    DRM_FORMAT_RGB888,
    DRM_FORMAT_XRGB8888,
    DRM_FORMAT_ARGB8888,
    DRM_FORMAT_XRGB4444,
    DRM_FORMAT_ARGB4444,
    DRM_FORMAT_XRGB1555,
    DRM_FORMAT_ARGB1555,
    DRM_FORMAT_YUV422,

    /*DRM_FORMAT_ARGB8888,
    DRM_FORMAT_XRGB8888,
    DRM_FORMAT_RGB888,*/
};

static int _plane_atomic_check(struct drm_plane *plane, struct drm_plane_state *state) {
    pr_info(">>%s:%d\n", __func__, __LINE__);
    return 0;
}

static void _plane_atomic_update(struct drm_plane *plane, struct drm_plane_state *state) {
    pr_info(">>%s:%d\n", __func__, __LINE__);
}

static const struct drm_plane_helper_funcs _plane_helper_funcs = {
    .atomic_check = _plane_atomic_check,
    .atomic_update = _plane_atomic_update,
};

static void _plane_destroy(struct drm_plane *plane) {
    drm_plane_helper_disable(plane);
    drm_plane_cleanup(plane);
}

static const struct drm_plane_funcs _plane_funcs = {
    /*.update_plane	= drm_atomic_helper_update_plane,
    .disable_plane	= drm_atomic_helper_disable_plane,*/
    .destroy		= _plane_destroy,
    /*.reset		= drm_atomic_helper_plane_reset,
    .atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
    .atomic_destroy_state = drm_atomic_helper_plane_destroy_state,*/
};


static void _crtc_init(struct drm_device *drm) {
    int ret;
    struct _device *private = drm->dev_private;
    struct drm_plane *plane = &private->plane;
    struct drm_crtc *crtc = &private->crtc;

    pr_info(">>%s:%d\n", __func__, __LINE__);
    /*ret = drm_crtc_init(drm, crtc, &_crtc_funcs);
    if (ret < 0) {
	pr_info(">>>>drm_crt_init failed!!!\n");
	drm_crtc_cleanup(crtc);
	return;
    }*/
    /*struct drm_plane *primary = create_primary_plane(drm);			// cuurent point*/
    //drm_crtc_init_with_planes(drm, crtc, primary, NULL, _crtc_funcs, NULL);	// current point*/

    plane->format_default = true;
    /* possible_crtc's will be filled in later by crtc_init */
    ret = drm_universal_plane_init(drm, plane, 0, &_plane_funcs, _formats, ARRAY_SIZE(_formats), DRM_PLANE_TYPE_PRIMARY);
    if (ret)
	return;
    drm_plane_helper_add(plane, &_plane_helper_funcs);

    ret = drm_crtc_init_with_planes(drm, crtc, plane, NULL, &_crtc_funcs);
    if (ret < 0) {
	drm_crtc_cleanup(crtc);
	return;
    }

    //drm_mode_crtc_set_gamma_size(crtc, 256);
    drm_crtc_helper_add(crtc, &_crtc_helper_funcs);	// TODO
}

static void _encoder_disable(struct drm_encoder *encoder) {
    pr_info(">>%s:%d\n", __func__, __LINE__);
}
static void _encoder_enable(struct drm_encoder *encoder) {
    pr_info(">>%s:%d\n", __func__, __LINE__);
}

static void _encoder_prepare(struct drm_encoder *encoder) {
    pr_info(">>%s:%d\n", __func__, __LINE__);
}
static void _encoder_mode_set(struct drm_encoder *encoder, struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode) {
    pr_info(">>%s:%d\n", __func__, __LINE__);
}
static void _encoder_commit(struct drm_encoder *encoder) {
    pr_info(">>%s:%d\n", __func__, __LINE__);
}

static bool _encoder_mode_fixup(struct drm_encoder *encoder, const struct drm_display_mode *mode, struct drm_display_mode *adj_mode) {
    pr_info(">>%s:%d\n", __func__, __LINE__);
    return true;
}
static const struct drm_encoder_helper_funcs _encoder_helper_funcs = {
    .mode_fixup = _encoder_mode_fixup,
    .prepare = _encoder_prepare,
    .mode_set = _encoder_mode_set,
    .disable = _encoder_disable,
    .enable = _encoder_enable,
    .commit = _encoder_commit,
};
static const struct drm_encoder_funcs _encoder_funcs = {
    .destroy = drm_encoder_cleanup,
};

static struct drm_encoder *_encoder_init(struct drm_device *drm) {
    struct _device *private =  drm->dev_private;
    struct drm_encoder *encoder = &private->encoder;

    pr_info(">>%s:%d\n", __func__, __LINE__);
    encoder->possible_crtcs = 0x1;	// 1 << drm_crtc_index(crtc)
    //encoder->possible_crtcs |= drm_crtc_mask(crtc);
    drm_encoder_init(drm, encoder, &_encoder_funcs, DRM_MODE_ENCODER_DAC); //TODO: add extra params "char *name"
    drm_encoder_helper_add(encoder, &_encoder_helper_funcs);
    return encoder;
}

static inline void pr_cmdline_mode(struct drm_cmdline_mode *mode) {
    pr_info(">>%s:%d %dx%d-%d@%d\n", __func__, __LINE__, mode->xres, mode->yres, mode->bpp, mode->refresh);
}
static int _connector_get_modes(struct drm_connector *connector) {
    int width = 1920, height = 1080;
    /*struct drm_display_mode *mode = drm_mode_create(dev);
    mode->hdisplay = 320;
    mode->vdisplay = 240;

    mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
    drm_mode_set_name(mode);*/

    //struct drm_display_mode *mode = drm_cvt_mode(connector->dev, width, height, 60, false, false, false);	// TODO: try GTF formula (drm_gtf_mode)
    struct drm_display_mode *mode = drm_gtf_mode(connector->dev, width, height, 60, false, false);	// TODO: try GTF formula (drm_gtf_mode)

    pr_info(">>%s:%d\n", __func__, __LINE__);
    pr_cmdline_mode(&connector->cmdline_mode);
    mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
    drm_mode_probed_add(connector, mode);

    return 1;
}
static int _connector_mode_valid(struct drm_connector *connector, struct drm_display_mode *mode) {
    //struct drm_device *dev = connector->dev;
    int bpp = 32;

    pr_info(">>%s:%d\n", __func__, __LINE__);
    pr_cmdline_mode(&connector->cmdline_mode);
    /*if ((mode->hdisplay % 8) != 0 || (mode->hsync_start % 8) != 0 || (mode->hsync_end % 8) != 0 || (mode->htotal % 8) != 0) {return MODE_H_ILLEGAL;}
    if (mode->crtc_hdisplay > 2048 || mode->crtc_hsync_start > 4096 || mode->crtc_hsync_end > 4096 || mode->crtc_htotal > 4096 || mode->crtc_vdisplay > 2048 || mode->crtc_vsync_start > 4096 || mode->crtc_vsync_end > 4096 || mode->crtc_vtotal > 4096) {return MODE_BAD;}*/

    if (connector->cmdline_mode.specified) {	// Validate the mode input by the user
	if (connector->cmdline_mode.bpp_specified)
		bpp = connector->cmdline_mode.bpp;
    }
    /*if ((mode->hdisplay * mode->vdisplay * (bpp/8)) > mdev->mc.vram_size) {
	if (connector->cmdline_mode.specified)
	    connector->cmdline_mode.specified = false;
	return MODE_BAD;
    }*/
    return MODE_OK;
}

static struct drm_encoder *_connector_best_encoder(struct drm_connector *connector) {
    int enc_id = connector->encoder_ids[0];
    pr_info(">>%s:%d\n", __func__, __LINE__);
    if (enc_id)
	return drm_encoder_find(connector->dev, enc_id);
    return NULL;
}
//static struct drm_encoder *_connector_best_encoder(struct drm_connector *connector) {return drm_encoder_find(connector->dev, connector->encoder_ids[0]);}
static const struct drm_connector_helper_funcs _connector_helper_funcs = {
    .get_modes = _connector_get_modes,
    .mode_valid = _connector_mode_valid,
    .best_encoder = _connector_best_encoder,
    //.best_encoder = drm_atomic_helper_best_encoder,
};

static enum drm_connector_status _connector_detect(struct drm_connector *connector, bool force) {
    pr_info(">>%s:%d\n", __func__, __LINE__);
    return connector_status_connected;
}
static const struct drm_connector_funcs _connector_funcs = {
    .dpms = drm_helper_connector_dpms,
    .detect = _connector_detect,
    .fill_modes = drm_helper_probe_single_connector_modes,
    .destroy = drm_connector_cleanup,
};


static struct drm_connector *_connector_init(struct drm_device *drm) {
    struct _device *private =  drm->dev_private;
    struct drm_connector *connector = &private->connector;

    pr_info(">>%s:%d\n", __func__, __LINE__);
    drm_connector_init(drm, connector, &_connector_funcs, DRM_MODE_CONNECTOR_VGA);
    drm_connector_helper_add(connector, &_connector_helper_funcs);
    drm_connector_register(connector);
    return connector;
}


static const struct drm_mode_config_funcs mode_funcs = {
    .fb_create = drm_fb_cma_create,
};

int _kms_init(struct drm_device *drm) {
    //struct _device *private = drm->dev_private;
    pr_info(">>%s:%d\n", __func__, __LINE__);
    drm_mode_config_init(drm);
    //
    drm->mode_config.min_width = 0;
    drm->mode_config.min_height = 0;
    drm->mode_config.max_width = 1920;
    drm->mode_config.max_height = 1080;
    //
    drm->mode_config.funcs = &mode_funcs;
    // dumb ioctl
    drm->mode_config.preferred_depth = 32;	//  here the place to change "bpp" global
    drm->mode_config.prefer_shadow = 1;	// 0|1?
    //
    drm->mode_config.fb_base = pci_resource_start(drm->pdev, 0); //  to remove
    //
    _crtc_init(drm);

    drm_mode_connector_attach_encoder(_connector_init(drm), _encoder_init(drm));
    drm_mode_config_reset(drm);			// reset all the states of crtc/plane/encoder/connector*/
    drm_helper_disable_unused_functions(drm);	// disable all the possible outputs/crtcs before entering KMS mode
    drm_kms_helper_poll_init(drm);		// init kms poll for handling hpd

    return 0;
}

void _kms_fini(struct drm_device *drm) {
    pr_info(">>%s:%d\n", __func__, __LINE__);
    drm_mode_config_cleanup(drm);
}
