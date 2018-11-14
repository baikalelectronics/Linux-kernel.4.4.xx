#ifndef _SMI_H
#define _SMI_H

#include <linux/io.h>
#include <linux/fb.h>
#include <linux/console.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_gem.h>
#include <drm/drm_legacy.h>

struct _device {
    struct device	*dev;
    struct drm_device	*drm;
    //struct pci_dev	*pdev;


    struct drm_plane plane;
    struct drm_crtc crtc;

    struct drm_encoder encoder;
    struct drm_connector connector;

    //struct drm_fbdev_cma *fbdev;
    struct drm_fb_helper helper;
    struct drm_framebuffer fb;

    //drm_local_map_t *mmio;
    //void __iomem *mmio;
    //drm_local_map_t *fb;
    drm_local_map_t *mmio;
    //drm_local_map_t *aperture;	// sarea;? // video area??
};

#define DRIVER_NAME "smi"

int _kms_init(struct drm_device *drm);
void _kms_fini(struct drm_device *drm);

#endif /* _SMI_H */