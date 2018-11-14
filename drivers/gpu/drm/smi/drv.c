#include <linux/mm.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <linux/dma-mapping.h>

#include <drm/drm_gem_cma_helper.h>

#include "smi.h"
#include "ctrl.h"
#include "fbdev.h"

static int _driver_unload(struct drm_device *dev) {
    struct _device *private = dev->dev_private;

    pr_info("%s:%d\n", __func__, __LINE__);
    kfree(private);
    dev->dev_private = NULL;
    return 0;
}

static inline u32 pll_fmt(unsigned od, unsigned n, unsigned m) {
    return (((od<<4)|n)<<8)|m;
}
static inline void _ctrl_init(struct drm_device *dev) {
    pr_info(">>%s:%d\n", __func__, __LINE__);

    ctrl_update(dev, VGA_CONFIGURATION, VGA_CONFIGURATION_PLL|VGA_CONFIGURATION_MODE);	// set panel pll and graphic mode via mmio_88

    //ctrl_write(dev, MXCLK_PLL_CTRL, MXCLK_PLL_CTRL_POWER|compute_pll(290000));		// set main chip clock. Master Clock Control: MXCLK_PLL
    ctrl_write(dev, MXCLK_PLL_CTRL, MXCLK_PLL_CTRL_POWER|pll_fmt(0, 12, 243));		// set main chip clock. Master Clock Control: MXCLK_PLL @290MHz
    ctrl_wrup(dev, MODE0_GATE, MODE0_GATE_MCLK_112MHZ|MODE0_GATE_M2XCLK_336MHZ);	// set memory and master clocks

    ctrl_clear(dev, MISC_CTRL, MISC_CTRL_LOCALMEM_RESET);				// Reset the memory controller. If the memory controller is not reset in SM750, the system might hang when sw accesses the memory.
    ctrl_update(dev, MISC_CTRL, MISC_CTRL_LOCALMEM_RESET);				// The memory should be resetted after changing the MXCLK.

    ctrl_update(dev, SYSTEM_CTRL, SYSTEM_CTRL_DE_ABORT);				/* engine reset */
    ctrl_clear(dev, SYSTEM_CTRL, SYSTEM_CTRL_DE_ABORT);
}

static inline void pr_local_map(drm_local_map_t *map) {
    pr_info("type:%d mtrr:%d handle:%p flags:%08x size:%08lx offset:%08llx\n", map->type, map->mtrr, map->handle, map->flags, map->size, map->offset);
}

static int _driver_load(struct drm_device *dev, unsigned long flags) {
	struct _device *dev_priv = dev->dev_private;
	struct pci_dev *pdev = dev->pdev;
	int ret;

	pr_info(">>%s:%d\n", __func__, __LINE__);
	dev_priv = kzalloc(sizeof(*dev_priv), GFP_KERNEL);
	if (dev_priv == NULL)
		return -ENOMEM;
	dev_priv->drm = dev;
	dev->dev_private = dev_priv;

	pci_set_master(pdev);


	//ret = drm_legacy_addmap(dev, pci_resource_start(pdev, 0), pci_resource_len(pdev, 0), _DRM_REGISTERS/*_DRM_FRAME_BUFFER*/, _DRM_WRITE_COMBINING, &dev_priv->fb);
	//if (ret)
	//    return ret;
	ret = drm_legacy_addmap(dev, pci_resource_start(dev->pdev, 1), pci_resource_len(dev->pdev, 1), _DRM_REGISTERS, _DRM_READ_ONLY|_DRM_DRIVER, &dev_priv->mmio);
	if (ret) {
		return ret;
	}
	pr_local_map(dev_priv->mmio);
	//ret = drm_legacy_addmap(dev, pci_resource_start(pdev, 6), pci_resource_len(pdev, 6), _DRM_FRAME_BUFFER, _DRM_WRITE_COMBINING, &dev_priv->aperture);
	//if (ret)
	//    return ret;

	_ctrl_init(dev);	// hw_init?

	ret = _kms_init(dev);
	if (ret)
	    goto err;

	ret = _fbdev_init(dev);
	if (ret)
	    goto err;
	//drm_kms_helper_poll_init(dev);	// init kms poll for handling hpd

	return 0;

err:
	_driver_unload(dev);
	return ret;

}


static int _drm_enable_vblank(struct drm_device *dev, unsigned int pipe) {
    pr_info("%s:%d\n", __func__, __LINE__);
    return 0;
}
static void _drm_disable_vblank(struct drm_device *dev, unsigned int pipe) {
    pr_info("%s:%d\n", __func__, __LINE__);
}

static const struct file_operations fops = {
    .owner              = THIS_MODULE,
    .open               = drm_open,
    .release            = drm_release,
    .unlocked_ioctl     = drm_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl       = drm_compat_ioctl,
#endif
    .poll               = drm_poll,
    .read               = drm_read,
    .llseek             = no_llseek,
    .mmap               = drm_gem_cma_mmap,
};

static int _kick_out_firmware_fb(struct pci_dev *pdev) {
    struct apertures_struct *ap;

    pr_info("%s:%d\n", __func__, __LINE__);
    return 0;
    ap = alloc_apertures(1);
    if (!ap)
	return -ENOMEM;

    ap->ranges[0].base = pci_resource_start(pdev, 0);
    ap->ranges[0].size = pci_resource_len(pdev, 0);
    remove_conflicting_framebuffers(ap, DRIVER_NAME"drmfb", pdev->resource[PCI_ROM_RESOURCE].flags & IORESOURCE_ROM_SHADOW); // sm750fb
    kfree(ap);

    return 0;
}

static struct drm_driver _drm_driver = {
    .driver_features	= DRIVER_GEM | DRIVER_MODESET,
    .load		= _driver_load,
    .unload		= _driver_unload,
    .set_busid		= drm_pci_set_busid,
    .get_vblank_counter = drm_vblank_no_hw_counter,
    .enable_vblank	= _drm_enable_vblank,
    .disable_vblank	= _drm_disable_vblank,
    .dumb_create	= drm_gem_cma_dumb_create,
    .dumb_map_offset	= drm_gem_cma_dumb_map_offset,
    .dumb_destroy	= drm_gem_dumb_destroy,
    .fops = &fops,
    .name		= DRIVER_NAME,
    .desc		= "silicon motion display engine",
    .date		= "20160715",
    .major		= 1,
    .minor		= 0,
};

static int _pci_probe(struct pci_dev *pdev, const struct pci_device_id *pid) {
    int ret;

    pr_info("%s:%d\n", __func__, __LINE__);
    ret = _kick_out_firmware_fb(pdev);
    if (ret)
	return ret;

    return drm_get_pci_dev(pdev, pid, &_drm_driver);	// pci_enable_device->pci_set_drvdata
}

static void _pci_remove(struct pci_dev *pdev) {
    pr_info("%s:%d\n", __func__, __LINE__);
    drm_put_dev(pci_get_drvdata(pdev));
}

static const struct pci_device_id _ids[] = {
    {PCI_DEVICE(0x126f, 0x718),},
    {PCI_DEVICE(0x126f, 0x750),},
    {PCI_DEVICE(0x126f, 0x501),},
    {0,}
};

static struct pci_driver _pci_driver = {
    .name = DRIVER_NAME,
    .id_table = _ids,
    .probe = _pci_probe,
    .remove = _pci_remove,
};

static int __init smi_init(void){
    return drm_pci_init(&_drm_driver, &_pci_driver);
}
static void __exit smi_exit(void) {drm_pci_exit(&_drm_driver, &_pci_driver);}

module_init(smi_init);
module_exit(smi_exit);

MODULE_AUTHOR("Andrew Khorolsky <a.khorolsky@baikalelectronics.ru>");
