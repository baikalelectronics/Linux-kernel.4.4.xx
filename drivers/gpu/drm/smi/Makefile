ccflags-y := -Iinclude/drm
smi-y := drv.o kms.o

smi-$(CONFIG_DRM_FBDEV_EMULATION) += fbdev.o

obj-$(CONFIG_DRM_SMI) += smi.o
