/*
 * Copyright (C) Baikal Electronics Co.Ltd
 * Author:Andrew Khorolsky <a.khorolsky@baikal-electronics.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _FBDEV_H
#define _FBDEV_H

#ifdef CONFIG_DRM_FBDEV_EMULATION
int _fbdev_init(struct drm_device *dev);
void _fbdev_fini(struct drm_device *dev);
#else
static inline int _fbdev_init(struct drm_device *dev) {return 0;}

static inline void _fbdev_fini(struct drm_device *dev) {}
#endif

#endif /* _FBDEV_H */
