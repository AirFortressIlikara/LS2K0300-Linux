// SPDX-License-Identifier: GPL-2.0+
/*
 * DRM driver for display panels connected to a Sitronix ST7789V
 * display controller in SPI mode.
 * Copyright 2021 CNflysky
 */
#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <video/mipi_display.h>

#define ST7789V_MY BIT(7)
#define ST7789V_MX BIT(6)
#define ST7789V_MV BIT(5)
#define ST7789V_RGB BIT(3)

static void st7789v_pipe_enable(struct drm_simple_display_pipe *pipe,
                                struct drm_crtc_state *crtc_state,
                                struct drm_plane_state *plane_state) {
  struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
  struct mipi_dbi *dbi = &dbidev->dbi;
  int idx;
  uint8_t addr_mode;
  if (!drm_dev_enter(pipe->crtc.dev, &idx)) return;
  DRM_DEBUG_KMS("\n");
  int ret = mipi_dbi_poweron_reset(dbidev);
  if (ret) drm_dev_exit(idx);
  // init seq begin
  mipi_dbi_command(dbi, MIPI_DCS_SET_PIXEL_FORMAT, 0x05);
  // Porch setting
  mipi_dbi_command(dbi, 0xB2, 0x0C, 0x0C, 0x00, 0x33, 0x33);
  // Gate control
  mipi_dbi_command(dbi, 0xB7, 0x35);
  // VCOM settings
  mipi_dbi_command(dbi, 0xBB, 0x19);
  // LCM Control
  mipi_dbi_command(dbi, 0xC0, 0x2C);
  // VDV and VRH Command Enable
  mipi_dbi_command(dbi, 0xC2, 0x01);
  // VRH Set
  mipi_dbi_command(dbi, 0xC3, 0x12);
  // VDV Set
  mipi_dbi_command(dbi, 0xC4, 0x20);
  // Frame Rate Control in Normal Mode
  mipi_dbi_command(dbi, 0xC6, 0x0F);
  // Power control 1
  mipi_dbi_command(dbi, 0xD0, 0xA4, 0xA1);
  // Positive Voltage Gamma Control
  mipi_dbi_command(dbi, 0xE0, 0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54,
                   0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23);
  // Negative Voltage Gamma Control
  mipi_dbi_command(dbi, 0xE1, 0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44,
                   0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23);
  mipi_dbi_command(dbi, MIPI_DCS_ENTER_INVERT_MODE);
  switch (dbidev->rotation) {
    default:
      addr_mode = 0;
      break;
    case 90:
      addr_mode = ST7789V_MX | ST7789V_MV;
      break;
    case 180:
      addr_mode = ST7789V_MX | ST7789V_MY;
      break;
    case 270:
      addr_mode = ST7789V_MY | ST7789V_MV;
      break;
  }
  //if colors were inverted(blue as red),then uncomment the following line:
  //addr_mode |= ST7789V_RGB;
  mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);
  mipi_dbi_command(dbi, MIPI_DCS_EXIT_SLEEP_MODE);
  mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_ON);
  msleep(20);
  // init seq end
  mipi_dbi_enable_flush(dbidev, crtc_state, plane_state);
}

static const struct drm_simple_display_pipe_funcs st7789v_pipe_funcs = {
    .enable = st7789v_pipe_enable,
    .disable = mipi_dbi_pipe_disable,
    .update = mipi_dbi_pipe_update,
    .prepare_fb = drm_gem_fb_simple_display_pipe_prepare_fb,
};

static const struct drm_display_mode st7789v_mode = {
    DRM_SIMPLE_MODE(240, 240, 26, 26),
};

DEFINE_DRM_GEM_CMA_FOPS(st7789v_fops);

static struct drm_driver st7789v_driver = {
    .driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
    .fops = &st7789v_fops,
    DRM_GEM_CMA_DRIVER_OPS_VMAP,
    .debugfs_init = mipi_dbi_debugfs_init,
    .name = "st7789v",
    .desc = "Sitronix ST7789V",
    .date = "20211022",
    .major = 1,
    .minor = 0,
};

static const struct of_device_id st7789v_of_match[] = {
    {.compatible = "sitronix,st7789v_240x240"},
    {},
};

MODULE_DEVICE_TABLE(of, st7789v_of_match);

static const struct spi_device_id st7789v_id[] = {
    {"st7789v_240x240", 0},
    {},
};

MODULE_DEVICE_TABLE(spi, st7789v_id);

static int st7789v_probe(struct spi_device *spi) {
  struct device *dev = &spi->dev;
  struct drm_device *drm;
  uint32_t rotation = 0;
  struct mipi_dbi_dev *dbidev =
      devm_drm_dev_alloc(dev, &st7789v_driver, struct mipi_dbi_dev, drm);
  if (IS_ERR(dbidev)) return PTR_ERR(dbidev);
  struct mipi_dbi *dbi = &dbidev->dbi;
  drm = &dbidev->drm;
  dbi->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
  if (IS_ERR(dbi->reset)) {
    DRM_DEV_ERROR(dev, "Failed to get gpio 'reset'\n");
    return PTR_ERR(dbi->reset);
  }
  struct gpio_desc *dc = devm_gpiod_get_optional(dev, "dc", GPIOD_OUT_LOW);
  if (IS_ERR(dc)) {
    DRM_DEV_ERROR(dev, "Failed to get gpio 'dc'\n");
    return PTR_ERR(dc);
  }
  dbidev->backlight = devm_of_find_backlight(dev);
  if (IS_ERR(dbidev->backlight)) return PTR_ERR(dbidev->backlight);
  device_property_read_u32(dev, "rotation", &rotation);
  int ret = mipi_dbi_spi_init(spi, dbi, dc);
  if (ret) return ret;
  ret = mipi_dbi_dev_init(dbidev, &st7789v_pipe_funcs, &st7789v_mode, rotation);
  if (ret) return ret;
  drm_mode_config_reset(drm);
  ret = drm_dev_register(drm, 0);
  if (ret) return ret;
  spi_set_drvdata(spi, drm);
  drm_fbdev_generic_setup(drm, 0);
  return 0;
}

static int st7789v_remove(struct spi_device *spi) {
  struct drm_device *drm = spi_get_drvdata(spi);
  drm_dev_unplug(drm);
  drm_atomic_helper_shutdown(drm);
  return 0;
}

static void st7789v_shutdown(struct spi_device *spi) {
  drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static struct spi_driver st7789v_spi_driver = {
    .driver =
        {
            .name = "st7789v",
            .of_match_table = st7789v_of_match,
        },
    .id_table = st7789v_id,
    .probe = st7789v_probe,
    .remove = st7789v_remove,
    .shutdown = st7789v_shutdown,
};

module_spi_driver(st7789v_spi_driver);
MODULE_DESCRIPTION("Sitronix ST7789V DRM driver");
MODULE_AUTHOR("CNflysky <cnflysky@qq.com>");
MODULE_LICENSE("GPL");
