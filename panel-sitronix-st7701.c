// SPDX-License-Identifier: GPL-2.0+
/**
 *-----------------------------------------------------------------------------
 * @brief Melbourne Instruments Sitronix ST7701 panel display driver.
 *	      Most of this code is based on the original ST7701 driver written
 *        by Jagan Teki from Amarula Solutions.
 * @copyright 2021-2023 Melbourne Instruments, Australia
 *-----------------------------------------------------------------------------
 */

#include <linux/fb.h>
#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <linux/backlight.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <video/mipi_display.h>

/* Command2 BKx selection command */
#define DSI_CMD2BKX_SEL			0xFF

/* Command2, BK0 commands */
#define DSI_CMD2_BK0_PVGAMCTRL		0xB0 /* Positive Voltage Gamma Control */
#define DSI_CMD2_BK0_NVGAMCTRL		0xB1 /* Negative Voltage Gamma Control */
#define DSI_CMD2_BK0_LNESET		0xC0 /* Display Line setting */
#define DSI_CMD2_BK0_PORCTRL		0xC1 /* Porch control */
#define DSI_CMD2_BK0_INVSEL		0xC2 /* Inversion selection, Frame Rate Control */

/* Command2, BK1 commands */
#define DSI_CMD2_BK1_VRHS		0xB0 /* Vop amplitude setting */
#define DSI_CMD2_BK1_VCOM		0xB1 /* VCOM amplitude setting */
#define DSI_CMD2_BK1_VGHSS		0xB2 /* VGH Voltage setting */
#define DSI_CMD2_BK1_TESTCMD		0xB3 /* TEST Command Setting */
#define DSI_CMD2_BK1_VGLS		0xB5 /* VGL Voltage setting */
#define DSI_CMD2_BK1_PWCTLR1		0xB7 /* Power Control 1 */
#define DSI_CMD2_BK1_PWCTLR2		0xB8 /* Power Control 2 */
#define DSI_CMD2_BK1_SPD1		0xC1 /* Source pre_drive timing set1 */
#define DSI_CMD2_BK1_SPD2		0xC2 /* Source EQ2 Setting */
#define DSI_CMD2_BK1_MIPISET1		0xD0 /* MIPI Setting 1 */

/**
 * Command2 with BK function selection.
 *
 * BIT[4, 0]: [CN2, BKXSEL]
 * 10 = CMD2BK0, Command2 BK0
 * 11 = CMD2BK1, Command2 BK1
 * 00 = Command2 disable
 */
#define DSI_CMD2BK1_SEL			0x11
#define DSI_CMD2BK0_SEL			0x10
#define DSI_CMD2BKX_SEL_NONE		0x00

/* Command2, BK0 bytes */
#define DSI_LINESET_LINE		0x69
#define DSI_LINESET_LDE_EN		BIT(7)
#define DSI_LINESET_LINEDELTA		GENMASK(1, 0)
#define DSI_CMD2_BK0_LNESET_B1		DSI_LINESET_LINEDELTA
#define DSI_CMD2_BK0_LNESET_B0		(DSI_LINESET_LDE_EN | DSI_LINESET_LINE)
#define DSI_INVSEL_DEFAULT		GENMASK(5, 4)
#define DSI_INVSEL_NLINV		GENMASK(2, 0)
#define DSI_INVSEL_RTNI			GENMASK(2, 1)
#define DSI_CMD2_BK0_INVSEL_B1		DSI_INVSEL_RTNI
#define DSI_CMD2_BK0_INVSEL_B0		(DSI_INVSEL_DEFAULT | DSI_INVSEL_NLINV)
#define DSI_CMD2_BK0_PORCTRL_B0(m)	((m)->vtotal - (m)->vsync_end)
#define DSI_CMD2_BK0_PORCTRL_B1(m)	((m)->vsync_start - (m)->vdisplay)

/* Command2, BK1 bytes */
#define DSI_CMD2_BK1_VRHA_SET		0x45
#define DSI_CMD2_BK1_VCOM_SET		0x13
#define DSI_CMD2_BK1_VGHSS_SET		GENMASK(2, 0)
#define DSI_CMD2_BK1_TESTCMD_VAL	BIT(7)
#define DSI_VGLS_DEFAULT		BIT(6)
#define DSI_VGLS_SEL			GENMASK(2, 0)
#define DSI_CMD2_BK1_VGLS_SET		(DSI_VGLS_DEFAULT | DSI_VGLS_SEL)
#define DSI_PWCTLR1_AP			BIT(7) /* Gamma OP bias, max */
#define DSI_PWCTLR1_APIS		BIT(2) /* Source OP input bias, min */
#define DSI_PWCTLR1_APOS		BIT(0) /* Source OP output bias, min */
#define DSI_CMD2_BK1_PWCTLR1_SET	(DSI_PWCTLR1_AP | DSI_PWCTLR1_APIS | \
                    DSI_PWCTLR1_APOS)
#define DSI_PWCTLR2_AVDD		BIT(5) /* AVDD 6.6v */
#define DSI_PWCTLR2_AVCL		0x0    /* AVCL -4.4v */
#define DSI_CMD2_BK1_PWCTLR2_SET	(DSI_PWCTLR2_AVDD | DSI_PWCTLR2_AVCL)
#define DSI_SPD1_T2D			BIT(3)
#define DSI_CMD2_BK1_SPD1_SET		(GENMASK(6, 4) | DSI_SPD1_T2D)
#define DSI_CMD2_BK1_SPD2_SET		DSI_CMD2_BK1_SPD1_SET
#define DSI_MIPISET1_EOT_EN		BIT(3)
#define DSI_CMD2_BK1_MIPISET1_SET	(BIT(7) | DSI_MIPISET1_EOT_EN)

struct st7701_panel_desc {
    const struct drm_display_mode *mode;
    unsigned int lanes;
    unsigned long flags;
    enum mipi_dsi_pixel_format format;
    const char *const *supply_names;
    unsigned int num_supplies;
    unsigned int panel_sleep_delay;
};

struct st7701 {
    struct drm_panel panel;
    struct mipi_dsi_device *dsi;
    const struct st7701_panel_desc *desc;
    struct backlight_device *backlight;
    struct regulator_bulk_data *supplies;
    struct gpio_desc *reset;
    unsigned int sleep_delay;
};

static inline struct st7701 *panel_to_st7701(struct drm_panel *panel)
{
    return container_of(panel, struct st7701, panel);
}

static inline int st7701_dsi_write(struct st7701 *st7701, const void *seq,
                   size_t len)
{
    return mipi_dsi_dcs_write_buffer(st7701->dsi, seq, len);
}

#define ST7701_DSI(st7701, seq...)				\
    {							\
        const u8 d[] = { seq };				\
        st7701_dsi_write(st7701, d, ARRAY_SIZE(d));	\
    }

static void st7701_init_sequence(struct st7701 *st7701)
{
    ST7701_DSI(st7701, MIPI_DCS_EXIT_SLEEP_MODE);
    msleep(120);

    /* Command2, BK3 */
    ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x13);
    ST7701_DSI(st7701, 0xEF, 0x08);

    /* Command2, BK0 */
    ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x10);
    ST7701_DSI(st7701, 0xC0, 0xE9, 0x03);
    ST7701_DSI(st7701, 0xC1, 0x12, 0x0C);
    ST7701_DSI(st7701, 0xC2, 0x20, 0x0A);
    ST7701_DSI(st7701, 0xC7, 0x04);
    ST7701_DSI(st7701, 0xCC, 0x10);
    ST7701_DSI(st7701, 0xB0, 0x07, 0x14, 0x9C, 0x0B, 0x10, 0x06, 0x08, 0x09, 0x08, 0x22, 0x02,
                       0x4F, 0x0E, 0x66, 0x2D, 0x1C);
    ST7701_DSI(st7701, 0xB1, 0x09, 0x17, 0x9E, 0x0F, 0x11, 0x06, 0x0C, 0x08, 0x08, 0x26, 0x04,
                       0x51, 0x10, 0x6A, 0x33, 0x1D);

    /* Command2, BK1 */
    ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x11);
    ST7701_DSI(st7701, 0xB0, 0x4D);
    ST7701_DSI(st7701, 0xB1, 0x43);
    ST7701_DSI(st7701, 0xB2, 0x84);
    ST7701_DSI(st7701, 0xB3, 0x80);
    ST7701_DSI(st7701, 0xB5, 0x45);
    ST7701_DSI(st7701, 0xB7, 0x85);
    ST7701_DSI(st7701, 0xB8, 0x33);
    ST7701_DSI(st7701, 0xC1, 0x78);
    ST7701_DSI(st7701, 0xC2, 0x78);
    ST7701_DSI(st7701, 0xD0, 0x88);

    ST7701_DSI(st7701, 0xE0, 0x00, 0x00, 0x02);
    ST7701_DSI(st7701, 0xE1, 0x06, 0xA0, 0x08, 0xA0, 0x05, 0xA0, 0x07,
           0xA0, 0x00, 0x44, 0x44);
    ST7701_DSI(st7701, 0xE2, 0x30, 0x30, 0x44, 0x44, 0x6E, 0xA0, 0x00,
           0x00, 0x6E, 0xA0, 0x00, 0x00);
    ST7701_DSI(st7701, 0xE3, 0x00, 0x00, 0x33, 0x33);
    ST7701_DSI(st7701, 0xE4, 0x44, 0x44);
    ST7701_DSI(st7701, 0xE5, 0x0D, 0x69, 0x0A, 0xA0, 0x0F, 0x6B, 0x0A, 0xA0,
           0x09, 0x65, 0x0A, 0xA0, 0x0B, 0x67, 0x0A, 0xA0);
    ST7701_DSI(st7701, 0xE6, 0x00, 0x00, 0x33, 0x33);
    ST7701_DSI(st7701, 0xE7, 0x44, 0x44);
    ST7701_DSI(st7701, 0xE8, 0x0C, 0x68, 0x0A, 0xA0, 0x0E, 0x6A, 0x0A,
           0xA0, 0x08, 0x64, 0x0A, 0xA0, 0x0A, 0x66, 0x0A, 0xA0);
    ST7701_DSI(st7701, 0xE9, 0x36, 0x00);
    ST7701_DSI(st7701, 0xEB, 0x00, 0x01, 0xE4, 0xE4, 0x44, 0x88, 0x40);
    ST7701_DSI(st7701, 0xED, 0xFF, 0x45, 0x67, 0xFB, 0x01, 0x2A, 0xFC,
           0xFF, 0xFF, 0xCF, 0xA2, 0x10, 0xBF, 0x76, 0x54, 0xFF);
    ST7701_DSI(st7701, 0xEF, 0x10, 0x0D, 0x04, 0x08, 0x3F, 0x1F);
    ST7701_DSI(st7701, 0x36, 0x10);

    /* Disable Command2 */
    ST7701_DSI(st7701, DSI_CMD2BKX_SEL,
           0x77, 0x01, 0x00, 0x00, 0x00);
    ST7701_DSI(st7701, MIPI_DCS_SET_DISPLAY_ON);
}

static int st7701_prepare(struct drm_panel *panel)
{
    struct st7701 *st7701 = panel_to_st7701(panel);

    // Make an input and it will get pulled up
    msleep(1);
    gpiod_direction_input(st7701->reset);
    msleep(150);
    printk(KERN_INFO "st7701_prepare");
    st7701_init_sequence(st7701);
    return 0;
}

static int st7701_enable(struct drm_panel *panel)
{
    return 0;
}

static int st7701_disable(struct drm_panel *panel)
{
    return 0;
}

static int st7701_unprepare(struct drm_panel *panel)
{
    return 0;
}

static int st7701_get_modes(struct drm_panel *panel)                
{
    struct st7701 *st7701 = panel_to_st7701(panel);
    const struct drm_display_mode *desc_mode = st7701->desc->mode;
    struct drm_display_mode *mode;

    mode = drm_mode_duplicate(panel->drm, desc_mode);
    if (!mode) {
        dev_err(&st7701->dsi->dev, "failed to add mode %ux%u@%u\n",
            desc_mode->hdisplay, desc_mode->vdisplay,
            drm_mode_vrefresh(desc_mode));
        return -ENOMEM;
    }

    drm_mode_set_name(mode);
    drm_mode_probed_add(panel->connector, mode);

    panel->connector->display_info.width_mm = desc_mode->width_mm;
    panel->connector->display_info.height_mm = desc_mode->height_mm;
    return 1;
}

static const struct drm_panel_funcs st7701_funcs = {
    .disable	= st7701_disable,
    .unprepare	= st7701_unprepare,
    .prepare	= st7701_prepare,
    .enable		= st7701_enable,
    .get_modes	= st7701_get_modes,
};

static const struct drm_display_mode ts8550b_mode = {
    .clock		= 27500,

    .hdisplay	= 480,
    .hsync_start	= 480 + 38,
    .hsync_end	= 480 + 38 + 12,
    .htotal		= 480 + 38 + 12 + 12,

    .vdisplay	= 854,
    .vsync_start	= 854 + 18,
    .vsync_end	= 854 + 18 + 8,
    .vtotal		= 854 + 18 + 8 + 4,

    .width_mm	= 69,
    .height_mm	= 139,

    .type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static const char * const ts8550b_supply_names[] = {
    "VCC",
    "IOVCC",
};

static const struct st7701_panel_desc ts8550b_desc = {
    .mode = &ts8550b_mode,
    .lanes = 2,
    .flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS,
    .format = MIPI_DSI_FMT_RGB888,
    .supply_names = ts8550b_supply_names,
    .num_supplies = ARRAY_SIZE(ts8550b_supply_names),
    .panel_sleep_delay = 80, /* panel need extra 80ms for sleep out cmd */
};

static int st7701_dsi_probe(struct mipi_dsi_device *dsi)
{
    const struct st7701_panel_desc *desc;
    struct st7701 *st7701;
    int ret, i;

    st7701 = devm_kzalloc(&dsi->dev, sizeof(*st7701), GFP_KERNEL);
    if (!st7701)
        return -ENOMEM;

    desc = of_device_get_match_data(&dsi->dev);
    dsi->mode_flags = desc->flags;
    dsi->format = desc->format;
    dsi->lanes = desc->lanes;

    st7701->supplies = devm_kcalloc(&dsi->dev, desc->num_supplies,
                    sizeof(*st7701->supplies),
                    GFP_KERNEL);
    if (!st7701->supplies)
        return -ENOMEM;

    for (i = 0; i < desc->num_supplies; i++)
        st7701->supplies[i].supply = desc->supply_names[i];

    ret = devm_regulator_bulk_get(&dsi->dev, desc->num_supplies,
                      st7701->supplies);
    if (ret < 0)
        return ret;

    st7701->reset = devm_gpiod_get(&dsi->dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(st7701->reset)) {
        dev_err(&dsi->dev, "Couldn't get our reset GPIO\n");
        return PTR_ERR(st7701->reset);
    }

    drm_panel_init(&st7701->panel);
    st7701->panel.dev = &dsi->dev;
    st7701->panel.funcs = &st7701_funcs;

    /**
     * Once sleep out has been issued, ST7701 IC required to wait 120ms
     * before initiating new commands.
     *
     * On top of that some panels might need an extra delay to wait, so
     * add panel specific delay for those cases. As now this panel specific
     * delay information is referenced from those panel BSP driver, example
     * ts8550b and there is no valid documentation for that.
     */
    st7701->sleep_delay = 120 + desc->panel_sleep_delay;
    drm_panel_add(&st7701->panel);

    mipi_dsi_set_drvdata(dsi, st7701);
    st7701->dsi = dsi;
    st7701->desc = desc;
    return mipi_dsi_attach(dsi);
}

static int st7701_dsi_remove(struct mipi_dsi_device *dsi)
{
    struct st7701 *st7701 = mipi_dsi_get_drvdata(dsi);

    mipi_dsi_detach(dsi);
    drm_panel_remove(&st7701->panel);
    return 0;
}

static const struct of_device_id st7701_of_match[] = {
    { .compatible = "techstar,ts8550b", .data = &ts8550b_desc },
    { }
};
MODULE_DEVICE_TABLE(of, st7701_of_match);

static struct mipi_dsi_driver st7701_dsi_driver = {
    .probe		= st7701_dsi_probe,
    .remove		= st7701_dsi_remove,
    .driver = {
        .name		= "st7701",
        .of_match_table	= st7701_of_match,
    },
};
module_mipi_dsi_driver(st7701_dsi_driver);

MODULE_AUTHOR("Melbourne Instruments");
MODULE_DESCRIPTION("Melbourne Instruments Sitronix ST7701 LCD Panel Driver");
MODULE_LICENSE("GPL");
