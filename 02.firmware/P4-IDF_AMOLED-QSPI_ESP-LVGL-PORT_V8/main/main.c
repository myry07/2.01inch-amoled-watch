#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "lv_demos.h"

#include "esp_lcd_co5300.h"
#include "esp_lcd_touch_chsc6417.h"

/* LCD size */
#define EXAMPLE_LCD_H_RES   (368)
#define EXAMPLE_LCD_V_RES   (448)

/* LCD settings */
#define EXAMPLE_LCD_SPI_NUM         (SPI2_HOST)
#define EXAMPLE_LCD_BITS_PER_PIXEL  (16) //LCD像素位数示例
#define EXAMPLE_LCD_DRAW_BUFF_DOUBLE (1)
#define EXAMPLE_LCD_DRAW_BUFF_HEIGHT (50)
#define EXAMPLE_LCD_BL_ON_LEVEL     (1)

/* LCD pins */
#define EXAMPLE_LCD_CS            (GPIO_NUM_7)
#define EXAMPLE_LCD_PCLK          (GPIO_NUM_8)
#define EXAMPLE_LCD_DATA0         (GPIO_NUM_23)
#define EXAMPLE_LCD_DATA1         (GPIO_NUM_22)
#define EXAMPLE_LCD_DATA2         (GPIO_NUM_21)
#define EXAMPLE_LCD_DATA3         (GPIO_NUM_20)
#define EXAMPLE_LCD_RST           (GPIO_NUM_5)
#define EXAMPLE_LCD_GPIO_BL       (-1)

/* Touch settings */
#define EXAMPLE_TOUCH_I2C_NUM       (0)
#define EXAMPLE_TOUCH_I2C_CLK_HZ    (400000)

/* LCD touch pins */
#define EXAMPLE_TOUCH_I2C_SCL       (GPIO_NUM_6)
#define EXAMPLE_TOUCH_I2C_SDA       (GPIO_NUM_4)
#define EXAMPLE_TOUCH_RST           (GPIO_NUM_3)
#define EXAMPLE_TOUCH_INT           (GPIO_NUM_2)

static const char *TAG = "test";

// LVGL image declare
LV_IMG_DECLARE(esp_logo)

/* LCD IO and panel */
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;
static esp_lcd_touch_handle_t touch_handle = NULL;

/* LVGL display and touch */
static lv_display_t *lvgl_disp = NULL;
static lv_indev_t *lvgl_touch_indev = NULL;

static esp_err_t app_lcd_init(void)
{
    esp_err_t ret = ESP_OK;
#if(EXAMPLE_LCD_GPIO_BL)
{
    /* LCD backlight */
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_LCD_GPIO_BL,
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
}
#endif

    /* LCD initialization */
    ESP_LOGD(TAG, "Initialize SPI bus");
   const spi_bus_config_t buscfg = CO5300_PANEL_BUS_QSPI_CONFIG(EXAMPLE_LCD_PCLK,
                                                                 EXAMPLE_LCD_DATA0,
                                                                 EXAMPLE_LCD_DATA1,
                                                                 EXAMPLE_LCD_DATA2,
                                                                 EXAMPLE_LCD_DATA3,
                                                                 EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * EXAMPLE_LCD_BITS_PER_PIXEL / 8);
    ESP_RETURN_ON_ERROR(spi_bus_initialize(EXAMPLE_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGD(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = CO5300_PANEL_IO_QSPI_CONFIG(EXAMPLE_LCD_CS, NULL, NULL);

    co5300_vendor_config_t vendor_config = {
        // .flags = {
        //     .use_qspi_interface = 0,
        // },
    };

    // 将 LCD 连接到 SPI 总线
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_LCD_SPI_NUM, &io_config, &lcd_io), err, TAG, "New panel IO failed");

    ESP_LOGD(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = EXAMPLE_LCD_BITS_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_co5300(lcd_io, &panel_config, &lcd_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(lcd_panel);
    esp_lcd_panel_init(lcd_panel);
    esp_lcd_panel_disp_on_off(lcd_panel, true);

#if(EXAMPLE_LCD_GPIO_BL)
    /* LCD backlight on */
    ESP_ERROR_CHECK(gpio_set_level(EXAMPLE_LCD_GPIO_BL, EXAMPLE_LCD_BL_ON_LEVEL));
#endif

    return ret;

err:
    if (lcd_panel) {
        esp_lcd_panel_del(lcd_panel);
    }
    if (lcd_io) {
        esp_lcd_panel_io_del(lcd_io);
    }
    spi_bus_free(EXAMPLE_LCD_SPI_NUM);
    return ret;
}

static esp_err_t app_touch_init(void)
{
    /* Initilize I2C */
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_TOUCH_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = EXAMPLE_TOUCH_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = EXAMPLE_TOUCH_I2C_CLK_HZ
    };
    ESP_RETURN_ON_ERROR(i2c_param_config(EXAMPLE_TOUCH_I2C_NUM, &i2c_conf), TAG, "I2C configuration failed");
    ESP_RETURN_ON_ERROR(i2c_driver_install(EXAMPLE_TOUCH_I2C_NUM, i2c_conf.mode, 0, 0, 0), TAG, "I2C initialization failed");

    /* Initialize touch HW */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES,
        .y_max = EXAMPLE_LCD_V_RES,
        .rst_gpio_num = EXAMPLE_TOUCH_RST,
        .int_gpio_num = EXAMPLE_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CHSC6417_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)EXAMPLE_TOUCH_I2C_NUM, &tp_io_config, &tp_io_handle), TAG, "");
    return esp_lcd_touch_new_i2c_chsc6417(tp_io_handle, &tp_cfg, &touch_handle);
}

static esp_err_t app_lvgl_init(void)
{
    /* Initialize LVGL */
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,         /* LVGL task priority */
        .task_stack = 4096*2,         /* LVGL 任务堆栈大小*/
        .task_affinity = -1,        /* LVGL task pinned to core (-1 is no affinity) */
        .task_max_sleep_ms = 500,   /* Maximum sleep in LVGL task */
        .timer_period_ms = 5        /* LVGL timer tick period in ms */
    };
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_io,
        .panel_handle = lcd_panel,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT,
        .double_buffer = EXAMPLE_LCD_DRAW_BUFF_DOUBLE,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = false,//单色
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = true,
        },
        .flags = {
            .buff_dma = false,
            .buff_spiram = true,
            .sw_rotate = true,
            .full_refresh = false,
            .direct_mode = false,
        }
    };
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    /* Add touch input (for selected screen) */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = lvgl_disp,
        .handle = touch_handle,
    };
    lvgl_touch_indev = lvgl_port_add_touch(&touch_cfg);

    return ESP_OK;
}

static void app_main_display(void)
{
    /* Task lock */
    lvgl_port_lock(0);

    // lv_demo_music();
    lv_demo_widgets();
    // lv_demo_stress(); /* LVGL 压力测试 */

    /* Task unlock */
    lvgl_port_unlock();
}

void app_main(void)
{
    /* LCD HW initialization */
    ESP_ERROR_CHECK(app_lcd_init());

    /* Touch initialization */
    ESP_ERROR_CHECK(app_touch_init());

    /* LVGL initialization */
    ESP_ERROR_CHECK(app_lvgl_init());

    /* Show LVGL objects */
    app_main_display();
}
