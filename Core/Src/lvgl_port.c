
/*
 * porting ili9341 LCD module for LVGL
 * reference: https://docs.lvgl.io/master/integration/driver/display/lcd_stm32_guide.html#lcd-stm32-guide
 */
#include "lvgl.h"
#include "./src/drivers/display/ili9341/lv_ili9341.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

#define LCD_H_RES       (240)
#define LCD_V_RES       (320)
#define BUS_SPI1_POLL_TIMEOUT 0x1000U


// TaskHandle_t LvglTaskHandle;
lv_display_t *lcd_disp;
volatile int lcd_bus_busy = 0;

/* defined in main.c */
extern SPI_HandleTypeDef 	hspi1;
extern DMA_HandleTypeDef 	hdma_spi1_tx;
#define LCD_CS_GPIO_Port 	SPI_CS_GPIO_Port
#define LCD_CS_Pin  	 	SPI_CS_Pin
#define LCD_DCX_GPIO_Port 	LCD_DC_SEL_GPIO_Port
#define LCD_DCX_Pin	 	LCD_DC_SEL_Pin

void lcd_color_transfer_ready_cb(SPI_HandleTypeDef *hspi)
{
  /* CS high */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
  lcd_bus_busy = 0;
  lv_display_flush_ready(lcd_disp);
}

/* Initialize LCD bus, reset LCD */
static int32_t lcd_init(void)
{
//   GPIO_InitTypeDef GPIO_InitStruct = {0};
//   GPIO_InitStruct.Pin = LCD_CS_Pin | LCD_DCX_Pin;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Mode =  GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//   HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);
//   HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
//   HAL_GPIO_WritePin(LCD_DCX_GPIO_Port, LCD_DCX_Pin, GPIO_PIN_SET);
//   vTaskDelay(10);
  /* Register SPI Tx Complete Callback */
  HAL_SPI_RegisterCallback(&hspi1, HAL_SPI_TX_COMPLETE_CB_ID, lcd_color_transfer_ready_cb);

  /* reset LCD */
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);
  vTaskDelay(100);
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);
  vTaskDelay(100);
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_DCX_GPIO_Port, LCD_DCX_Pin, GPIO_PIN_SET);
  return HAL_OK;
}

/* Platform-specific implementation of the LCD send command function. In general this should use polling transfer. */
static void lcd_send_cmd(lv_display_t *disp, const uint8_t *cmd, size_t cmd_size, const uint8_t *param, size_t param_size)
{
  LV_UNUSED(disp);
  while (lcd_bus_busy);   /* wait until previous transfer is finished */
  /* Set the SPI in 8-bit mode */
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  HAL_SPI_Init(&hspi1);
  /* DCX low (command) */
  HAL_GPIO_WritePin(LCD_DCX_GPIO_Port, LCD_DCX_Pin, GPIO_PIN_RESET);
  /* CS low */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
  /* send command */
  if (HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd, cmd_size, BUS_SPI1_POLL_TIMEOUT) == HAL_OK) {
    /* DCX high (data) */
    HAL_GPIO_WritePin(LCD_DCX_GPIO_Port, LCD_DCX_Pin, GPIO_PIN_SET);
    /* for short data blocks we use polling transfer */
    HAL_SPI_Transmit(&hspi1, (uint8_t *)param, (uint16_t)param_size, BUS_SPI1_POLL_TIMEOUT);
    /* CS high */
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
  }
}

/* Platform-specific implementation of the LCD send color function. For better performance this should use DMA transfer.
 * In case of a DMA transfer a callback must be installed to notify LVGL about the end of the transfer.
 */
static void lcd_send_color(lv_display_t *disp, const uint8_t *cmd, size_t cmd_size, uint8_t *param, size_t param_size)
{
        LV_UNUSED(disp);
        while (lcd_bus_busy);   /* wait until previous transfer is finished */
        /* Set the SPI in 8-bit mode */
        hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
        HAL_SPI_Init(&hspi1);
        /* DCX low (command) */
        HAL_GPIO_WritePin(LCD_DCX_GPIO_Port, LCD_DCX_Pin, GPIO_PIN_RESET);
        /* CS low */
        HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
        /* send command */
        if (HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd, cmd_size, BUS_SPI1_POLL_TIMEOUT) == HAL_OK) {
                /* DCX high (data) */
                HAL_GPIO_WritePin(LCD_DCX_GPIO_Port, LCD_DCX_Pin, GPIO_PIN_SET);
                /* for color data use DMA transfer */
                /* Set the SPI in 16-bit mode to match endianess */
                hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
                HAL_SPI_Init(&hspi1);
                lcd_bus_busy = 1;
                HAL_SPI_Transmit_DMA(&hspi1, param, (uint16_t)param_size / 2);
                /* NOTE: CS will be reset in the transfer ready callback */
        }
}

static void ui_init(lv_display_t *disp);

void LVGL_Task(void *argument)
{
  /* Initialize LVGL */
  lv_init();
  /* Initialize LCD I/O */
  if (lcd_init() != 0) {
    return;
  }

  /* Create the LVGL display object and the LCD display driver */
  lcd_disp = lv_ili9341_create(LCD_H_RES, LCD_V_RES, LV_LCD_FLAG_NONE, lcd_send_cmd, lcd_send_color);
  lv_display_set_rotation(lcd_disp, LV_DISPLAY_ROTATION_270);

  /* Allocate draw buffers on the heap. In this example we use two partial buffers of 1/10th size of the screen */
  lv_color_t * buf1 = NULL;
  lv_color_t * buf2 = NULL;
  uint32_t buf_size = LCD_H_RES * LCD_V_RES / 10 * lv_color_format_get_size(lv_display_get_color_format(lcd_disp));
  // printf("buffer size: %lu, need 2time\r\n", buf_size);
  buf1 = lv_malloc(buf_size);
  if (buf1 == NULL) {
    LV_LOG_ERROR("display draw buffer malloc failed");
    return;
  }

  buf2 = lv_malloc(buf_size);
  if (buf2 == NULL) {
    LV_LOG_ERROR("display buffer malloc failed");
    lv_free(buf1);
    return;
  }

  lv_display_set_buffers(lcd_disp, buf1, buf2, buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);

  ui_init(lcd_disp);

  for(;;) {
    /* The task running lv_timer_handler should have lower priority than that running `lv_tick_inc` */
    lv_timer_handler();
    /* raise the task priority of LVGL and/or reduce the handler period can improve the performance */
    vTaskDelay(LV_DEF_REFR_PERIOD);
  }
}


void ui_init(lv_display_t *disp)
{
  lv_obj_t *obj;
  /* set screen background to white */
  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_white(), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_100, 0);

  /* create label */
  obj = lv_label_create(scr);
  lv_obj_set_align(obj, LV_ALIGN_CENTER);
  lv_obj_set_height(obj, LV_SIZE_CONTENT);
  lv_obj_set_width(obj, LV_SIZE_CONTENT);
  lv_obj_set_style_text_font(obj, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(obj, lv_color_black(), 0);
  lv_label_set_text(obj, "Hello World!");
}
