#include "ST77916.h"

static const char *TAG_LCD = "ST77916";

esp_lcd_panel_handle_t panel_handle = NULL;

void ST7701_Reset(){
  Set_EXIO(TCA9554_EXIO2,false);
  vTaskDelay(pdMS_TO_TICKS(10));
  Set_EXIO(TCA9554_EXIO2,true);
  vTaskDelay(pdMS_TO_TICKS(50));
}
void LCD_Init() {        
  ST77916_Init();
  Backlight_Init();
}

void test_draw_bitmap(esp_lcd_panel_handle_t panel_handle)
{
  uint16_t row_line = 20;

  uint8_t byte_per_pixel = 2;
  uint8_t *color = (uint8_t *)heap_caps_calloc(1, 20 * EXAMPLE_LCD_HEIGHT * byte_per_pixel, MALLOC_CAP_DMA);


  for (int j = 0; j < 18; j++) {
      for (int i = 0; i < 20 * EXAMPLE_LCD_HEIGHT; i++) {
          //memset(color + i * byte_per_pixel,  (0xff/20*j) < 8 + 0xff/20*j, byte_per_pixel);
          color[i * byte_per_pixel + 0] = 0xff;
          color[i * byte_per_pixel + 1] = 0xff;
      }
      esp_lcd_panel_draw_bitmap(panel_handle, 0, j * row_line, EXAMPLE_LCD_HEIGHT, (j + 1) * row_line, color);
      vTaskDelay(pdMS_TO_TICKS(10));
  }
  free(color);
}

void draw_bit(esp_lcd_panel_handle_t panel_handle, uint16_t x, uint16_t y, uint16_t color)
{
        esp_lcd_panel_draw_bitmap(panel_handle, x, y, x + 1, y + 1, &color);
}

int QSPI_Init(void){
  static const spi_bus_config_t host_config = {            
    .data0_io_num = ESP_PANEL_LCD_SPI_IO_DATA0,                    
    .data1_io_num = ESP_PANEL_LCD_SPI_IO_DATA1,                   
    .sclk_io_num = ESP_PANEL_LCD_SPI_IO_SCK,                   
    .data2_io_num = ESP_PANEL_LCD_SPI_IO_DATA2,                    
    .data3_io_num = ESP_PANEL_LCD_SPI_IO_DATA3,                    
    .data4_io_num = -1,                       
    .data5_io_num = -1,                      
    .data6_io_num = -1,                       
    .data7_io_num = -1,                      
    .max_transfer_sz = ESP_PANEL_HOST_SPI_MAX_TRANSFER_SIZE, 
    .flags = SPICOMMON_BUSFLAG_MASTER,       
    .intr_flags = 0,                            
  };
  if(spi_bus_initialize(ESP_PANEL_HOST_SPI_ID_DEFAULT, &host_config, SPI_DMA_CH_AUTO) != ESP_OK){
    printf("The SPI initialization failed.\r\n");
    return 0;
  }
  printf("The SPI initialization succeeded.\r\n");
  
  const esp_lcd_panel_io_spi_config_t io_config ={
    .cs_gpio_num = ESP_PANEL_LCD_SPI_IO_CS,               
    .dc_gpio_num = -1,                  
    .spi_mode = ESP_PANEL_LCD_SPI_MODE,                      
    .pclk_hz = ESP_PANEL_LCD_SPI_CLK_HZ,     
    .trans_queue_depth = ESP_PANEL_LCD_SPI_TRANS_QUEUE_SZ,            
    .on_color_trans_done = NULL,                            
    .user_ctx = NULL,                   
    .lcd_cmd_bits = ESP_PANEL_LCD_SPI_CMD_BITS,                 
    .lcd_param_bits = ESP_PANEL_LCD_SPI_PARAM_BITS,                
    .flags = {                          
      .dc_low_on_data = 0,            
      .octal_mode = 0,                
      .quad_mode = 1,                 
      .sio_mode = 0,                  
      .lsb_first = 0,                 
      .cs_high_active = 0,            
    },                                  
  };
  esp_lcd_panel_io_handle_t io_handle = NULL;
  if(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)ESP_PANEL_HOST_SPI_ID_DEFAULT, &io_config, &io_handle) != ESP_OK){
    printf("Failed to set LCD communication parameters -- SPI\r\n");
    return 0;
  }
  printf("LCD communication parameters are set successfully -- SPI\r\n");

  printf("Install LCD driver of st77916\r\n");
  st77916_vendor_config_t vendor_config={  
    .flags = {
      .use_qspi_interface = 1,
    },
  };
  esp_lcd_panel_dev_config_t panel_config={
    .reset_gpio_num = EXAMPLE_LCD_PIN_NUM_RST,                                
    .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,                   
    // .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,                       
    .bits_per_pixel = EXAMPLE_LCD_COLOR_BITS,                                 
    .flags = {                                                    
      .reset_active_high = 0,                                   
    },                                                            
    .vendor_config = (void *) &vendor_config,                                  
  };
  esp_lcd_new_panel_st77916(io_handle, &panel_config, &panel_handle);

  esp_lcd_panel_reset(panel_handle);
  esp_lcd_panel_init(panel_handle);
  // esp_lcd_panel_invert_color(panel_handle,false);

  esp_lcd_panel_disp_on_off(panel_handle, true);
  test_draw_bitmap(panel_handle);
  return 1;
}

void ST77916_Init() {
  ST7701_Reset();
  if(!QSPI_Init()){
    printf("ST77916 Failed to be initialized\r\n");
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Backlight program

uint8_t LCD_Backlight = 75;
static ledc_channel_config_t ledc_channel;
void Backlight_Init(void)
{
    ESP_LOGI(TAG_LCD, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_LCD_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_LS_MODE,
        .timer_num = LEDC_HS_TIMER,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel.channel    = LEDC_HS_CH0_CHANNEL;
    ledc_channel.duty       = 0;
    ledc_channel.gpio_num   = EXAMPLE_LCD_PIN_NUM_BK_LIGHT;
    ledc_channel.speed_mode = LEDC_LS_MODE;
    ledc_channel.timer_sel  = LEDC_HS_TIMER;
    ledc_channel_config(&ledc_channel);
    ledc_fade_func_install(0);
    
    Set_Backlight(LCD_Backlight);      //0~100    
}
void Set_Backlight(uint8_t Light)
{   
    if(Light > Backlight_MAX) Light = Backlight_MAX;
    uint16_t Duty = LEDC_MAX_Duty-(81*(Backlight_MAX-Light));
    if(Light == 0) 
        Duty = 0;
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, Duty);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
}
// end Backlight program