#include "BAT_Driver.h"
#include "PCF85063.h"
#include "PWR_Key.h"
#include "QMI8658.h"
#include "ST77916.h"
#include "Scene.h"
#include "TCA9554PWR.h"
#include "esp_log.h"
static const char* TAG0 = "RENDER";
static const char* TAG1 = "FLUID_SIM";
static Scene scene;
void Driver_Loop(void* parameter)
{
    // Wireless_Init();
    while (1) {
        PCF85063_Loop();
        BAT_Get_Volts();
        PWR_Loop();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL);
}
void Driver_Init(void)
{
    PWR_Init();
    BAT_Init();
    I2C_Init();
    EXIO_Init();
    PCF85063_Init();
    QMI8658_Init();
    xTaskCreatePinnedToCore(
        Driver_Loop,
        "Other Driver task",
        4096,
        NULL,
        11,
        NULL,
        tskNO_AFFINITY);
}
void FluidSimLoop(void* parameter)
{
    int64_t start_time = esp_timer_get_time();
    for (int i = 1;; ++i) {
        QMI8658_Loop();
        scene.update(scene.getdt(), Vector3(0 - Accel.y * 1400, Accel.x * 1400, 0));
        scene.render(panel_handle);
        if (i % 1000 == 0) {
            ESP_LOGI(TAG1, "Finished %i FluidSimLoop, duration: %lld ms", i, (esp_timer_get_time() - start_time) / 1000);
            start_time = esp_timer_get_time();
        }
    }
    vTaskDelete(NULL);
}
extern "C" void app_main();
void app_main(void)
{
    Driver_Init();
    LCD_Init();
    scene.init(360, 1e-3, 1.0F, 5.0F, 10.0f, 1e4F, 0.2F);
    scene.add_object(Vec(0.5, 0.3), 1000);
    ESP_LOGI(TAG1, "Finished init.");
    xTaskCreatePinnedToCore(
        FluidSimLoop,
        "FluidSim",
        10240,
        NULL,
        10,
        NULL,
        tskNO_AFFINITY);
}
