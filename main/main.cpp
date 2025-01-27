#include "BAT_Driver.h"
#include "PCF85063.h"
#include "PWR_Key.h"
#include "QMI8658.h"
#include "ST77916.h"
#include "Scene.h"
#include "TCA9554PWR.h"

#include "dsp_platform.h"
#include "esp_dsp.h"
#include "esp_log.h"
#include <vector>
static const char* TAG_IMU = "IMU";
static const char* TAG = "DSP";
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
    EXIO_Init(); // Example Initialize EXIO
    PCF85063_Init();
    QMI8658_Init();
    xTaskCreatePinnedToCore(
        Driver_Loop,
        "Other Driver task",
        4096,
        NULL,
        3,
        NULL,
        tskNO_AFFINITY);
}
void FluidSimLoop(void* parameter)
{
    for (int i = 0;; ++i) {
        int64_t start_time = esp_timer_get_time();
        QMI8658_Loop();
        scene.update(scene.getdt(), Vector3(0 - Accel.y, -Accel.x, 0));

        int64_t end_time = esp_timer_get_time();
        ESP_LOGI(TAG1, "Finished %i, FluidSimLoop duration: %lld ms", i, (end_time - start_time) / 1000);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}
void Render(void* parameter){
    while (1)
    {
        test_draw_bitmap(panel_handle);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}
extern "C" void app_main();
void app_main(void)
{
    Driver_Init();
    LCD_Init();
    scene.init(360, 1e-3, 1.0F, 5.0F, 10.0f, 1e4F, 0.2F);
    scene.add_object(Vec(0.3, 0.2));
    xTaskCreatePinnedToCore(
        FluidSimLoop,
        "FluidSim",
        10240,
        NULL,
        10,
        NULL,
        tskNO_AFFINITY);

    xTaskCreatePinnedToCore(
        FluidSimLoop,
        "Render",
        10240,
        NULL,
        10,
        NULL,
        tskNO_AFFINITY);
}
