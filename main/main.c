#include "ST77916.h"
#include "PCF85063.h"
#include "QMI8658.h"
#include "TCA9554PWR.h"
#include "BAT_Driver.h"
#include "PWR_Key.h"
static const char *TAG_IMU = "IMU";
void Driver_Loop(void *parameter)
{
    // Wireless_Init();
    while(1)
    {
        QMI8658_Loop();
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
    EXIO_Init();                    // Example Initialize EXIO
    PCF85063_Init();
    QMI8658_Init();
    xTaskCreatePinnedToCore(
        Driver_Loop, 
        "Other Driver task",
        4096, 
        NULL, 
        3, 
        NULL, 
        0);
}
void app_main(void)
{
    Driver_Init();

    LCD_Init();

    while (1) {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(10));
        // 输出加速度计数据和陀螺仪数据
        ESP_LOGI(TAG_IMU, "Accel: x=%f, y=%f, z=%f", Accel.x, Accel.y, Accel.z);
        ESP_LOGI(TAG_IMU, "Gyro: x=%f, y=%f, z=%f", Gyro.x, Gyro.y, Gyro.z);
    }
}






