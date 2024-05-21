#include <Arduino.h>
#include <lgfx_user/LGFX_ESP32S3_RGB_ESP32-8048S070.h>
#include <fpt_engine.h>
#include <fpt_engine_data.h>
#include <lvgl.h>
#include <SPI.h>
#include <FS.h>
#include <SPIFFS.h>
#include <FreeRTOS.h>
#include "ui/ui.h"

#define PORT_Main 17
#define LGFX_AUTODETECT
#define LGFX_USE_V1
#define LOGO_FILENAME "/logo.bmp"

SemaphoreHandle_t MAINSemaphore;
FPTEngine DATAEngine;
FPTEngine_Data MAINData;

static const uint16_t screenWidth = 800;
static const uint16_t screenHeight = 480;

unsigned char BrightnessLevel;
bool ChangeBrightnessFlag;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[2][screenWidth * 10];

_LGFX gfx;
LGFX_Sprite screen;

void UpdateDisplay(void * pvParameters);
void DecodeCanbus(void * pvParameters);
//void UpdateParameter();
void logo_show();

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    if (gfx.getStartCount() == 0)
    { // Processing if not yet started
        gfx.startWrite();
    }
    gfx.pushImageDMA(area->x1, area->y1, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1, (lgfx::rgb565_t *) &color_p->full);
    lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
    uint16_t touchX, touchY;

    data->state = LV_INDEV_STATE_REL;

    if (gfx.getTouch(&touchX, &touchY))
    {
        data->state = LV_INDEV_STATE_PR;

        /*Set the coordinates*/
        data->point.x = touchX;
        data->point.y = touchY;
    }
}

void setup()
{
    Serial0.begin(115200);
    if(DATAEngine.Init(PORT_Main)){      
        Serial0.println("Main Communication Initialized");
    }else{
        Serial0.println("Main Communication Failed");
        //while(1);
    }

    gfx.begin();
    gfx.setBrightness(100);
    logo_show();
    delay(1000);
    lv_init();
    lv_disp_draw_buf_init(&draw_buf, buf[0], buf[1], screenWidth * 10);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    ui_init();

    if(MAINSemaphore==NULL){
        MAINSemaphore = xSemaphoreCreateMutex();
        if((MAINSemaphore)!=NULL){
            xSemaphoreGive(MAINSemaphore);  
        }
    }
    xTaskCreatePinnedToCore(UpdateDisplay, "Update Display", 100000, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(DecodeCanbus, "Decode Canbus", 5000, NULL, 1, NULL, 1);

}

void loop()
{
}   

void UpdateDisplay(void * pvParameters){
    Serial0.println("UI Task Started");
    while(1){
        lv_timer_handler();
        if(ChangeBrightnessFlag){
            gfx.setBrightness(BrightnessLevel);
            ChangeBrightnessFlag = false;
        }
        //UpdateParameter();
        vTaskDelay(5/portTICK_PERIOD_MS);
    }
}

void DecodeCanbus(void * pvParameters){
    Serial0.println("J1939 Task Started");
    while(1){
        if(xSemaphoreTake(MAINSemaphore, (TickType_t)5) ==  pdTRUE){
            DATAEngine.Refresh(&MAINData);
            xSemaphoreGive(MAINSemaphore);
        }
    }
}
/*
void UpdateParameter(){
    if(xSemaphoreTake(MAINSemaphore, (TickType_t)5) == pdTRUE){
        if(DATAEngine.Connected){
        //Value Main
        lv_label_set_text(ui_Value_Oil_Press_Main, String(MAINData.mOil_press,1).c_str());
        lv_label_set_text(ui_Value_Cool_Temp_Main, String(MAINData.mCool_temp,1).c_str());
        lv_label_set_text(ui_Value_Load_Main, String(MAINData.mLoad,0).c_str());
        lv_label_set_text(ui_Value_Fuel_Rate_Main, String(MAINData.mFuel_rate,1).c_str());

        //Value Fluid Sensors
        lv_label_set_text(ui_Value_Cooltime_PS_FS, String(MAINData.fsCoolPS,1).c_str());
        lv_label_set_text(ui_Value_Cooltime_SB_FS, String(MAINData.fsCoolSB,1).c_str());
        lv_label_set_text(ui_Value_Oil_Temp_FS, String(MAINData.fsOil_temp,1).c_str());
        lv_label_set_text(ui_Value_Oil_Press_FS, String(MAINData.fsOil_press,1).c_str());
        lv_label_set_text(ui_Value_Fuel_Temp_FS, String(MAINData.fsFuel_temp,1).c_str());
        lv_label_set_text(ui_Value_Load_FS, String(MAINData.fsLoad,0).c_str());
        lv_label_set_text(ui_Value_Engine_RPM_PS, String(MAINData.fsEngine_RPM,0).c_str());
        
        //Value In Exh Sensors
        lv_label_set_text(ui_In_Take_P_PS_Value, String(MAINData.IES_ITP_PS,1).c_str());
        lv_label_set_text(ui_In_Take_P_SB_Value, String(MAINData.IES_ITP_SB,1).c_str());
        lv_label_set_text(ui_In_Take_T_PS_Value, String(MAINData.IES_ITT_PS,1).c_str());
        lv_label_set_text(ui_In_Take_T_SB_Value, String(MAINData.IES_ITT_SB,1).c_str());
        lv_label_set_text(ui_Load_Value_IE_Sensors, String(MAINData.IES_Load,0).c_str());
        lv_label_set_text(ui_Exhaust_T_PS_Value, String(MAINData.IES_ET_PS,1).c_str());
        lv_label_set_text(ui_Exhaust_T_SB_Value, String(MAINData.IES_ET_SB,1).c_str());

        //Value Speed
        lv_label_set_text(ui_Value_Speed_To_MCU, String(MAINData.SPD_SPD_TO_MCU,0).c_str());
        lv_label_set_text(ui_Value_Speed_Request, String(MAINData.SPD_SPD_REQ,1).c_str());
        lv_label_set_text(ui_Value_Load_Speed, String(MAINData.SPD_LOAD,1).c_str());
        lv_label_set_text(ui_Value_Battery_Volt, String(MAINData.SPD_BATVOL,1).c_str());

        //Value Statistics
        lv_label_set_text(ui_Value_Run_Hours_Statistics, String(MAINData.STT_RH,0).c_str());
        lv_label_set_text(ui_Value_Tot_Fuel_Used_Statistics, String(MAINData.STT_TFU,1).c_str());
    }
    else{
        //Value Main
        lv_label_set_text(ui_Value_Oil_Press_Main, "***");
        lv_label_set_text(ui_Value_Cool_Temp_Main, "***");
        lv_label_set_text(ui_Value_Load_Main, "***");
        lv_label_set_text(ui_Value_Fuel_Rate_Main, "***");

        //Value Fluid Sensors
        lv_label_set_text(ui_Value_Cooltime_PS_FS, "***");
        lv_label_set_text(ui_Value_Cooltime_SB_FS, "***");
        lv_label_set_text(ui_Value_Oil_Temp_FS, "***");
        lv_label_set_text(ui_Value_Oil_Press_FS, "***");
        lv_label_set_text(ui_Value_Fuel_Temp_FS, "***");
        lv_label_set_text(ui_Value_Load_FS, "***");
        lv_label_set_text(ui_Value_Engine_RPM_PS, "***");
        
        //Value In Exh Sensors
        lv_label_set_text(ui_In_Take_P_PS_Value, "***");
        lv_label_set_text(ui_In_Take_P_SB_Value, "***");
        lv_label_set_text(ui_In_Take_T_PS_Value, "***");
        lv_label_set_text(ui_In_Take_T_SB_Value, "***");
        lv_label_set_text(ui_Load_Value_IE_Sensors, "***");
        lv_label_set_text(ui_Exhaust_T_PS_Value, "***");
        lv_label_set_text(ui_Exhaust_T_SB_Value, "***");

        //Value Speed
        lv_label_set_text(ui_Value_Speed_To_MCU, "***");
        lv_label_set_text(ui_Value_Speed_Request, "***");
        lv_label_set_text(ui_Value_Battery_Volt, "***");
        lv_label_set_text(ui_Value_Load_Speed, "***");

        //Value Statistics
        lv_label_set_text(ui_Value_Run_Hours_Statistics, "***");
        lv_label_set_text(ui_Value_Tot_Fuel_Used_Statistics, "***");
    }
    xSemaphoreGive(MAINSemaphore);
 }
}
*/
void logo_show()
{   
    if(SPIFFS.begin()){
        File bmpFile = SPIFFS.open(LOGO_FILENAME); 
        if(bmpFile){
            gfx.fillScreen(TFT_WHITE);
            gfx.drawBmp(&bmpFile,0,0,0,0,0,0,1.0,1.0,middle_centre);
            bmpFile.close();
            SPIFFS.end();
        }
    }
}
