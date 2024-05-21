SET(SOURCES screens/ui_Main.c
    screens/ui_Fluid_Sensors.c
    screens/ui_In_Exh_Sensors.c
    screens/ui_Speed.c
    screens/ui_Alarmlist.c
    screens/ui_History.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c)

add_library(ui ${SOURCES})
