#ifndef LV_CONF_H
#define LV_CONF_H

/*******************
 * GENERAL SETTINGS
 *******************/
#define LV_COLOR_DEPTH                 16
#define LV_COLOR_16_SWAP               1        // <-- REQUIRED for TFT_eSPI (tft.setSwapBytes(true))
#define LV_TICK_CUSTOM                 1
#define LV_TICK_CUSTOM_SYS_TIME_EXPR   (millis())

/* Heap for LVGL objects (adjust as you grow UI) */
#define LV_MEM_SIZE    (64U * 1024U)

/* Logging (keep off unless debugging) */
#define LV_USE_LOG      0
/* #define LV_LOG_LEVEL LV_LOG_LEVEL_WARN */

/*******************
 * THEMES / FONTSs
 *******************/
#define LV_USE_THEME_DEFAULT 1
#define LV_USE_THEME_BASIC   1
#define LV_USE_THEME_MONO    0

/* Include a couple of fonts */
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_20 1

/*******************
 * DEV UTILS (optional)
 *******************/
#define LV_USE_PERF_MONITOR 0   // shows CPU/FPS in top-left (optional)

/* Core */
#define LV_USE_ANIMATION 1

/* Widgets we use */
#define LV_USE_CANVAS 1
#define LV_USE_LINE   1

/* Software renderer (usually 1 by default) */
#define LV_USE_DRAW_SW 1


#endif /* LV_CONF_H */
