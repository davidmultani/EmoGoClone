/*
  ===========================================================
   EMO FACE PRO — Enhanced with Voice Reactions & Music Sync
  ===========================================================
  Board: ESP32
  Display: ST7789 170x320 portrait, X offset 35
  Display Pins: SCK=14, MOSI=13, CS=15, DC=2, RST=4, BL=21
  
  *** IMPORTANT ACCELEROMETER ORIENTATION ***
  MMA8452Q mounted with:
  - X axis: Top/Bottom (Top = +X, Bottom = -X)
  - Y axis: Left/Right (Left = +Y, Right = -Y)  
  - Z axis: Forward/Backward (Forward = +Z, Backward = -Z)

  I2C (MMA8452Q): SDA=32, SCL=33
  
  I2S Mic: WS=27, BCLK=26, DOUT=25, SEL=GND (LEFT channel)
  I2S Out: DIN=19, BCLK=18, LRCLK=5
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <time.h>
#include <esp_sntp.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <DNSServer.h>

#include <lvgl.h>
#include <LovyanGFX.hpp>
#include <esp_heap_caps.h>
#include <driver/ledc.h>
#include <driver/i2s.h>

// ----------------- WiFi Configuration ------------------
#define WIFI_SSID "Your-WIFI-SSID"
#define WIFI_PASSWORD "Your-WIFI-Password"

#define WIFI_SSID2 ""
#define WIFI_PASSWORD2 ""

#define ENABLE_TTS 1
#define ENABLE_VOICE_REACTIONS 1
#define ENABLE_MUSIC_BEAT_DETECTION 1
#define ENABLE_MOOD_MEMORY 1
#define ENABLE_GESTURES 1
#define ENABLE_TIME_FEATURES 1
#define ENABLE_WIFI 1
#define ENABLE_WEB_SERVER 1

// ===================== I2S Pins =====================
#define MIC_PORT       I2S_NUM_0
#define MIC_BCLK_PIN   26
#define MIC_WS_PIN     27
#define MIC_DATA_PIN   25
#define MIC_IS_LEFT    1

#define SPK_PORT       I2S_NUM_1
#define SPK_BCLK_PIN   18
#define SPK_WS_PIN     5
#define SPK_DOUT_PIN   19

// ================== Audio Config =================
#define MIC_SAMPLE_RATE    16000
#define SPK_SAMPLE_RATE    22050
#define SAMPLE_SHIFT       8
#define DMA_LEN            64
#define MIC_SOFT_GAIN      0.5f  

#define MUSIC_THRESHOLD 0.2f    
#define BEAT_DECAY 0.7f

#define TTS_SAMPLE_RATE 8000
#define TTS_BUFFER_SIZE 512

// ================== Music Reaction Config =================
#define EYEBROW_DANCE_SPEED 8.0f  
#define EYEBALL_JUMP_STRENGTH 12.0f  
#define BEAT_IMPACT_DURATION 150

// ================== Web Server Config =================
WebServer webServer(80);
DNSServer dnsServer;
const byte DNS_PORT = 53;
bool web_server_active = false;
bool ap_mode = false;
String ap_ssid = "EmoFacePro-AP";
String ap_password = "12345678";

// ================== Alarm System Config =================
#define MAX_ALARMS 5
#define ALARM_RING_DURATION 30000  // 30 seconds of continuous ringing
#define ALARM_SNOOZE_DURATION 300000  // 5 minutes snooze

// ================== Hardware Pins =================
static const int I2C_SDA = 32;
static const int I2C_SCL = 33;

static const int SCR_W = 170;
static const int SCR_H = 320;
static const int EYE_W = 56;
static const int EYE_H = 44;

static int BLINK_OFS_X = 0;
static int BLINK_OFS_Y = 0;

// Web configurable settings
static struct {
  int music_sensitivity = 8;
  float mic_gain = 0.5f;
  int eyebrow_dance_speed = 8;
  int eyeball_jump_strength = 12;
  bool auto_emotion = true;
  bool voice_reactions = true;
  bool music_reactions = true;
  String wifi_ssid = WIFI_SSID;
  String wifi_password = WIFI_PASSWORD;
  String timezone = "IST-5:30";
  int sleep_hour = 23;
  int wake_hour = 7;
  int brightness = 255;
  
  // New settings
  bool lip_sync_enabled = true;
  int lip_sync_sensitivity = 5;
  bool notifications_enabled = true;
  int alarm_volume = 7;
} config;

static const long  GMT_OFFSET_SEC = 5.5 * 3600;
static const int   DAYLIGHT_OFFSET_SEC = 0;

static const char* NTP_SERVER1 = "in.pool.ntp.org";
static const char* NTP_SERVER2 = "asia.pool.ntp.org";
static const char* NTP_SERVER3 = "pool.ntp.org";

static int SLEEP_HOUR = 23;
static int WAKE_HOUR = 7;
static const int SLEEP_MINUTE = 0;
static const int WAKE_MINUTE = 0;

static const lv_color_t COL_BG    = lv_color_hex(0x000000);
static const lv_color_t COL_PUPIL = lv_color_hex(0xFFFFFF);

static const lv_color_t COL_EYE[7] = {
  lv_color_hex(0x00E5FF),  // Normal - Light Blue
  lv_color_hex(0xFF3366),  // Angry - Red
  lv_color_hex(0x33FF99),  // Happy - Green
  lv_color_hex(0xFFCC00),  // Curious - Yellow
  lv_color_hex(0x9966FF),  // Sleepy - Purple
  lv_color_hex(0xFF6600),  // Excited - Orange
  lv_color_hex(0x00CCFF)   // Sad - Blue
};

enum Emotion { EMO_NORMAL=0, EMO_ANGRY, EMO_HAPPY, EMO_CURIOUS, EMO_SLEEPY, EMO_EXCITED, EMO_SAD, EMO_COUNT };

static const char* VOICE_PHRASES[EMO_COUNT] = {
  "I'm feeling normal",
  "I'm getting angry!",
  "I'm so happy!",
  "I'm curious about that",
  "I'm getting sleepy...",
  "Wow, this is exciting!",
  "I'm feeling a bit sad..."
};

enum Gesture { GESTURE_NONE, GESTURE_SHAKE, GESTURE_NOD, GESTURE_TILT_LEFT, GESTURE_TILT_RIGHT, GESTURE_DOUBLE_TAP };
static const uint32_t GESTURE_COOLDOWN = 2000;
static const uint32_t VOICE_COMMAND_TIMEOUT = 5000;

static const int DRAW_BUF_LINES = 15;
static const uint32_t LVGL_UPDATE_MS = 5;
static const uint32_t POWER_SAVE_TIMEOUT = 300000;
static const uint32_t WIFI_CONNECT_TIMEOUT = 10000;  
static const uint32_t WIFI_RETRY_INTERVAL = 60000;  

// ================== Alarm System ==================
struct Alarm {
  bool enabled;
  int hour;
  int minute;
  int days[7]; // 0=Sun, 1=Mon, ..., 6=Sat
  String label;
  int type; // 0=gentle, 1=standard, 2=energetic
  bool snoozed;
  int snooze_minutes;
};

static Alarm alarms[MAX_ALARMS];
static bool alarm_triggered = false;
static int current_alarm_index = -1;
static uint32_t alarm_start_time = 0;
static uint32_t last_alarm_check = 0;
static uint32_t last_alarm_beep = 0;
static bool alarm_ringing = false;
static int alarm_ring_counter = 0;
static lv_obj_t *alarm_label = nullptr;

// ================== Enhanced Lip Animation ==================
static lv_obj_t *mouth_top = nullptr;
static lv_obj_t *mouth_bottom = nullptr;
static float lip_openness = 0.5f;
static uint32_t last_lip_update = 0;
static float target_lip_openness = 0.5f;
static float lip_transition_progress = 1.0f;

// ================== Particle System ==================
static lv_obj_t *particles[20];
static bool particles_active = false;
static uint32_t particle_last_update = 0;

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel;
  lgfx::Bus_SPI      _bus;
  lgfx::Light_PWM    _light;
public:
  LGFX() {
    { auto cfg=_bus.config();
      cfg.spi_host=SPI2_HOST; cfg.spi_mode=0; cfg.freq_write=80000000; cfg.freq_read=20000000;
      cfg.spi_3wire=true; cfg.use_lock=true; cfg.dma_channel=SPI_DMA_CH_AUTO;
      cfg.pin_sclk=14; cfg.pin_mosi=13; cfg.pin_miso=-1; cfg.pin_dc=2;
      _bus.config(cfg); _panel.setBus(&_bus);
    }
    { auto cfg=_panel.config();
      cfg.pin_cs=15; cfg.pin_rst=4; cfg.pin_busy=-1;
      cfg.panel_width=170; cfg.panel_height=320; cfg.memory_width=240; cfg.memory_height=320;
      cfg.offset_x=35; cfg.offset_y=0; cfg.invert=true; cfg.rgb_order=false; cfg.readable=false;
      cfg.dummy_read_pixel=8; cfg.dummy_read_bits=1; cfg.bus_shared=true;
      _panel.config(cfg);
    }
    { auto cfg=_light.config();
      cfg.pin_bl=21; cfg.invert=false; cfg.freq=10000; cfg.pwm_channel=LEDC_CHANNEL_0;
      _light.config(cfg); _panel.setLight(&_light);
    }
    setPanel(&_panel);
  }
} tft;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t *lvBuf1=nullptr, *lvBuf2=nullptr;
static lv_disp_drv_t disp_drv;

static void IRAM_ATTR lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *a, lv_color_t *color_p) {
  uint16_t w = (a->x2 - a->x1 + 1), h = (a->y2 - a->y1 + 1);
  tft.pushImage(a->x1, a->y1, w, h, (const uint16_t*)color_p);
  lv_disp_flush_ready(drv);
}

static lv_style_t style_eye, style_pupil, style_brow;
static lv_style_t style_calib_text, style_calib_box;
static lv_style_t style_time_text;

static void create_styles() {
  lv_style_init(&style_eye);
  lv_style_set_bg_opa(&style_eye, LV_OPA_COVER);
  lv_style_set_radius(&style_eye, 12);
  lv_style_set_border_width(&style_eye, 0);
  lv_style_set_shadow_width(&style_eye, 16);
  lv_style_set_shadow_spread(&style_eye, 2);

  lv_style_init(&style_pupil);
  lv_style_set_bg_opa(&style_pupil, LV_OPA_COVER);
  lv_style_set_border_width(&style_pupil, 0);
  lv_style_set_radius(&style_pupil, 6);
  lv_style_set_shadow_width(&style_pupil, 6);
  lv_style_set_shadow_color(&style_pupil, lv_color_white());
  lv_style_set_bg_color(&style_pupil, COL_PUPIL);

  lv_style_init(&style_brow);
  lv_style_set_line_width(&style_brow, 4);
  lv_style_set_line_rounded(&style_brow, true);
  
  lv_style_init(&style_calib_text);
  lv_style_set_text_color(&style_calib_text, lv_color_white());
  lv_style_set_text_font(&style_calib_text, &lv_font_montserrat_14);
  lv_style_set_text_align(&style_calib_text, LV_TEXT_ALIGN_CENTER);
  
  lv_style_init(&style_calib_box);
  lv_style_set_bg_color(&style_calib_box, lv_color_hex(0x202020));
  lv_style_set_bg_opa(&style_calib_box, LV_OPA_80);
  lv_style_set_radius(&style_calib_box, 10);
  lv_style_set_pad_all(&style_calib_box, 10);
  
  lv_style_init(&style_time_text);
  lv_style_set_text_color(&style_time_text, lv_color_hex(0x00E5FF));
  lv_style_set_text_font(&style_time_text, &lv_font_montserrat_14);
  lv_style_set_text_align(&style_time_text, LV_TEXT_ALIGN_CENTER);
}

struct Eye;
static void set_brow_line(Eye &e, int tilt);
static void create_eye(Eye &e, int x, int y);
static void blink_one(Eye &e, uint16_t t_ms);

struct Eye {
  lv_obj_t *eye=nullptr, *pupil=nullptr, *lidTop=nullptr, *lidBot=nullptr;
  lv_obj_t *glint=nullptr, *brow=nullptr;
  lv_point_t brow_pts[2];
  int pupil_w=12, pupil_h=12;
  int pupil_dx=0, pupil_dy=0;
  int target_dx=0, target_dy=0;
  bool is_blinking=false;
  float k=0.4f, d=0.7f, vx=0, vy=0;
  int x=0, y=0;
  
  float music_dx = 0;
  float music_dy = 0;
  uint32_t last_beat_time = 0;
  float beat_decay = 0.7f;

  void updatePhysics() {
    float ax = (target_dx - pupil_dx)*k - vx*d;
    float ay = (target_dy - pupil_dy)*k - vy*d;
    vx += ax; vy += ay;
    pupil_dx += vx + music_dx; 
    pupil_dy += vy + music_dy;

    int max_dx = (EYE_W - pupil_w)/2;
    int max_dy = (EYE_H - pupil_h)/2;
    
    pupil_dx = constrain(pupil_dx, -max_dx, max_dx);
    pupil_dy = constrain(pupil_dy, -max_dy, max_dy);

    if (pupil) {
      lv_obj_set_pos(pupil, 
          (EYE_W - pupil_w)/2 + pupil_dx,
          (EYE_H - pupil_h)/2 + pupil_dy);
    }
    
    if (glint) {
      lv_obj_set_pos(glint, 
          (EYE_W - pupil_w)/2 + pupil_dx - 4,
          (EYE_H - pupil_h)/2 + pupil_dy - 4);
    }
    
    music_dx *= beat_decay;
    music_dy *= beat_decay;
    
    if (fabs(music_dx) < 0.1f) music_dx = 0;
    if (fabs(music_dy) < 0.1f) music_dy = 0;
  }
  
  void addBeatImpact(float strength_x, float strength_y) {
    music_dx += strength_x;
    music_dy += strength_y;
    last_beat_time = millis();
  }
  
  void setTarget(int dx, int dy) { 
    target_dx = dx; 
    target_dy = dy; 
  }
  
  void setPupilSize(int w, int h) { 
    pupil_w = w; 
    pupil_h = h; 
    if (pupil) lv_obj_set_size(pupil, w, h); 
    updatePhysics(); 
  }
  
  void resetToCenter() {
    pupil_dx = 0;
    pupil_dy = 0;
    target_dx = 0;
    target_dy = 0;
    vx = 0;
    vy = 0;
    music_dx = 0;
    music_dy = 0;
    updatePhysics();
  }
};

static Eye eyeL, eyeR;

static Emotion current_emotion=EMO_NORMAL, target_emotion=EMO_NORMAL, prev_emotion=EMO_NORMAL;
static Emotion last_stable_emotion=EMO_NORMAL;
static float emotion_blend=1.0f;
static const float emotion_transition_speed=0.15f;
static uint16_t blink_speed=120;
static uint32_t nextBlinkAt=0;
static uint32_t emotion_stable_since=0;
static uint32_t last_activity_time=0;
static bool power_save_mode=false;
static bool calibration_mode = false;
static bool wifi_connected = false;
static bool time_synced = false;
static bool scheduled_sleep = false;
static bool lvgl_initialized = false;

// ACCELEROMETER CALIBRATION - ADJUSTED FOR NEW ORIENTATION
// When device is flat on table facing forward:
// X (top/bottom): 0g
// Y (left/right): 0g  
// Z (forward/backward): -1g (gravity pulling backward)
static float accel_zero_x = 0;
static float accel_zero_y = 0;
static float accel_zero_z = -1.0f;
static bool accel_calibrated = false;

static bool i2s_spk_init_success = false;

static float hpf_prev_in  = 0.0f;
static float hpf_prev_out = 0.0f;

#if ENABLE_WIFI
static WiFiMulti wifiMulti;
static uint32_t wifi_last_attempt = 0;
static bool wifi_connecting = false;
#endif

static lv_obj_t *time_label = nullptr;
static lv_obj_t *wifi_status_label = nullptr;

static float beat_history[8] = {0};
static int beat_history_index = 0;
static uint32_t last_beat_time = 0;
static float beat_bpm = 0;
static bool beat_detected = false;

static float music_dance_phase = 0;
static float music_dance_intensity = 0;
static uint32_t last_music_update = 0;
static float current_loudness = 0;

struct MoodMemory {
  Emotion emotion;
  uint32_t timestamp;
  float intensity;
};
static MoodMemory mood_history[24];
static int mood_history_count = 0;

static Gesture last_gesture = GESTURE_NONE;
static uint32_t last_gesture_time = 0;

static bool voice_command_mode = false;
static uint32_t voice_command_start = 0;

static int battery_level = 100;
static bool charging = false;
static uint32_t last_battery_update = 0;

static float emotion_intensity[EMO_COUNT] = {0};
static Emotion dominant_emotion_today = EMO_NORMAL;

static Preferences preferences;

#if ENABLE_TTS
static bool tts_speaking = false;
static const char* current_tts_text = nullptr;
static size_t tts_text_pos = 0;
static uint32_t tts_next_char_time = 0;
#endif

// ================== FORWARD DECLARATIONS ==================
static void playTone(float freq, int ms, float vol = 0.35f);
static void playMelody(const float* notes, const int* durations, int count, float vol = 0.35f);
static void set_emotion(Emotion emo, bool proofBlink = true);
static void enhancedSetEmotion(Emotion emo, bool withVoice = true, bool immediate = false);
static void updateMoodMemory(Emotion e, float intensity);
static void loadMoodHistory();
static void calibrateAccelerometerZero();
static void calibrateBlinkOffset();
static void autoCalibrateBlink();
static void initTime();

// Time functions
#if ENABLE_TIME_FEATURES
static int getCurrentHour();
static int getCurrentMinute();
static String getCurrentTimeString();
#endif

// Web server functions
#if ENABLE_WEB_SERVER
static void updateBrightness();
static void saveConfig();
static void loadConfig();
static void startAPMode();
static void initWebServer();
static void handleWebServer();
static void handleRoot();
static void handleSave();
static void handleCalibrate();
static void handleReboot();
static void handleTestMusic();
static void handleTestEmotion();
static void handleTestBrightness();
static void handleTestBrightnessRestore();
static void handleReconnectWiFi();
static void handleFactoryReset();
#endif

// New feature functions
static void initAlarmSystem();
static void saveAlarms();
static void loadAlarms();
static void checkAlarms();
static void triggerAlarm(int alarm_index);
static void stopAlarm();
static void snoozeAlarm();
static void updateAlarmRinging();
static String getNextAlarmTime();
static int countActiveAlarms();

// Enhanced animation functions
static void updateLipAnimation(float loudness, bool beat_detected);
static void setLipShapeForEmotion(Emotion e);
static void lipSyncToMusic(float loudness, float bpm);
static void updateLipTransition();

static void initParticleSystem();
static void updateParticles(Emotion e);
static void spawnParticles(int x, int y, int count, lv_color_t color);

// Web server handlers for new features
#if ENABLE_WEB_SERVER
static void handleAlarms();
static void handleSaveAlarm();
static void handleDeleteAlarm();
static void handleTestAlarm();
#endif

static inline void set_top_lid(void *obj, int32_t v) {
  lv_obj_t *o = (lv_obj_t*)obj;
  lv_obj_set_height(o, v);
  lv_obj_set_pos(o, 0, 0);
}

static inline void set_bottom_lid(void *obj, int32_t v) {
  lv_obj_t *o = (lv_obj_t*)obj;
  lv_obj_set_height(o, v);
  lv_obj_set_pos(o, 0, EYE_H - v);
}

static void set_brow_line(Eye &e, int tilt) {
  e.brow_pts[0] = {4,          2 + tilt};
  e.brow_pts[1] = {EYE_W - 4,  6 - tilt};
  lv_line_set_points(e.brow, e.brow_pts, 2);
}

static void create_eye(Eye &e, int x, int y) {
  e.x = x;
  e.y = y;
  
  e.eye = lv_obj_create(lv_scr_act());
  lv_obj_set_size(e.eye, EYE_W, EYE_H);
  lv_obj_set_pos(e.eye, x, y);
  lv_obj_add_style(e.eye, &style_eye, 0);
  lv_style_set_bg_color(&style_eye, COL_EYE[EMO_NORMAL]);
  lv_style_set_shadow_color(&style_eye, COL_EYE[EMO_NORMAL]);
  
  lv_obj_clear_flag(e.eye, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_pad_all(e.eye, 0, 0);
  lv_obj_set_style_border_width(e.eye, 0, 0);
  lv_obj_set_style_outline_width(e.eye, 0, 0);

  e.pupil = lv_obj_create(e.eye);
  lv_obj_add_style(e.pupil, &style_pupil, 0);
  lv_obj_set_size(e.pupil, 12, 12);
  
  int pupil_x = (EYE_W - 12) / 2;
  int pupil_y = (EYE_H - 12) / 2;
  lv_obj_set_pos(e.pupil, pupil_x, pupil_y);
  
  e.pupil_dx = 0;
  e.pupil_dy = 0;
  e.target_dx = 0;
  e.target_dy = 0;
  
  lv_obj_set_style_pad_all(e.pupil, 0, 0);
  lv_obj_set_style_border_width(e.pupil, 0, 0);
  lv_obj_set_style_outline_width(e.pupil, 0, 0);

  e.glint = lv_obj_create(e.eye);
  lv_obj_set_size(e.glint, 4, 4);
  lv_obj_set_style_bg_color(e.glint, lv_color_white(), 0);
  lv_obj_set_style_bg_opa(e.glint, LV_OPA_80, 0);
  lv_obj_set_style_radius(e.glint, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_pos(e.glint, pupil_x - 4, pupil_y - 4);
  
  lv_obj_set_style_pad_all(e.glint, 0, 0);

  e.lidTop = lv_obj_create(e.eye);
  lv_obj_set_size(e.lidTop, EYE_W, 0);
  lv_obj_set_pos(e.lidTop, 0, 0);
  lv_obj_set_style_bg_color(e.lidTop, COL_BG, 0);
  lv_obj_set_style_bg_opa(e.lidTop, LV_OPA_COVER, 0);
  lv_obj_set_style_pad_all(e.lidTop, 0, 0);

  e.lidBot = lv_obj_create(e.eye);
  lv_obj_set_size(e.lidBot, EYE_W, 0);
  lv_obj_set_pos(e.lidBot, 0, EYE_H);
  lv_obj_set_style_bg_color(e.lidBot, COL_BG, 0);
  lv_obj_set_style_bg_opa(e.lidBot, LV_OPA_COVER, 0);
  lv_obj_set_style_pad_all(e.lidBot, 0, 0);

  e.brow = lv_line_create(lv_scr_act());
  lv_obj_add_style(e.brow, &style_brow, 0);
  lv_obj_set_pos(e.brow, x, y - 18);
  e.brow_pts[0] = {4, 2};
  e.brow_pts[1] = {EYE_W - 4, 6};
  lv_line_set_points(e.brow, e.brow_pts, 2);

  lv_obj_move_foreground(e.pupil);
  lv_obj_move_foreground(e.glint);
  lv_obj_move_foreground(e.lidTop);
  lv_obj_move_foreground(e.lidBot);
  lv_obj_move_foreground(e.brow);
}

static void blink_one(Eye &e, uint16_t t_ms) {
  if (!e.lidTop || !e.lidBot) return;
  
  const int OVERLAP=2, close_h=(EYE_H+OVERLAP)/2;

  auto easeOut   = lv_anim_path_ease_out;
  auto easeInOut = lv_anim_path_ease_in_out;

  lv_anim_t a1; lv_anim_init(&a1);
  lv_anim_set_var(&a1, e.lidTop);
  lv_anim_set_time(&a1, (uint16_t)(t_ms * 0.55));
  lv_anim_set_values(&a1, 0, close_h);
  lv_anim_set_exec_cb(&a1, set_top_lid);
  lv_anim_set_path_cb(&a1, easeOut);

  lv_anim_t a2; lv_anim_init(&a2);
  lv_anim_set_var(&a2, e.lidBot);
  lv_anim_set_time(&a2, (uint16_t)(t_ms * 0.55));
  lv_anim_set_values(&a2, 0, close_h);
  lv_anim_set_exec_cb(&a2, set_bottom_lid);
  lv_anim_set_path_cb(&a2, easeOut);

  lv_anim_t a3; lv_anim_init(&a3);
  lv_anim_set_var(&a3, e.lidTop);
  lv_anim_set_delay(&a3, (uint16_t)(t_ms * 0.55));
  lv_anim_set_time(&a3, (uint16_t)(t_ms * 0.45));
  lv_anim_set_values(&a3, close_h, 0);
  lv_anim_set_exec_cb(&a3, set_top_lid);
  lv_anim_set_path_cb(&a3, easeInOut);

  lv_anim_t a4; lv_anim_init(&a4);
  lv_anim_set_var(&a4, e.lidBot);
  lv_anim_set_delay(&a4, (uint16_t)(t_ms * 0.55));
  lv_anim_set_time(&a4, (uint16_t)(t_ms * 0.45));
  lv_anim_set_values(&a4, close_h, 0);
  lv_anim_set_exec_cb(&a4, set_bottom_lid);
  lv_anim_set_path_cb(&a4, easeInOut);

  lv_anim_start(&a1); lv_anim_start(&a2);
  lv_anim_start(&a3); lv_anim_start(&a4);

  lv_timer_t *cleanup = lv_timer_create([](lv_timer_t *t){
    Eye *ee = (Eye*)t->user_data;
    if (ee && ee->lidTop && ee->lidBot) {
      lv_obj_set_pos(ee->lidTop, 0, 0);
      lv_obj_set_pos(ee->lidBot, 0, EYE_H);
    }
    lv_timer_del(t);
  }, t_ms + 40, &e);
  (void)cleanup;
}

static void blink_eyes(uint16_t t) {
  if (!eyeL.eye || !eyeR.eye || !eyeL.lidTop || !eyeR.lidTop || 
      !eyeL.lidBot || !eyeR.lidBot) return;
  
  if (eyeL.is_blinking || eyeR.is_blinking) return;
  eyeL.is_blinking = eyeR.is_blinking = true;
  blink_one(eyeL, t); blink_one(eyeR, t);
  lv_timer_t *done = lv_timer_create([](lv_timer_t *tm){
    eyeL.is_blinking=false; eyeR.is_blinking=false; lv_timer_del(tm);
  }, t+40, nullptr);
  (void)done;
}

static void playTone(float freq, int ms, float vol) {
  if (power_save_mode || !i2s_spk_init_success || scheduled_sleep) return;
  
  const int N = (SPK_SAMPLE_RATE * ms)/1000;
  int16_t frame[2];
  size_t bytesWritten;
  for (int n=0;n<N;n++) {
    float t = (float)n / SPK_SAMPLE_RATE;
    float s = sinf(2.0f * 3.1415926f * freq * t);
    int16_t v = (int16_t)(s * vol * 32767);
    frame[0]=v; frame[1]=v;
    i2s_write(SPK_PORT, frame, sizeof(frame), &bytesWritten, portMAX_DELAY);
  }
}

static void playMelody(const float* notes, const int* durations, int count, float vol) {
  if (power_save_mode || !i2s_spk_init_success || scheduled_sleep) return;
  
  for (int i = 0; i < count; i++) {
    playTone(notes[i], durations[i], vol);
    delay(durations[i] * 0.8f); // Short pause between notes
  }
}

static void emotionAudioNotification(Emotion e) {
  if (power_save_mode || !i2s_spk_init_success || scheduled_sleep || !config.notifications_enabled) return;
  
  // Adjust volume based on time of day
  float volume_multiplier = 1.0f;
  int current_hour = getCurrentHour();
  if (current_hour >= 22 || current_hour <= 7) { // Quiet hours
    volume_multiplier = 0.3f;
  }
  
  switch(e){
    case EMO_SLEEPY: {
      // Gentle descending lullaby with reverb effect
      float notes[] = {440.0f, 392.0f, 349.2f, 329.6f, 293.7f, 261.6f};
      int durations[] = {300, 300, 300, 300, 400, 600};
      for (int i = 0; i < 6; i++) {
        playTone(notes[i], durations[i], 0.15f * volume_multiplier);
        if (i < 5) delay(50); // Small pause between notes
      }
    } break;
    
    case EMO_SAD: {
      // Slow, melancholic minor progression with echo
      float notes[] = {523.3f, 493.9f, 440.0f, 392.0f, 349.2f, 329.6f};
      int durations[] = {400, 400, 400, 400, 600, 800};
      for (int i = 0; i < 6; i++) {
        playTone(notes[i], durations[i], 0.2f * volume_multiplier);
        // Add echo
        if (i < 3) {
          delay(100);
          playTone(notes[i] * 0.5f, durations[i]/2, 0.1f * volume_multiplier);
        }
        if (i < 5) delay(100);
      }
    } break;
    
    case EMO_NORMAL: {
      // Calm, reassuring tones
      float notes[] = {392.0f, 440.0f, 523.3f, 587.3f, 659.3f};
      int durations[] = {200, 200, 200, 200, 400};
      for (int i = 0; i < 5; i++) {
        playTone(notes[i], durations[i], 0.18f * volume_multiplier);
        delay(80);
      }
    } break;
    
    case EMO_CURIOUS: {
      // Rising, questioning pattern with arpeggio
      float base_freq = 440.0f;
      for (int i = 0; i < 4; i++) {
        float freq = base_freq * pow(2, i/12.0f);
        playTone(freq, 150, 0.25f * volume_multiplier);
        delay(120);
      }
      // Quick descending run
      for (int i = 3; i >= 0; i--) {
        float freq = base_freq * pow(2, i/12.0f);
        playTone(freq, 100, 0.2f * volume_multiplier);
        delay(80);
      }
    } break;
    
    case EMO_HAPPY: {
      // Upbeat, happy major progression with harmony
      float melody[] = {523.3f, 659.3f, 784.0f, 880.0f};
      float harmony[] = {392.0f, 493.9f, 587.3f, 698.5f};
      int durations[] = {150, 150, 150, 300};
      
      for (int i = 0; i < 4; i++) {
        // Play chord (melody + harmony)
        int16_t frame[2];
        size_t bytesWritten;
        int N = (SPK_SAMPLE_RATE * durations[i])/1000;
        
        for (int n=0;n<N;n++) {
          float t = (float)n / SPK_SAMPLE_RATE;
          float s1 = sinf(2.0f * 3.1415926f * melody[i] * t);
          float s2 = sinf(2.0f * 3.1415926f * harmony[i] * t);
          float s = (s1 + s2) * 0.5f;
          int16_t v = (int16_t)(s * 0.3f * volume_multiplier * 32767);
          frame[0]=v; frame[1]=v;
          i2s_write(SPK_PORT, frame, sizeof(frame), &bytesWritten, portMAX_DELAY);
        }
        
        if (i < 3) delay(100);
      }
    } break;
    
    case EMO_EXCITED: {
      // Fast, energetic sequence with crescendo
      for (int repeat = 0; repeat < 2; repeat++) {
        float base = 440.0f;
        for (int i = 0; i < 8; i++) {
          float freq = base * (1.0f + i * 0.1f);
          int duration = 60 + i * 10;
          float vol = 0.25f + (i * 0.02f);
          playTone(freq, duration, vol * volume_multiplier);
          delay(40);
        }
        if (repeat == 0) delay(200);
      }
    } break;
    
    case EMO_ANGRY: {
      // Aggressive, dissonant pattern with percussive elements
      float freqs[] = {196.0f, 233.1f, 261.6f, 311.1f, 349.2f};
      for (int i = 0; i < 5; i++) {
        // Main tone
        playTone(freqs[i], 120, 0.35f * volume_multiplier);
        
        // Add percussive element (high frequency burst)
        if (i % 2 == 0) {
          delay(20);
          playTone(freqs[i] * 2.5f, 40, 0.2f * volume_multiplier);
        }
        
        delay(100);
      }
      
      // Final low rumble
      playTone(130.8f, 500, 0.4f * volume_multiplier);
    } break;
    
    default: {
      // Pleasant two-tone notification
      playTone(523.3f, 200, 0.25f * volume_multiplier);
      delay(150);
      playTone(659.3f, 250, 0.25f * volume_multiplier);
    } break;
  }
}

static uint16_t blinkDurationForEmotion(Emotion e) {
  switch(e) {
    case EMO_SLEEPY:  return 260;
    case EMO_SAD:     return 160;
    case EMO_NORMAL:  return 120;
    case EMO_CURIOUS: return 110;
    case EMO_HAPPY:   return 105;
    case EMO_EXCITED: return  90;
    case EMO_ANGRY:   return  95;
    default:          return 120;
  }
}

static uint32_t blinkIntervalForEmotion(Emotion e) {
  switch(e) {
    case EMO_SLEEPY:  return 9000;
    case EMO_SAD:     return 2600;
    case EMO_NORMAL:  return 2000;
    case EMO_CURIOUS: return 1500;
    case EMO_HAPPY:   return 1200;
    case EMO_EXCITED: return  700;
    case EMO_ANGRY:   return  900;
    default:          return 2000;
  }
}

static void scheduleNextBlink() {
  uint32_t interval = blinkIntervalForEmotion(current_emotion);
  float jitterFrac = (current_emotion == EMO_SLEEPY) ? 0.45f : 0.30f;
  int32_t jitter = (int32_t)(interval * jitterFrac);
  interval = interval + random(-jitter, jitter + 1);
  if (current_emotion == EMO_SLEEPY && random(100) < 25) interval += (uint32_t)(interval * 0.8f);
  if (interval < 300) interval = 300;
  blink_speed = blinkDurationForEmotion(current_emotion);
  nextBlinkAt = millis() + interval;
}

static void set_emotion(Emotion emo, bool proofBlink) { 
  if (emo >= EMO_COUNT) return;
  
  if (millis() - emotion_stable_since < 1000) return;
  
  prev_emotion = current_emotion; 
  target_emotion = emo; 
  current_emotion = emo;
  
  // Immediate color change (no transition)
  lv_style_set_bg_color(&style_eye, COL_EYE[emo]);
  lv_style_set_shadow_color(&style_eye, COL_EYE[emo]);
  lv_obj_refresh_style(eyeL.eye, LV_PART_MAIN, LV_STYLE_PROP_ANY);
  lv_obj_refresh_style(eyeR.eye, LV_PART_MAIN, LV_STYLE_PROP_ANY);
  
  last_stable_emotion = emo;
  emotion_stable_since = millis();
  last_activity_time = millis();

  switch (emo) {
    case EMO_ANGRY:   eyeL.setTarget(-8,0); eyeR.setTarget( 8,0); eyeL.setPupilSize(10,10); eyeR.setPupilSize(10,10); set_brow_line(eyeL,6);  set_brow_line(eyeR,-6); break;
    case EMO_HAPPY:   eyeL.setTarget(0,-6); eyeR.setTarget(0,-6); eyeL.setPupilSize(16,16); eyeR.setPupilSize(16,16); set_brow_line(eyeL,-4); set_brow_line(eyeR, 4); break;
    case EMO_SAD:     eyeL.setTarget(0, 8); eyeR.setTarget(0, 8); eyeL.setPupilSize(10,14); eyeR.setPupilSize(10,14); set_brow_line(eyeL,3);  set_brow_line(eyeR,-3); break;
    case EMO_SLEEPY:  eyeL.setTarget(0, 4); eyeR.setTarget(0, 4); eyeL.setPupilSize( 8, 8); eyeR.setPupilSize( 8, 8); set_brow_line(eyeL,1);  set_brow_line(eyeR,-1); break;
    case EMO_EXCITED: eyeL.setTarget(random(-8,8),random(-8,8)); eyeR.setTarget(random(-8,8),random(-8,8)); eyeL.setPupilSize(18,18); eyeR.setPupilSize(18,18); set_brow_line(eyeL,-5); set_brow_line(eyeR,5); break;
    case EMO_CURIOUS: eyeL.setTarget(4,-2); eyeR.setTarget(4,-2); eyeL.setPupilSize(14,14); eyeR.setPupilSize(14,14); set_brow_line(eyeL,4);  set_brow_line(eyeR, 4); break;
    default:          eyeL.setTarget(0,0);  eyeR.setTarget(0,0);  eyeL.setPupilSize(12,12); eyeR.setPupilSize(12,12); set_brow_line(eyeL,0);  set_brow_line(eyeR, 0); break;
  }
  
  setLipShapeForEmotion(emo);
  scheduleNextBlink();
  if (proofBlink) nextBlinkAt = millis() + 400;
}

// ================== WEB SERVER FUNCTIONS ==================
#if ENABLE_WEB_SERVER
static void updateBrightness() {
  if (!power_save_mode && !scheduled_sleep) {
    tft.setBrightness(config.brightness);
  }
}

static void saveConfig() {
  preferences.begin("config", false);
  preferences.putInt("music_sens", config.music_sensitivity);
  preferences.putFloat("mic_gain", config.mic_gain);
  preferences.putInt("eyebrow_speed", config.eyebrow_dance_speed);
  preferences.putInt("eyeball_str", config.eyeball_jump_strength);
  preferences.putBool("auto_emo", config.auto_emotion);
  preferences.putBool("voice_react", config.voice_reactions);
  preferences.putBool("music_react", config.music_reactions);
  preferences.putString("wifi_ssid", config.wifi_ssid);
  preferences.putString("wifi_pass", config.wifi_password);
  preferences.putString("timezone", config.timezone);
  preferences.putInt("sleep_hour", config.sleep_hour);
  preferences.putInt("wake_hour", config.wake_hour);
  preferences.putInt("brightness", config.brightness);
  
  // New settings
  preferences.putBool("lip_sync", config.lip_sync_enabled);
  preferences.putInt("lip_sens", config.lip_sync_sensitivity);
  preferences.putBool("notifications", config.notifications_enabled);
  preferences.putInt("alarm_vol", config.alarm_volume);
  
  preferences.end();
  
  updateBrightness();
}

static void loadConfig() {
  preferences.begin("config", true);
  config.music_sensitivity = preferences.getInt("music_sens", 8);
  config.mic_gain = preferences.getFloat("mic_gain", 0.5f);
  config.eyebrow_dance_speed = preferences.getInt("eyebrow_speed", 8);
  config.eyeball_jump_strength = preferences.getInt("eyeball_str", 12);
  config.auto_emotion = preferences.getBool("auto_emo", true);
  config.voice_reactions = preferences.getBool("voice_react", true);
  config.music_reactions = preferences.getBool("music_react", true);
  config.wifi_ssid = preferences.getString("wifi_ssid", WIFI_SSID);
  config.wifi_password = preferences.getString("wifi_pass", WIFI_PASSWORD);
  config.timezone = preferences.getString("timezone", "IST-5:30");
  config.sleep_hour = preferences.getInt("sleep_hour", 23);
  config.wake_hour = preferences.getInt("wake_hour", 7);
  config.brightness = preferences.getInt("brightness", 255);
  
  // New settings
  config.lip_sync_enabled = preferences.getBool("lip_sync", true);
  config.lip_sync_sensitivity = preferences.getInt("lip_sens", 5);
  config.notifications_enabled = preferences.getBool("notifications", true);
  config.alarm_volume = preferences.getInt("alarm_vol", 7);
  
  preferences.end();
  
  SLEEP_HOUR = config.sleep_hour;
  WAKE_HOUR = config.wake_hour;
  updateBrightness();
}

static void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<meta charset='UTF-8'>";
  html += "<title>Emo Face Pro Configuration</title>";
  html += "<style>";
  html += "* { box-sizing: border-box; }";
  html += "body{font-family:'Segoe UI',Arial,sans-serif;margin:0;padding:20px;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);min-height:100vh;}";
  html += ".container{max-width:800px;margin:auto;background:rgba(255,255,255,0.95);padding:30px;border-radius:20px;box-shadow:0 10px 30px rgba(0,0,0,0.3);backdrop-filter:blur(10px);}";
  html += "h1{color:#333;text-align:center;margin-bottom:30px;font-size:2.5em;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);-webkit-background-clip:text;-webkit-text-fill-color:transparent;}";
  html += "h2{color:#555;border-bottom:3px solid #667eea;padding-bottom:10px;margin-top:40px;font-size:1.8em;}";
  html += ".form-group{margin:20px 0;padding:15px;background:rgba(255,255,255,0.7);border-radius:10px;transition:all 0.3s;}";
  html += ".form-group:hover{background:rgba(255,255,255,0.9);transform:translateY(-2px);box-shadow:0 5px 15px rgba(0,0,0,0.1);}";
  html += "label{display:block;margin-bottom:10px;font-weight:bold;color:#444;font-size:1.1em;}";
  html += "input[type=text],input[type=password],input[type=number],select{width:100%;padding:12px;border:2px solid #ddd;border-radius:8px;font-size:16px;transition:border 0.3s;}";
  html += "input:focus,select:focus{border-color:#667eea;outline:none;}";
  html += "input[type=range]{width:100%;height:8px;border-radius:4px;background:#ddd;outline:none;opacity:0.7;transition:opacity 0.2s;}";
  html += "input[type=range]:hover{opacity:1;}";
  html += "input[type=range]::-webkit-slider-thumb{width:20px;height:20px;border-radius:50%;background:#667eea;cursor:pointer;}";
  html += ".range-container{display:flex;align-items:center;gap:15px;}";
  html += ".range-value{display:inline-block;min-width:40px;padding:5px 10px;background:#667eea;color:white;border-radius:5px;text-align:center;font-weight:bold;font-size:1.1em;}";
  html += ".checkbox-group{display:flex;align-items:center;gap:10px;margin:15px 0;}";
  html += ".checkbox-group input[type=checkbox]{width:20px;height:20px;}";
  html += ".btn{display:inline-block;padding:15px 30px;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);color:white;border:none;border-radius:8px;cursor:pointer;font-size:16px;font-weight:bold;text-decoration:none;transition:all 0.3s;margin:10px 5px;}";
  html += ".btn:hover{transform:translateY(-3px);box-shadow:0 10px 20px rgba(102,126,234,0.3);}";
  html += ".section{margin:30px 0;padding:20px;background:linear-gradient(135deg,rgba(102,126,234,0.1) 0%,rgba(118,75,162,0.1) 100%);border-radius:15px;border-left:5px solid #667eea;}";
  html += ".status{background:linear-gradient(135deg,#e8f5e8 0%,#d4edda 100%);padding:20px;border-radius:10px;margin:20px 0;border:2px solid #28a745;}";
  html += ".status p{margin:10px 0;font-size:1.1em;}";
  html += ".actions-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:15px;margin:20px 0;}";
  html += ".emoji-icon{font-size:1.5em;margin-right:10px;}";
  html += ".alarm-status{background:linear-gradient(135deg,#fff3cd 0%,#ffeaa7 100%);padding:15px;border-radius:8px;margin:15px 0;border:2px solid #ffc107;}";
  html += ".alarm-item{padding:10px;background:#f8f9fa;border-radius:5px;margin:5px 0;border-left:4px solid #667eea;}";
  html += "@media (max-width:600px){.container{padding:15px;} h1{font-size:2em;} .actions-grid{grid-template-columns:1fr;}}";
  html += "</style>";
  html += "</head><body>";
  html += "<div class='container'>";
  html += "<h1>🤖 Emo Face Pro Configuration</h1>";
  
  // Status section
  html += "<div class='section'>";
  html += "<h2>📊 System Status</h2>";
  html += "<div class='status'>";
  html += "<p><strong>WiFi:</strong> ";
  html += wifi_connected ? "✅ Connected (" + WiFi.SSID() + ")" : "❌ Disconnected";
  html += "</p>";
  html += "<p><strong>IP Address:</strong> ";
  html += wifi_connected ? WiFi.localIP().toString() : (ap_mode ? WiFi.softAPIP().toString() : "N/A");
  html += "</p>";
  html += "<p><strong>Time:</strong> ";
  html += time_synced ? "✅ Synchronized" : "❌ Not synchronized";
  html += "</p>";
  html += "<p><strong>Current Emotion:</strong> ";
  html += String(current_emotion);
  html += "</p>";
  html += "<p><strong>Battery:</strong> ";
  html += String(battery_level);
  html += "%</p>";
  html += "<p><strong>Brightness:</strong> ";
  html += String(config.brightness);
  html += "%</p>";
  
  // Alarm status
  html += "<div class='alarm-status'>";
  html += "<p><strong>Alarm Status:</strong> ";
  html += alarm_triggered ? "🔔 Ringing" : "✅ Quiet";
  html += "</p>";
  html += "<p><strong>Next Alarm:</strong> ";
  html += getNextAlarmTime();
  html += "</p>";
  html += "<p><strong>Active Alarms:</strong> ";
  html += String(countActiveAlarms());
  html += "</p>";
  html += "</div>";
  
  html += "</div>";
  html += "</div>";
  
  html += "<form method='post' action='/save' onsubmit='return validateForm()'>";
  
  // WiFi Settings
  html += "<div class='section'>";
  html += "<h2>📶 WiFi Settings</h2>";
  html += "<div class='form-group'>";
  html += "<label for='wifi_ssid'>SSID:</label>";
  html += "<input type='text' id='wifi_ssid' name='wifi_ssid' value='";
  html += config.wifi_ssid;
  html += "' required placeholder='Enter WiFi network name'>";
  html += "</div>";
  html += "<div class='form-group'>";
  html += "<label for='wifi_password'>Password:</label>";
  html += "<input type='password' id='wifi_password' name='wifi_password' value='";
  html += config.wifi_password;
  html += "' placeholder='Enter WiFi password'>";
  html += "</div>";
  html += "</div>";
  
  // Display Settings
  html += "<div class='section'>";
  html += "<h2>💡 Display Settings</h2>";
  html += "<div class='form-group'>";
  html += "<label>Screen Brightness: <span class='range-value' id='brightnessValue'>";
  html += String(config.brightness);
  html += "%</span></label>";
  html += "<input type='range' name='brightness' min='10' max='100' value='";
  html += String(config.brightness);
  html += "' oninput=\"brightnessValue.innerHTML=this.value+'%'\">";
  html += "</div>";
  html += "</div>";
  
  // Music Reaction Settings
  html += "<div class='section'>";
  html += "<h2>🎵 Music Reaction Settings</h2>";
  html += "<div class='form-group'>";
  html += "<label>Music Sensitivity: <span class='range-value' id='sensValue'>";
  html += String(config.music_sensitivity);
  html += "</span></label>";
  html += "<div class='range-container'>";
  html += "<span>Low</span><input type='range' name='music_sensitivity' min='1' max='10' value='";
  html += String(config.music_sensitivity);
  html += "' oninput=\"sensValue.innerHTML=this.value\"><span>High</span>";
  html += "</div>";
  html += "</div>";
  html += "<div class='form-group'>";
  html += "<label>Microphone Gain: <span class='range-value' id='gainValue'>";
  html += String(config.mic_gain, 1);
  html += "</span></label>";
  html += "<input type='range' name='mic_gain' min='0.1' max='2.0' step='0.1' value='";
  html += String(config.mic_gain, 1);
  html += "' oninput=\"gainValue.innerHTML=parseFloat(this.value).toFixed(1)\">";
  html += "</div>";
  html += "<div class='form-group'>";
  html += "<label>Eyebrow Dance Speed: <span class='range-value' id='eyebrowValue'>";
  html += String(config.eyebrow_dance_speed);
  html += "</span></label>";
  html += "<input type='range' name='eyebrow_speed' min='1' max='20' value='";
  html += String(config.eyebrow_dance_speed);
  html += "' oninput=\"eyebrowValue.innerHTML=this.value\">";
  html += "</div>";
  html += "<div class='form-group'>";
  html += "<label>Eyeball Jump Strength: <span class='range-value' id='eyeballValue'>";
  html += String(config.eyeball_jump_strength);
  html += "</span></label>";
  html += "<input type='range' name='eyeball_strength' min='1' max='20' value='";
  html += String(config.eyeball_jump_strength);
  html += "' oninput=\"eyeballValue.innerHTML=this.value\">";
  html += "</div>";
  html += "<div class='form-group'>";
  html += "<label>Lip Sync Sensitivity: <span class='range-value' id='lipSyncValue'>";
  html += String(config.lip_sync_sensitivity);
  html += "</span></label>";
  html += "<div class='range-container'>";
  html += "<span>Low</span><input type='range' name='lip_sync_sensitivity' min='1' max='10' value='";
  html += String(config.lip_sync_sensitivity);
  html += "' oninput=\"lipSyncValue.innerHTML=this.value\"><span>High</span>";
  html += "</div>";
  html += "</div>";
  html += "</div>";
  
  // Behavior Settings
  html += "<div class='section'>";
  html += "<h2>⚙️ Behavior Settings</h2>";
  html += "<div class='checkbox-group'>";
  html += "<input type='checkbox' id='auto_emotion' name='auto_emotion' ";
  html += config.auto_emotion ? "checked" : "";
  html += ">";
  html += "<label for='auto_emotion'>Auto Emotion Detection</label>";
  html += "</div>";
  html += "<div class='checkbox-group'>";
  html += "<input type='checkbox' id='voice_reactions' name='voice_reactions' ";
  html += config.voice_reactions ? "checked" : "";
  html += ">";
  html += "<label for='voice_reactions'>Voice Reactions</label>";
  html += "</div>";
  html += "<div class='checkbox-group'>";
  html += "<input type='checkbox' id='music_reactions' name='music_reactions' ";
  html += config.music_reactions ? "checked" : "";
  html += ">";
  html += "<label for='music_reactions'>Music Reactions</label>";
  html += "</div>";
  html += "<div class='checkbox-group'>";
  html += "<input type='checkbox' id='lip_sync_enabled' name='lip_sync_enabled' ";
  html += config.lip_sync_enabled ? "checked" : "";
  html += ">";
  html += "<label for='lip_sync_enabled'>Lip Sync Animation</label>";
  html += "</div>";
  html += "<div class='checkbox-group'>";
  html += "<input type='checkbox' id='notifications_enabled' name='notifications_enabled' ";
  html += config.notifications_enabled ? "checked" : "";
  html += ">";
  html += "<label for='notifications_enabled'>Notifications</label>";
  html += "</div>";
  html += "<div class='form-group'>";
  html += "<label for='timezone'>Timezone:</label>";
  html += "<select id='timezone' name='timezone'>";
  html += "<option value='IST-5:30'";
  if (config.timezone == "IST-5:30") html += " selected";
  html += ">IST (India)</option>";
  html += "<option value='EST+5:00'";
  if (config.timezone == "EST+5:00") html += " selected";
  html += ">EST (USA East)</option>";
  html += "<option value='PST+8:00'";
  if (config.timezone == "PST+8:00") html += " selected";
  html += ">PST (USA West)</option>";
  html += "<option value='GMT+0:00'";
  if (config.timezone == "GMT+0:00") html += " selected";
  html += ">GMT (London)</option>";
  html += "<option value='CET-1:00'";
  if (config.timezone == "CET-1:00") html += " selected";
  html += ">CET (Europe)</option>";
  html += "<option value='JST-9:00'";
  if (config.timezone == "JST-9:00") html += " selected";
  html += ">JST (Japan)</option>";
  html += "</select>";
  html += "</div>";
  html += "<div class='form-group'>";
  html += "<label for='sleep_hour'>Sleep Time (24h format):</label>";
  html += "<input type='number' id='sleep_hour' name='sleep_hour' min='0' max='23' value='";
  html += String(config.sleep_hour);
  html += "'>";
  html += "</div>";
  html += "<div class='form-group'>";
  html += "<label for='wake_hour'>Wake Time (24h format):</label>";
  html += "<input type='number' id='wake_hour' name='wake_hour' min='0' max='23' value='";
  html += String(config.wake_hour);
  html += "'>";
  html += "</div>";
  html += "<div class='form-group'>";
  html += "<label>Alarm Volume: <span class='range-value' id='alarmVolValue'>";
  html += String(config.alarm_volume);
  html += "</span></label>";
  html += "<div class='range-container'>";
  html += "<span>Quiet</span><input type='range' name='alarm_volume' min='1' max='10' value='";
  html += String(config.alarm_volume);
  html += "' oninput=\"alarmVolValue.innerHTML=this.value\"><span>Loud</span>";
  html += "</div>";
  html += "</div>";
  html += "</div>";
  
  html += "<div class='form-group' style='text-align:center;'>";
  html += "<button type='submit' class='btn'>💾 Save All Settings</button>";
  html += "</div>";
  html += "</form>";
  
  // Actions section
  html += "<div class='section'>";
  html += "<h2>🚀 Quick Actions</h2>";
  html += "<div class='actions-grid'>";
  html += "<a href='/alarms' class='btn'><span class='emoji-icon'>⏰</span>Alarm Settings</a>";
  html += "<a href='/calibrate' class='btn'><span class='emoji-icon'>🎯</span>Calibrate</a>";
  html += "<a href='/reboot' class='btn'><span class='emoji-icon'>🔄</span>Reboot</a>";
  html += "<a href='/wifi' class='btn'><span class='emoji-icon'>📶</span>Reconnect WiFi</a>";
  html += "<a href='/test/music' class='btn'><span class='emoji-icon'>🎵</span>Test Music</a>";
  html += "<a href='/test/emotion' class='btn'><span class='emoji-icon'>😊</span>Test Emotions</a>";
  html += "<a href='/test/brightness' class='btn'><span class='emoji-icon'>💡</span>Test Brightness</a>";
  html += "<a href='/test/alarm' class='btn'><span class='emoji-icon'>🔔</span>Test Alarm</a>";
  html += "<a href='/factory_reset' class='btn' style='background:linear-gradient(135deg,#ff416c 0%,#ff4b2b 100%);'><span class='emoji-icon'>⚠️</span>Factory Reset</a>";
  html += "</div>";
  html += "</div>";
  
  // Active Alarms Display
  html += "<div class='section'>";
  html += "<h2>⏰ Active Alarms</h2>";
  int activeCount = 0;
  for (int i = 0; i < MAX_ALARMS; i++) {
    if (alarms[i].enabled) {
      activeCount++;
      html += "<div class='alarm-item'>";
      html += "<strong>" + alarms[i].label + "</strong> - ";
      html += String(alarms[i].hour).length() == 1 ? "0" + String(alarms[i].hour) : String(alarms[i].hour);
      html += ":";
      html += String(alarms[i].minute).length() == 1 ? "0" + String(alarms[i].minute) : String(alarms[i].minute);
      html += " ";
      String days[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
      for (int d = 0; d < 7; d++) {
        if (alarms[i].days[d]) html += days[d].substring(0, 1) + " ";
      }
      html += "</div>";
    }
  }
  if (activeCount == 0) {
    html += "<p>No alarms set. <a href='/alarms'>Set alarms here</a></p>";
  }
  html += "</div>";
  
  html += "<div style='text-align:center;margin-top:40px;color:#666;font-size:0.9em;'>";
  html += "<p>Emo Face Pro v2.1 | ";
  html += __DATE__;
  html += " | ";
  html += wifi_connected ? "Station Mode" : (ap_mode ? "AP Mode" : "Disconnected");
  html += "</p>";
  html += "</div>";
  
  html += "</div>";
  
  // JavaScript
  html += "<script>";
  html += "function validateForm() {";
  html += "  var sleep = document.getElementById('sleep_hour').value;";
  html += "  var wake = document.getElementById('wake_hour').value;";
  html += "  if(sleep < 0 || sleep > 23 || wake < 0 || wake > 23) {";
  html += "    alert('Sleep and wake hours must be between 0 and 23');";
  html += "    return false;";
  html += "  }";
  html += "  if(parseInt(sleep) === parseInt(wake)) {";
  html += "    alert('Sleep and wake times cannot be the same');";
  html += "    return false;";
  html += "  }";
  html += "  return true;";
  html += "}";
  html += "document.addEventListener('DOMContentLoaded', function() {";
  html += "  var sliders = document.querySelectorAll('input[type=range]');";
  html += "  sliders.forEach(function(slider) {";
  html += "    slider.addEventListener('input', function() {";
  html += "      var valueDisplay = this.parentElement.querySelector('.range-value');";
  html += "      if(valueDisplay) {";
        html += "        if(this.name === 'brightness') {";
  html += "          valueDisplay.innerHTML = this.value + '%';";
  html += "        } else {";
  html += "          valueDisplay.innerHTML = this.value;";
  html += "        }";
  html += "      }";
  html += "    });";
  html += "  });";
  html += "});";
  html += "</script>";
  html += "</body></html>";
  
  webServer.send(200, "text/html", html);
}

static void handleSave() {
  if (webServer.method() == HTTP_POST) {
    for (int i = 0; i < webServer.args(); i++) {
      String name = webServer.argName(i);
      String value = webServer.arg(i);
      
      if (name == "music_sensitivity") {
        config.music_sensitivity = value.toInt();
      } else if (name == "mic_gain") {
        config.mic_gain = value.toFloat();
      } else if (name == "eyebrow_speed") {
        config.eyebrow_dance_speed = value.toInt();
      } else if (name == "eyeball_strength") {
        config.eyeball_jump_strength = value.toInt();
      } else if (name == "wifi_ssid") {
        config.wifi_ssid = value;
      } else if (name == "wifi_password") {
        config.wifi_password = value;
      } else if (name == "timezone") {
        config.timezone = value;
      } else if (name == "sleep_hour") {
        config.sleep_hour = value.toInt();
      } else if (name == "wake_hour") {
        config.wake_hour = value.toInt();
      } else if (name == "brightness") {
        config.brightness = value.toInt();
        updateBrightness();
      } else if (name == "lip_sync_sensitivity") {
        config.lip_sync_sensitivity = value.toInt();
      } else if (name == "alarm_volume") {
        config.alarm_volume = value.toInt();
      } else if (name == "auto_emotion") {
        config.auto_emotion = true;
      } else if (name == "voice_reactions") {
        config.voice_reactions = true;
      } else if (name == "music_reactions") {
        config.music_reactions = true;
      } else if (name == "lip_sync_enabled") {
        config.lip_sync_enabled = true;
      } else if (name == "notifications_enabled") {
        config.notifications_enabled = true;
      }
    }
    
    if (!webServer.hasArg("auto_emotion")) config.auto_emotion = false;
    if (!webServer.hasArg("voice_reactions")) config.voice_reactions = false;
    if (!webServer.hasArg("music_reactions")) config.music_reactions = false;
    if (!webServer.hasArg("lip_sync_enabled")) config.lip_sync_enabled = false;
    if (!webServer.hasArg("notifications_enabled")) config.notifications_enabled = false;
    
    saveConfig();
    
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'></head><body style='font-family:Arial;text-align:center;padding:50px;'>";
    html += "<div style='max-width:500px;margin:auto;background:#f0f0f0;padding:30px;border-radius:10px;'>";
    html += "<h1 style='color:#4CAF50;'>✅ Settings Saved!</h1>";
    html += "<p>Configuration has been updated successfully.</p>";
    html += "<a href='/' style='display:inline-block;margin-top:20px;padding:10px 20px;background:#4CAF50;color:white;text-decoration:none;border-radius:5px;'>Return to Dashboard</a>";
    html += "</div></body></html>";
    
    webServer.send(200, "text/html", html);
    
    if (webServer.hasArg("wifi_ssid")) {
      #if ENABLE_WIFI
      wifi_connected = false;
      WiFi.disconnect();
      delay(100);
      #endif
    }
  } else {
    webServer.send(405, "text/plain", "Method Not Allowed");
  }
}

static void handleCalibrate() {
  calibration_mode = true;
  Serial.println("Entering calibration mode...");
  Serial.println("Place device flat on a surface and press 'c' to calibrate accelerometer");
  Serial.println("Press 'x' to reset calibration");
  Serial.println("Press 'a' to auto-calibrate blink");
  
  // Show calibration message on screen
  lv_obj_t *calib_msg = lv_label_create(lv_scr_act());
  lv_label_set_text(calib_msg, "CALIBRATION MODE\nPlace device flat\nPress 'c' to calibrate\nPress 'x' to reset");
  lv_obj_align(calib_msg, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_text_align(calib_msg, LV_TEXT_ALIGN_CENTER, 0);
  
  String html = "<html><body><h1>Calibration Mode Activated</h1>";
  html += "<p>Check serial monitor for calibration instructions.</p>";
  html += "<p><a href='/'>Back to config</a></p></body></html>";
  webServer.send(200, "text/html", html);
}

static void handleReboot() {
  String html = "<html><body><h1>Rebooting...</h1>";
  html += "<p>Emo Face Pro will restart in 3 seconds.</p></body></html>";
  webServer.send(200, "text/html", html);
  delay(3000);
  ESP.restart();
}

static void handleTestMusic() {
  enhancedSetEmotion(EMO_EXCITED, false);
  String html = "<html><body><h1>Music Reaction Test</h1>";
  html += "<p>Playing test tones and activating music reactions...</p>";
  html += "<p><a href='/'>Back to config</a></p></body></html>";
  webServer.send(200, "text/html", html);
  
  if (i2s_spk_init_success) {
    for(int i = 0; i < 5; i++) {
      playTone(440 + i*100, 200, 0.3f);
      delay(300);
    }
  }
}

static void handleReconnectWiFi() {
  wifi_connected = false;
  String html = "<html><body><h1>Reconnecting WiFi...</h1>";
  html += "<p>Attempting to reconnect to WiFi network.</p>";
  html += "<p><a href='/'>Back to config</a></p></body></html>";
  webServer.send(200, "text/html", html);
}

static void handleTestBrightness() {
  static int test_brightness = 50;
  test_brightness = (test_brightness == 50) ? 100 : 50;
  
  int old_brightness = config.brightness;
  config.brightness = test_brightness;
  updateBrightness();
  
  String html = "<html><body><h1>Brightness Test</h1>";
  html += "<p>Brightness set to ";
  html += String(test_brightness);
  html += "%</p>";
  html += "<p><a href='/'>Back to config</a></p>";
  html += "<script>";
  html += "setTimeout(function() {";
  html += "  window.location.href = '/test/brightness/restore?old=";
  html += String(old_brightness);
  html += "';";
  html += "}, 2000);";
  html += "</script>";
  html += "</body></html>";
  
  webServer.send(200, "text/html", html);
}

static void handleTestBrightnessRestore() {
  if (webServer.hasArg("old")) {
    config.brightness = webServer.arg("old").toInt();
    updateBrightness();
  }
  
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void handleTestEmotion() {
  static int emotion_index = 0;
  Emotion emotions[] = {EMO_NORMAL, EMO_HAPPY, EMO_SAD, EMO_EXCITED, EMO_ANGRY, EMO_CURIOUS, EMO_SLEEPY};
  
  enhancedSetEmotion(emotions[emotion_index], true);
  emotion_index = (emotion_index + 1) % 7;
  
  String html = "<html><body><h1>Emotion Test</h1>";
  html += "<p>Testing emotion: ";
  html += String(emotion_index);
  html += "</p>";
  html += "<p><a href='/test/emotion'>Test Next Emotion</a></p>";
  html += "<p><a href='/'>Back to config</a></p>";
  html += "</body></html>";
  
  webServer.send(200, "text/html", html);
}

static void handleTestAlarm() {
  triggerAlarm(0);
  String html = "<html><body><h1>Alarm Test</h1>";
  html += "<p>Test alarm triggered!</p>";
  html += "<p><a href='/'>Back to config</a></p>";
  html += "</body></html>";
  webServer.send(200, "text/html", html);
}

static void handleFactoryReset() {
  String html = "<html><body style='text-align:center;padding:50px;'>";
  
  if (webServer.hasArg("confirm")) {
    preferences.begin("config", false);
    preferences.clear();
    preferences.end();
    
    preferences.begin("face", false);
    preferences.clear();
    preferences.end();
    
    preferences.begin("mood", false);
    preferences.clear();
    preferences.end();
    
    preferences.begin("alarms", false);
    preferences.clear();
    preferences.end();
    
    html += "<h1 style='color:#4CAF50;'>Factory Reset Complete!</h1>";
    html += "<p>All settings have been reset to defaults.</p>";
    html += "<p>The device will reboot in 5 seconds...</p>";
    html += "<script>setTimeout(function(){ window.location.href='/'; }, 5000);</script>";
    
    webServer.send(200, "text/html", html);
    
    delay(5000);
    ESP.restart();
  } else {
    html += "<h1 style='color:#ff4444;'>⚠️ Factory Reset ⚠️</h1>";
    html += "<div style='max-width:500px;margin:auto;background:#ffe6e6;padding:20px;border-radius:10px;border:2px solid #ff4444;'>";
    html += "<p>This will reset ALL settings to factory defaults:</p>";
    html += "<ul style='text-align:left;'>";
    html += "<li>WiFi credentials</li>";
    html += "<li>Display settings</li>";
    html += "<li>Music reaction settings</li>";
    html += "<li>Alarm settings</li>";
    html += "<li>Calibration data</li>";
    html += "<li>Mood history</li>";
    html += "</ul>";
    html += "<p><strong>This action cannot be undone!</strong></p>";
    html += "</div>";
    html += "<div style='margin-top:30px;'>";
    html += "<a href='/factory_reset?confirm=true' style='padding:15px 30px;background:#ff4444;color:white;text-decoration:none;border-radius:5px;margin-right:20px;'>Confirm Reset</a>";
    html += "<a href='/' style='padding:15px 30px;background:#666;color:white;text-decoration:none;border-radius:5px;'>Cancel</a>";
    html += "</div>";
    html += "</body></html>";
    
    webServer.send(200, "text/html", html);
  }
}

static void handleAlarms() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<meta charset='UTF-8'>";
  html += "<title>Alarm Settings - Emo Face Pro</title>";
  html += "<style>";
  html += "* { box-sizing: border-box; }";
  html += "body{font-family:'Segoe UI',Arial,sans-serif;margin:0;padding:20px;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);min-height:100vh;}";
  html += ".container{max-width:800px;margin:auto;background:rgba(255,255,255,0.95);padding:30px;border-radius:20px;box-shadow:0 10px 30px rgba(0,0,0,0.3);}";
  html += "h1{color:#333;text-align:center;margin-bottom:30px;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);-webkit-background-clip:text;-webkit-text-fill-color:transparent;}";
  html += ".alarm-card{background:linear-gradient(135deg,#f5f7fa 0%,#c3cfe2 100%);border-radius:15px;padding:20px;margin:15px 0;box-shadow:0 5px 15px rgba(0,0,0,0.1);}";
  html += ".alarm-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:15px;}";
  html += ".alarm-time{font-size:2em;font-weight:bold;color:#333;}";
  html += ".alarm-label{font-size:1.2em;color:#555;margin:10px 0;}";
  html += ".day-selector{display:flex;gap:5px;margin:10px 0;}";
  html += ".day-btn{padding:5px 10px;border-radius:5px;border:1px solid #ddd;cursor:pointer;background:#fff;}";
  html += ".day-btn.active{background:#667eea;color:white;}";
  html += ".btn{padding:10px 20px;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);color:white;border:none;border-radius:8px;cursor:pointer;margin:5px;text-decoration:none;display:inline-block;}";
  html += ".btn-danger{background:linear-gradient(135deg,#ff416c 0%,#ff4b2b 100%);}";
  html += ".btn-success{background:linear-gradient(135deg,#4CAF50 0%,#2E7D32 100%);}";
  html += ".form-group{margin:15px 0;}";
  html += "input[type=time],input[type=text],select{padding:10px;border:2px solid #ddd;border-radius:5px;width:100%;}";
  html += ".alarm-type{display:flex;gap:10px;margin:10px 0;}";
  html += ".type-option{padding:10px;border:2px solid #ddd;border-radius:5px;cursor:pointer;background:#fff;}";
  html += ".type-option.active{border-color:#667eea;background:#e8f0fe;}";
  html += ".switch{position:relative;display:inline-block;width:60px;height=34px;}";
  html += ".switch input{opacity:0;width:0;height:0;}";
  html += ".slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background-color:#ccc;transition:.4s;border-radius:34px;}";
  html += ".slider:before{position:absolute;content:\"\";height:26px;width:26px;left:4px;bottom:4px;background-color:white;transition:.4s;border-radius:50%;}";
  html += "input:checked + .slider{background-color:#667eea;}";
  html += "input:checked + .slider:before{transform:translateX(26px);}";
  html += "</style>";
  html += "<script>";
  html += "function toggleDay(btn, alarmIndex, dayIndex) {";
  html += "  btn.classList.toggle('active');";
  html += "  var daysInput = document.getElementById('days_' + alarmIndex);";
  html += "  if(!daysInput) return;";
  html += "  var days = daysInput.value.split(',').map(Number);";
  html += "  days[dayIndex] = days[dayIndex] ? 0 : 1;";
  html += "  daysInput.value = days.join(',');";
  html += "}";
  html += "function setAlarmType(type, alarmIndex) {";
  html += "  document.querySelectorAll('#type_' + alarmIndex + ' .type-option').forEach(el => el.classList.remove('active'));";
  html += "  event.target.classList.add('active');";
  html += "  document.getElementById('type_input_' + alarmIndex).value = type;";
  html += "}";
  html += "function testAlarm() {";
  html += "  fetch('/test/alarm');";
  html += "  alert('Test alarm triggered! Check your device.');";
  html += "}";
  html += "function toggleAlarm(index, enabled) {";
  html += "  document.getElementById('enabled_' + index).value = enabled ? '1' : '0';";
  html += "}";
  html += "function saveAlarm(index) {";
  html += "  var form = document.getElementById('alarm_form_' + index);";
  html += "  form.submit();";
  html += "}";
  html += "function deleteAlarm(index) {";
  html += "  if(confirm('Delete this alarm?')) {";
  html += "    window.location.href = '/delete_alarm?index=' + index;";
  html += "  }";
  html += "}";
  html += "function addAlarm() {";
  html += "  var time = document.getElementById('newAlarmTime').value;";
  html += "  var label = document.getElementById('newAlarmLabel').value || 'New Alarm';";
  html += "  if(!time) { alert('Please set a time'); return; }";
  html += "  window.location.href = '/save_alarm?time=' + time + '&label=' + encodeURIComponent(label);";
  html += "}";
  html += "</script>";
  html += "</head><body>";
  html += "<div class='container'>";
  html += "<h1>⏰ Alarm Settings</h1>";
  
  html += "<div style='text-align:center;margin-bottom:30px;'>";
  html += "<a href='/' class='btn'>← Back to Main</a>";
  html += "<button class='btn' onclick='testAlarm()'>🔔 Test Alarm</button>";
  html += "</div>";
  
  for (int i = 0; i < MAX_ALARMS; i++) {
    html += "<div class='alarm-card'>";
    html += "<div class='alarm-header'>";
    html += "<div class='alarm-time'>";
    html += String(alarms[i].hour).length() == 1 ? "0" + String(alarms[i].hour) : String(alarms[i].hour);
    html += ":";
    html += String(alarms[i].minute).length() == 1 ? "0" + String(alarms[i].minute) : String(alarms[i].minute);
    html += "</div>";
    html += "<label class='switch'>";
    html += "<input type='checkbox' " + String(alarms[i].enabled ? "checked" : "") + " onchange='toggleAlarm(" + String(i) + ", this.checked)'>";
    html += "<span class='slider'></span>";
    html += "</label>";
    html += "</div>";
    
    html += "<form id='alarm_form_" + String(i) + "' method='post' action='/save_alarm'>";
    html += "<input type='hidden' name='index' value='" + String(i) + "'>";
    html += "<input type='hidden' id='enabled_" + String(i) + "' name='enabled' value='" + String(alarms[i].enabled ? "1" : "0") + "'>";
    
    html += "<div class='form-group'>";
    html += "<label>Time:</label>";
    html += "<input type='time' name='time' value='";
    html += String(alarms[i].hour).length() == 1 ? "0" + String(alarms[i].hour) : String(alarms[i].hour);
    html += ":";
    html += String(alarms[i].minute).length() == 1 ? "0" + String(alarms[i].minute) : String(alarms[i].minute);
    html += "' required>";
    html += "</div>";
    
    html += "<div class='form-group'>";
    html += "<label>Label:</label>";
    html += "<input type='text' name='label' value='" + alarms[i].label + "' placeholder='Wake Up Alarm'>";
    html += "</div>";
    
    html += "<div class='form-group'>";
    html += "<label>Repeat:</label>";
    html += "<div class='day-selector'>";
    String days[] = {"S", "M", "T", "W", "T", "F", "S"};
    String daysValue = "";
    for (int d = 0; d < 7; d++) {
      html += "<div class='day-btn " + String(alarms[i].days[d] ? "active" : "") + "' onclick='toggleDay(this, " + String(i) + ", " + String(d) + ")'>" + days[d] + "</div>";
      daysValue += String(alarms[i].days[d] ? "1" : "0");
      if (d < 6) daysValue += ",";
    }
    html += "<input type='hidden' id='days_" + String(i) + "' name='days' value='" + daysValue + "'>";
    html += "</div>";
    html += "</div>";
    
    html += "<div class='form-group'>";
    html += "<label>Alarm Type:</label>";
    html += "<div class='alarm-type' id='type_" + String(i) + "'>";
    html += "<div class='type-option " + String(alarms[i].type == 0 ? "active" : "") + "' onclick='setAlarmType(0, " + String(i) + ")'>Gentle</div>";
    html += "<div class='type-option " + String(alarms[i].type == 1 ? "active" : "") + "' onclick='setAlarmType(1, " + String(i) + ")'>Standard</div>";
    html += "<div class='type-option " + String(alarms[i].type == 2 ? "active" : "") + "' onclick='setAlarmType(2, " + String(i) + ")'>Energetic</div>";
    html += "</div>";
    html += "<input type='hidden' id='type_input_" + String(i) + "' name='type' value='" + String(alarms[i].type) + "'>";
    html += "</div>";
    
    html += "<div style='text-align:center;'>";
    html += "<button type='button' class='btn btn-success' onclick='saveAlarm(" + String(i) + ")'>💾 Save Alarm</button>";
    html += "<button type='button' class='btn btn-danger' onclick='deleteAlarm(" + String(i) + ")'>🗑️ Delete</button>";
    html += "</div>";
    
    html += "</form>";
    html += "</div>";
  }
  
  html += "<div class='alarm-card'>";
  html += "<h3 style='text-align:center;'>Add New Alarm</h3>";
  html += "<div class='form-group'>";
  html += "<label>Time:</label>";
  html += "<input type='time' id='newAlarmTime' required>";
  html += "</div>";
  html += "<div class='form-group'>";
  html += "<label>Label:</label>";
  html += "<input type='text' id='newAlarmLabel' placeholder='Wake Up Alarm'>";
  html += "</div>";
  html += "<div style='text-align:center;'>";
  html += "<button type='button' class='btn' onclick='addAlarm()'>➕ Add Alarm</button>";
  html += "</div>";
  html += "</div>";
  
  html += "</div>";
  html += "</body></html>";
  
  webServer.send(200, "text/html", html);
}

static void handleSaveAlarm() {
  if (webServer.method() == HTTP_POST) {
    int index = webServer.arg("index").toInt();
    if (index >= 0 && index < MAX_ALARMS) {
      String timeStr = webServer.arg("time");
      int colonPos = timeStr.indexOf(':');
      if (colonPos > 0) {
        alarms[index].hour = timeStr.substring(0, colonPos).toInt();
        alarms[index].minute = timeStr.substring(colonPos + 1).toInt();
      }
      
      alarms[index].label = webServer.arg("label");
      alarms[index].enabled = webServer.arg("enabled") == "1";
      alarms[index].type = webServer.arg("type").toInt();
      
      String daysStr = webServer.arg("days");
      int dayIndex = 0;
      int startPos = 0;
      while (dayIndex < 7 && startPos < daysStr.length()) {
        int endPos = daysStr.indexOf(',', startPos);
        if (endPos == -1) endPos = daysStr.length();
        alarms[index].days[dayIndex] = daysStr.substring(startPos, endPos).toInt();
        startPos = endPos + 1;
        dayIndex++;
      }
      
      saveAlarms();
    }
    
    webServer.sendHeader("Location", "/alarms");
    webServer.send(303);
  } else {
    String timeStr = webServer.arg("time");
    String label = webServer.arg("label");
    
    // Find empty slot
    for (int i = 0; i < MAX_ALARMS; i++) {
      if (!alarms[i].enabled && alarms[i].label == "Alarm " + String(i + 1)) {
        int colonPos = timeStr.indexOf(':');
        if (colonPos > 0) {
          alarms[i].hour = timeStr.substring(0, colonPos).toInt();
          alarms[i].minute = timeStr.substring(colonPos + 1).toInt();
        }
        alarms[i].label = label;
        alarms[i].enabled = true;
        for (int d = 0; d < 7; d++) alarms[i].days[d] = (d < 5); // Weekdays default
        
        saveAlarms();
        break;
      }
    }
    
    webServer.sendHeader("Location", "/alarms");
    webServer.send(303);
  }
}

static void handleDeleteAlarm() {
  int index = webServer.arg("index").toInt();
  if (index >= 0 && index < MAX_ALARMS) {
    alarms[index].enabled = false;
    alarms[index].label = "Alarm " + String(index + 1);
    saveAlarms();
  }
  
  webServer.sendHeader("Location", "/alarms");
  webServer.send(303);
}

static void startAPMode() {
  WiFi.mode(WIFI_AP);
  ap_mode = true;
  
  uint8_t mac[6];
  WiFi.macAddress(mac);
  
  char apName[32];
  sprintf(apName, "EmoFacePro-%02X%02X", mac[4], mac[5]);
  ap_ssid = String(apName);
  
  Serial.println("Starting Access Point...");
  Serial.print("AP SSID: ");
  Serial.println(ap_ssid);
  Serial.print("AP Password: ");
  Serial.println(ap_password);
  
  if (!WiFi.softAP(ap_ssid.c_str(), ap_password.c_str())) {
    Serial.println("AP Failed to start!");
    return;
  }
  
  delay(100);
  
  IPAddress apIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(apIP);
  
  initWebServer();
  
  web_server_active = true;
  wifi_connected = false;
  
  if (wifi_status_label) {
    lv_label_set_text(wifi_status_label, "AP Mode");
    lv_obj_set_style_text_color(wifi_status_label, lv_color_hex(0xFFCC00), 0);
  }
  
  Serial.println("=== Access Point Started ===");
  Serial.println("Connect to WiFi: " + ap_ssid);
  Serial.println("Password: " + ap_password);
  Serial.println("Open browser to: http://" + apIP.toString());
  Serial.println("=============================");
  
  if (i2s_spk_init_success) {
    playTone(523, 200, 0.3f);
    delay(100);
    playTone(659, 200, 0.3f);
    delay(100);
    playTone(784, 300, 0.3f);
  }
}

static void initWebServer() {
  webServer.on("/", handleRoot);
  webServer.on("/save", HTTP_POST, handleSave);
  webServer.on("/calibrate", handleCalibrate);
  webServer.on("/reboot", handleReboot);
  webServer.on("/test/music", handleTestMusic);
  webServer.on("/test/emotion", handleTestEmotion);
  webServer.on("/test/brightness", handleTestBrightness);
  webServer.on("/test/brightness/restore", handleTestBrightnessRestore);
  webServer.on("/test/alarm", handleTestAlarm);
  webServer.on("/wifi", handleReconnectWiFi);
  webServer.on("/factory_reset", handleFactoryReset);
  webServer.on("/alarms", handleAlarms);
  webServer.on("/save_alarm", handleSaveAlarm);
  webServer.on("/delete_alarm", handleDeleteAlarm);
  
  webServer.onNotFound([]() {
    String message = "File Not Found\n\n";
    message += "URI: " + webServer.uri() + "\n";
    message += "Method: ";
    message += (webServer.method() == HTTP_GET ? "GET" : "POST");
    message += "\n";
    webServer.send(404, "text/plain", message);
  });
  
  webServer.begin();
  web_server_active = true;
  Serial.println("Web server started");
}

static void handleWebServer() {
  if (web_server_active) {
    webServer.handleClient();
  }
}

#endif  // ENABLE_WEB_SERVER

struct Star { float x,y,phase,speed; uint8_t size,base; };
static const int STAR_COUNT = 32;
static Star stars[STAR_COUNT];
static bool starlight_enabled = true;

static void stars_init() {
  for (int i=0;i<STAR_COUNT;i++) {
    stars[i].x = random(0, SCR_W);
    stars[i].y = random(0, SCR_H);
    stars[i].phase = random(0, 628) / 100.0f;
    stars[i].speed = 0.8f + random(0, 120)/100.0f;
    stars[i].size  = (random(100)<80) ? 1 : 2;
    stars[i].base  = 40 + random(0, 80);
  }
}

static void stars_draw() {
  if (!starlight_enabled) return;
  tft.startWrite();
  uint32_t ms = millis();
  for (int i=0;i<STAR_COUNT;i++) {
    float t = stars[i].phase + (ms * 0.001f) * stars[i].speed;
    float s = 0.5f + 0.5f * sinf(t);
    uint8_t b = stars[i].base + (uint8_t)(s * (200 - stars[i].base));
    uint16_t col = ((b & 0xF8) << 8) | ((b & 0xFC) << 3) | (b >> 3);
    int xi = (int)stars[i].x, yi=(int)stars[i].y;
    if (stars[i].size==1) tft.drawPixel(xi, yi, col); 
    else tft.fillRect(xi, yi, 2, 2, col);
  }
  tft.endWrite();
}

namespace MMA8452Q {
  uint8_t addr = 0x1D;
  const uint8_t REG_WHOAMI    = 0x0D;
  const uint8_t REG_CTRL1     = 0x2A;
  const uint8_t REG_XYZ_CFG   = 0x0E;
  const uint8_t REG_OUT_X_MSB = 0x01;

  bool present = false;

  uint8_t read8(uint8_t reg) {
    Wire.beginTransmission(addr); 
    Wire.write(reg); 
    Wire.endTransmission(false);
    Wire.requestFrom((int)addr, 1);
    return Wire.available() ? Wire.read() : 0xFF;
  }
  
  void write8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr); 
    Wire.write(reg); 
    Wire.write(val); 
    Wire.endTransmission();
  }

  bool init() {
    for (uint8_t a : { (uint8_t)0x1D, (uint8_t)0x1C }) {
      addr = a;
      uint8_t who = read8(REG_WHOAMI);
      if (who == 0x2A) { 
        present = true; 
        break; 
      }
    }
    if (!present) return false;

    uint8_t c1 = read8(REG_CTRL1);
    write8(REG_CTRL1, c1 & ~0x01);
    write8(REG_XYZ_CFG, 0x00);
    write8(REG_CTRL1, (c1 & ~(0b111 << 3)) | (0b010 << 3) | 0x01);
    return true;
  }

  void read(float &ax_g, float &ay_g, float &az_g) {
    ax_g=ay_g=az_g=0;
    if (!present) return;
    Wire.beginTransmission(addr); 
    Wire.write(REG_OUT_X_MSB); 
    Wire.endTransmission(false);
    Wire.requestFrom((int)addr, 6);
    if (Wire.available() < 6) return;

    int16_t x = (Wire.read()<<8) | Wire.read();
    int16_t y = (Wire.read()<<8) | Wire.read();
    int16_t z = (Wire.read()<<8) | Wire.read();

    x >>= 2; y >>= 2; z >>= 2;
    const float SCALE = 1.0f / 4096.0f;
    ax_g = x * SCALE; ay_g = y * SCALE; az_g = z * SCALE;
  }
}

static inline float dc_block(float x) {
  const float R = 0.995f;
  float y = x - hpf_prev_in + R * hpf_prev_out;
  hpf_prev_in  = x;
  hpf_prev_out = y;
  return y;
}

static void i2s_mic_init() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = MIC_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = MIC_IS_LEFT ? I2S_CHANNEL_FMT_ONLY_LEFT
                                  : I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = DMA_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  #if defined(ESP_IDF_VERSION) && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0))
    i2s_pin_config_t pins = {
      .mck_io_num   = I2S_PIN_NO_CHANGE,
      .bck_io_num   = MIC_BCLK_PIN,
      .ws_io_num    = MIC_WS_PIN,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num  = MIC_DATA_PIN
    };
  #else
    i2s_pin_config_t pins = {
      .bck_io_num   = MIC_BCLK_PIN,
      .ws_io_num    = MIC_WS_PIN,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num  = MIC_DATA_PIN
    };
  #endif

  esp_err_t err = i2s_driver_install(MIC_PORT, &cfg, 0, NULL);
  if (err != ESP_OK) return;
  
  err = i2s_set_pin(MIC_PORT, &pins);
  if (err != ESP_OK) return;
  
  i2s_zero_dma_buffer(MIC_PORT);
}

static void i2s_spk_init() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SPK_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 6,
    .dma_buf_len = DMA_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  #if defined(ESP_IDF_VERSION) && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0))
    i2s_pin_config_t pins = {
      .mck_io_num   = I2S_PIN_NO_CHANGE,
      .bck_io_num   = SPK_BCLK_PIN,
      .ws_io_num    = SPK_WS_PIN,
      .data_out_num = SPK_DOUT_PIN,
      .data_in_num  = I2S_PIN_NO_CHANGE
    };
  #else
    i2s_pin_config_t pins = {
      .bck_io_num   = SPK_BCLK_PIN,
      .ws_io_num    = SPK_WS_PIN,
      .data_out_num = SPK_DOUT_PIN,
      .data_in_num  = I2S_PIN_NO_CHANGE
    };
  #endif

  esp_err_t err = i2s_driver_install(SPK_PORT, &cfg, 0, NULL);
  if (err != ESP_OK) {
    i2s_spk_init_success = false;
    return;
  }
  
  err = i2s_set_pin(SPK_PORT, &pins);
  if (err != ESP_OK) {
    i2s_spk_init_success = false;
    return;
  }
  
  i2s_zero_dma_buffer(SPK_PORT);
  
  i2s_set_clk(SPK_PORT, SPK_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
  
  i2s_spk_init_success = true;
}

static float mic_read_loudness() {
  static int32_t mic_buffer[DMA_LEN];
  size_t bytesRead = 0;
  
  esp_err_t err = i2s_read(MIC_PORT, (void*)mic_buffer, sizeof(mic_buffer), &bytesRead, 0);
  if (err != ESP_OK || bytesRead == 0) return 0.0f;

  int samples = bytesRead / sizeof(int32_t);
  float peak = 0.0f;
  
  for (int i = 0; i < samples; i++) {
    float s = (float)(mic_buffer[i] >> SAMPLE_SHIFT);
    s = dc_block(s);
    s *= config.mic_gain;
    
    float a = fabsf(s);
    if (a > peak) peak = a;
  }
  
  float scaled_peak = peak / (500000.0f / (config.music_sensitivity * 0.5f));
  return fminf(1.0f, scaled_peak);
}

// ================== ALARM SYSTEM ==================
static void initAlarmSystem() {
  for (int i = 0; i < MAX_ALARMS; i++) {
    alarms[i].enabled = false;
    alarms[i].hour = 7;
    alarms[i].minute = 0;
    for (int d = 0; d < 7; d++) {
      alarms[i].days[d] = (d < 5);
    }
    alarms[i].label = "Alarm " + String(i + 1);
    alarms[i].type = 1;
    alarms[i].snoozed = false;
    alarms[i].snooze_minutes = 5;
  }
  
  loadAlarms();
  
  alarm_label = lv_label_create(lv_scr_act());
  lv_obj_add_style(alarm_label, &style_time_text, 0);
  lv_obj_set_width(alarm_label, SCR_W);
  lv_obj_set_pos(alarm_label, 0, 50);
  lv_label_set_text(alarm_label, "");
  lv_obj_set_style_text_align(alarm_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_color(alarm_label, lv_color_hex(0xFFCC00), 0);
}

static void saveAlarms() {
  preferences.begin("alarms", false);
  for (int i = 0; i < MAX_ALARMS; i++) {
    String prefix = "alarm" + String(i);
    preferences.putBool((prefix + "_enabled").c_str(), alarms[i].enabled);
    preferences.putInt((prefix + "_hour").c_str(), alarms[i].hour);
    preferences.putInt((prefix + "_minute").c_str(), alarms[i].minute);
    preferences.putString((prefix + "_label").c_str(), alarms[i].label);
    preferences.putInt((prefix + "_type").c_str(), alarms[i].type);
    
    uint8_t days_bitmap = 0;
    for (int d = 0; d < 7; d++) {
      if (alarms[i].days[d]) days_bitmap |= (1 << d);
    }
    preferences.putUChar((prefix + "_days").c_str(), days_bitmap);
  }
  preferences.end();
}

static void loadAlarms() {
  preferences.begin("alarms", true);
  for (int i = 0; i < MAX_ALARMS; i++) {
    String prefix = "alarm" + String(i);
    alarms[i].enabled = preferences.getBool((prefix + "_enabled").c_str(), false);
    alarms[i].hour = preferences.getInt((prefix + "_hour").c_str(), 7);
    alarms[i].minute = preferences.getInt((prefix + "_minute").c_str(), 0);
    alarms[i].label = preferences.getString((prefix + "_label").c_str(), "Alarm " + String(i + 1));
    alarms[i].type = preferences.getInt((prefix + "_type").c_str(), 1);
    
    uint8_t days_bitmap = preferences.getUChar((prefix + "_days").c_str(), 0x1F);
    for (int d = 0; d < 7; d++) {
      alarms[i].days[d] = (days_bitmap & (1 << d)) != 0;
    }
  }
  preferences.end();
}

static void checkAlarms() {
  if (!time_synced || alarm_triggered) return;
  
  uint32_t now = millis();
  if (now - last_alarm_check < 60000) return;
  
  last_alarm_check = now;
  
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return;
  
  int current_hour = timeinfo.tm_hour;
  int current_minute = timeinfo.tm_min;
  int current_day = timeinfo.tm_wday;
  
  for (int i = 0; i < MAX_ALARMS; i++) {
    if (!alarms[i].enabled || alarms[i].snoozed) continue;
    
    if (!alarms[i].days[current_day]) continue;
    
    if (alarms[i].hour == current_hour && alarms[i].minute == current_minute) {
      triggerAlarm(i);
      return;
    }
  }
}

static void triggerAlarm(int alarm_index) {
  alarm_triggered = true;
  current_alarm_index = alarm_index;
  alarm_start_time = millis();
  alarm_ringing = true;
  alarm_ring_counter = 0;
  
  Alarm &alarm = alarms[alarm_index];
  
  Serial.println("ALARM TRIGGERED: " + alarm.label);
  Serial.println("Alarm will ring continuously for " + String(ALARM_RING_DURATION/1000) + " seconds");
  
  if (alarm_label) {
    lv_label_set_text(alarm_label, ("⏰ " + alarm.label).c_str());
    lv_obj_set_style_text_color(alarm_label, lv_color_hex(0xFF3366), 0);
  }
  
  enhancedSetEmotion(EMO_EXCITED, false);
}

static void updateAlarmRinging() {
  if (!alarm_triggered || !alarm_ringing) return;
  
  uint32_t now = millis();
  if (now - alarm_start_time > ALARM_RING_DURATION) {
    // Alarm has been ringing long enough, stop it
    stopAlarm();
    return;
  }
  
  // Ring continuously with different patterns based on alarm type
  if (now - last_alarm_beep > 500) { // Update every 500ms
    last_alarm_beep = now;
    alarm_ring_counter++;
    
    Alarm &alarm = alarms[current_alarm_index];
    float volume = config.alarm_volume * 0.1f;
    
    switch(alarm.type) {
      case 0: // Gentle
        if (alarm_ring_counter % 4 == 0) {
          playTone(440, 300, volume * 0.6f);
          blink_eyes(80);
        }
        break;
        
      case 1: // Standard
        if (alarm_ring_counter % 2 == 0) {
          playTone(880, 200, volume * 0.8f);
          blink_eyes(60);
        }
        if (alarm_ring_counter % 8 == 0) {
          playTone(523, 400, volume * 0.8f);
        }
        break;
        
      case 2: // Energetic
        playTone(440 + (alarm_ring_counter % 3) * 110, 100, volume);
        if (alarm_ring_counter % 4 == 0) {
          blink_eyes(40);
          // Make eyes flash red
          lv_obj_set_style_bg_color(eyeL.eye, lv_color_hex(0xFF6600), 0);
          lv_obj_set_style_bg_color(eyeR.eye, lv_color_hex(0xFF6600), 0);
          lv_timer_t *reset_color = lv_timer_create([](lv_timer_t *t){
            lv_obj_set_style_bg_color(eyeL.eye, COL_EYE[EMO_EXCITED], 0);
            lv_obj_set_style_bg_color(eyeR.eye, COL_EYE[EMO_EXCITED], 0);
            lv_timer_del(t);
          }, 100, nullptr);
        }
        break;
    }
  }
}

static void stopAlarm() {
  if (!alarm_triggered) return;
  
  alarm_triggered = false;
  alarm_ringing = false;
  
  if (alarm_label) {
    lv_label_set_text(alarm_label, "");
  }
  
  enhancedSetEmotion(current_emotion, false);
  
  Serial.println("Alarm stopped");
}

static void snoozeAlarm() {
  if (!alarm_triggered || current_alarm_index == -1) return;
  
  alarms[current_alarm_index].snoozed = true;
  
  lv_timer_t *snooze_timer = lv_timer_create([](lv_timer_t *t) {
    int alarm_idx = (int)(intptr_t)t->user_data;
    if (alarm_idx >= 0 && alarm_idx < MAX_ALARMS) {
      alarms[alarm_idx].snoozed = false;
    }
    lv_timer_del(t);
  }, alarms[current_alarm_index].snooze_minutes * 60000, (void*)(intptr_t)current_alarm_index);
  
  stopAlarm();
  enhancedSetEmotion(EMO_SLEEPY, false);
}

static String getNextAlarmTime() {
  for (int i = 0; i < MAX_ALARMS; i++) {
    if (alarms[i].enabled && !alarms[i].snoozed) {
      return String(alarms[i].hour).length() == 1 ? "0" + String(alarms[i].hour) : String(alarms[i].hour) + ":" + 
             (String(alarms[i].minute).length() == 1 ? "0" + String(alarms[i].minute) : String(alarms[i].minute));
    }
  }
  return "No alarms";
}

static int countActiveAlarms() {
  int count = 0;
  for (int i = 0; i < MAX_ALARMS; i++) {
    if (alarms[i].enabled) count++;
  }
  return count;
}

// ================== ENHANCED LIP ANIMATION ==================
static void initLipAnimation() {
  mouth_top = lv_obj_create(lv_scr_act());
  lv_obj_set_size(mouth_top, 32, 6);
  lv_obj_set_style_radius(mouth_top, 12, 0);
  lv_obj_set_style_bg_color(mouth_top, lv_color_hex(0xD43F3F), 0);
  lv_obj_set_style_bg_opa(mouth_top, LV_OPA_COVER, 0);
  lv_obj_set_pos(mouth_top, (SCR_W-32)/2, 200);
  
  mouth_bottom = lv_obj_create(lv_scr_act());
  lv_obj_set_size(mouth_bottom, 32, 6);
  lv_obj_set_style_radius(mouth_bottom, 12, 0);
  lv_obj_set_style_bg_color(mouth_bottom, lv_color_hex(0xD43F3F), 0);
  lv_obj_set_style_bg_opa(mouth_bottom, LV_OPA_COVER, 0);
  lv_obj_set_pos(mouth_bottom, (SCR_W-32)/2, 206);
  
  lv_obj_move_foreground(mouth_top);
  lv_obj_move_foreground(mouth_bottom);
}

static void updateLipTransition() {
  if (lip_transition_progress < 1.0f) {
    lip_transition_progress += 0.05f;
    if (lip_transition_progress > 1.0f) lip_transition_progress = 1.0f;
    
    float eased_progress = lip_transition_progress * lip_transition_progress * (3.0f - 2.0f * lip_transition_progress);
    lip_openness = lip_openness * (1.0f - eased_progress) + target_lip_openness * eased_progress;
  }
}

static void updateLipAnimation(float loudness, bool beat_detected) {
  if (!config.lip_sync_enabled || scheduled_sleep) return;
  
  uint32_t now = millis();
  if (now - last_lip_update < 30) return;
  
  last_lip_update = now;
  
  updateLipTransition();
  
  if (loudness > 0.1f && config.music_reactions) {
    lipSyncToMusic(loudness, beat_bpm);
    return;
  }
  
  // Breathing animation when no music
  float breath = sin(millis() * 0.002f) * 0.3f + 0.7f;
  float current_openness = lip_openness * breath;
  
  int height = 3 + (int)(current_openness * 9);
  int separation = 2 + (int)((1.0f - current_openness) * 3);
  
  // Smooth animation using LVGL animations
  lv_anim_t anim_top, anim_bottom;
  lv_anim_init(&anim_top);
  lv_anim_init(&anim_bottom);
  
  lv_anim_set_var(&anim_top, mouth_top);
  lv_anim_set_var(&anim_bottom, mouth_bottom);
  lv_anim_set_time(&anim_top, 100);
  lv_anim_set_time(&anim_bottom, 100);
  
  lv_anim_set_values(&anim_top, lv_obj_get_height(mouth_top), height);
  lv_anim_set_values(&anim_bottom, lv_obj_get_height(mouth_bottom), height);
  
  lv_anim_set_exec_cb(&anim_top, [](void *obj, int32_t v) {
    lv_obj_set_height((lv_obj_t*)obj, v);
  });
  lv_anim_set_exec_cb(&anim_bottom, [](void *obj, int32_t v) {
    lv_obj_set_height((lv_obj_t*)obj, v);
  });
  
  lv_anim_start(&anim_top);
  lv_anim_start(&anim_bottom);
  
  int mouth_width = 28 + (int)(current_openness * 12);
  lv_obj_set_width(mouth_top, mouth_width);
  lv_obj_set_width(mouth_bottom, mouth_width);
  
  lv_obj_set_x(mouth_top, (SCR_W - mouth_width)/2);
  lv_obj_set_x(mouth_bottom, (SCR_W - mouth_width)/2);
  lv_obj_set_y(mouth_bottom, 200 + height + separation);
}

static void lipSyncToMusic(float loudness, float bpm) {
  float time = millis() * 0.001f;
  
  // Multiple sine waves for complex lip movement
  float movement1 = sin(time * bpm * 0.01f) * 0.5f + 0.5f;
  float movement2 = sin(time * bpm * 0.02f + 1.0f) * 0.3f;
  float movement3 = cos(time * bpm * 0.005f + 2.0f) * 0.2f;
  
  float openness = (movement1 + movement2 + movement3) * loudness * 2.0f;
  openness = constrain(openness, 0.0f, 1.0f);
  
  target_lip_openness = openness;
  lip_transition_progress = 0.0f;
  
  int max_height = 12;
  int min_height = 2;
  int height = min_height + (int)(openness * (max_height - min_height));
  
  int min_width = 24;
  int max_width = 40;
  int width = min_width + (int)(openness * (max_width - min_width));
  
  int separation = 1 + (int)((1.0f - openness) * 3);
  
  lv_anim_t anim_top, anim_bottom;
  lv_anim_init(&anim_top);
  lv_anim_init(&anim_bottom);
  
  lv_anim_set_var(&anim_top, mouth_top);
  lv_anim_set_var(&anim_bottom, mouth_bottom);
  lv_anim_set_time(&anim_top, 80);
  lv_anim_set_time(&anim_bottom, 80);
  lv_anim_set_path_cb(&anim_top, lv_anim_path_ease_out);
  lv_anim_set_path_cb(&anim_bottom, lv_anim_path_ease_out);
  
  lv_anim_set_values(&anim_top, lv_obj_get_height(mouth_top), height);
  lv_anim_set_values(&anim_bottom, lv_obj_get_height(mouth_bottom), height);
  
  lv_anim_set_exec_cb(&anim_top, [](void *obj, int32_t v) {
    lv_obj_set_height((lv_obj_t*)obj, v);
  });
  lv_anim_set_exec_cb(&anim_bottom, [](void *obj, int32_t v) {
    lv_obj_set_height((lv_obj_t*)obj, v);
  });
  
  lv_anim_start(&anim_top);
  lv_anim_start(&anim_bottom);
  
  lv_obj_set_width(mouth_top, width);
  lv_obj_set_width(mouth_bottom, width);
  lv_obj_set_x(mouth_top, (SCR_W - width)/2);
  lv_obj_set_x(mouth_bottom, (SCR_W - width)/2);
  lv_obj_set_y(mouth_bottom, 200 + height + separation);
  
  // Color intensity based on loudness
  uint8_t color_intensity = 160 + (int)(openness * 95);
  lv_color_t lip_color = lv_color_make(color_intensity, 40, 40);
  lv_obj_set_style_bg_color(mouth_top, lip_color, 0);
  lv_obj_set_style_bg_color(mouth_bottom, lip_color, 0);
}

static void setLipShapeForEmotion(Emotion e) {
  if (!config.lip_sync_enabled) return;
  
  float target_openness;
  int width;
  lv_color_t color = lv_color_hex(0xD43F3F);
  
  switch(e) {
    case EMO_HAPPY:
      target_openness = 0.8f;
      width = 36;
      color = lv_color_hex(0xFF3366);
      break;
    case EMO_SAD:
      target_openness = 0.3f;
      width = 30;
      color = lv_color_hex(0x9933CC);
      break;
    case EMO_EXCITED:
      target_openness = 0.9f;
      width = 40;
      color = lv_color_hex(0xFF6600);
      break;
    case EMO_ANGRY:
      target_openness = 0.4f;
      width = 28;
      color = lv_color_hex(0xFF0000);
      break;
    case EMO_SLEEPY:
      target_openness = 0.2f;
      width = 24;
      color = lv_color_hex(0x9966FF);
      break;
    case EMO_CURIOUS:
      target_openness = 0.5f;
      width = 26;
      color = lv_color_hex(0x33CCCC);
      break;
    default:
      target_openness = 0.5f;
      width = 32;
      break;
  }
  
  target_lip_openness = target_openness;
  lip_transition_progress = 0.0f;
  
  int height = 3 + (int)(target_openness * 9);
  int separation = 2 + (int)((1.0f - target_openness) * 3);
  
  lv_anim_t anim;
  lv_anim_init(&anim);
  lv_anim_set_time(&anim, 300);
  lv_anim_set_path_cb(&anim, lv_anim_path_ease_out);
  
  lv_anim_set_var(&anim, mouth_top);
  lv_anim_set_values(&anim, lv_obj_get_height(mouth_top), height);
  lv_anim_set_exec_cb(&anim, [](void *obj, int32_t v) {
    lv_obj_set_height((lv_obj_t*)obj, v);
  });
  lv_anim_start(&anim);
  
  lv_anim_set_var(&anim, mouth_bottom);
  lv_anim_set_values(&anim, lv_obj_get_height(mouth_bottom), height);
  anim.exec_cb = [](void *obj, int32_t v) {
    lv_obj_set_height((lv_obj_t*)obj, v);
  };
  lv_anim_start(&anim);
  
  lv_obj_set_width(mouth_top, width);
  lv_obj_set_width(mouth_bottom, width);
  lv_obj_set_x(mouth_top, (SCR_W - width)/2);
  lv_obj_set_x(mouth_bottom, (SCR_W - width)/2);
  lv_obj_set_y(mouth_bottom, 200 + height + separation);
  
  lv_obj_set_style_bg_color(mouth_top, color, 0);
  lv_obj_set_style_bg_color(mouth_bottom, color, 0);
}

// ================== PARTICLE SYSTEM ==================
static void initParticleSystem() {
  for (int i = 0; i < 20; i++) {
    particles[i] = lv_obj_create(lv_scr_act());
    lv_obj_set_size(particles[i], 2, 2);
    lv_obj_set_style_radius(particles[i], LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(particles[i], LV_OPA_0, 0);
    lv_obj_add_flag(particles[i], LV_OBJ_FLAG_HIDDEN);
  }
}

static void updateParticles(Emotion e) {
  if (!particles_active) return;
  
  uint32_t now = millis();
  if (now - particle_last_update < 16) return;
  
  particle_last_update = now;
  
  for (int i = 0; i < 20; i++) {
    lv_obj_t *p = particles[i];
    if (lv_obj_has_flag(p, LV_OBJ_FLAG_HIDDEN)) continue;
    
    int x = lv_obj_get_x(p);
    int y = lv_obj_get_y(p);
    
    y -= 2;
    lv_obj_set_pos(p, x, y);
    
    lv_opa_t opa = lv_obj_get_style_bg_opa(p, 0);
    if (opa > 10) {
      lv_obj_set_style_bg_opa(p, opa - 10, 0);
    } else {
      lv_obj_add_flag(p, LV_OBJ_FLAG_HIDDEN);
    }
    
    if (y < -10) {
      lv_obj_add_flag(p, LV_OBJ_FLAG_HIDDEN);
    }
  }
}

static void spawnParticles(int x, int y, int count, lv_color_t color) {
  particles_active = true;
  
  for (int i = 0; i < min(count, 20); i++) {
    lv_obj_t *p = particles[i];
    
    int px = x + random(-10, 10);
    int py = y + random(-10, 10);
    
    lv_obj_set_pos(p, px, py);
    lv_obj_set_style_bg_color(p, color, 0);
    lv_obj_set_style_bg_opa(p, LV_OPA_COVER, 0);
    lv_obj_clear_flag(p, LV_OBJ_FLAG_HIDDEN);
  }
}

// Music reaction functions
static void updateEyebrowDance(float loudness, float beat_bpm) {
  static uint32_t last_eyebrow_update = 0;
  uint32_t now = millis();
  
  if (!config.music_reactions || now - last_eyebrow_update < 20) return;
  
  last_eyebrow_update = now;
  
  if (loudness > 0.1f) { 
    float phase_increment = (loudness * 0.15f) + (beat_bpm / 800.0f);
    music_dance_phase += phase_increment * (config.eyebrow_dance_speed * 0.5f);
    
    if (music_dance_phase > 6.283f) music_dance_phase -= 6.283f;
    
    float dance1 = sin(music_dance_phase);
    float dance2 = sin(music_dance_phase * 1.7f + 1.0f);
    float dance3 = cos(music_dance_phase * 0.6f + 2.0f);
    
    float eyebrow_tilt = (dance1 * 0.5f + dance2 * 0.3f + dance3 * 0.2f) * 15.0f * loudness;
    
    float eyebrow_left = eyebrow_tilt * 0.8f + sin(music_dance_phase * 0.9f) * 5.0f * loudness;
    float eyebrow_right = -eyebrow_tilt * 0.8f + cos(music_dance_phase * 0.8f + 1.5f) * 5.0f * loudness;
    
    set_brow_line(eyeL, (int)eyebrow_left);
    set_brow_line(eyeR, (int)eyebrow_right);
    
    int line_width = 3 + (int)(loudness * 4);
    lv_obj_set_style_line_width(eyeL.brow, line_width, 0);
    lv_obj_set_style_line_width(eyeR.brow, line_width, 0);
    
  } else {
    static Emotion last_emotion = EMO_NORMAL;
    if (current_emotion != last_emotion) {
      switch(current_emotion) {
        case EMO_ANGRY:   set_brow_line(eyeL, 6);  set_brow_line(eyeR, -6); break;
        case EMO_HAPPY:   set_brow_line(eyeL, -4); set_brow_line(eyeR, 4);  break;
        case EMO_SAD:     set_brow_line(eyeL, 3);  set_brow_line(eyeR, -3); break;
        case EMO_SLEEPY:  set_brow_line(eyeL, 1);  set_brow_line(eyeR, -1); break;
        case EMO_EXCITED: set_brow_line(eyeL, -5); set_brow_line(eyeR, 5);  break;
        case EMO_CURIOUS: set_brow_line(eyeL, 4);  set_brow_line(eyeR, 4);  break;
        default:          set_brow_line(eyeL, 0);  set_brow_line(eyeR, 0);  break;
      }
      last_emotion = current_emotion;
    }
  }
}

static void updateEyeballJump(float loudness, bool beat_detected_flag) {
  static uint32_t last_jump_update = 0;
  uint32_t now = millis();
  
  if (!config.music_reactions || now - last_jump_update < 15) return;
  
  last_jump_update = now;
  
  if (loudness > 0.1f) { 
    float gentle_x = sin(music_dance_phase * 1.5f) * 4.0f * loudness;
    float gentle_y = cos(music_dance_phase * 1.2f + 0.5f) * 3.0f * loudness;
    
    eyeL.music_dx += gentle_x * 0.15f;
    eyeL.music_dy += gentle_y * 0.15f;
    eyeR.music_dx += gentle_x * 0.15f;
    eyeR.music_dy += gentle_y * 0.15f;
    
    if (beat_detected_flag && loudness > 0.3f) {
      float jump_strength = (config.eyeball_jump_strength * 0.5f) * loudness;
      
      float angle = random(0, 628) / 100.0f;
      float jump_x = cos(angle) * jump_strength;
      float jump_y = sin(angle) * jump_strength;
      
      eyeL.addBeatImpact(jump_x, jump_y);
      eyeR.addBeatImpact(jump_x, jump_y);
      
      if (loudness > 0.5f && !eyeL.is_blinking) {
        blink_eyes(40 + (int)(loudness * 60));
      }
      
      int pulse_size = 6 + (int)(loudness * 16);
      eyeL.setPupilSize(pulse_size, pulse_size);
      eyeR.setPupilSize(pulse_size, pulse_size);
      
      if (loudness > 0.6f) {
        lv_obj_set_style_shadow_width(eyeL.eye, 30, 0);
        lv_obj_set_style_shadow_width(eyeR.eye, 30, 0);
        lv_obj_set_style_shadow_spread(eyeL.eye, 8, 0);
        lv_obj_set_style_shadow_spread(eyeR.eye, 8, 0);
      }
    }
    
    static uint32_t last_shadow_decay = 0;
    if (now - last_shadow_decay > 80) {
      last_shadow_decay = now;
      int current_shadow = lv_obj_get_style_shadow_width(eyeL.eye, 0);
      if (current_shadow > 16) {
        lv_obj_set_style_shadow_width(eyeL.eye, current_shadow - 2, 0);
        lv_obj_set_style_shadow_width(eyeR.eye, current_shadow - 2, 0);
      }
      if (lv_obj_get_style_shadow_spread(eyeL.eye, 0) > 2) {
        lv_obj_set_style_shadow_spread(eyeL.eye, 2, 0);
        lv_obj_set_style_shadow_spread(eyeR.eye, 2, 0);
      }
    }
  }
}

static void updateBeatDetection() {
  if (!config.music_reactions) return;
  
  float loudness = mic_read_loudness();
  current_loudness = loudness;
  
  beat_history[beat_history_index] = loudness;
  beat_history_index = (beat_history_index + 1) % 8;
  
  float avg = 0;
  for(int i = 0; i < 8; i++) {
    avg += beat_history[i];
  }
  avg /= 8.0f;
  
  bool beat_detected_flag = false;
  
  if (loudness > avg * 1.3f && loudness > MUSIC_THRESHOLD) {
    uint32_t now = millis();
    if (now - last_beat_time > 150) {
      beat_detected_flag = true;
      beat_detected = true;
      
      if (last_beat_time > 0) {
        float interval = (now - last_beat_time) / 1000.0f;
        float instant_bpm = 60.0f / interval;
        
        beat_bpm = beat_bpm * 0.6f + instant_bpm * 0.4f;
      }
      last_beat_time = now;
      
      if (config.auto_emotion && loudness > 0.4f && current_emotion != EMO_EXCITED && random(100) < 50) {
        set_emotion(EMO_EXCITED);
      }
    }
  } else {
    beat_detected = false;
  }
  
  updateEyebrowDance(loudness, beat_bpm);
  updateEyeballJump(loudness, beat_detected_flag);
  
  music_dance_intensity = music_dance_intensity * 0.9f + loudness * 0.1f;
  last_music_update = millis();
}

#if ENABLE_GESTURES
// UPDATED GESTURE DETECTION FOR NEW ACCELEROMETER ORIENTATION
static Gesture detectGesture(float ax, float ay, float az) {
  static float last_ax = 0, last_ay = 0, last_az = 0;
  
  float dx = ax - last_ax;
  float dy = ay - last_ay;
  float dz = az - last_az;
  
  last_ax = ax; last_ay = ay; last_az = az;
  
  float change_magnitude = sqrt(dx*dx + dy*dy + dz*dz);
  
  if (change_magnitude > 0.8f) {
    return GESTURE_SHAKE;
  }
  
  static float x_history[5] = {0}; // Now using X axis (top/bottom) for nod detection
  static int x_index = 0;
  x_history[x_index] = ax;
  x_index = (x_index + 1) % 5;
  
  int direction_changes = 0;
  for(int i = 1; i < 5; i++) {
    if ((x_history[i] - x_history[i-1]) * (x_history[i-1] - x_history[i-2]) < 0) {
      direction_changes++;
    }
  }
  
  if (direction_changes >= 3 && fabs(ax) > 0.3f) {
    return GESTURE_NOD;
  }
  
  // For tilt detection, now use Y axis (left/right)
  if (ay < -0.4f) return GESTURE_TILT_LEFT;
  if (ay > 0.4f) return GESTURE_TILT_RIGHT;
  
  return GESTURE_NONE;
}

static void handleGesture(Gesture g) {
  if (millis() - last_gesture_time < GESTURE_COOLDOWN) return;
  
  Emotion next_emotion, prev_emotion_local;
  
  switch(g) {
    case GESTURE_SHAKE:
      set_emotion(EMO_EXCITED);
      break;
      
    case GESTURE_NOD:
      set_emotion(EMO_HAPPY);
      break;
      
    case GESTURE_TILT_LEFT:
      next_emotion = (Emotion)((current_emotion + 1) % EMO_COUNT);
      set_emotion(next_emotion);
      break;
      
    case GESTURE_TILT_RIGHT:
      prev_emotion_local = (Emotion)((current_emotion - 1 + EMO_COUNT) % EMO_COUNT);
      set_emotion(prev_emotion_local);
      break;
      
    default:
      break;
  }
  
  last_gesture_time = millis();
}
#endif

static void enhancedBlink() {
  if (eyeL.is_blinking || eyeR.is_blinking) return;
  
  if (current_loudness > 0.2f) {
    blink_speed = 60 + (int)(current_loudness * 60);
    if (random(100) < current_loudness * 40) {
      blink_eyes(blink_speed);
    }
  } else {
    switch(current_emotion) {
      case EMO_SLEEPY:
        blink_speed = 300;
        break;
      case EMO_EXCITED:
        blink_speed = 80;
        if (random(100) < 30) {
          blink_eyes(blink_speed);
          delay(100);
        }
        break;
      case EMO_SAD:
        blink_speed = 180;
        break;
      case EMO_HAPPY:
        blink_speed = 100;
        break;
      case EMO_ANGRY:
        blink_speed = 90;
        break;
      default:
        blink_speed = 120;
        break;
    }
  }
  
  blink_eyes(blink_speed);
}

static void enhancedSetEmotion(Emotion emo, bool withVoice, bool immediate) {
  if (emo >= EMO_COUNT) return;
  
  if (current_emotion == emo && !immediate) return;
  
  prev_emotion = current_emotion;
  target_emotion = emo;
  
  if (immediate) {
    current_emotion = emo;
    emotion_blend = 1.0f;
  } else {
    emotion_blend = 0.0f;
  }
  
  // Immediate color change (no transition)
  lv_style_set_bg_color(&style_eye, COL_EYE[emo]);
  lv_style_set_shadow_color(&style_eye, COL_EYE[emo]);
  lv_obj_refresh_style(eyeL.eye, LV_PART_MAIN, LV_STYLE_PROP_ANY);
  lv_obj_refresh_style(eyeR.eye, LV_PART_MAIN, LV_STYLE_PROP_ANY);
  
  #if ENABLE_MOOD_MEMORY
  float intensity = 1.0f;
  updateMoodMemory(emo, intensity);
  #endif
  
  if (withVoice && !scheduled_sleep && config.voice_reactions) {
    emotionAudioNotification(emo);
  }
  
  setLipShapeForEmotion(emo);
  
  switch(emo) {
    case EMO_HAPPY:
      for(int i = 0; i < 3; i++) {
        int x = random(20, SCR_W - 20);
        int y = random(60, SCR_H - 60);
        tft.drawPixel(x, y, TFT_YELLOW);
      }
      break;
      
    case EMO_EXCITED:
      for(int i = 0; i < 5; i++) {
        lv_obj_set_x(eyeL.eye, lv_obj_get_x(eyeL.eye) + 1);
        lv_obj_set_x(eyeR.eye, lv_obj_get_x(eyeR.eye) + 1);
        delay(20);
        lv_obj_set_x(eyeL.eye, lv_obj_get_x(eyeL.eye) - 1);
        lv_obj_set_x(eyeR.eye, lv_obj_get_x(eyeR.eye) - 1);
        delay(20);
      }
      break;
  }
  
  switch (emo) {
    case EMO_ANGRY:   eyeL.setTarget(-8,0); eyeR.setTarget( 8,0); eyeL.setPupilSize(10,10); eyeR.setPupilSize(10,10); set_brow_line(eyeL,6);  set_brow_line(eyeR,-6); break;
    case EMO_HAPPY:   eyeL.setTarget(0,-6); eyeR.setTarget(0,-6); eyeL.setPupilSize(16,16); eyeR.setPupilSize(16,16); set_brow_line(eyeL,-4); set_brow_line(eyeR, 4); break;
    case EMO_SAD:     eyeL.setTarget(0, 8); eyeR.setTarget(0, 8); eyeL.setPupilSize(10,14); eyeR.setPupilSize(10,14); set_brow_line(eyeL,3);  set_brow_line(eyeR,-3); break;
    case EMO_SLEEPY:  eyeL.setTarget(0, 4); eyeR.setTarget(0, 4); eyeL.setPupilSize( 8, 8); eyeR.setPupilSize( 8, 8); set_brow_line(eyeL,1);  set_brow_line(eyeR,-1); break;
    case EMO_EXCITED: eyeL.setTarget(random(-8,8),random(-8,8)); eyeR.setTarget(random(-8,8),random(-8,8)); eyeL.setPupilSize(18,18); eyeR.setPupilSize(18,18); set_brow_line(eyeL,-5); set_brow_line(eyeR,5); break;
    case EMO_CURIOUS: eyeL.setTarget(4,-2); eyeR.setTarget(4,-2); eyeL.setPupilSize(14,14); eyeR.setPupilSize(14,14); set_brow_line(eyeL,4);  set_brow_line(eyeR, 4); break;
    default:          eyeL.setTarget(0,0);  eyeR.setTarget(0,0);  eyeL.setPupilSize(12,12); eyeR.setPupilSize(12,12); set_brow_line(eyeL,0);  set_brow_line(eyeR, 0); break;
  }
  
  scheduleNextBlink();
  last_activity_time = millis();
  emotion_stable_since = millis();
}

#if ENABLE_WIFI
static void initWiFiNonBlocking() {
  static uint32_t wifi_start_time = 0;
  static bool wifi_init_done = false;
  
  uint32_t now = millis();
  
  if (!wifi_init_done) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    
    if (config.wifi_ssid != "" && config.wifi_password != "") {
      wifiMulti.addAP(config.wifi_ssid.c_str(), config.wifi_password.c_str());
    }
    
    wifi_start_time = now;
    wifi_init_done = true;
    wifi_connecting = true;
    
    if (wifi_status_label) {
      lv_label_set_text(wifi_status_label, "WiFi: Connecting...");
      lv_obj_set_style_text_color(wifi_status_label, lv_color_hex(0xFFCC00), 0);
    }
    
    return;
  }
  
  if (wifi_connecting) {
    if (wifiMulti.run() == WL_CONNECTED) {
      wifi_connected = true;
      wifi_connecting = false;
      ap_mode = false;

      Serial.print("Connected to WiFi! IP address: ");
      Serial.println(WiFi.localIP());
      
      if (wifi_status_label) {
        lv_label_set_text(wifi_status_label, "WiFi: Connected");
        lv_obj_set_style_text_color(wifi_status_label, lv_color_hex(0x33FF99), 0);
      }
      
      initWebServer();
      initTime();
    }
    else if (now - wifi_start_time > WIFI_CONNECT_TIMEOUT) {
      wifi_connecting = false;
      
      #if ENABLE_WEB_SERVER
      startAPMode();
      #else
      if (wifi_status_label) {
        lv_label_set_text(wifi_status_label, "WiFi: Failed");
        lv_obj_set_style_text_color(wifi_status_label, lv_color_hex(0xFF3366), 0);
      }
      #endif
    }
  }
}

static void reconnectWiFiIfNeeded() {
  static uint32_t last_check = 0;
  uint32_t now = millis();
  
  if (now - last_check > WIFI_RETRY_INTERVAL) {
    last_check = now;
    
    if (WiFi.status() != WL_CONNECTED && !wifi_connecting && !ap_mode) {
      wifi_connected = false;
      wifi_connecting = true;
      wifi_last_attempt = now;
      
      if (wifi_status_label) {
        lv_label_set_text(wifi_status_label, "WiFi: Reconnecting...");
        lv_obj_set_style_text_color(wifi_status_label, lv_color_hex(0xFFCC00), 0);
      }
    }
  }
}
#endif

#if ENABLE_TIME_FEATURES
static void initTime() {
  if (!wifi_connected) return;
  
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, 
             NTP_SERVER1, NTP_SERVER2, NTP_SERVER3);
  
  sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
  sntp_set_time_sync_notification_cb([](struct timeval *tv) {
    time_synced = true;
    if (i2s_spk_init_success) {
      playTone(784, 100, 0.3f);
      playTone(988, 150, 0.3f);
    }
  });
  
  time_synced = false;
}
#endif

// ================== TIME FUNCTIONS ==================
#if ENABLE_TIME_FEATURES
static int getCurrentHour() {
  if (!time_synced) return -1;
  
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return -1;
  
  return timeinfo.tm_hour;
}

static int getCurrentMinute() {
  if (!time_synced) return -1;
  
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return -1;
  
  return timeinfo.tm_min;
}

static String getCurrentTimeString() {
  if (!time_synced) return "--:--:--";
  
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "Time Error";
  
  char timeStr[9];
  strftime(timeStr, sizeof(timeStr), "%H:%M:%S", &timeinfo);
  return String(timeStr);
}
#endif

static void updateTimeDisplay() {
  if (time_label && time_synced) {
    lv_label_set_text(time_label, getCurrentTimeString().c_str());
  }
}

static void checkSleepSchedule() {
  if (!time_synced) return;
  
  int current_hour = getCurrentHour();
  int current_minute = getCurrentMinute();
  
  if (current_hour == -1) return;
  
  bool should_sleep = (current_hour >= SLEEP_HOUR) || (current_hour < WAKE_HOUR);
  
  if (should_sleep && !scheduled_sleep) {
    scheduled_sleep = true;
    power_save_mode = true;
    tft.setBrightness(10);
    
    enhancedSetEmotion(EMO_SLEEPY, false);
    
    static bool played_sleep_tone = false;
    if (!played_sleep_tone && i2s_spk_init_success) {
      playTone(330, 500, 0.1f);
      played_sleep_tone = true;
    }
    
  } 
  else if (!should_sleep && scheduled_sleep) {
    scheduled_sleep = false;
    power_save_mode = false;
    tft.setBrightness(config.brightness);
    
    if (i2s_spk_init_success) {
      for(int i = 0; i < 3; i++) {
        playTone(440 + i*110, 150, 0.2f);
        delay(200);
      }
    }
    
    enhancedSetEmotion(EMO_NORMAL, true);
  }
}

static void saveSettings() {
  preferences.begin("face", false);
  preferences.putInt("blinkX", BLINK_OFS_X);
  preferences.putInt("blinkY", BLINK_OFS_Y);
  preferences.putInt("brightness", 255);
  
  preferences.putFloat("accelZeroX", accel_zero_x);
  preferences.putFloat("accelZeroY", accel_zero_y);
  preferences.putFloat("accelZeroZ", accel_zero_z);
  preferences.putBool("accelCalibrated", accel_calibrated);
  
  preferences.end();
}

static void loadSettings() {
  preferences.begin("face", true);
  BLINK_OFS_X = preferences.getInt("blinkX", 0);
  BLINK_OFS_Y = preferences.getInt("blinkY", 0);
  
  accel_zero_x = preferences.getFloat("accelZeroX", 0.0f);
  accel_zero_y = preferences.getFloat("accelZeroY", 0.0f);
  accel_zero_z = preferences.getFloat("accelZeroZ", -1.0f);
  accel_calibrated = preferences.getBool("accelCalibrated", false);
  
  preferences.end();
}

static void createTimeDisplay() {
  time_label = lv_label_create(lv_scr_act());
  lv_obj_add_style(time_label, &style_time_text, 0);
  lv_obj_set_width(time_label, SCR_W);
  lv_obj_set_pos(time_label, 0, 10);
  lv_label_set_text(time_label, "--:--:--");
  lv_obj_set_style_text_align(time_label, LV_TEXT_ALIGN_CENTER, 0);
  
  wifi_status_label = lv_label_create(lv_scr_act());
  lv_obj_add_style(wifi_status_label, &style_calib_text, 0);
  lv_obj_set_width(wifi_status_label, SCR_W);
  lv_obj_set_pos(wifi_status_label, 0, 30);
  lv_label_set_text(wifi_status_label, "WiFi: Off");
  lv_obj_set_style_text_color(wifi_status_label, lv_color_hex(0xFF3366), 0);
  lv_obj_set_style_text_align(wifi_status_label, LV_TEXT_ALIGN_CENTER, 0);
}

static void updateMoodMemory(Emotion e, float intensity) {
  if (mood_history_count < 24) {
    mood_history[mood_history_count] = {e, millis(), intensity};
    mood_history_count++;
  } else {
    for (int i = 0; i < 23; i++) {
      mood_history[i] = mood_history[i + 1];
    }
    mood_history[23] = {e, millis(), intensity};
  }
  
  preferences.begin("mood", false);
  preferences.putBytes("history", mood_history, sizeof(mood_history));
  preferences.putInt("count", mood_history_count);
  preferences.end();
}

static void loadMoodHistory() {
  preferences.begin("mood", true);
  preferences.getBytes("history", mood_history, sizeof(mood_history));
  mood_history_count = preferences.getInt("count", 0);
  preferences.end();
}

// Helper function to check device stability
static bool isDeviceStable(float ax, float ay, float az, float threshold = 0.1f) {
  static float prev_ax = 0, prev_ay = 0, prev_az = 0;
  static uint32_t stable_start = 0;
  
  float change = fabs(ax - prev_ax) + fabs(ay - prev_ay) + fabs(az - prev_az);
  
  prev_ax = ax;
  prev_ay = ay;
  prev_az = az;
  
  if (change < threshold) {
    if (stable_start == 0) {
      stable_start = millis();
    }
    // Device has been stable for 1 second
    return (millis() - stable_start) > 1000;
  } else {
    stable_start = 0;
    return false;
  }
}

// UPDATED CALIBRATION FOR NEW ACCELEROMETER ORIENTATION
static void calibrateAccelerometerZero() {
  if (!MMA8452Q::present) {
    Serial.println("Accelerometer not found!");
    return;
  }
  
  Serial.println("Calibrating accelerometer... Keep device flat on table facing forward.");
  Serial.println("Expected readings when flat: X=0, Y=0, Z=-1.0g");
  
  float sum_x = 0, sum_y = 0, sum_z = 0;
  const int samples = 200;
  
  for (int i = 0; i < samples; i++) {
    float ax, ay, az;
    MMA8452Q::read(ax, ay, az);
    sum_x += ax;
    sum_y += ay;
    sum_z += az;
    delay(10);
  }
  
  // Calculate averages
  accel_zero_x = sum_x / samples; // Should be ~0 when flat
  accel_zero_y = sum_y / samples; // Should be ~0 when flat
  accel_zero_z = (sum_z / samples) + 1.0f; // Should be ~0 when flat (since actual reading is -1.0g)
  
  accel_calibrated = true;
  saveSettings();
  
  Serial.println("Calibration complete!");
  Serial.printf("Raw averages: X=%.3f, Y=%.3f, Z=%.3f\n", 
                sum_x/samples, sum_y/samples, sum_z/samples);
  Serial.printf("Zero offsets: X=%.3f, Y=%.3f, Z=%.3f\n", 
                accel_zero_x, accel_zero_y, accel_zero_z);
  Serial.println("Note: Z offset includes +1.0g correction for gravity");
  
  // Visual confirmation
  enhancedSetEmotion(EMO_HAPPY, false);
  for(int i = 0; i < 3; i++) {
    playTone(880 + i*110, 100, 0.3f);
    delay(150);
  }
}

static void calibrateBlinkOffset() {
  Serial.println("Blink offset calibration mode");
  Serial.println("Adjusting blink offset...");
  
  // Test different offsets
  for(int x = -20; x <= 0; x += 5) {
    for(int y = -20; y <= 0; y += 5) {
      BLINK_OFS_X = x;
      BLINK_OFS_Y = y;
      Serial.printf("Testing X=%d, Y=%d\n", x, y);
      blink_eyes(100);
      delay(1000);
    }
  }
  
  // Reset to default
  BLINK_OFS_X = 0;
  BLINK_OFS_Y = 0;
  
  saveSettings();
  Serial.println("Blink offset calibration saved");
}

static void autoCalibrateBlink() {
  Serial.println("Auto calibrating blink offset...");
  
  // Simple auto-calibration - use defaults
  BLINK_OFS_X = 0;
  BLINK_OFS_Y = 0;
  
  saveSettings();
  Serial.println("Auto calibration complete - using default offsets");
  
  // Visual confirmation
  enhancedSetEmotion(EMO_HAPPY, false);
  playTone(523, 200, 0.3f);
  delay(100);
  playTone(659, 200, 0.3f);
}

#if ENABLE_TTS
static void updateTTS() {
  if (!tts_speaking || !current_tts_text) return;
  
  uint32_t now = millis();
  if (now < tts_next_char_time) return;
  
  if (tts_text_pos >= strlen(current_tts_text)) {
    tts_speaking = false;
    current_tts_text = nullptr;
    tts_text_pos = 0;
    return;
  }
  
  char c = current_tts_text[tts_text_pos];
  if (c != ' ') {
    float freq = 300 + (c % 26) * 50;
    playTone(freq, 50, 0.2f);
  }
  
  tts_text_pos++;
  tts_next_char_time = now + 150;
}
#endif

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Emo Face Pro v2.1...");
  Serial.println("Accelerometer orientation: X=Top/Bottom, Y=Left/Right, Z=Forward/Backward");
  
  loadSettings();
  loadConfig();
  
  tft.init(); 
  tft.setRotation(0); 
  tft.setBrightness(config.brightness); 
  tft.fillScreen(TFT_BLACK);
  
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(SCR_W/2 - 60, SCR_H/2 - 20);
  tft.print("Starting...");
  
  delay(300);
  tft.fillScreen(TFT_BLACK);

  lv_init();
  size_t buf_size = SCR_W * DRAW_BUF_LINES;
  
  lvBuf1 = (lv_color_t*)heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_DMA);
  lvBuf2 = (lv_color_t*)heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_DMA);
  
  if (!lvBuf1 || !lvBuf2) {
    lvBuf1 = (lv_color_t*)malloc(buf_size * sizeof(lv_color_t));
    lvBuf2 = (lv_color_t*)malloc(buf_size * sizeof(lv_color_t));
    if (!lvBuf1 || !lvBuf2) {
      Serial.println("Failed to allocate LVGL buffers!");
      while(1) delay(1000);
    }
  }
  
  lv_disp_draw_buf_init(&draw_buf, lvBuf1, lvBuf2, buf_size);
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCR_W; 
  disp_drv.ver_res = SCR_H; 
  disp_drv.flush_cb = lvgl_flush_cb; 
  disp_drv.draw_buf = &draw_buf; 
  disp_drv.antialiasing = 1;
  lv_disp_drv_register(&disp_drv);
  
  create_styles();
  lv_obj_set_style_bg_color(lv_scr_act(), COL_BG, 0);

  createTimeDisplay();

  const int leftX  = (SCR_W - (2*EYE_W + 18)) / 2;
  const int rightX = leftX + EYE_W + 18;
  const int eyeY = 108 + 20;
  
  create_eye(eyeL, leftX, eyeY);
  create_eye(eyeR, rightX, eyeY);

  initLipAnimation();
  initAlarmSystem();
  initParticleSystem();

  stars_init();
  
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!MMA8452Q::init()) {
    Serial.println("MMA8452Q not found!");
  } else {
    Serial.println("MMA8452Q initialized");
  }
  
  i2s_mic_init();
  i2s_spk_init();
  
  for(int i = 0; i < 8; i++) {
    beat_history[i] = 0;
  }
  
  #if ENABLE_MOOD_MEMORY
  loadMoodHistory();
  #endif
  
  if (MMA8452Q::present && !accel_calibrated) {
    Serial.println("Accelerometer needs calibration");
  }

  if (i2s_spk_init_success) {
    // Play startup melody
    float startup_notes[] = {523.3f, 659.3f, 784.0f, 987.8f, 1174.7f};
    int startup_durations[] = {150, 150, 150, 150, 300};
    playMelody(startup_notes, startup_durations, 5, 0.3f);
  }

  enhancedSetEmotion(EMO_NORMAL, false, true);
  setLipShapeForEmotion(EMO_NORMAL);
  scheduleNextBlink();
  last_activity_time = millis();
  
  lvgl_initialized = true;
  
  #if ENABLE_WIFI
  wifi_last_attempt = millis();
  #endif
  
  Serial.println("======================================");
  Serial.println("Emo Face Pro v2.1 Ready!");
  Serial.println("Features: Improved Audio Notifications, Continuous Alarms");
  Serial.println("Accelerometer: X=Top/Bottom, Y=Left/Right, Z=Forward/Backward");
  Serial.println("Serial Commands:");
  Serial.println("  0-6: Change emotion");
  Serial.println("  b: Blink");
  Serial.println("  c: Calibrate accelerometer");
  Serial.println("  x: Reset calibration");
  Serial.println("  a: Auto-calibrate blink");
  Serial.println("  p: Toggle power save");
  Serial.println("  w: Toggle WiFi");
  Serial.println("  r: Reconnect WiFi");
  Serial.println("  n: Normal emotion");
  Serial.println("  o: Center pupils");
  Serial.println("  s: Start AP mode");
  Serial.println("  A: Test alarm (rings for 30 seconds)");
  Serial.println("  S: Stop alarm");
  Serial.println("  Z: Snooze alarm");
  Serial.println("  L: Toggle lip sync");
  Serial.println("  P: Spawn particles");
  Serial.println("======================================");
}

void loop() {
  static uint32_t lastLvglUpdate = 0;
  static uint32_t lastTimeUpdate = 0;
  static uint32_t lastScheduleCheck = 0;
  static uint32_t lastEmotionCheck = 0;
  static uint32_t lastAccelUpdate = 0;
  static uint32_t lastAlarmCheck = 0;
  static Emotion detected_emotion = EMO_NORMAL;
  uint32_t now = millis();
  
  if (!lvgl_initialized) {
    delay(100);
    return;
  }

  if (now - lastLvglUpdate >= LVGL_UPDATE_MS) {
    lv_timer_handler();
    lastLvglUpdate = now;
    
    updateLipTransition();
    updateParticles(current_emotion);
  }

  #if ENABLE_WEB_SERVER
  handleWebServer();
  #endif

  #if ENABLE_WIFI
  if (!wifi_connected && !web_server_active && !ap_mode) {
    initWiFiNonBlocking();
  }
  
  if (wifi_connected) {
    reconnectWiFiIfNeeded();
  }
  #endif

  if (now - lastTimeUpdate >= 1000) {
    lastTimeUpdate = now;
    updateTimeDisplay();
    
    if (now - lastAlarmCheck >= 60000) {
      lastAlarmCheck = now;
      checkAlarms();
    }
  }
  
  if (now - lastScheduleCheck >= 60000) {
    lastScheduleCheck = now;
    checkSleepSchedule();
  }

  // Update alarm ringing continuously
  if (alarm_triggered) {
    updateAlarmRinging();
  }

  #if ENABLE_TTS
  if (tts_speaking) {
    updateTTS();
  }
  #endif

  if (!calibration_mode) {
    if (now - lastAccelUpdate >= 10) {
      lastAccelUpdate = now;
      
      if (MMA8452Q::present) {
        float ax, ay, az;
        MMA8452Q::read(ax, ay, az);
        
        // Apply calibration offsets
        if (accel_calibrated) {
          ax -= accel_zero_x;
          ay -= accel_zero_y;
          az -= accel_zero_z;
        }
        
        // UPDATED PUPIL MOVEMENT FOR NEW ORIENTATION
        // X axis (top/bottom) controls vertical pupil movement
        // Y axis (left/right) controls horizontal pupil movement
        int dx = (int)(-ay * 12.0f);  // Use Y axis for horizontal movement
        int dy = (int)(-ax * 10.0f);  // Use X axis for vertical movement (inverted)
        
        if (abs(dx) < 2) dx = 0;
        if (abs(dy) < 2) dy = 0;
        
        dx = constrain(dx, -20, 20);
        dy = constrain(dy, -15, 15);
        
        eyeL.setTarget(dx, dy);
        eyeR.setTarget(dx, dy);
        
        // Z axis (forward/backward) controls pupil size
        // When leaning forward (Z positive), pupils get smaller
        // When leaning backward (Z negative), pupils get larger
        int pupil_adjust = (int)((az + 1.0f) * 6.0f);  // Z ranges from -1 to +1
        pupil_adjust = constrain(pupil_adjust, -6, 6);
        
        int base_size = 12;
        int new_size = base_size + pupil_adjust;
        new_size = constrain(new_size, 8, 20);
        eyeL.setPupilSize(new_size, new_size);
        eyeR.setPupilSize(new_size, new_size);

        #if ENABLE_GESTURES
        static uint32_t last_gesture_check = 0;
        if (now - last_gesture_check > 100) {
          last_gesture_check = now;
          Gesture g = detectGesture(ax, ay, az);
          if (g != GESTURE_NONE && g != last_gesture) {
            handleGesture(g);
            last_gesture = g;
          }
        }
        #endif

        // =========== IMPROVED EMOTION DETECTION ===========
        if (config.auto_emotion && now - lastEmotionCheck > 1000) {  // Check every 1 second
          lastEmotionCheck = now;
          
          // Check device orientation for emotion detection
          bool is_stable = isDeviceStable(ax, ay, az, 0.08f);
          
          Emotion e = detected_emotion; // Start with current emotion
          
          if (is_stable) {
            // Device is stable - detect emotion based on orientation
            if (az > 0.7f) {  // Leaning forward
              e = EMO_CURIOUS;
            } 
            else if (az < -0.7f) {  // Leaning backward
              e = EMO_SAD;
            }
            else if (ay > 0.6f) {  // Tilted left (positive Y)
              e = EMO_ANGRY;
            } 
            else if (ay < -0.6f) {  // Tilted right (negative Y)
              e = EMO_HAPPY;
            }
            else if (ax > 0.6f) {  // Top down (positive X)
              e = EMO_EXCITED;
            }
            else if (ax < -0.6f) {  // Bottom up (negative X)
              e = EMO_SLEEPY;
            }
            else {
              // Device is flat and stable
              e = EMO_NORMAL;
            }
          }
          else {
            // Device is moving/shaking
            float movement_magnitude = sqrtf(ax*ax + ay*ay + az*az);
            if (movement_magnitude > 1.2f) {
              e = EMO_EXCITED;
            }
          }
          
          // Only update if emotion changed
          if (e != detected_emotion) {
            detected_emotion = e;
            enhancedSetEmotion(e);
            
            // Play sound for emotion changes (except when returning to normal quietly)
            if (e != EMO_NORMAL || !is_stable) {
              emotionAudioNotification(e);
            }
            
            setLipShapeForEmotion(e);
            last_activity_time = now;
          }
        }
        // =================================================
      }
    }

    #if ENABLE_MUSIC_BEAT_DETECTION
    static uint32_t last_beat_update = 0;
    if (now - last_beat_update > 40) {
      last_beat_update = now;
      updateBeatDetection();
      
      updateLipAnimation(current_loudness, beat_detected);
    }
    #endif

    eyeL.updatePhysics();
    eyeR.updatePhysics();
    
    if (emotion_blend < 1.0f) {
      emotion_blend += emotion_transition_speed;
      if (emotion_blend > 1.0f) emotion_blend = 1.0f;
    }

    if ((int32_t)(now - nextBlinkAt) >= 0 && !eyeL.is_blinking) {
      enhancedBlink();
      scheduleNextBlink();
    }

    stars_draw();
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c=='\n' || c=='\r') continue;
    
    last_activity_time = now;
    
    switch(c) {
      case '0': case '1': case '2': case '3': case '4': case '5': case '6': {
        Emotion e = (Emotion)(c - '0');
        enhancedSetEmotion(e);
        emotionAudioNotification(e);
        Serial.print("Emotion set to: ");
        Serial.println(e);
      } break;
      
      case 'b': 
        enhancedBlink(); 
        Serial.println("Blink triggered");
        break;
        
      case 't': 
        starlight_enabled = !starlight_enabled; 
        Serial.print("Starlight: ");
        Serial.println(starlight_enabled ? "ON" : "OFF");
        break;
        
      case 'c': 
        if (MMA8452Q::present) {
          calibrateAccelerometerZero();
          Serial.println("Accelerometer calibrated");
        } else {
          Serial.println("Accelerometer not present!");
        }
        break;
        
      case 'a':
        autoCalibrateBlink();
        Serial.println("Blink auto-calibrated");
        break;
        
      case 'x':
        accel_zero_x = 0;
        accel_zero_y = 0;
        accel_zero_z = -1.0f;
        accel_calibrated = false;
        saveSettings();
        set_emotion(EMO_NORMAL);
        Serial.println("Calibration reset");
        break;
        
      case 'p': 
        if (!scheduled_sleep) {
          power_save_mode = !power_save_mode;
          tft.setBrightness(power_save_mode ? 30 : config.brightness);
          Serial.print("Power save: ");
          Serial.println(power_save_mode ? "ON" : "OFF");
        }
        break;
        
      case 'w':
        #if ENABLE_WIFI
        if (WiFi.getMode() == WIFI_MODE_STA) {
          WiFi.mode(WIFI_MODE_NULL);
          wifi_connected = false;
          time_synced = false;
          if (wifi_status_label) {
            lv_label_set_text(wifi_status_label, "WiFi: OFF");
            lv_obj_set_style_text_color(wifi_status_label, lv_color_hex(0xFF3366), 0);
          }
          Serial.println("WiFi disabled");
        } else {
          WiFi.mode(WIFI_STA);
          wifi_connected = false;
          wifi_connecting = true;
          wifi_last_attempt = millis();
          Serial.println("WiFi enabled - connecting...");
        }
        #endif
        break;
        
      case 'r':
        #if ENABLE_WIFI
        WiFi.disconnect();
        delay(100);
        WiFi.mode(WIFI_STA);
        wifi_connected = false;
        wifi_connecting = true;
        wifi_last_attempt = millis();
        Serial.println("WiFi reconnecting...");
        #endif
        break;
        
      case 'n':
        enhancedSetEmotion(EMO_NORMAL);
        Serial.println("Normal emotion");
        break;
        
      case 'v':
        voice_command_mode = true;
        voice_command_start = millis();
        Serial.println("Voice command mode");
        break;
        
      case 'o':
        eyeL.resetToCenter();
        eyeR.resetToCenter();
        Serial.println("Pupils centered");
        break;
        
      case 's':
        #if ENABLE_WEB_SERVER
        if (!web_server_active) {
          startAPMode();
          Serial.println("AP mode started");
        }
        #endif
        break;
        
      case 'A':
        triggerAlarm(0);
        Serial.println("Test alarm triggered - will ring for 30 seconds");
        break;
        
      case 'S':
        stopAlarm();
        Serial.println("Alarm stopped");
        break;
        
      case 'Z':
        snoozeAlarm();
        Serial.println("Alarm snoozed for 5 minutes");
        break;
        
      case 'L':
        config.lip_sync_enabled = !config.lip_sync_enabled;
        Serial.print("Lip sync: ");
        Serial.println(config.lip_sync_enabled ? "ON" : "OFF");
        break;
        
      case 'P':
        spawnParticles(SCR_W/2, SCR_H/2, 10, COL_EYE[current_emotion]);
        Serial.println("Particles spawned");
        break;
        
      default:
        Serial.print("Unknown command: ");
        Serial.println(c);
        break;
    }
  }

  delay(1);
}