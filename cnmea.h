#ifndef __NMEA_H__
#define __NMEA_H__
#ifdef __cplusplus
extern "C" {
#endif

// clang-format off
// clang-format on
#define CNMEA_ENABLE_LOG 1

#if CNMEA_ENABLE_LOG
#ifdef CLOG
#include "clog.h"
#else
#define log(level, fmt, ...) printf(fmt, ##__VA_ARGS__)
#endif
#else
#define log(level, fmt, ...)
#endif

#ifndef ERROR
#define ERROR 1
#endif
#ifndef SUCCESS
#define SUCCESS 0
#endif
#ifndef BOOL
#define BOOL int
#endif
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#ifndef NULL
#define NULL 0
#endif

#ifndef U8
typedef unsigned char U8;
#endif
#ifndef S8
typedef signed char S8;
#endif
#ifndef U16
typedef unsigned short U16;
#endif
#ifndef S16
typedef signed short S16;
#endif
#ifndef U32
typedef unsigned int U32;
#endif
#ifndef S32
typedef signed int S32;
#endif

#define MAX_DATETIME_LEN 6
#define MAX_NMEA_LINE_LEN 256
#define MAX_SNR_NUM 180

// gps模拟包的开关，为1时，使用代码中的模拟包
#define GPS_SIMU_SWITCH 0

typedef enum { EN_RMC = 0, EN_VTG, EN_GGA, EN_GSA, EN_GSV, EN_GLL } NMEA_TYPE_EN;

typedef enum { EN_GS_A = 0, EN_GS_V, EN_GS_INV } GPS_STATE_EN;

typedef enum { EN_EAST = 0, EN_SOUTH, EN_WEST, EN_NORTH, EN_INV } ORIENTATION_EN;

// 定位类型
typedef enum {

  EN_GM_A,
  EN_GM_D,
  EN_GM_E,
  EN_GM_N,

  EN_GM_INV
} GPS_MODE_EN;

typedef struct {
  int index;
  int count;
  double lat[5];
  double lng[5];
  double lat_average;
  double lng_average;
} gps_average_struct;

typedef struct {
  U16 sat_no;
  U8 snr;
} snr_node_struct;

typedef struct {
  U8 datetime[MAX_DATETIME_LEN];
  U8 state;

  // 方便准确值表示，结合ind值使用
  U32 latitude;
  U8 lat_ind;
  U32 longitude;
  U8 long_ind;

  // 字符串传输过程避免多次转换
  char lat_str[16];
  char lat_ind_str[2];
  char lng_str[16];
  char lng_ind_str[2];

  U16 speed;

  U16 course;
  U16 magnetic_value;
  U8 magnetic_ind;
  U8 mode;

  // 方便存储，但是会引入转换误差
  float f_latitude;
  float f_longitude;
  float f_altitude;
  float f_speed;

  U8 sat_uesed;
  U16 msl_altitude;
  U16 hdop;

  U16 snr_index;
  snr_node_struct snr_list[MAX_SNR_NUM];

  gps_average_struct gps_average;
} nmea_parsed_struct;

extern nmea_parsed_struct gps;

int get_gps_info(char *buf, U16 len);

#ifdef __cplusplus
}
#endif

#endif
