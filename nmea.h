#ifndef __NMEA_H__
#define __NMEA_H__
#ifdef __cplusplus
extern "C"
{
#endif

  // clang-format off
  // clang-format on

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

#ifndef CLOG
#define log(...)
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
#define SEPERATE_BD 1

  typedef enum
  {
    EN_RMC = 0,
    EN_VTG,
    EN_GGA,
    EN_GSA,
    EN_GSV,
    EN_GLL
  } NMEA_TYPE_EN;

  typedef enum
  {
    EN_GS_A = 0,
    EN_GS_V,
    EN_GS_INV
  } GPS_STATE_EN;

  typedef enum
  {
    EN_EAST = 0,
    EN_SOUTH,
    EN_WEST,
    EN_NORTH,
    EN_INV
  } ORIENTATION_EN;

  // 定位类型
  typedef enum
  {

    EN_GM_A,
    EN_GM_D,
    EN_GM_E,
    EN_GM_N,

    EN_GM_INV
  } GPS_MODE_EN;

  typedef struct
  {
    int index;
    int count;
    double lat[5];
    double lng[5];
    double lat_average;
    double lng_average;
  } gps_average_struct;

  typedef struct
  {
    U16 sat_no;
    U8 snr;
  } snr_node_struct;

  typedef struct
  {
    U8 datetime[MAX_DATETIME_LEN];
    U8 state;
    U32 latitude;
    U8 lat_ind;
    char lat_str[16];
    char lat_ind_str[2];
    U32 longitude;
    U8 long_ind;
    char lng_str[16];
    char lng_ind_str[2];
    U16 speed;
    U16 course;
    U16 magnetic_value;
    U8 magnetic_ind;
    U8 mode;

    U8 sat_uesed;
    U16 msl_altitude;
    U16 hdop;

    U16 snr_index;
    snr_node_struct snr_list[MAX_SNR_NUM];

    gps_average_struct gps_average;
  } nmea_parsed_struct;

  extern nmea_parsed_struct gps;

  int get_gps_info(char *buf);

#ifdef __cplusplus
}
#endif

#endif
