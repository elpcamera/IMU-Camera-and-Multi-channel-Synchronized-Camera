#ifndef SYNC_SDK_H
#define SYNC_SDK_H

#include <stdint.h>
#include <stdbool.h>

#if defined(_MSC_VER)
#  ifdef SYNC_LIB_LIB
#    define SYNC_API __declspec(dllexport)
#  else
#    define SYNC_API __declspec(dllimport)
#  endif
#elif defined(__GNUC__) || defined(__clang__)
#  define SYNC_API __attribute__((visibility("default")))
#else
#  define SYNC_API
#endif

#ifdef __cplusplus
extern "C" {
#endif
/* ==================== Data structure definition ==================== */
#pragma pack(push, 8)

/* Maximum number of cameras supported by SDK. Current logical device index rangeis 0 ~ 5.  */
#define SYNC_MAX_CAMERA_COUNT 20

/* Max per-frame ICM42688 IMU sample points.  */
#define SYNC_IMU_SAMPLE_COUNT 11

/* Max per-frame AK09940 Magnetometer sample points.  */
#define SYNC_MTT_SAMPLE_COUNT 5

/*
 * Camera raw image left hidden encoding area width, unit: pixels.
 * SDK internally parses exposure time and ICM42688 IMU data from this area;
 * The image provided to the callback has this area cropped.
 */
#define SYNC_EFFECT_AREA_WIDTH 160

/*
 * Image format enum. 
 * Currently, SDK internally supports recognition of MJPG and YUY2 input formats. 
 * MJPG input is decoded to NV12 via GStreamer mppjpegdec;
 * External callback output format fixed toCropped NV12. 
 */
typedef enum SyncCamFormat {
    /* Unknown or unsupported image format.  */
    SYNC_FORMAT_UNKNOWN = 0,

    /* MJPG (Motion JPEG) compression format.  */
    SYNC_FORMAT_MJPG = 1,

    /* YUY2 / YUV422 packed format.  */
    SYNC_FORMAT_YUY2 = 2,

    /* NV12 format: Y plane + interleaved UV plane.  */
    SYNC_FORMAT_NV12 = 3
} SyncCamFormat;



/*
 * Device information. 
 * Enumerated and returned to caller by SYNC_EnumDevices(). 
 */


typedef struct SyncDeviceInfo {
    /* Device name, e.g. Camera0001: Camera0001.  */
    char name[256];

    /* Device VID/PID string, e.g. 32e40001.  */
    char vidpid[16];

    /* Device path, e.g. /dev/video0.  */
    char path[256];

    /* Logical device index, e.g. Camera0001 => 0, Camera0002 => 1.  */
    uint32_t number;

    /* Whether the device currently exists.  */
    bool isExist;

    /* Whether the device is currently opened.  */
    bool isOpened;
} SyncDeviceInfo;

/*
 * ICM42688 floating-point IMU data. 
 * One element represents one IMU sample point. 
 */
typedef struct SyncICM42688Data {
    /* Sampling timestamp.  */
    uint64_t uTime;

    /* X-axis acceleration.  */
    float fAccData_X;

    /* Y-axis acceleration.  */
    float fAccData_Y;

    /* Z-axis acceleration.  */
    float fAccData_Z;

    /* X-axis gyroscope angular velocity.  */
    float fGyroData_X;

    /* Y-axis gyroscope angular velocity.  */
    float fGyroData_Y;

    /* Z-axis gyroscope angular velocity.  */
    float fGyroData_Z;
} SyncICM42688Data;

/*
 * AK09940 magnetometer data. 
 * One element represents one magnetometer sample point. 
 */
typedef struct SyncAK09940Data {
    /* Sampling timestamp.  */
    uint64_t uTime;

    /* X-axis raw magnetic field data.  */
    int32_t iX;

    /* Y-axis raw magnetic field data.  */
    int32_t iY;

    /* Z-axis raw magnetic field data.  */
    int32_t iZ;

    /* Sensor temperature.  */
    float Temp;

    /* Status bits.  */
    int32_t iStatusBit;
} SyncAK09940Data;

/*
 * Image format description supported by the device. 
 * Returned by SYNC_GetDeviceFormats(). 
 */
typedef struct SyncFormatInfo {
    /* Image format type.  */
    SyncCamFormat fmt;

    /* Image width.  */
    int width;

    /* Image height.  */
    int height;

    /* Image frame rate.  */
    int fps;
} SyncFormatInfo;

/*
 * Single camera output data.
 *
 * data is always NV12 with left hidden area cropped:
 * - width = original image width - SYNC_EFFECT_AREA_WIDTH
 * - height = original image height
 * - Y plane size: width * height
 * - UV plane size: width * height / 2
 * - Total size: width * height * 3 / 2
 *
 * The SDK first parses the exposure start/end timestamps and
 * ICM42688 IMU data from the hidden encoding area, then crops the hidden area and calls back by synced group. 
 *
 * The data pointer inside the callback is only valid before the callback returns;
 * If asynchronous usage is required, please copy it yourself.
 */
typedef struct SyncCamData {
    /* NV12 pixel data first address.  */
    const uint8_t* data;

    /* Image width.  */
    int width;

    /* Image height.  */
    int height;

    /* Total image bytes.  */
    int bufSize;

    /* Current image format, normally SYNC_FORMAT_NV12.  */
    SyncCamFormat format;

    /* Exposure start timestamp.  */
    uint64_t startExpouseTime;

    /* Exposure end timestamp.  */
    uint64_t endExpouseTime;

    /* IMU data array attached to the current frame.  */
    SyncICM42688Data imu_data[SYNC_IMU_SAMPLE_COUNT];

    /* Magnetometer data array attached to the current frame.  */
    SyncAK09940Data mtt_data[SYNC_MTT_SAMPLE_COUNT];

    /* SDK logical device index: Camera0001 => 0, Camera0002 => 1 ... */
    int deviceIdx;

    /* Raw device index in the underlying capture library.  */
    uint32_t rawIndex;

    /* Is current frame valid.  */
    bool isValid;
} SyncCamData;

/*
 * Multi-channel sync image group. 
 * One callback returns one synced group, which can contain 1 to SYNC_MAX_CAMERA_COUNT images. 
 * SDK matches synchronization based on the exposure timestamps in the hidden encoding area of each image. 
 */
typedef struct SyncCamGroupData {
    /* Image data of each camera in the current group.  */
    SyncCamData camData[SYNC_MAX_CAMERA_COUNT];

    /* Actual device count in current group.  */
    int deviceCount;
} SyncCamGroupData;

#pragma pack(pop)

/*
 * Image callback function type. 
 *
 * image: 
 *   Pointer to a group of synced image data, containing multiple SyncCamData. 
 *
 * userData: 
 *   Custom parameter passed when registering callback, will be passed back as is.
 */
typedef void (*ImageCallback)(const SyncCamGroupData* image, void* userData);

/* ==================== SDK interface ==================== */
SYNC_API int SYNC_Initialize(void);
SYNC_API void SYNC_Release(void);
SYNC_API const char* SYNC_GetVersion(void);
SYNC_API int SYNC_GetLastError(void);
SYNC_API const char* SYNC_GetErrorText(int errorCode);

SYNC_API int SYNC_EnumDevices(SyncDeviceInfo* devices, int maxCount, int* count);
SYNC_API int SYNC_OpenDevice(uint32_t deviceIndex);
SYNC_API int SYNC_CloseDevice(uint32_t deviceIndex);
SYNC_API int SYNC_GetDeviceFormats(uint32_t deviceIndex, SyncFormatInfo* formats, int maxCount, int* count);
SYNC_API int SYNC_SetDeviceFormat(uint32_t deviceIndex, int formatIndex);
SYNC_API int SYNC_GetDeviceFormatIndex(uint32_t deviceIndex);

SYNC_API int SYNC_SetImageCallback(ImageCallback callback, void* userData);
SYNC_API void SYNC_SetCaptureFormat(SyncCamFormat format);

/* Open by logical device ID: defaults to opening the first deviceCount existing devices starting from 0, and call back by synced group.  */
SYNC_API int SYNC_StartCapture(int deviceCount);

/* Open by logical device ID array, e.g. {0, 2, 4}, and call back by synced group.  */
SYNC_API int SYNC_StartCaptureByIndices(const int* deviceIndices, int deviceCount);

SYNC_API int SYNC_StopCapture(void);
SYNC_API bool SYNC_IsCapturing(void);

SYNC_API bool SYNC_SaveNV12ToJPG(const uint8_t* nv12_buf, int w, int h, const char* filePath, int quality);
SYNC_API float SYNC_GetFrameRate(uint32_t deviceIndex);

#ifdef __cplusplus
}
#endif


#endif // SYNC_SDK_H
