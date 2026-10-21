#ifndef SYNC_SDK_H
#define SYNC_SDK_H
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#if defined(_MSC_VER)
#  ifdef SYNC_LIB_LIB
#    define SYNC_API __declspec(dllexport)
#  else
#    define SYNC_API __declspec(dllimport)
#  endif
#else
#  define SYNC_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 数据结构定义 ==================== */
#pragma pack(push,8)
    enum CamFormat {
        FORMAT_UNKNOWN = 0,
        FORMAT_MJPG = 1,
        FORMAT_RGB24 = 2,
        FORMAT_YUV422 = 3,
        FORMAT_NV12 = 4
    };

typedef struct DeviceInfo {
    char name[256];
    char vidpid[16];
    int devNumber;
    char path[256];
    DeviceInfo()
    {
        memset(name, 0, 256);
        memset(vidpid, 0, 256);
        memset(path, 0, 256);
        devNumber = -1;
    }
}DeviceInfo;
typedef struct {
    uint64_t  uTime;
    float     fAccData_X;
    float     fAccData_Y;
    float     fAccData_Z;
    float     fGyroData_X;
    float     fGyroData_Y;
    float     fGyroData_Z;
}sICM42688_XYZ_float;
typedef struct sAK09940_XYZ_int {
    uint64_t	uTime;
    int32_t		iX;
    int32_t		iY;
    int32_t		iZ;
    float		Temp;
    int32_t		iStatusBit;
}sAK09940_XYZ_int;
typedef struct FormatInfo {
    CamFormat fmt;
    int width;
    int height;
    int fps;
    FormatInfo()
    {
        fmt = FORMAT_UNKNOWN;
        width = 0;
        height = 0;
        fps = 0;
    }
}FormatInfo;
typedef struct CamData {
    uint8_t* data;
    int width;
    int height;
    int bufSize;
    CamFormat format;
    uint64_t startExpouseTime;
    uint64_t endExpouseTime;
    sICM42688_XYZ_float imu_data[11];
    sAK09940_XYZ_int mtt_data[5];
    uint32_t deviceIdx;
    bool isValid;

}CamData;

typedef struct CamGroupData {
    struct CamData camData[20];
    int  deviceCount;
}CamGroupData;
#pragma pack(pop)
typedef void (*ImageCallback)(const CamGroupData* image, void* userData);

/* ==================== SDK 接口 ==================== */
SYNC_API int SYNC_Initialize();
SYNC_API void SYNC_Release();
SYNC_API const char* SYNC_GetVersion();
SYNC_API const int SYNC_GetLastError();
SYNC_API const char* SYNC_GetErrorText(int errorCode);

SYNC_API int SYNC_EnumDevices(DeviceInfo* devices, int maxCount, int* count);
SYNC_API int SYNC_OpenDevice(uint32_t deviceIndex);
SYNC_API int SYNC_CloseDevice(uint32_t deviceIndex);
SYNC_API bool SYNC_IsOpening(uint32_t deviceIndex);
SYNC_API int SYNC_GetDeviceFormats(uint32_t deviceIndex, FormatInfo* formats, int maxCount, int* count);
SYNC_API int SYNC_SetDeviceFormat(uint32_t deviceIndex, int formatIndex);
SYNC_API int SYNC_GetDeviceFormatIndex(uint32_t deviceIndex);

SYNC_API int SYNC_SetImageCallback(ImageCallback callback, void* userData);
SYNC_API void SYNC_SetCaptureFormat(CamFormat format);
SYNC_API int SYNC_StartCapture(int deviceCount);
SYNC_API int SYNC_StopCapture();
SYNC_API bool SYNC_IsCapturing();
SYNC_API bool SYNC_SaveToBMP(const uint8_t* buf, CamFormat format,int w, int h, const char* filename);
SYNC_API bool SYNC_SaveToJPG(const uint8_t* buf, CamFormat format,int w, int h, const char* filename, int quality = 85);
SYNC_API float SYNC_GetFrameRate();

#ifdef __cplusplus
}
#endif

#endif // SYNC_SDK_H

