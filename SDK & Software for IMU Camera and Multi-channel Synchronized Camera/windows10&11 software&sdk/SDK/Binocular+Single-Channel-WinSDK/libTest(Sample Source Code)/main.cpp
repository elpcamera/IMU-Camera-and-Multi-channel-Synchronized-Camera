#include <cstdint>
#include <cstring>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <cstdio>
#include <deque>
#include <string>
#include <condition_variable>
#include <windows.h>
#include <cinttypes>
#include <algorithm>
#include "synclib.h"
#include <conio.h>
#include <ctime>
#include <cerrno>
#include <direct.h>
#include <map>

//Global constant configuration
constexpr uint64_t kSyncThresholdUs = 100;
constexpr size_t   kFrameQueueMax = 20;
constexpr int      kJpgQuality = 85;
constexpr int      kCaptureTimeoutMs = 8000;
constexpr int      kMaxDevices = 8;


//Global variables
int sync_method = 0;                                    //Sync method flag
std::mutex              g_queueMtx[kMaxDevices];        //Queue mutex
std::condition_variable g_syncCv;                       //Condition variable
std::mutex mtx;                                         //Global variable mutex
std::deque<CamData> g_frameQueues[kMaxDevices];         //Queue
std::vector<int>        g_selectedDevIdx;               //Container (stores the selected device indices)
std::atomic<bool>       g_captureDone{ false };         //Capture flag
int g_actualDeviceCount = 0;                            // Actual number of opened devices
short begin_x = 0, begin_y = 0;                         //Start position for printing image data
short end_x = 0, end_y = 0;                             //End position for printing image data
//Device group mode
enum DeviceGroupMode
{
    SINGLE,             //All monocular
    DOUBLE_AND_SINGLE   //All binocular
};
DeviceGroupMode groupMode = SINGLE;
int first_single_Idx = -1;          //Index of the first monocular device among the opened devices
int doubleCount = 0, singleCount = 0;//Number of binocular and monocular devices among the opened devices
//Get format name
static const char* format_name(int fmt)
{
    switch (fmt)
    {
    case FORMAT_RGB24:  return "RGB24";
    case FORMAT_MJPG:   return "MJPG";
    case FORMAT_YUV422: return "YUV422";
    case FORMAT_NV12:   return "NV12";
    default:            return "UNKNOWN";
    }
}

//Create directory
static void ensure_dir(const std::string& path)
{
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(path), ec);
}

//Set mouse cursor position
void setCursorPos(short x, short y)
{
    COORD coord = { x, y };
    SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coord);
}

//Get mouse cursor position
void getCursorPos(short& x, short& y)
{
    CONSOLE_SCREEN_BUFFER_INFO csbi;
    GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &csbi);
    x = csbi.dwCursorPosition.X;
    y = csbi.dwCursorPosition.Y;
}

//Print image data
void printCamData(const CamData& camData)
{
    //Print basic image information
    printf("Camera %2d:%s,%dx%d,size:%ld bytes,fps=%.1f                                                                                                                              \n",
        camData.deviceIdx+1, format_name(camData.format), camData.width, camData.height, camData.bufSize, SYNC_GetFrameRate());
    printf("exposure:start =%12llu us   end =%12llu us   duration = %llu us                                                                                                         \n",
        camData.startExpouseTime, camData.endExpouseTime, camData.endExpouseTime - camData.startExpouseTime);

    //Print IMU data
    for (int i = 0; i < 11; i++)
    {
            printf("[%2d] fAcc = (x: %-12f, y : %-12f, z : %-12f)    fGyro = (x: %-10f, y : %-10f, z : %-10f)    ",
            i,camData.imu_data[i].fAccData_X, camData.imu_data[i].fAccData_Y, camData.imu_data[i].fAccData_Z,
            camData.imu_data[i].fGyroData_X, camData.imu_data[i].fGyroData_Y, camData.imu_data[i].fGyroData_Z);
        printf("time=%-11llu us                                                           \n", camData.imu_data[i].uTime);
    }
}

//Check whether multiple frames are synchronized: max exposure start (end) time - min exposure start (end) time <= kSyncThresholdUs is considered synchronized
static bool is_sync_frames(const std::vector<CamData>& frames, int& dropDevIdx)
{
    dropDevIdx = -1;
    if (frames.size() < 2) return false;
    //Find the max and min exposure start (end) times
    uint64_t min_ts = sync_method == 0 ? frames[0].startExpouseTime:frames[0].endExpouseTime;
    uint64_t max_ts = sync_method == 0 ? frames[0].startExpouseTime : frames[0].endExpouseTime;
    int      min_idx = 0;
    for (size_t i = 0; i < frames.size(); ++i)
    {
        uint64_t ts = (sync_method==0? frames[i].startExpouseTime:frames[i].endExpouseTime);
        if (ts < min_ts)
        {
            min_ts = ts;
            min_idx = static_cast<int>(i);
        }
        if (ts > max_ts)
        {
            max_ts = ts;
        }
    }
    //Store the index of the device with the min exposure start (end) time
    dropDevIdx = min_idx;
    //Compute the difference
    if (max_ts - min_ts <= kSyncThresholdUs)
        return true;
    return false;
}

bool spliceBuff(std::vector<CamData>& frames, uint8_t* result, int totalWidth, int totalHeight)
{
    if (frames.size() < 2)
        return false;
    CamFormat fmt = frames[0].format;
    if (fmt == CamFormat::FORMAT_RGB24)
    {
        int sBeginy = 0;
        if (first_single_Idx != -1)//Mixed monocular and binocular
        {
            //Splice binocular first
            for (size_t cnt = 0; cnt < doubleCount; cnt++)
            {
                uint8_t* dest_ptr = result + cnt * frames[cnt].width * frames[cnt].height * 3;
                const uint8_t* src_ptr = frames[cnt].data;
                //1. Splice binocular
                if (!frames[cnt].isValid)
                    memset(dest_ptr, 0, frames[cnt].width * frames[cnt].height * 3);
                else
                    memcpy(dest_ptr, src_ptr, frames[cnt].width * frames[cnt].height * 3);
                dest_ptr += frames[cnt].width * frames[cnt].height * 3;
                sBeginy += frames[cnt].height;
            }
            int curFrameIndex = first_single_Idx == -1 ? 0 : first_single_Idx;
            //Splice monocular
            int totalRow = (static_cast<int>(frames.size())) / 2 + curFrameIndex;
            int curRow = curFrameIndex;
            int  curRowCol = (static_cast<int>(frames.size()) - curFrameIndex >= 2) ? 2 : 1;

            for (int row = curRow; row < totalRow; row++)
            {
                curRowCol = (static_cast<int>(frames.size()) - curFrameIndex >= 2) ? 2 : 1;
                //Compute the max height of the two images
                int maxHeight = curRowCol == 2 ?
                    (frames[curFrameIndex].height > frames[curFrameIndex + 1].height ?
                        frames[curFrameIndex].height : frames[curFrameIndex + 1].height)
                    : frames[curFrameIndex].height;

                for (int y = 0; y < maxHeight; y++)
                {
                    uint8_t* line_out = result + (y + sBeginy) * totalWidth * 3;
                    //Left image
                    int left_width = frames[curFrameIndex].width;
                    int left_height = frames[curFrameIndex].height;
                    uint8_t* src_line = frames[curFrameIndex].data + y * left_width * 3;
                    if (y < left_height)
                    {
                        if (!frames[curFrameIndex].isValid)
                            memset(line_out, 0, left_width * 3);
                        else
                            memcpy(line_out, src_line, left_width * 3);
                        line_out += left_width * 3;
                    }
                    else
                    {
                        line_out += left_width * 3;
                    }

                    if (curRowCol == 1)continue;
                    //Right image
                    int right_width = frames[curFrameIndex + 1].width;
                    int right_height = frames[curFrameIndex + 1].height;
                    src_line = frames[curFrameIndex + 1].data + y * right_width * 3;
                    if (y < right_height)
                    {
                        if (!frames[curFrameIndex + 1].isValid)
                            memset(line_out, 0, right_width * 3);
                        else
                            memcpy(line_out, src_line, right_width * 3);
                        line_out += right_width * 3;
                    }
                    else
                    {
                        line_out += right_width * 3;
                    }
                    line_out += (totalWidth - (left_width + right_width)) * 3;
                }
                curFrameIndex += curRowCol;
                sBeginy += maxHeight;

            }
        }
        else
        {
            for (int y = 0; y < totalHeight; y++)
            {
                uint8_t* dest_line = result + y * totalWidth * 3;
                for (int c = 0; c < frames.size(); c++)
                {
                    int w = frames[c].width;
                    int h = frames[c].height;
                    uint8_t* src_line = frames[c].data + y * w * 3;

                    if (y < h)
                        memcpy(dest_line, src_line, w * 3);
                    dest_line += w * 3;
                }
            }
        }
    }
    else if(fmt == CamFormat::FORMAT_NV12)
    {
        int sBeginy = 0;
        if (first_single_Idx != -1)//Mixed monocular and binocular
        {
            memset(result, 16, totalWidth * totalHeight);                         // Y all black
            memset(result + totalWidth * totalHeight, 128, totalWidth * totalHeight / 2); // UV gray

            // ---------- 2. Splice the Y plane ----------
            int sBeginY = 0;  // Start line number of the current row in the Y plane

            // 2.1 Binocular Y (each binocular occupies a full row)
            for (int i = 0; i < doubleCount; ++i) {
                int w = frames[i].width;
                int h = frames[i].height;
                // Copy row by row or as a whole block
                for (int y = 0; y < h; ++y) {
                    uint8_t* dst = result + (sBeginY + y) * totalWidth;
                    const uint8_t* src = frames[i].data + y * w;
                    memcpy(dst, src, w);
                }
                sBeginY += h;
            }

            // 2.2 Monocular Y (spliced horizontally in pairs)
            if (first_single_Idx != -1) {
                int idx = first_single_Idx;
                int totalFrames = static_cast<int>(frames.size());
                while (idx < totalFrames) {
                    int count = (totalFrames - idx >= 2) ? 2 : 1;  // Number of frames in the current row
                    // Compute the max height of this row
                    int maxH = frames[idx].height;
                    if (count == 2) maxH = maxH>frames[idx + 1].height? maxH: frames[idx + 1].height;

                    for (int y = 0; y < maxH; ++y) {
                        uint8_t* dst = result + (sBeginY + y) * totalWidth;
                        // Left frame
                        int left_w = frames[idx].width;
                        int left_h = frames[idx].height;
                        if (y < left_h) {
                            const uint8_t* src = frames[idx].data + y * left_w;
                            memcpy(dst, src, left_w);
                        }
                        // If the left side is invalid, memcpy will not overwrite the already filled background (filled black earlier)
                        dst += left_w;

                        if (count == 2) {
                            int right_w = frames[idx + 1].width;
                            int right_h = frames[idx + 1].height;
                            if (y < right_h) {
                                const uint8_t* src = frames[idx + 1].data + y * right_w;
                                memcpy(dst, src, right_w);
                            }
                            dst += right_w;
                        }
                        // The remaining width is already filled with the background, no extra action needed
                    }
                    sBeginY += maxH;
                    idx += count;
                }
            }

            // ---------- 3. Splice the UV plane ----------
            uint8_t* uv_start = result + totalWidth * totalHeight;  // UV plane start
            sBeginY = 0;  // Start line number in the UV plane (UV row = half of a Y row)

            // 3.1 Binocular UV
            for (int i = 0; i < doubleCount; ++i) {
                int w = frames[i].width;
                int h = frames[i].height;
                int uv_h = h / 2;
                const uint8_t* src_uv = frames[i].data + w * h;  // UV start address
                for (int y = 0; y < uv_h; ++y) {
                    uint8_t* dst = uv_start + (sBeginY + y) * totalWidth;
                    memcpy(dst, src_uv + y * w, w);
                }
                sBeginY += uv_h;
            }

            // 3.2 Monocular UV
            if (first_single_Idx != -1) {
                int idx = first_single_Idx;
                int totalFrames = static_cast<int>(frames.size());
                while (idx < totalFrames) {
                    int count = (totalFrames - idx >= 2) ? 2 : 1;
                    // UV plane row height
                    int maxH = frames[idx].height / 2;
                    if (count == 2) maxH = maxH>frames[idx + 1].height / 2? maxH: frames[idx + 1].height/2;

                    for (int y = 0; y < maxH; ++y) {
                        uint8_t* dst = uv_start + (sBeginY + y) * totalWidth;
                        // Left
                        int left_w = frames[idx].width;
                        int left_h = frames[idx].height;
                        int left_uv_h = left_h / 2;
                        if (y < left_uv_h) {
                            const uint8_t* src = frames[idx].data + left_w * left_h + y * left_w;
                            memcpy(dst, src, left_w);
                        }
                        dst += left_w;

                        if (count == 2) {
                            int right_w = frames[idx + 1].width;
                            int right_h = frames[idx + 1].height;
                            int right_uv_h = right_h / 2;
                            if (y < right_uv_h) {
                                const uint8_t* src = frames[idx + 1].data + right_w * right_h + y * right_w;
                                memcpy(dst, src, right_w);
                            }
                            dst += right_w;
                        }
                    }
                    sBeginY += maxH;
                    idx += count;
                }
            }
        }
        else
        {
            uint8_t* dest_ptr = result; // Y plane start
            for (int y = 0; y < totalHeight; y++) {
                uint8_t* dest_line = result + y * totalWidth;
                for (int c = 0; c < frames.size(); c++) {
                    int w = frames[c].width;
                    int h = frames[c].height;

                    if (y < h) {
                        // Valid row: copy the source Y row
                        uint8_t* src_line = frames[c].data + y * w;
                        memcpy(dest_line, src_line, w);
                    }
                    else {
                        // Beyond the frame height, fill black (Y = 16)
                        memset(dest_line, 16, w);
                    }
                    dest_line += w;
                }
            }

            // ---------- Copy the UV plane ----------
            // The UV plane immediately follows the Y plane
            uint8_t* uv_start = result + totalWidth * totalHeight;
            for (int y = 0; y < totalHeight / 2; y++) {
                uint8_t* dest_line = uv_start + y * totalWidth;
                for (int c = 0; c < frames.size(); c++) {
                    int w = frames[c].width;
                    int h = frames[c].height;

                    // Number of UV rows of the frame = h / 2
                    if (y < h / 2) {
                        // Valid row: copy the source UV row
                        uint8_t* src_line = frames[c].data + w * h + y * w;
                        memcpy(dest_line, src_line, w);
                    }
                    else {
                        // Beyond the frame UV height, fill gray (UV = 128)
                        memset(dest_line, 128, w);
                    }
                    dest_line += w;
                }
            }
        }
    }
    else
    {
        printf("Unknown format.\n");
     }
    


    return true;
}
// Create directory
static bool createDirIfNotExist(const char* dirPath)
{
    int ret = _mkdir(dirPath);
    if (ret == 0 || errno == EEXIST)
        return true;
    printf("Failed to create directory: %s, errno=%d\n", dirPath, errno);
    return false;
}

//Get a year-month-day hour-minute-second millisecond string, format 20260618112823156
static std::string getTimeMsStr()
{
    auto now = std::chrono::system_clock::now();
    std::time_t sec = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
    tm t{};
    localtime_s(&t, &sec);
    char buf[32] = { 0 };
    snprintf(buf, sizeof(buf),
        "%04d%02d%02d%02d%02d%02d%03lld",
        t.tm_year + 1900,
        t.tm_mon + 1,
        t.tm_mday,
        t.tm_hour,
        t.tm_min,
        t.tm_sec,
        (long long)ms);
    return std::string(buf);
}

//Save multiple image data (image + basic image information + IMU data)
static bool saveGroupInfo(const uint8_t* spliceBuf,
    const std::vector<CamData>& frames,int res_w,int res_h)
{
    //Save directory
    const std::string rootDir = "./picture";
    if (!createDirIfNotExist(rootDir.c_str()))
        return false;

    //Generate a real-time millisecond timestamp string (subfolder name + file name)
    std::string timeName = getTimeMsStr();
    //Full subfolder path ./picture/20260618120127696
    std::string subDir = rootDir + "/" + timeName;
    //Create a directory named after the millisecond timestamp string
    if (!createDirIfNotExist(subDir.c_str()))
        return false;

    std::string jpg_name = subDir + "/" + timeName + ".jpg";
    std::string txt_name = subDir + "/" + timeName + ".txt";

    int devCount = frames.size();
    if (devCount == 0)
    {
        printf("No camera data, skipping save\n");
        return false;
    }
    int stitch_w = res_w;
    int stitch_h = res_h;

    // Save spliced image
    if (!SYNC_SaveToJPG(spliceBuf, frames[0].format, stitch_w, stitch_h, jpg_name.c_str()))// , kJpgQuality))
    {
        printf("Failed to save JPG: %s\n", jpg_name.c_str());
        return false;
    }

    // Write image data and IMU data to the file
    FILE* fp = nullptr;
    fopen_s(&fp, txt_name.c_str(), "w");
    if (!fp)
    {
        printf("Failed to open txt file: %s\n", txt_name.c_str());
        return false;
    }

    //Write exposure time
    uint64_t minExpTime = (sync_method == 0 ? frames[0].startExpouseTime : frames[0].endExpouseTime);
    uint64_t maxExpTime = (sync_method == 0 ? frames[0].startExpouseTime : frames[0].endExpouseTime);
    for (size_t cam_idx = 0; cam_idx < frames.size(); ++cam_idx)
    {
        const CamData& d = frames[cam_idx];
        if (sync_method == 0)
        {
            minExpTime=minExpTime > d.startExpouseTime ? d.startExpouseTime : minExpTime;
            maxExpTime= maxExpTime< d.startExpouseTime ? d.startExpouseTime : maxExpTime;
            fprintf(fp, "Device %d exposure start time=%llu\n", d.deviceIdx+1, d.startExpouseTime);
        }
        else
        {
            minExpTime = minExpTime > d.endExpouseTime ? d.endExpouseTime : minExpTime;
            maxExpTime = maxExpTime < d.endExpouseTime ? d.endExpouseTime : maxExpTime;
            fprintf(fp, "Device %d exposure end time=%llu\n", d.deviceIdx + 1, d.endExpouseTime);
        }
    }
    
    if (sync_method == 0)
    {
        fprintf(fp, "Min exposure start time=%lluus\n", (unsigned long long)minExpTime);
        fprintf(fp, "Max exposure start time=%lluus\n", (unsigned long long)maxExpTime);
    }
       
    else
    {
        fprintf(fp, "Min exposure end time=%lluus\n", (unsigned long long)minExpTime);
        fprintf(fp, "Max exposure end time=%lluus\n", (unsigned long long)maxExpTime);
    }
    fprintf(fp, "Difference:%lluus\n\n", maxExpTime - minExpTime);
    
    fprintf(fp, "Information for each device:\n");
    for (size_t cam_idx = 0; cam_idx < frames.size(); ++cam_idx)
    {
        const CamData& d = frames[cam_idx];
        fprintf(fp, "[camera_%zu]\n", cam_idx + 1);
        fprintf(fp, "width=%d\n", d.width);
        fprintf(fp, "height=%d\n", d.height);
        fprintf(fp, "bufSize=%d\n", d.bufSize);
        fprintf(fp, "format=%s\n", format_name(d.format));
        fprintf(fp, "startExpouseTime=%" PRIu64 "\n", d.startExpouseTime);
        fprintf(fp, "endExpouseTime=%" PRIu64 "\n", d.endExpouseTime);
        fprintf(fp, "deviceIdx=%d\n", d.deviceIdx);
        fprintf(fp, "isValid=%d\n\n", d.isValid ? 1 : 0);

        //Write IMU data
        fprintf(fp, "IMU Data (Acc + Gyro):\n");
        fprintf(fp, "┌────┬──────────┬──────────┬──────────┬──────────┬──────────┬──────────┬────────────────┐\n");
        fprintf(fp, "│ Index │ Acc(X)  │ Acc(Y)  │ Acc(Z)  │ Gyro(X)  │ Gyro(Y)  │ Gyro(Z) │ Timestamp         │\n");
        fprintf(fp, "├────┼──────────┼──────────┼──────────┼──────────┼──────────┼──────────┼────────────────┤\n");
        for (int i = 0; i < 11; ++i)
        {
            const auto& imu = d.imu_data[i];
            fprintf(fp, "│%-4d│%10.6f   │%10.6f   │%10.6f   │%10.6f   │%10.6f   │%10.6f │%14" PRIu64 " │\n",
                i + 1,
                imu.fAccData_X, imu.fAccData_Y, imu.fAccData_Z,
                imu.fGyroData_X, imu.fGyroData_Y, imu.fGyroData_Z,
                imu.uTime);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
    printf("Synchronized frame saved: %s / %s\n", jpg_name.c_str(), txt_name.c_str());
    return true;
}

// Image callback function
void ImageCallbackFunc(const CamGroupData* image, void* /*userData*/)
{
    int deviceCount = image->deviceCount;
    if (deviceCount <= 0 || deviceCount > kMaxDevices)
    {
        printf("Warning: SDK returned an invalid device count %d, using the actual opened device count %d\n", deviceCount, g_actualDeviceCount);
        //Get the actual number of opened devices
        deviceCount = g_actualDeviceCount;
    }
    for (int i = 0;i < deviceCount; ++i)
    {
        const CamData cam = image->camData[i];
        if (!cam.data || cam.bufSize <= 0 || !cam.isValid)
            continue;
        
        uint32_t devIndex = cam.deviceIdx;
        if (devIndex < 0 || devIndex >= kMaxDevices)
        {
            printf("Error: device index out of range devIndex=%d, exceeding the max device count %d\n", devIndex, kMaxDevices);
            continue;
        }
        {
            //Push each image into the queue by device index
            std::lock_guard<std::mutex> lock(g_queueMtx[devIndex]);
            auto& q = g_frameQueues[devIndex];
            if (q.size() >= kFrameQueueMax) q.pop_front();
            q.push_back(std::move(cam));
        }
       
    }
}

//Main function
int main(int argc, char* argv[])
{
   
    ensure_dir("./");
    for (int i = 0; i < kMaxDevices; i++)
    {
        std::lock_guard<std::mutex> lock(g_queueMtx[i]);
        g_frameQueues[i].clear();
    }
    g_selectedDevIdx.clear();
    g_captureDone.store(false);
    bool isStop = true;

    printf("========== Sync SDK Multi-Camera Synchronized Capture ==========\n");
    //1. Initialize SDK
    int ret = SYNC_Initialize();
    if (ret != 0)
    {
        printf("SDK initialization failed, err: %d -> %s\n", ret, SYNC_GetErrorText(SYNC_GetLastError()));
        system("pause");
        return 1;
    }
    printf("SDK initialized successfully\n");
    printf("SDK version: %s\n", SYNC_GetVersion());

    //2. Enumerate devices
    DeviceInfo dev_list[kMaxDevices] = {};
    int dev_count = 0;
    ret = SYNC_EnumDevices(dev_list, kMaxDevices, &dev_count);
    if (ret != 0 || dev_count <= 0 || dev_count > kMaxDevices)
    {
        printf("Failed to enumerate devices or abnormal device count: count=%d\n", dev_count);
        SYNC_Release();
        system("pause");
        return 1;
    }
    printf("Total devices detected: %d\n\n", dev_count);
    if (dev_list[0].devNumber != 0)
    {
        printf("No master device, the program cannot be used\n");
        SYNC_Release();
        system("pause");
        return 1;
    }
    std::vector<int> opened_devs;

    //3. Get formats, set formats, open devices
    int required_dev_count = 2;                         //Number of devices to open
    std::map<uint32_t, std::vector<FormatInfo>> formats;//Store formats as key-value pairs (key: device number, value: formats supported by the device)
    
    //Open the first required_dev_count devices in a loop
    for (int i = 0; i < dev_count; ++i)
    {

        uint32_t dev_num = dev_list[i].devNumber;
        printf("----- Device %s (PID:%u) -----\n", dev_list[i].name, dev_num+1);
        FormatInfo fmt_list[100] = {};
        int fmt_cnt = 0;
        //Get the device's formats
        ret = SYNC_GetDeviceFormats(dev_num, fmt_list, 100, &fmt_cnt);
        if (ret != 0 || fmt_cnt <= 0)
        {
            printf("  Failed to get the format list: %s\n", SYNC_GetErrorText(SYNC_GetLastError()));
            continue;
        }
        std::vector<FormatInfo> fmts;
        //Print formats
        printf("  Supported formats:\n");
        for (int f = 0; f < fmt_cnt; ++f)
        {
            printf("    [%d] %s %dx%d @%dfps\n",
                f, format_name(fmt_list[f].fmt),
                fmt_list[f].width, fmt_list[f].height, fmt_list[f].fps);
            fmts.push_back(fmt_list[f]);
        }
        formats[dev_num] = fmts;
        //Set the device's format
        ret = SYNC_SetDeviceFormat(dev_list[i].devNumber, 0);
        if (ret != 0)
        {
            printf("  Failed to set format: %s\n", SYNC_GetErrorText(SYNC_GetLastError()));
            continue;
        }
        if (opened_devs.size() >= required_dev_count)continue;
        //Open device
        ret = SYNC_OpenDevice(dev_list[i].devNumber);
        if (ret != 0)
            printf("  Failed to open device %s: %s\n", dev_list[i].name, SYNC_GetErrorText(SYNC_GetLastError()));
        else
        {
            printf("  Device %s configured and opened successfully\n\n", dev_list[i].name);
            opened_devs.push_back(dev_list[i].devNumber);
        }
        
    }
   /* ret = SYNC_OpenDevice(dev_list[0].number);
    if (ret != 0)
        printf("  Failed to open device %d: %s\n", dev_list[0].number, SYNC_GetErrorText(SYNC_GetLastError()));
    else
    {
        printf("  Device %d configured and opened successfully\n\n", dev_list[0].number);
        opened_devs.push_back(dev_list[0].number);
    }
    ret = SYNC_OpenDevice(dev_list[1].number);
    if (ret != 0)
        printf("  Failed to open device %d: %s\n", dev_list[1].number, SYNC_GetErrorText(SYNC_GetLastError()));
    else
    {
        printf("  Device %d configured and opened successfully\n\n", dev_list[1].number);
        opened_devs.push_back(dev_list[1].number);
    }*/

    if (opened_devs.size() < 2)
    {
        printf("Fewer than 2 devices opened successfully, exiting\n");
        for (int dev : opened_devs) SYNC_CloseDevice(dev);
        SYNC_Release();
        system("pause");
        return 1;
    }

    // Record the actual number of opened devices
    g_actualDeviceCount = static_cast<int>(opened_devs.size());

    printf("========== Capture preparation complete ==========\n");
    printf("Selected devices: %zu, sync threshold: %lldus, timeout: %d ms\n\n",
        opened_devs.size(), kSyncThresholdUs, kCaptureTimeoutMs);
    //4. Register the callback function
    ret = SYNC_SetImageCallback(ImageCallbackFunc, nullptr);
    if (ret != 0)
    {
        printf("Failed to register the image callback\n");
        for (int devIndex : opened_devs)
        {
            SYNC_CloseDevice(dev_list[devIndex].devNumber);
        }
        SYNC_Release();
        system("pause");
        return 1;
    }

    //5. Set the capture format
    SYNC_SetCaptureFormat(FORMAT_RGB24);
    char c = dev_list[0].vidpid[4];
    if ((c>='A'&&c<='C')|| (c >= 'a' && c <= 'c'))
        sync_method = 0;
    else if((c >= 'D' && c <= 'F') || (c >= 'd' && c <= 'f'))
        sync_method = 1;
    //Get device group mode
    int master_width = formats[0][0].width;
    int master_height = formats[0][0].height;
    for (int i = 1; i < opened_devs.size(); i++)
    {
        uint32_t dNum = opened_devs[i];
        int w = formats[dNum][0].width;
        int h= formats[dNum][0].height;
        if ((master_width - 160) / 2 >= w-160)
        {
            groupMode = DOUBLE_AND_SINGLE;
            first_single_Idx = i;
            break;
        }
        
    }
    if (first_single_Idx != -1)
    {
        doubleCount = first_single_Idx;
        singleCount = static_cast<int>(opened_devs.size()) - doubleCount;
    }
   
    //6. Start synchronized image capture
    ret = SYNC_StartCapture((int)opened_devs.size());
    if (ret != 0)
    {
        printf("Failed to start capture: %s\n", SYNC_GetErrorText(SYNC_GetLastError()));
        for (int dev : opened_devs) SYNC_CloseDevice(dev);
        SYNC_Release();
        system("pause");
        return 1;
    }
    isStop = false;
    printf("Capture started, waiting for synchronized frames (press s/S to save an image, press k/K to stop(start) capture, press q/Q to exit)...\n");

    //The main thread loops to fetch frames, perform the sync check, save images, and write files
    auto begin_now = std::chrono::system_clock::now();
    long long begin_ms = std::chrono::duration_cast<std::chrono::milliseconds>(begin_now.time_since_epoch()).count();
    std::vector<CamData> frame_candidates;
    bool syncTimeout = false;
    bool isQuit = false;
    uint64_t frameCount = 0;
    getCursorPos(begin_x, begin_y);
    while (true)
    {
        if (_kbhit())
        {
            int ch = _getch(); // Read key
            switch (ch)
            {
            case 's'://Save image
            case 'S':
                g_captureDone.store(true);
                break;
            case 'q'://Exit the program
            case 'Q':
                isQuit = true;
                break;
            case 'k'://Pause (start) capture
            case 'K':
                if (isStop)
                {
                    //Start capture
                    isStop = false;
                    SYNC_StartCapture((int)opened_devs.size());
                    setCursorPos(end_x, end_y);
                    printf("Capture started, waiting for synchronized frames...\n");
                    frameCount = 0;
                }
                else
                {
                    //Pause capture
                    getCursorPos(end_x, end_y);
                    printf("Capture stopped, press k/K to continue...\n");
                    isStop = true;
                    SYNC_StopCapture();
                }
                continue;
            }
        }
        if (isQuit) break;
        if (isStop)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        frame_candidates.clear();

        //1. Fetch frames from the queue
        bool all_ready = true;
        for (size_t idx = 0; idx < opened_devs.size(); ++idx)
        {
            int dev = opened_devs[idx];
            {
                std::lock_guard<std::mutex> lock(g_queueMtx[dev]);
                if (g_frameQueues[dev].empty())
                {
                    all_ready = false;
                    break;
                }
            }
            frame_candidates.push_back(g_frameQueues[dev].front());
        }
        if (!all_ready)
        {
            auto now = std::chrono::system_clock::now();
            long long now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            if (now_ms - begin_ms >= kCaptureTimeoutMs)
            {
                syncTimeout = true;
            }
            if (syncTimeout)
            {
                printf("Capture timeout! Failed to get synchronized frames within %dms\n", kCaptureTimeoutMs);
                getCursorPos(begin_x, begin_y);
                SYNC_StopCapture();
                for (int dev : opened_devs) SYNC_CloseDevice(dev);
                SYNC_Release();
                break;
            }
            continue;
        }
        //2. Check synchronization
        int drop_idx = -1;
        if (!is_sync_frames(frame_candidates, drop_idx))
        {
            auto now = std::chrono::system_clock::now();
            long long now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            if (now_ms - begin_ms >= kCaptureTimeoutMs)
            {
                syncTimeout = true;
            }
            if (syncTimeout)
            {
                printf("Capture timeout! Failed to get synchronized frames within %dms\n", kCaptureTimeoutMs);
                getCursorPos(begin_x, begin_y);
                SYNC_StopCapture();
                for (int dev : opened_devs) SYNC_CloseDevice(dev);
                SYNC_Release();
                break;
            }
            //Not synchronized, discard the old frame
            int devIndex = frame_candidates[drop_idx].deviceIdx;
            std::lock_guard<std::mutex> lock(g_queueMtx[devIndex]);
            g_frameQueues[devIndex].pop_front();
            //printf("Frame dropped\n");
            continue;
        }
        // Update the timeout start point
        begin_now = std::chrono::system_clock::now();
        begin_ms = std::chrono::duration_cast<std::chrono::milliseconds>(begin_now.time_since_epoch()).count();
        frameCount++;

        //Print synchronized frames
        setCursorPos(begin_x, begin_y);
        printf("----------------------------------------------------------------------------------------------------------------------------------------------------------\n");
        for (size_t i = 0; i < frame_candidates.size(); i++)
        {
            printCamData(frame_candidates[i]);
            printf("\n");
        }
        printf("\n");
        getCursorPos(end_x, end_y);

        if (!g_captureDone.load())//Check whether s/S was pressed
        {
            //Dequeue
            for (int idx : opened_devs)
            {
                std::lock_guard<std::mutex> lock(g_queueMtx[idx]);
                g_frameQueues[idx].pop_front();
            }
            continue;
        }
        g_captureDone.store(false);
        getCursorPos(begin_x, begin_y);

        //4. Splice images
        int devCount = opened_devs.size();
        int colorSize = 0;
        if (frame_candidates[0].format == CamFormat::FORMAT_NV12)colorSize = 2;
        else if (frame_candidates[0].format == CamFormat::FORMAT_RGB24)colorSize = 1;
        uint8_t* splice_res = nullptr;
        int res_w = 0;
        int res_h = 0;
        if (groupMode == SINGLE)
        {
            int maxHeight = frame_candidates[0].height;
            for (size_t i = 0; i < frame_candidates.size(); i++)
           {
                maxHeight = maxHeight > frame_candidates[i].height ? maxHeight : frame_candidates[i].height;
                res_w += frame_candidates[i].width;
           }
            res_h = maxHeight;
        }
            
        else
        {
            int maxWidth = frame_candidates[0].width;
            for (int i = 0; i < first_single_Idx; i++)
            {
                maxWidth = maxWidth > frame_candidates[0].width ? maxWidth : frame_candidates[0].width;
                res_h += frame_candidates[0].height;
            }
            for (int i = first_single_Idx; i < frame_candidates.size(); i+=2)
            {
                int maxHeight = 0;
                int curCol = static_cast<int>(frame_candidates.size()-i+1)/2;
                if (curCol == 1)
                {
                    maxHeight = frame_candidates[i].height;
                    maxWidth = maxWidth > frame_candidates[i].width? maxWidth : frame_candidates[i].width;
                }
                else
                {
                    maxHeight= frame_candidates[i].height > frame_candidates[i + 1].height ? frame_candidates[i].height : frame_candidates[i + 1].height;
                    maxWidth = maxWidth > frame_candidates[i].width + frame_candidates[i + 1].width ? maxWidth : frame_candidates[i].width + frame_candidates[i + 1].width;
                }
               
                res_h += maxHeight;
            }
            res_w = maxWidth;
   
        }
        splice_res = new uint8_t[res_w * res_h * 3 / colorSize];
        memset(splice_res, 0, res_w* res_h * 3 / colorSize);
       
        if (!splice_res)
        {
            setCursorPos(end_x, end_y);
            printf("Out of memory, save failed.\n");
            delete[] splice_res;
            splice_res = nullptr;
            continue;
        }
        spliceBuff(frame_candidates, splice_res,res_w,res_h);

        // Save multiple frames
        if (saveGroupInfo(splice_res, frame_candidates,res_w,res_h))
        {
            setCursorPos(end_x, end_y);
            printf("Synchronized frame saved successfully\n");
        }
        //Free memory, dequeue
        delete[] splice_res;
        splice_res = nullptr;
        for (int idx : opened_devs)
        {
            std::lock_guard<std::mutex> lock(g_queueMtx[idx]);
            g_frameQueues[idx].pop_front();
        }
    }
    //7. Stop capture
    SYNC_StopCapture();
    //Close all devices
    for (int dev : opened_devs) SYNC_CloseDevice(dev);
    //8. Release SDK resources
    SYNC_Release();
    printf("\n========== Program exited normally ==========\n");
    system("pause");
    return 0;
}
