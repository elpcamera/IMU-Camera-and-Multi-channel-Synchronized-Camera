#include <cstdio>
#include <cstdint>
#include <cinttypes>
#include <cstring>
#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <algorithm>
#include <atomic>
#include <csignal>
#include <thread>
#include <chrono>
#include <sys/select.h>
#include <unistd.h>
#include <termios.h>
#include <sys/stat.h>   
#include <errno.h>     
#include <ctime>       
#include "synclib.h"

// Global flag: whether each camera uses exposure end time for sync determination
static bool g_useEndTime[SYNC_MAX_CAMERA_COUNT] = {false};
// Program running flag, set to false upon receiving SIGINT, for graceful exit
static std::atomic<bool> g_running(true);

// ======================== Utility Functions ========================
// Convert image format enum to readable string
// Param fmt: Image format enum value
// Return: Corresponding string constant
static const char* format_name(SyncCamFormat fmt)
{
    switch (fmt) {
    case SYNC_FORMAT_MJPG:   return "MJPG";
    case SYNC_FORMAT_YUY2:   return "YUY2";
    case SYNC_FORMAT_NV12:   return "NV12";
    default:                 return "UNKNOWN";
    }
}

namespace {

// Sync determination threshold: exposure start/end time difference <= 100us is considered synced
constexpr uint64_t kSyncThresholdUs = 100;
// Screen refresh min interval (ms), 0 means refresh every synced frame received
constexpr int kMinRefreshIntervalMs = 0;

// Struct to store a frame and its metadata
struct StoredFrame {
    std::vector<uint8_t> buffer; // Image data buffer (deep copy)
    SyncCamData meta;            // Frame metadata
};

// Global mutex to protect all shared data (queues, selection list, refresh timestamps, etc.)
std::mutex g_queueMutex;
// Frame queue for each camera, max cache 12 frames
std::deque<StoredFrame> g_frameQueues[SYNC_MAX_CAMERA_COUNT];
// Device index list selected by user
std::vector<int> g_selectedIndices;
// Last screen refresh time, used for rate limiting
std::chrono::steady_clock::time_point g_lastRefreshTime;
// Mutex to protect standard output (printf), prevent multi-threaded print garbling
std::mutex g_printMutex;

// Save deep copy of the most recent valid synced frame group
std::mutex g_lastFrameMutex;
std::vector<StoredFrame> g_lastSyncedFrames;

// ======================== Save Text Information ========================
// Write synced frame metadata and IMU data to text file
// Param frames: Pointer list of synced frames (verified), each points to a StoredFrame
// Param minStartExposure: Minimum exposure start time of this frame group, used as filename
// Return: true on success, false otherwise
static bool write_frame_info_text(const std::vector<StoredFrame*>& frames, uint64_t minStartExposure)
{
    char txtPath[256];
    snprintf(txtPath, sizeof(txtPath), "%" PRIu64 ".txt", minStartExposure);

    FILE* fp = fopen(txtPath, "w");
    if (!fp) return false;

    // Used for global statistics: select timestamp based on g_useEndTime
    uint64_t globalMinTime = UINT64_MAX;
    uint64_t globalMaxTime = 0;
    bool useEnd = false;
    bool first = true;

    for (size_t i = 0; i < frames.size(); ++i) {
        const SyncCamData& frame = frames[i]->meta;

        int idx = frame.deviceIdx;
        uint64_t t = (g_useEndTime[idx]) ? frame.endExpouseTime : frame.startExpouseTime;
        if (t < globalMinTime) globalMinTime = t;
        if (t > globalMaxTime) globalMaxTime = t;
        if (first) {
            useEnd = g_useEndTime[idx];
            first = false;
        }

        // Calculate minimum difference between frame and IMU timestamp
        int64_t minDiff = -1;
        for (int imuIndex = 0; imuIndex < SYNC_IMU_SAMPLE_COUNT; ++imuIndex) {
            const SyncICM42688Data& imu = frame.imu_data[imuIndex];
            int64_t diff = std::abs(static_cast<int64_t>(imu.uTime) - static_cast<int64_t>(t));
            if (minDiff < 0 || diff < minDiff) {
                minDiff = diff;
            }
        }

        // Write metadata for each camera
        fprintf(fp, "[camera_%zu]\n", i + 1);
        fprintf(fp, "width=%d\n", frame.width);
        fprintf(fp, "height=%d\n", frame.height);
        fprintf(fp, "bufSize=%d\n", frame.bufSize);
        fprintf(fp, "format=%s\n", format_name(frame.format));
        fprintf(fp, "startExpouseTime=%" PRIu64 "\n", frame.startExpouseTime);
        fprintf(fp, "endExpouseTime=%" PRIu64 "\n", frame.endExpouseTime);
        fprintf(fp, "duration=%4" PRIu64 " us\n", frame.endExpouseTime - frame.startExpouseTime);
        // Construct expRowText
        char expRowText[256];
        const char* timeLabel = g_useEndTime[idx] ? "Exposure End Time" : "Exposure Start Time";
        if (minDiff >= 0) {
            snprintf(expRowText, sizeof(expRowText), "%s=%" PRIu64 "(Min time diff between frame and IMU: %" PRId64 "us)", timeLabel, t, minDiff);
        } else {
            snprintf(expRowText, sizeof(expRowText), "%s=%" PRIu64, timeLabel, t);
        }

        fprintf(fp, "deviceIdx=%d\n", frame.deviceIdx);
        fprintf(fp, "rawIndex=%u\n", frame.rawIndex);
        fprintf(fp, "isValid=%d\n", frame.isValid ? 1 : 0);
        fprintf(fp, "\n");

        // IMU Table
        const char* tableTop = "+--------+-----------------+-----------------+-----------------+-----------------+-----------------+-----------------+--------------------+\n";
        const char* tableHeader = "| Index  | Acceleration(X) | Acceleration(Y) | Acceleration(Z) | Gyroscope(X)    | Gyroscope(Y)    | Gyroscope(Z)    | Timestamp          |\n";
        const char* tableMid = "+--------+-----------------+-----------------+-----------------+-----------------+-----------------+-----------------+--------------------+\n";
        const char* tableSpanTop = "+--------+-----------------+-----------------+-----------------+-----------------+-----------------+-----------------+--------------------+\n";
        const char* tableSpanBot = "+--------+-----------------+-----------------+-----------------+-----------------+-----------------+-----------------+--------------------+\n";
        const char* tableBottomNormal = "+--------+-----------------+-----------------+-----------------+-----------------+-----------------+-----------------+--------------------+\n";

        enum ItemType { ITEM_IMU, ITEM_EXP };
        struct TableItem {
            ItemType type;
            int imuIndex;
        };

        std::vector<TableItem> items;
        bool expInserted = false;
        for (int imuIndex = 0; imuIndex < SYNC_IMU_SAMPLE_COUNT; ++imuIndex) {
            if (!expInserted && frame.imu_data[imuIndex].uTime > t) {
                items.push_back({ITEM_EXP, -1});
                expInserted = true;
            }
            items.push_back({ITEM_IMU, imuIndex});
        }
        if (!expInserted) {
            items.push_back({ITEM_EXP, -1});
        }

        fprintf(fp, "%s", tableTop);
        fprintf(fp, "%s", tableHeader);

        for (size_t idx_item = 0; idx_item < items.size(); ++idx_item) {
            bool isFirst = (idx_item == 0);
            bool isLast = (idx_item == items.size() - 1);
            ItemType curType = items[idx_item].type;
            ItemType prevType = isFirst ? ITEM_IMU : items[idx_item - 1].type;

            if (curType == ITEM_IMU) {
                if (isFirst) {
                    fprintf(fp, "%s", tableMid);
                } else if (prevType == ITEM_IMU) {
                    fprintf(fp, "%s", tableMid);
                } else if (prevType == ITEM_EXP) {
                    fprintf(fp, "%s", tableSpanBot);
                }

                const SyncICM42688Data& imu = frame.imu_data[items[idx_item].imuIndex];
                fprintf(fp, "| %-6d | %15.6f | %15.6f | %15.6f | %15.6f | %15.6f | %15.6f | %18" PRIu64 " |\n",
                        items[idx_item].imuIndex + 1,
                        imu.fAccData_X, imu.fAccData_Y, imu.fAccData_Z,
                        imu.fGyroData_X, imu.fGyroData_Y, imu.fGyroData_Z,
                        imu.uTime);

                if (isLast) {
                    fprintf(fp, "%s", tableBottomNormal);
                }
            } else { // curType == ITEM_EXP
                if (!isLast) {
                    fprintf(fp, "%s", tableSpanTop);
                    fprintf(fp, "  %s\n", expRowText);
                } else {
                    fprintf(fp, "%s", tableBottomNormal);
                    fprintf(fp, "%s\n", expRowText);
                }
            }
        }
        fprintf(fp, "\n");
    }
    fprintf(fp, "# Exposure start time for each device\n");
    for (size_t i = 0; i < frames.size(); ++i) {
        const SyncCamData& frame = frames[i]->meta;
        fprintf(fp, "Device %zu: Exposure Start Time = %" PRIu64 " us\n", i + 1, frame.startExpouseTime);
    }
    fprintf(fp, "\n");

    // Global statistics at end of file
    if (useEnd) {
        fprintf(fp, "# Global exposure statistics (based on Exposure End Time)\n");
        fprintf(fp, "Earliest exposure end time = %" PRIu64 " us\n", globalMinTime);
        fprintf(fp, "Latest exposure end time = %" PRIu64 " us\n", globalMaxTime);
    } else {
        fprintf(fp, "# Global exposure statistics (based on Exposure Start Time)\n");
        fprintf(fp, "Earliest exposure start time = %" PRIu64 " us\n", globalMinTime);
        fprintf(fp, "Latest exposure start time = %" PRIu64 " us\n", globalMaxTime);
    }
    fprintf(fp, "Exposure time difference = %" PRIu64 " us\n", globalMaxTime - globalMinTime);

    fclose(fp);
    return true;
}



// ======================== Synchronization Logic ========================
// Determine if a group of frames is synchronized
static bool is_sync_frames(const std::vector<StoredFrame*>& frames, int& minSyncIndex)
{
    // Need at least two frames
    if (frames.size() < 2) return false;

    // Get frame timestamp based on global setting
    auto get_time = [](const StoredFrame* frame) -> uint64_t {
        int idx = frame->meta.deviceIdx;
        if (idx >= 0 && idx < SYNC_MAX_CAMERA_COUNT && g_useEndTime[idx])
            return frame->meta.endExpouseTime;
        else
            return frame->meta.startExpouseTime;
    };

    uint64_t minTime = get_time(frames[0]);
    uint64_t maxTime = minTime;
    minSyncIndex = 0;

    // Iterate frames to find min and max timestamps
    for (size_t i = 1; i < frames.size(); ++i) {
        uint64_t ts = get_time(frames[i]);
        if (ts < minTime) {
            minTime = ts;
            minSyncIndex = static_cast<int>(i);
        }
        if (ts > maxTime) maxTime = ts;
    }
    return (maxTime - minTime) <= kSyncThresholdUs;
}


static bool save_things(const std::vector<StoredFrame*>& frames)
{
    if (frames.empty()) return false;

    // 1. Get current system time (ms)
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    time_t time_now = std::chrono::system_clock::to_time_t(now);
    struct tm* tm_info = localtime(&time_now);

    // 2. Construct timestamp string (e.g., 20260617_205837123)
    char timeStr[64];
    size_t len = strftime(timeStr, sizeof(timeStr), "%Y%m%d_%H%M%S", tm_info);
    snprintf(timeStr + len, sizeof(timeStr) - len, "%03d", static_cast<int>(now_ms.count()));

    // Create directory
    char dirPath[256];
    snprintf(dirPath, sizeof(dirPath), "./%s", timeStr);
    if (mkdir(dirPath, 0755) != 0 && errno != EEXIST) {
        fprintf(stderr, "\n[Save] Failed to create directory: %s\n", dirPath);
        fflush(stderr);
        return false;
    }

    uint64_t minStartExposure = frames[0]->meta.startExpouseTime;

    for (size_t i = 0; i < frames.size(); ++i) {
        const SyncCamData& frame = frames[i]->meta;
        if (!frame.data) return false;

        if (frame.startExpouseTime < minStartExposure) {
            minStartExposure = frame.startExpouseTime;
        }
    }

    // Determine if stitched bino/mono cameras
    bool isBinoStitched = false;
    int max_cropped_w = 0;
    for (size_t i = 0; i < frames.size(); ++i) {
        if (frames[i]->meta.width > max_cropped_w) {
            max_cropped_w = frames[i]->meta.width;
        }
        if (frames[i]->meta.width > 2700) {
            isBinoStitched = true;
        }
    }

    int stitchedWidth = 0;
    int stitchedHeight = 0;
    std::vector<int> deviceXOffset(frames.size(), 0);
    std::vector<int> deviceYOffset(frames.size(), 0);

    if (isBinoStitched) {
        int row0_w = 0;
        int max_bino_h = 0;
        for (size_t i = 0; i < frames.size(); ++i) {
            if (frames[i]->meta.width == max_cropped_w) {
                deviceYOffset[i] = 0;
                deviceXOffset[i] = row0_w;
                row0_w += frames[i]->meta.width;
                if (frames[i]->meta.height > max_bino_h) {
                    max_bino_h = frames[i]->meta.height;
                }
            }
        }

        int row1_w = 0;
        int max_mono_h = 0;
        for (size_t i = 0; i < frames.size(); ++i) {
            if (frames[i]->meta.width < max_cropped_w) {
                deviceYOffset[i] = max_bino_h;
                deviceXOffset[i] = row1_w;
                row1_w += frames[i]->meta.width;
                if (frames[i]->meta.height > max_mono_h) {
                    max_mono_h = frames[i]->meta.height;
                }
            }
        }
        stitchedWidth = (row0_w > row1_w) ? row0_w : row1_w;
        stitchedHeight = max_bino_h + max_mono_h;
    } else {
        int commonHeight = -1;
        int row0_w = 0;
        for (size_t i = 0; i < frames.size(); ++i) {
            if (commonHeight == -1) {
                commonHeight = frames[i]->meta.height;
            } else if (frames[i]->meta.height != commonHeight) {
                fprintf(stderr, "\n[Save] Error: Camera height mismatch: current=%d, prev=%d\n", frames[i]->meta.height, commonHeight);
                fflush(stderr);
                return false;
            }
            deviceYOffset[i] = 0;
            deviceXOffset[i] = row0_w;
            row0_w += frames[i]->meta.width;
        }
        stitchedWidth = row0_w;
        stitchedHeight = commonHeight;
    }

    std::vector<uint8_t> stitchedNv12(static_cast<size_t>(stitchedWidth) * static_cast<size_t>(stitchedHeight) * 3U / 2U, 0);
    uint8_t* dstY = stitchedNv12.data();
    uint8_t* dstUV = stitchedNv12.data() + stitchedWidth * stitchedHeight;

    memset(dstUV, 128, static_cast<size_t>(stitchedWidth) * static_cast<size_t>(stitchedHeight) / 2U);

    for (size_t i = 0; i < frames.size(); ++i) {
        const uint8_t* srcNv12 = frames[i]->buffer.data();
        const uint8_t* srcY = srcNv12;
        const int w = frames[i]->meta.width;
        const int h = frames[i]->meta.height;
        const uint8_t* srcUV = srcNv12 + w * h;

        int xOffset = deviceXOffset[i];
        int yOffset = deviceYOffset[i];

        for (int y = 0; y < h; ++y) {
            memcpy(dstY + (yOffset + y) * stitchedWidth + xOffset, srcY + y * w, w);
        }
        for (int y = 0; y < h / 2; ++y) {
            memcpy(dstUV + (yOffset / 2 + y) * stitchedWidth + xOffset, srcUV + y * w, w);
        }
    }
    char jpgPath[512];
    snprintf(jpgPath, sizeof(jpgPath), "%s/%s.jpg", dirPath, timeStr);
    
    bool ret = SYNC_SaveNV12ToJPG(stitchedNv12.data(), stitchedWidth, stitchedHeight, jpgPath, 90);

    if (ret) {
        char txtPath[512];
        snprintf(txtPath, sizeof(txtPath), "%s/%s.txt", dirPath, timeStr);
        
        bool txtRet = write_frame_info_text(frames, minStartExposure);
        if (txtRet) {
            char defaultTxtPath[256];
            snprintf(defaultTxtPath, sizeof(defaultTxtPath), "%" PRIu64 ".txt", minStartExposure);
            if (rename(defaultTxtPath, txtPath) != 0) {
                fprintf(stderr, "\n[Save] Failed to rename text file: %s -> %s\n", defaultTxtPath, txtPath);
                fflush(stderr);
            }
        }

        printf("\033[2J\033[H");
        fprintf(stderr, "\n[Save] Image saved: %s\n", jpgPath);
        if (txtRet) {
            fprintf(stderr, "[Save] Text saved: %s\n", txtPath);
        } else {
            fprintf(stderr, "[Save] Text save failed\n");
        }
        fflush(stderr);
    } else {
        fprintf(stderr, "\n[Save] Image save failed\n");
        fflush(stderr);
    }
    return ret;
}

// ======================== Print Synced Frames ========================
static void print_synced_frames(const std::vector<StoredFrame*>& frames)
{
    std::lock_guard<std::mutex> lock(g_printMutex);

    {
        std::lock_guard<std::mutex> lastLock(g_lastFrameMutex);
        g_lastSyncedFrames.clear();
        for (StoredFrame* f : frames) {
            StoredFrame copy;
            copy.buffer = f->buffer;
            copy.meta = f->meta;
            copy.meta.data = copy.buffer.data();
            g_lastSyncedFrames.push_back(std::move(copy));
        }
    }

    std::vector<StoredFrame*> sorted = frames;
    std::sort(sorted.begin(), sorted.end(),
              [](const StoredFrame* a, const StoredFrame* b) {
                  return a->meta.deviceIdx < b->meta.deviceIdx;
              });

    printf("\033[H");

    for (size_t i = 0; i < sorted.size(); ++i) {
        const SyncCamData& meta = sorted[i]->meta;
        float fps = SYNC_GetFrameRate(meta.deviceIdx);

        printf("Camera %d: %s %dx%d, size=%d bytes, fps=%.2f\n",
               meta.deviceIdx,
               format_name(meta.format),
               meta.width, meta.height,
               meta.bufSize,
               fps);
        printf("exposure: start=%12" PRIu64 " us end=%12" PRIu64 " us, duration=%8" PRIu64 " us\n",
               meta.startExpouseTime, meta.endExpouseTime,
               meta.endExpouseTime - meta.startExpouseTime);

        for (int imu_idx = 0; imu_idx < SYNC_IMU_SAMPLE_COUNT-1; ++imu_idx) {
            const SyncICM42688Data& imu = meta.imu_data[imu_idx];
            printf("[g] 0x%X Acc=(%12.6f,%12.6f,%12.6f) Gyro=(%12.6f,%12.6f,%12.6f) time=%11" PRIu64 " us\n",
                   imu_idx,
                   imu.fAccData_X, imu.fAccData_Y, imu.fAccData_Z,
                   imu.fGyroData_X, imu.fGyroData_Y, imu.fGyroData_Z,
                   imu.uTime);
        }
        printf("\n");
    }
    fflush(stdout);
}
} // namespace

// ======================== Keyboard Detection Thread ========================
static void keypress_thread_func()
{
    struct termios orig_termios;
    struct termios raw_termios;
    tcgetattr(STDIN_FILENO, &orig_termios);
    raw_termios = orig_termios;

    raw_termios.c_lflag &= ~(ECHO | ICANON);
    raw_termios.c_lflag |= ISIG; 
    raw_termios.c_cc[VMIN] = 1;
    raw_termios.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw_termios);

    while (g_running) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000;
        int ret = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &tv);
        if (ret > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
            char ch;
            if (read(STDIN_FILENO, &ch, 1) == 1 && ch == ' ') {
                std::lock_guard<std::mutex> lastLock(g_lastFrameMutex);
                if (!g_lastSyncedFrames.empty()) {
                    std::vector<StoredFrame*> ptrs;
                    for (auto& frame : g_lastSyncedFrames) {
                        ptrs.push_back(&frame);
                    }
                    save_things(ptrs);
                } else {
                    fprintf(stderr, "\n[Warning] No synced frame data, please wait...\n");
                    fflush(stderr);
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}
// ======================== Signal Handling ========================
static void signal_handler(int /*sig*/)
{
    g_running = false;
    struct termios orig_termios;
    tcgetattr(STDIN_FILENO, &orig_termios);
    system("stty cooked echo");
    fprintf(stderr, "\033[2J\033[H");
    fprintf(stderr, "\n[Signal] Received termination signal, preparing to exit...\n");
    fflush(stderr);
}

// ======================== SDK Callback ========================
void ImageCallbackFunction(const SyncCamGroupData* image, void* /*userData*/)
{
    if (!image || image->deviceCount <= 0) return;

    std::lock_guard<std::mutex> lock(g_queueMutex);

    for (int i = 0; i < image->deviceCount && i < SYNC_MAX_CAMERA_COUNT; ++i) {
        const SyncCamData& frame = image->camData[i];
        if (!frame.data || !frame.isValid || frame.width <= 0 || frame.height <= 0 || frame.bufSize <= 0)
            continue;
        if (frame.deviceIdx < 0 || frame.deviceIdx >= SYNC_MAX_CAMERA_COUNT)
            continue;

        StoredFrame stored;
        stored.buffer.assign(frame.data, frame.data + frame.bufSize);
        stored.meta = frame;
        stored.meta.data = stored.buffer.data();

        std::deque<StoredFrame>& queue = g_frameQueues[frame.deviceIdx];
        if (queue.size() >= 12) queue.pop_front();
        queue.push_back(std::move(stored));
    }

    while (true) {
        if (g_selectedIndices.size() < 2) break;

        std::vector<StoredFrame*> candidates;
        candidates.reserve(g_selectedIndices.size());
        bool allReady = true;
        for (int dev : g_selectedIndices) {
            if (dev < 0 || dev >= SYNC_MAX_CAMERA_COUNT || g_frameQueues[dev].empty()) {
                allReady = false;
                break;
            }
            candidates.push_back(&g_frameQueues[dev].front());
        }
        if (!allReady) break;

        int minIdx = -1;
        if (!is_sync_frames(candidates, minIdx)) {
            if (minIdx >= 0 && minIdx < (int)candidates.size()) {
                int dropDev = candidates[minIdx]->meta.deviceIdx;
                if (dropDev >= 0 && dropDev < SYNC_MAX_CAMERA_COUNT && !g_frameQueues[dropDev].empty()) {
                    g_frameQueues[dropDev].pop_front();
                    continue;
                }
            }
            break;
        }

        auto now = std::chrono::steady_clock::now();
        bool shouldRefresh = false;
        if (g_lastRefreshTime.time_since_epoch().count() == 0) {
            shouldRefresh = true;
        } else {
            auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_lastRefreshTime).count();
            if (elapsedMs >= kMinRefreshIntervalMs) {
                shouldRefresh = true;
            }
        }

        if (shouldRefresh) {
            print_synced_frames(candidates);
            g_lastRefreshTime = now;
        }

        for (StoredFrame* f : candidates) {
            int dev = f->meta.deviceIdx;
            if (dev >= 0 && dev < SYNC_MAX_CAMERA_COUNT && !g_frameQueues[dev].empty())
                g_frameQueues[dev].pop_front();
        }
    }
}

// ======================== Hardware PID Identification ========================
static unsigned int extract_hardware_pid(const char* vidpid) {
    unsigned int pid = 0;
    if (vidpid) {
        size_t len = strlen(vidpid);
        if (len >= 2) {
            sscanf(vidpid + len - 2, "%u", &pid);
        }
    }
    return pid;
}

// ======================== main ========================
int main(int argc, char* argv[])
{
    (void)argc; (void)argv;

    std::signal(SIGINT, signal_handler);

    {
        std::lock_guard<std::mutex> lock(g_queueMutex);
        g_selectedIndices.clear();
        for (int i = 0; i < SYNC_MAX_CAMERA_COUNT; ++i)
            g_frameQueues[i].clear();
        g_lastRefreshTime = std::chrono::steady_clock::time_point();
    }

    fprintf(stderr, "\033[2J\033[H");
    fprintf(stderr, "synccap SDK Testing Program \n");
    fprintf(stderr, "SDK Version: %s\n\n", SYNC_GetVersion());
    fprintf(stderr, "Press Space to capture...\n");
    fflush(stderr);

    // 1. Initialize SDK
    if (SYNC_Initialize() == 0) {
        fprintf(stderr, "Error: SYNC_Initialize failed: %s\n", SYNC_GetErrorText(SYNC_GetLastError()));
        return 1;
    }

    // 2. Enumerate Devices
    SyncDeviceInfo deviceInfo[SYNC_MAX_CAMERA_COUNT] = {};
    int deviceCount = 0;
    if (SYNC_EnumDevices(deviceInfo, SYNC_MAX_CAMERA_COUNT, &deviceCount) == 0) {
        fprintf(stderr, "Error: SYNC_EnumDevices failed: %s\n", SYNC_GetErrorText(SYNC_GetLastError()));
        SYNC_Release();
        return 1;
    }

    fprintf(stderr, "\n[Device Enumeration] Detected %d devices:\n", deviceCount);
    for (int i = 0; i < SYNC_MAX_CAMERA_COUNT; ++i) {
        if (deviceInfo[i].isExist) {
            const char* vidpid = deviceInfo[i].vidpid;
            if (strlen(vidpid) >= 5) {
                char ch = vidpid[4];
                if (ch == 'a' || ch == 'b' || ch == 'c') {
                    g_useEndTime[deviceInfo[i].number] = false;
                } else if (ch == 'd' || ch == 'e' || ch == 'f') {
                    g_useEndTime[deviceInfo[i].number] = true;
                }
            }
            fprintf(stderr, "  Device %u: %s, vidpid=%s, useEndTime=%d\n",
                    deviceInfo[i].number, deviceInfo[i].name, vidpid, g_useEndTime[deviceInfo[i].number] ? 1 : 0);
        }
    }
    if (deviceCount < 2) {
        fprintf(stderr, "Error: At least 2 devices required for synchronized capture\n");
        SYNC_Release();
        return 1;
    }

    bool hasStartTime = false;
    bool hasEndTime = false;
    for (int i = 0; i < SYNC_MAX_CAMERA_COUNT; ++i) {
        if (deviceInfo[i].isExist) {
            const char* vidpid = deviceInfo[i].vidpid;
            if (strlen(vidpid) >= 5) {
                char ch = vidpid[4];
                if (ch == 'a' || ch == 'b' || ch == 'c') {
                    hasStartTime = true;
                } else if (ch == 'd' || ch == 'e' || ch == 'f') {
                    hasEndTime = true;
                }
            }
        }
    }
    if (hasStartTime && hasEndTime) {
        fprintf(stderr, "Error: Mixed sync types detected (some start time sync, some end time sync), cannot proceed!\n");
        SYNC_Release();
        return 1;
    }

    // 3. Get and Set Format
    std::vector<int> selectedIndices;
    selectedIndices.reserve(SYNC_MAX_CAMERA_COUNT);
    bool hasPidSuffix01 = false;
    int FormatIndex = 0;
    int firstWidth = -1;
    int firstHeight = -1;
    int firstFps = -1;
    SyncCamFormat firstFmt = SYNC_FORMAT_MJPG;

    bool isBinoCamera = false;
    for (int i = 0; i < SYNC_MAX_CAMERA_COUNT; ++i) {
        if (!deviceInfo[i].isExist) continue;
        unsigned int hwPid = extract_hardware_pid(deviceInfo[i].vidpid);
        if (hwPid == 1) {
            hasPidSuffix01 = true;
            SyncFormatInfo formats[100];
            int formatCount = 0;
            if (SYNC_GetDeviceFormats(deviceInfo[i].number, formats, 100, &formatCount) == 0) {
                fprintf(stderr, "Error: Failed to get master device formats\n");
                SYNC_Release();
                return 1;
            }
            if (formatCount > 0) {
                int w = formats[0].width;
                if (w - 160 > 2700) {
                    isBinoCamera = true;
                }
            }
            break;
        }
    }

    if (hasPidSuffix01 == false) {
        fprintf(stderr, "Error: Master device (PID ending in 01) not found, cannot determine sync baseline\n");
        SYNC_Release();
        return 1;
    }

    for (int i = 0; i < SYNC_MAX_CAMERA_COUNT; ++i) {
        if (!deviceInfo[i].isExist) continue;

        SyncFormatInfo formats[100];
        int formatCount = 0;
        if (SYNC_GetDeviceFormats(deviceInfo[i].number, formats, 100, &formatCount) == 0) {
            fprintf(stderr, "Error: Device %u format access failed: %s\n", deviceInfo[i].number, SYNC_GetErrorText(SYNC_GetLastError()));
            SYNC_Release();
            return 1;
        }
        
        printf("[Format] Device %u supports %d types:\n", deviceInfo[i].number, formatCount);
        for (int fmtIndex = 0; fmtIndex < formatCount; ++fmtIndex) {
            printf("    %d: %s %dx%d %dfps\n",
                        fmtIndex,
                        format_name(formats[fmtIndex].fmt),
                        formats[fmtIndex].width,
                        formats[fmtIndex].height,
                        formats[fmtIndex].fps);
        }
        if (formatCount <= 0) {
            fprintf(stderr, "Error: Device %u has no formats\n", deviceInfo[i].number);
            SYNC_Release();
            return 1;
        }
        if (SYNC_SetDeviceFormat(deviceInfo[i].number, FormatIndex) == 0) {
            fprintf(stderr, "Error: Device %u format set failed: %s\n", deviceInfo[i].number, SYNC_GetErrorText(SYNC_GetLastError()));
            SYNC_Release();
            return 1;
        }

        if (firstWidth == -1) {
            firstWidth = formats[FormatIndex].width;
            firstHeight = formats[FormatIndex].height;
            firstFps = formats[FormatIndex].fps;
            firstFmt = formats[FormatIndex].fmt;
        } else {
            if (isBinoCamera) {
                if (formats[FormatIndex].fps != firstFps ||
                    formats[FormatIndex].fmt != firstFmt) {
                    fprintf(stderr, "Error: Device %u default format (excl. WH) mismatch! Current: %dfps %s, Prev: %dfps %s\n",
                            deviceInfo[i].number,
                            formats[FormatIndex].fps, format_name(formats[FormatIndex].fmt),
                            firstFps, format_name(firstFmt));
                    SYNC_Release();
                    return 1;
                }
            } else {
                if (formats[FormatIndex].height != firstHeight ||
                    formats[FormatIndex].fps != firstFps ||
                    formats[FormatIndex].fmt != firstFmt) {
                    fprintf(stderr, "Error: Device %u default format (excl. W) mismatch! Current: %dx%d %dfps %s, Prev: %dx%d %dfps %s\n",
                            deviceInfo[i].number,
                            formats[FormatIndex].width, formats[FormatIndex].height, formats[FormatIndex].fps, format_name(formats[FormatIndex].fmt),
                            firstWidth, firstHeight, firstFps, format_name(firstFmt));
                    SYNC_Release();
                    return 1;
                }
            }
        }

        fprintf(stderr, "[Format Setup] Device %d (pid=%u) set to format: %dx%d %dfps\n",
                i, deviceInfo[i].number, formats[FormatIndex].width, formats[FormatIndex].height, formats[FormatIndex].fps);

        selectedIndices.push_back(deviceInfo[i].number);
    }

    if (hasPidSuffix01 == false) {
        fprintf(stderr, "Error: Master device (PID ending in 01) not found\n");
        SYNC_Release();
        return 1;
    }

    // 4. Automatic Device Selection
    if (selectedIndices.size() < 2) {
        fprintf(stderr, "Error: Not enough devices available for synchronization (need at least 2).\n");
        SYNC_Release();
        return 1;
    }

    fprintf(stderr, "\n[Device Auto-Selection] Automatically selected all %zu devices:\n", selectedIndices.size());
    for (size_t i = 0; i < selectedIndices.size(); ++i) {
        int idx = selectedIndices[i];
        const SyncDeviceInfo* devPtr = nullptr;
        for (int d = 0; d < SYNC_MAX_CAMERA_COUNT; ++d) {
            if (deviceInfo[d].isExist && (int)deviceInfo[d].number == idx) {
                devPtr = &deviceInfo[d];
                break;
            }
        }
        if (devPtr) {
            fprintf(stderr, "  Device %d: %s, pid=%u\n", idx, devPtr->name, devPtr->number);
        } else {
            fprintf(stderr, "  Device %d: Unknown, pid=%d\n", idx, idx);
        }
    }

    {
        std::lock_guard<std::mutex> lock(g_queueMutex);
        g_selectedIndices = selectedIndices;
    }

    // 5. Register Callback
    if (SYNC_SetImageCallback(ImageCallbackFunction, nullptr) == 0) {
        fprintf(stderr, "Error: SYNC_SetImageCallback failed: %s\n", SYNC_GetErrorText(SYNC_GetLastError()));
        SYNC_Release();
        return 1;
    }

    // 6. Capture
    printf("\033[2J\033[H");
    fflush(stdout);

    std::thread keyThread(keypress_thread_func);

    // 7. Start Capture
    if (SYNC_StartCaptureByIndices(selectedIndices.data(), static_cast<int>(selectedIndices.size())) == 0) {
        fprintf(stderr, "Error: SYNC_StartCaptureByIndices failed: %s\n", SYNC_GetErrorText(SYNC_GetLastError()));
        g_running = false;
        if (keyThread.joinable()) {
            keyThread.join();
        }
        SYNC_Release();
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 8. Wait for Ctrl+C
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    keyThread.join();

    // 9. Stop and Release
    fprintf(stderr, "\n[Stop] Stopping capture and releasing resources...\n");
    SYNC_StopCapture();
    SYNC_Release();

    fprintf(stderr, "\nProgram exited normally.\n");
    return 0;
}
