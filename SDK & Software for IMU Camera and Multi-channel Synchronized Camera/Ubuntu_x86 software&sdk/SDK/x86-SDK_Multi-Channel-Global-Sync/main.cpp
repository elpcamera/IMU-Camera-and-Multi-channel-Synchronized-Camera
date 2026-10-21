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
// Program running flag, set to false upon SIGINT for graceful exit
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

// Sync threshold: exposure start/end time difference <= 100us is considered synced
constexpr uint64_t kSyncThresholdUs = 100;
// Screen refresh min interval (ms), 0 means refresh every synced frame
constexpr int kMinRefreshIntervalMs = 0;

// Struct to store a frame and its metadata
struct StoredFrame {
    std::vector<uint8_t> buffer; // Image data buffer (deep copy)
    SyncCamData meta;            // Frame metadata
};

// Global mutex protecting all shared data (queues, selection list, refresh timestamps, etc.)
std::mutex g_queueMutex;
// Frame queue for each camera, max cache 12 frames
std::deque<StoredFrame> g_frameQueues[SYNC_MAX_CAMERA_COUNT];
// Device index list selected by user
std::vector<int> g_selectedIndices;
// Last screen refresh time, used for rate limiting
std::chrono::steady_clock::time_point g_lastRefreshTime;
// Mutex protecting stdout (printf) to avoid multi-thread output corruption
std::mutex g_printMutex;

// Save deep copy of most recent valid synced frame group
std::mutex g_lastFrameMutex;
std::vector<StoredFrame> g_lastSyncedFrames;

// ======================== Save Text Information ========================
// Write synced frame metadata and IMU data to text file
// Param frames: Pointer list of synced frames, each points to a StoredFrame
// Param minStartExposure: Minimum exposure start time of this group, used as filename
// return value: successreturn true, failedreturn false
static bool write_frame_info_text(const std::vector<StoredFrame*>& frames, uint64_t minStartExposure)
{
    char txtPath[256];
    snprintf(txtPath, sizeof(txtPath), "%" PRIu64 ".txt", minStartExposure);

    FILE* fp = fopen(txtPath, "w");
    if (!fp) return false;

    // For global statistics: select timestamp based on g_useEndTime
    uint64_t globalMinTime = UINT64_MAX;
    uint64_t globalMaxTime = 0;
    bool useEnd = false;
    bool first = true;

    for (size_t i = 0; i < frames.size(); ++i) {
        const SyncCamData& frame = frames[i]->meta;

        // (select start or end based on device configuration)
        int idx = frame.deviceIdx;
        uint64_t t = (g_useEndTime[idx]) ? frame.endExpouseTime : frame.startExpouseTime;
        if (t < globalMinTime) globalMinTime = t;
        if (t > globalMaxTime) globalMaxTime = t;
        if (first) {
            useEnd = g_useEndTime[idx];
            first = false;
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
        fprintf(fp, "deviceIdx=%d\n", frame.deviceIdx);
        fprintf(fp, "rawIndex=%u\n", frame.rawIndex);
        fprintf(fp, "isValid=%d\n", frame.isValid ? 1 : 0);
        fprintf(fp, "\n");

        // IMU Table (kept unchanged)
        fprintf(fp, "+--------+-----------------+-----------------+-----------------+-----------------+-----------------+-----------------+--------------------+\n");
        fprintf(fp, "| Index  | Acceleration(X) | Acceleration(Y) | Acceleration(Z) | Gyroscope(X)    | Gyroscope(Y)    | Gyroscope(Z)    | Timestamp          |\n");
        fprintf(fp, "+--------+-----------------+-----------------+-----------------+-----------------+-----------------+-----------------+--------------------+\n");

        for (int imuIndex = 0; imuIndex < SYNC_IMU_SAMPLE_COUNT; ++imuIndex) {
            const SyncICM42688Data& imu = frame.imu_data[imuIndex];
            fprintf(fp,
                    "| %-6d | %15.6f | %15.6f | %15.6f | %15.6f | %15.6f | %15.6f | %18" PRIu64 " |\n",
                    imuIndex + 1,
                    imu.fAccData_X, imu.fAccData_Y, imu.fAccData_Z,
                    imu.fGyroData_X, imu.fGyroData_Y, imu.fGyroData_Z,
                    imu.uTime);
            if (imuIndex != SYNC_IMU_SAMPLE_COUNT - 1)
                fprintf(fp, "+--------+-----------------+-----------------+-----------------+-----------------+-----------------+-----------------+--------------------+\n");
        }
        fprintf(fp, "+--------+-----------------+-----------------+-----------------+-----------------+-----------------+-----------------+--------------------+\n");
        fprintf(fp, "\n");
    }
    fprintf(fp, "# Exposure start time of each device\n");
    for (size_t i = 0; i < frames.size(); ++i) {
        const SyncCamData& frame = frames[i]->meta;
        fprintf(fp, "Device %zu: Exposure start time = %" PRIu64 " us\n", i + 1, frame.startExpouseTime);
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
// Determine if a group of frames is synced (difference within threshold)
// Param frames: frame pointer list to check
// Param minSyncIndex: output index of frame with min timestamp
// Return: true if synced, false otherwise
static bool is_sync_frames(const std::vector<StoredFrame*>& frames, int& minSyncIndex)
{
    // Need at least two frames to determine
    if (frames.size() < 2) return false;

    // Get frame timestamp (start or end based on g_useEndTime)
    // Note: exposure end time is more stable on some devices, so it is configurable
    auto get_time = [](const StoredFrame* frame) -> uint64_t {
        int idx = frame->meta.deviceIdx;
        if (idx >= 0 && idx < SYNC_MAX_CAMERA_COUNT && g_useEndTime[idx])
            return frame->meta.endExpouseTime;   // Use exposure end time
        else
            return frame->meta.startExpouseTime; // Use exposure start time
    };

    // Initialize min/max timestamps and record min index
    uint64_t minTime = get_time(frames[0]);
    uint64_t maxTime = minTime;
    minSyncIndex = 0;

    // Iterate frames to find min and max timestamps
    for (size_t i = 1; i < frames.size(); ++i) {
        uint64_t ts = get_time(frames[i]);
        if (ts < minTime) {
            minTime = ts;
            minSyncIndex = static_cast<int>(i);   // Record position of earliest frame
        }
        if (ts > maxTime) maxTime = ts;
    }
    // Time difference <= threshold means synced
    return (maxTime - minTime) <= kSyncThresholdUs;
}


static bool save_things(const std::vector<StoredFrame*>& frames)
{
    if (frames.empty()) return false;

    // 1. Get current system time via chrono (millisecond precision)
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    time_t time_now = std::chrono::system_clock::to_time_t(now);
    struct tm* tm_info = localtime(&time_now);

    // 2. Construct timestamp string including milliseconds (e.g., 20260617_205837123)
    char timeStr[64];
    size_t len = strftime(timeStr, sizeof(timeStr), "%Y%m%d_%H%M%S", tm_info);
    snprintf(timeStr + len, sizeof(timeStr) - len, "%03d", static_cast<int>(now_ms.count()));

    // Create folder path
    char dirPath[256];
    snprintf(dirPath, sizeof(dirPath), "./%s", timeStr);
    if (mkdir(dirPath, 0755) != 0 && errno != EEXIST) {
        fprintf(stderr, "\n[Save] Failed to create directory: %s\n", dirPath);
        fflush(stderr);
        return false;
    }

    const int width = frames[0]->meta.width;
    const int height = frames[0]->meta.height;
    uint64_t minStartExposure = frames[0]->meta.startExpouseTime;

    for (size_t i = 0; i < frames.size(); ++i) {
        const SyncCamData& frame = frames[i]->meta;
        if (!frame.data || frame.width != width || frame.height != height)
            return false;
        if (frame.startExpouseTime < minStartExposure)
            minStartExposure = frame.startExpouseTime;
    }

    const int stitchedWidth = width * static_cast<int>(frames.size());
    const int stitchedHeight = height;
    std::vector<uint8_t> stitchedNv12(static_cast<size_t>(stitchedWidth) * static_cast<size_t>(stitchedHeight) * 3U / 2U, 0);
    uint8_t* dstY = stitchedNv12.data();
    uint8_t* dstUV = stitchedNv12.data() + stitchedWidth * stitchedHeight;

    for (size_t i = 0; i < frames.size(); ++i) {
        const uint8_t* srcNv12 = frames[i]->buffer.data();
        const uint8_t* srcY = srcNv12;
        const uint8_t* srcUV = srcNv12 + width * height;
        for (int y = 0; y < stitchedHeight; ++y) {
            memcpy(dstY + y * stitchedWidth + i * width, srcY + y * width, width);
        }
        for (int y = 0; y < stitchedHeight / 2; ++y) {
            memcpy(dstUV + y * stitchedWidth + i * width, srcUV + y * width, width);
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
                fprintf(stderr, "\n[Save] Failed to move text file: %s -> %s\n", defaultTxtPath, txtPath);
                fflush(stderr);
            }
        }

        // Clear screen and print clean success save log
        printf("\033[2J\033[H");
        fprintf(stderr, "\n[Save] Image saved: %s\n", jpgPath);
        if (txtRet) {
            fprintf(stderr, "[Save] Text file saved: %s\n", txtPath);
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
// Format and output synced frame info to screen (dynamic refresh, overwriting previous content)
// Also deep-copy this frame group into g_lastSyncedFrames for saving by space key
// Param frames: Synced frame pointer list to print
static void print_synced_frames(const std::vector<StoredFrame*>& frames)
{
    std::lock_guard<std::mutex> lock(g_printMutex);

    // Deep copy current synced frames for later save (frames may be consumed by the queue when space is pressed)
    {
        std::lock_guard<std::mutex> lastLock(g_lastFrameMutex);
        g_lastSyncedFrames.clear();
        for (StoredFrame* f : frames) {
            StoredFrame copy;
            copy.buffer = f->buffer;               // Copy image data
            copy.meta = f->meta;                   // Copy metadata
            copy.meta.data = copy.buffer.data();   // Fix data pointer to point to the copied buffer
            g_lastSyncedFrames.push_back(std::move(copy));
        }
    }

    // Sort by device index to ensure fixed output order (ascending)
    std::vector<StoredFrame*> sorted = frames;
    std::sort(sorted.begin(), sorted.end(),
              [](const StoredFrame* a, const StoredFrame* b) {
                  return a->meta.deviceIdx < b->meta.deviceIdx;
              });

    // Move cursor to top-left for dynamic refresh effect (overwrite previous output)
    printf("\033[H");

    // Print info for each device
    for (size_t i = 0; i < sorted.size(); ++i) {
        const SyncCamData& meta = sorted[i]->meta;
        float fps = SYNC_GetFrameRate(meta.deviceIdx); // Get real-time frame rate

        printf("Camera %d: %s %dx%d, size=%d bytes, fps=%.2f\n",
               meta.deviceIdx,
               format_name(meta.format),
               meta.width, meta.height,
               meta.bufSize,
               fps);
        printf("exposure: start=%12" PRIu64 " us end=%12" PRIu64 " us, duration=%8" PRIu64 " us\n",
               meta.startExpouseTime, meta.endExpouseTime,
               meta.endExpouseTime - meta.startExpouseTime);

        // Print IMU data (max SYNC_IMU_SAMPLE_COUNT samples per camera)
        // Note: only print up to SYNC_IMU_SAMPLE_COUNT-1 to avoid screen scrolling
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
// Independent thread using select to poll stdin; saves latest synced frames when space is pressed
static void keypress_thread_func()
{
    // Save original terminal attributes
    struct termios orig_termios;
    struct termios raw_termios;
    tcgetattr(STDIN_FILENO, &orig_termios);
    raw_termios = orig_termios;

    // Set raw mode: disable echo, canonical mode, signal handling, etc.
   raw_termios.c_lflag &= ~(ECHO | ICANON);
    raw_termios.c_lflag |= ISIG; 
    // Set input min chars to 1, timeout to 0 (immediate return)
    raw_termios.c_cc[VMIN] = 1;
    raw_termios.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw_termios);

    while (g_running) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000; // 100ms timeout
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
                    fprintf(stderr, "\n[Warning] No synced frame data yet, please press space later\n");
                    fflush(stderr);
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Restore terminal settings before thread exit
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}
// ======================== Signal Handling ========================
// SIGINT Signal Handling function sets g_running = false to exit the main loop
// Param sig: Signal number (unused)
static void signal_handler(int /*sig*/)
{
    g_running = false;
    // Restore terminal settings (if raw mode was set)
    struct termios orig_termios;
    tcgetattr(STDIN_FILENO, &orig_termios);
    system("stty cooked echo");
    fprintf(stderr, "\033[2J\033[H");
    fprintf(stderr, "\n[Signal] Received termination signal, preparing to exit...\n");
    fflush(stderr);
}

// ======================== SDK Callback ========================
// SDK image Callback Functions, invoked when a new image group arrives
// Param image: Image group returned by SDK (multiple cameras triggered simultaneously)
// Param userData: User-defined data pointer
void ImageCallbackFunction(const SyncCamGroupData* image, void* /*userData*/)
{
    // Check input parameter validity
    if (!image || image->deviceCount <= 0) return;

    // Lock global queue and selection list
    std::lock_guard<std::mutex> lock(g_queueMutex);

    // Store frame of each camera in this callback into corresponding device queue
    for (int i = 0; i < image->deviceCount && i < SYNC_MAX_CAMERA_COUNT; ++i) {
        const SyncCamData& frame = image->camData[i];
        // Skip invalid frames (null data, invalid flag, abnormal dimensions, etc.)
        if (!frame.data || !frame.isValid || frame.width <= 0 || frame.height <= 0 || frame.bufSize <= 0)
            continue;
        // Device index must be valid
        if (frame.deviceIdx < 0 || frame.deviceIdx >= SYNC_MAX_CAMERA_COUNT)
            continue;

        // Copy image data and metadata (deep copy because SDK callback data may be freed after return)
        StoredFrame stored;
        stored.buffer.assign(frame.data, frame.data + frame.bufSize);
        stored.meta = frame;
        stored.meta.data = stored.buffer.data(); // Point data pointer to the copied buffer

        // Get device queue; if over 12 frames, drop oldest and add new frame
        std::deque<StoredFrame>& queue = g_frameQueues[frame.deviceIdx];
        if (queue.size() >= 12) queue.pop_front();
        queue.push_back(std::move(stored));
    }

    // Continuously try to match synced frames (sliding window alignment)
    while (true) {
        // Need at least 2 devices selected for sync detection
        if (g_selectedIndices.size() < 2) break;

        // Collect front frame pointers of all selected devices
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
        // If a device queue is empty, wait for more frames
        if (!allReady) break;

        // Check whether these front frames are synced
        int minIdx = -1;
        if (!is_sync_frames(candidates, minIdx)) {
            // Not synced: drop frame with smallest timestamp (earliest exposed), then retry
            if (minIdx >= 0 && minIdx < (int)candidates.size()) {
                int dropDev = candidates[minIdx]->meta.deviceIdx;
                if (dropDev >= 0 && dropDev < SYNC_MAX_CAMERA_COUNT && !g_frameQueues[dropDev].empty()) {
                    g_frameQueues[dropDev].pop_front();
                    continue; // Recheck the new front frame
                }
            }
            break; // Cannot drop further (invalid index or empty queue), exit loop
        }

        // Synced frame found: determine if screen refresh needed (min interval)
        auto now = std::chrono::steady_clock::now();
        bool shouldRefresh = false;
        if (g_lastRefreshTime.time_since_epoch().count() == 0) {
            shouldRefresh = true; // First synced frame received, refresh immediately
        } else {
            auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_lastRefreshTime).count();
            if (elapsedMs >= kMinRefreshIntervalMs) {
                shouldRefresh = true;
            }
        }

        if (shouldRefresh) {
            print_synced_frames(candidates); // Output to screen and deep copy to g_lastSyncedFrames
            g_lastRefreshTime = now;          // Update last refresh time
        }

        // Consume this frame group (remove from queues to avoid reprocessing)
        for (StoredFrame* f : candidates) {
            int dev = f->meta.deviceIdx;
            if (dev >= 0 && dev < SYNC_MAX_CAMERA_COUNT && !g_frameQueues[dev].empty())
                g_frameQueues[dev].pop_front();
        }
        // Continue to next group (queue may have more frame combinations)
    }
}
// ======================== Hardware PID Identification for Master Device ========================
// Extract hardware PID from VIDPID string (e.g. "1D19:1001"), result is 1
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
    (void)argc; (void)argv; // Avoid unused parameter warning

    // ========== System Initialization Phase ==========
    // Register SIGINT Signal Handling function for graceful exit (cleanup on Ctrl+C)
    std::signal(SIGINT, signal_handler);

    // Initialize global queues and timestamps (locked for thread safety)
    {
        std::lock_guard<std::mutex> lock(g_queueMutex);
        g_selectedIndices.clear();                     // Clear selected device list
        for (int i = 0; i < SYNC_MAX_CAMERA_COUNT; ++i)
            g_frameQueues[i].clear();                  // Clear frame queue of each camera
        g_lastRefreshTime = std::chrono::steady_clock::time_point(); // Reset last refresh time
    }

    // Output all init info to stderr to avoid polluting stdout
    // (stdout is used for dynamic refresh; mixing init info would corrupt the display)
    fprintf(stderr, "\033[2J\033[H");                  // 
    fprintf(stderr, "synccap SDK Testing Program \n");
    fprintf(stderr, "SDK Version: %s\n\n", SYNC_GetVersion());
    fprintf(stderr, "Press Space to capture...\n");
    fflush(stderr);                                    // Ensure immediate output

    // ----- 1. Initialize SDK -----
    // Call SDK init function to allocate resources and prepare hardware communication
    if (SYNC_Initialize() == 0) {
        // Get and print detailed error info on init failure
        fprintf(stderr, "Error: SYNC_Initialize failed: %s\n", SYNC_GetErrorText(SYNC_GetLastError()));
        return 1;                                      // Nonzero exit code indicates error
    }

    // ----- 2. Enumerate Devices -----
    // Probe all connected camera devices and fill deviceInfo array
    SyncDeviceInfo deviceInfo[SYNC_MAX_CAMERA_COUNT] = {};
    int deviceCount = 0;
    if (SYNC_EnumDevices(deviceInfo, SYNC_MAX_CAMERA_COUNT, &deviceCount) == 0) {
        fprintf(stderr, "Error: SYNC_EnumDevices failed: %s\n", SYNC_GetErrorText(SYNC_GetLastError()));
        SYNC_Release();                                // Release initialized SDK resources
        return 1;
    }

    // Print logical index, name, VID/PID of each existing device and whether exposure end time is used
    fprintf(stderr, "\n[Device Enumeration] Detected %d devices:\n", deviceCount);
    for (int i = 0; i < SYNC_MAX_CAMERA_COUNT; ++i) {
        if (deviceInfo[i].isExist) {
            // Determine whether to use exposure end time based on the 5th char of vidpid being 'abc'
            const char* vidpid = deviceInfo[i].vidpid;
            if (strlen(vidpid) >= 5) {
                char ch = vidpid[4];
                if (ch == 'a' || ch == 'b' || ch == 'c') {
                    g_useEndTime[deviceInfo[i].number] = false;   // Start time
                } else if (ch == 'd' || ch == 'e' || ch == 'f') {
                    g_useEndTime[deviceInfo[i].number] = true;    // End time
                }
            }
            fprintf(stderr, "  Device %u: %s, vidpid=%s, useEndTime=%d\n",
                    deviceInfo[i].number, deviceInfo[i].name, vidpid, g_useEndTime[deviceInfo[i].number] ? 1 : 0);
        }
    }
    // Sync capture requires at least 2 devices to proceed
    if (deviceCount < 2) {
        fprintf(stderr, "Error: At least 2 devices required for synchronized save\n");
        SYNC_Release();
        return 1;
    }

    // ----- 3. Query and Auto-Set Format -----
    std::vector<int> selectedIndices;
    selectedIndices.reserve(SYNC_MAX_CAMERA_COUNT);
    bool hasPidSuffix01 = false;       // Flag whether master device (PID suffix 01) is detected
    int FormatIndex = 0;
    int firstWidth = -1;
    int firstHeight = -1;
    int firstFps = -1;
    SyncCamFormat firstFmt = SYNC_FORMAT_MJPG;

    // 1. Get formats of all existing devices
    for (int i = 0; i < SYNC_MAX_CAMERA_COUNT; ++i) {
        if (!deviceInfo[i].isExist) continue;

        unsigned int hwPid = extract_hardware_pid(deviceInfo[i].vidpid);
        if (hwPid == 1) {
            hasPidSuffix01 = true;
        }
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
            fprintf(stderr, "Error: Device %u has no available formats\n", deviceInfo[i].number);
            SYNC_Release();
            return 1;
        }
        if (SYNC_SetDeviceFormat(deviceInfo[i].number, FormatIndex) == 0) {
            fprintf(stderr, "Error: Device %u format set failed: %s\n", deviceInfo[i].number, SYNC_GetErrorText(SYNC_GetLastError()));
            SYNC_Release();
            return 1;
        }

        // Check whether the set format matches previous device
        if (firstWidth == -1) {
            firstWidth = formats[FormatIndex].width;
            firstHeight = formats[FormatIndex].height;
            firstFps = formats[FormatIndex].fps;
            firstFmt = formats[FormatIndex].fmt;
        } else {
            if (formats[FormatIndex].width != firstWidth ||
                formats[FormatIndex].height != firstHeight ||
                formats[FormatIndex].fps != firstFps ||
                formats[FormatIndex].fmt != firstFmt) {
                fprintf(stderr, "Error: Device %u default format mismatch with previous device! Current: %dx%d %dfps %s, Prev: %dx%d %dfps %s\n",
                        deviceInfo[i].number,
                        formats[FormatIndex].width, formats[FormatIndex].height, formats[FormatIndex].fps, format_name(formats[FormatIndex].fmt),
                        firstWidth, firstHeight, firstFps, format_name(firstFmt));
                SYNC_Release();
                return 1;
            }
        }

        fprintf(stderr, "[Format Setup] Device %d (pid=%u) set to format: %dx%d %dfps\n",
                i, deviceInfo[i].number, formats[FormatIndex].width, formats[FormatIndex].height, formats[FormatIndex].fps);

        selectedIndices.push_back(deviceInfo[i].number);
    }

    // If no device with PID suffix 01 is found, cannot determine master device for unified format setup
    if (hasPidSuffix01 == false) {
        fprintf(stderr, "Error: Master device (PID suffix 01) not found, cannot determine sync baseline device\n");
        SYNC_Release();
        return 1;
    }

    // ----- 4. Automatic Device Selection -----
    // Check if device count meets sync capture requirement (at least 2)
    if (selectedIndices.size() < 2) {
        fprintf(stderr, "Error: Not enough devices available for sync capture (need at least 2). \n");
        SYNC_Release();
        return 1;
    }

    fprintf(stderr, "\n[Device Auto-Selection] Automatically selected all %zu devices:\n", selectedIndices.size());
    for (size_t i = 0; i < selectedIndices.size(); ++i) {
        int idx = selectedIndices[i];
        // Find deviceInfo element with logical device index equal to idx
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
            fprintf(stderr, "  Device %d: Unknown device, pid=%d\n", idx, idx);
        }
    }

    // Copy user-selected device list to global g_selectedIndices under lock
    {
        std::lock_guard<std::mutex> lock(g_queueMutex);
        g_selectedIndices = selectedIndices;
    }

    // ----- 5. Register Callback -----
    // Set image Callback Functions ImageCallbackFunction, called when SDK captures a new image group
    if (SYNC_SetImageCallback(ImageCallbackFunction, nullptr) == 0) {
        fprintf(stderr, "Error: SYNC_SetImageCallback failed: %s\n", SYNC_GetErrorText(SYNC_GetLastError()));
        SYNC_Release();
        return 1;
    }

    // ----- 6. Start Capture: perform a full screen clear before capture -----
    // All init info already went to stderr; stdout is clean for dynamic refresh of synced frame info
    printf("\033[2J\033[H");   // Clear screen and move cursor to top-left
    fflush(stdout);

    // ----- 7. Start Capture -----
    // Notify SDK to start capturing from the device list; capture runs in background with callbacks invoked continuously
    if (SYNC_StartCaptureByIndices(selectedIndices.data(), static_cast<int>(selectedIndices.size())) == 0) {
        fprintf(stderr, "Error: SYNC_StartCaptureByIndices failed: %s\n", SYNC_GetErrorText(SYNC_GetLastError()));
        SYNC_Release();
        return 1;
    }

    // Start key detection thread (independent thread to listen for 'p' key capture)
    std::thread keyThread(keypress_thread_func);

    // Wait for the first frame group (first refresh triggers automatically in callback)
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ----- 8. Wait for Ctrl+C -----
    // Main thread loops checking g_running flag; on SIGINT, 
    // signal_handler sets g_running to false and the loop exits. 
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Reduce CPU usage
    }

    // Wait for key thread to finish (ensure thread resources are reclaimed)
    keyThread.join();

    // ----- 9. Stop and Release -----
    fprintf(stderr, "\n[Stop] Stopping capture and releasing resources...\n");
    SYNC_StopCapture();      // Stop image capture of all devices
    SYNC_Release();          // Release all SDK resources (including callbacks, device handles, etc.)

    fprintf(stderr, "\nProgram exited normally.\n");
    return 0;                // Normal exit
}
