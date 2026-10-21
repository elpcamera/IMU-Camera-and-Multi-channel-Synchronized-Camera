#include <cstdio>
#include <cstdint>
#include <cstring>
#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <csignal>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <ctime>
#include <vector>
#include "scamlib.h"

// ======================== Utility Functions ========================
static const char* format_name(CamFormat fmt) {
    switch (fmt) {
    case FORMAT_MJPG:   return "MJPG";
    case FORMAT_YUV422: return "YUV422";
    case FORMAT_RGB24:  return "RGB24";
    case FORMAT_NV12:   return "NV12";
    default:            return "UNKNOWN";
    }
}

// ======================== Global Variables ========================
std::atomic<bool> g_running{true};          // Program running flag
std::mutex g_frameMutex;                   // Protect latest frame
std::mutex g_printMutex;                  // Protect terminal output
static int g_targetDevice = -1;            // Target device logical ID

// Store deep copy of latest frame
struct StoredFrame {
    std::vector<uint8_t> buffer;
    CamData meta;
};
StoredFrame g_latestFrame;
bool g_hasFrame = false;                  // Whether a frame has been received
static bool g_useEndTime = false;   // true=useEnd time, false=useStart time
// ======================== Display Functions ========================
// Print current frame info to screen (dynamic refresh, overwriting previous)
static void display_frame(const StoredFrame& frame) {
    std::lock_guard<std::mutex> lock(g_printMutex);

    const CamData& meta = frame.meta;
    // Move cursor to top-left for overwrite refresh
    printf("\033[H");

    float fps = SCAM_GetFrameRate(g_targetDevice);
    printf("Camera %d: %s %dx%d, size=%d bytes, fps=%.2f\n",
           g_targetDevice,
           format_name(meta.format),
           meta.width, meta.height,
           meta.bufSize,
           fps);
    printf("exposure: start=%12lu us end=%12lu us, duration=%8lu us\n",
           meta.startExpouseTime, meta.endExpouseTime,
           meta.endExpouseTime - meta.startExpouseTime);

    // Print all 11 sets of IMU data
    for (int imu_idx = 0; imu_idx < 11; ++imu_idx) {
        const sICM42688_XYZ_float& imu = meta.imu_data[imu_idx];
        printf("[g] 0x%X Acc=(%12.6f,%12.6f,%12.6f) Gyro=(%12.6f,%12.6f,%12.6f) time=%11lu us\n",
               imu_idx,
               imu.fAccData_X, imu.fAccData_Y, imu.fAccData_Z,
               imu.fGyroData_X, imu.fGyroData_Y, imu.fGyroData_Z,
               imu.uTime);
    }
    printf("\n");
    fflush(stdout);
}

// ======================== Save Functions ========================
// Save current frame as JPG and write to text file
static void save_current_frame() {
    std::lock_guard<std::mutex> lock(g_frameMutex);
    if (!g_hasFrame) {
        fprintf(stderr, "\n[Warning] No frame received yet, cannot save\n");
        fflush(stderr);
        return;
    }

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
        return;
    }

    char jpgPath[512];
    snprintf(jpgPath, sizeof(jpgPath), "%s/%s.jpg", dirPath, timeStr);

    const CamData& meta = g_latestFrame.meta;
    printf("\n[Save] Saving image to %s ...\n", jpgPath);
    bool ret = SCAM_SaveToJPG(g_latestFrame.buffer.data(), meta.format, meta.width, meta.height, jpgPath, 90);

    if (ret) {
        char txtPath[512];
        snprintf(txtPath, sizeof(txtPath), "%s/%s.txt", dirPath, timeStr);

        FILE* fp = fopen(txtPath, "w");
        if (fp) {
            fprintf(fp, "[camera_1]\n");
            fprintf(fp, "width=%d\n", meta.width);
            fprintf(fp, "height=%d\n", meta.height);
            fprintf(fp, "bufSize=%d\n", meta.bufSize);
            fprintf(fp, "format=%s\n", format_name(meta.format));
            fprintf(fp, "startExpouseTime=%lu\n", meta.startExpouseTime);
            fprintf(fp, "endExpouseTime=%lu\n", meta.endExpouseTime);
            fprintf(fp, "duration=%lu us\n", meta.endExpouseTime - meta.startExpouseTime);
            fprintf(fp, "deviceIdx=%d\n", g_targetDevice);
            fprintf(fp, "\n");

            fprintf(fp, "+--------+-----------------+-----------------+-----------------+-----------------+-----------------+-----------------+--------------------+\n");
            fprintf(fp, "| Index  | Acceleration(X) | Acceleration(Y) | Acceleration(Z) | Gyroscope(X)    | Gyroscope(Y)    | Gyroscope(Z)    | Timestamp          |\n");
            fprintf(fp, "+--------+-----------------+-----------------+-----------------+-----------------+-----------------+-----------------+--------------------+\n");

            for (int imuIndex = 0; imuIndex < 11; ++imuIndex) {
                const sICM42688_XYZ_float& imu = meta.imu_data[imuIndex];
                fprintf(fp,
                        "| %-6d | %15.6f | %15.6f | %15.6f | %15.6f | %15.6f | %15.6f | %18lu |\n",
                        imuIndex + 1,
                        imu.fAccData_X, imu.fAccData_Y, imu.fAccData_Z,
                        imu.fGyroData_X, imu.fGyroData_Y, imu.fGyroData_Z,
                        imu.uTime);
                if (imuIndex != 10)
                    fprintf(fp, "+--------+-----------------+-----------------+-----------------+-----------------+-----------------+-----------------+--------------------+\n");
            }
            fprintf(fp, "+--------+-----------------+-----------------+-----------------+-----------------+-----------------+-----------------+--------------------+\n");
            fprintf(fp, "\n");

            fclose(fp);
            printf("[Save] imageSavesuccess!\n");
            printf("[Save] Text file saved: %s\n", txtPath);
        } else {
            printf("[Save] Image saved, but text file creation failed!\n");
        }
    } else {
        printf("[Save] Save failed: %s\n", SCAM_GetErrorText(SCAM_GetLastError()));
    }
    fflush(stdout);
}

// ======================== Keyboard Detection Thread ========================
static void keypress_thread_func() {
    // Set terminal to raw mode
    struct termios orig_termios, raw_termios;
    tcgetattr(STDIN_FILENO, &orig_termios);
    raw_termios = orig_termios;
    raw_termios.c_lflag &= ~(ECHO | ICANON);
    raw_termios.c_lflag |= ISIG;           // Keep Ctrl+C signal
    raw_termios.c_cc[VMIN] = 1;
    raw_termios.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw_termios);

    while (g_running) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        struct timeval tv = {0, 100000};   // 100ms timeout
        int ret = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &tv);
        if (ret > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
            char ch;
            if (read(STDIN_FILENO, &ch, 1) == 1 && ch == ' ') {
                save_current_frame();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

// ======================== Signal Handling ========================
static void signal_handler(int /*sig*/) {
    g_running = false;
    // Restore terminal (prevent raw mode residue)
    system("stty cooked echo");
    fprintf(stderr, "\nReceived termination signal, exiting...\n");
    fflush(stderr);
}

// ======================== Callback Functions ========================
void OnFrameCaptured(const CamData* image, void* userData) {
    if (!image || !g_running) return;

    // Deep copy data
    {
        std::lock_guard<std::mutex> lock(g_frameMutex);
        g_latestFrame.buffer.assign(image->data, image->data + image->bufSize);
        g_latestFrame.meta = *image;
        g_latestFrame.meta.data = g_latestFrame.buffer.data();
        g_hasFrame = true;
    }

    // Display current frame (refresh screen)

    display_frame(g_latestFrame);
}
int main() {
    // Register signals
    std::signal(SIGINT, signal_handler);

    // Clear screen and initialize display
    printf("\033[2J\033[H");
    printf("--- SCAM Single Camera Live Capture Demo (Press Space to save frame) ---\n");
    printf("SDK Version: %s\n", SCAM_GetVersion());
    fflush(stdout);

    // 1. Initialize SDK
    if (!SCAM_Initialize()) {
        fprintf(stderr, "Error: SCAM_Initialize failed: %s\n", SCAM_GetErrorText(SCAM_GetLastError()));
        return 1;
    }

    // 2. Enumerate Devices
    DeviceInfo devices[10];
    memset(devices, 0, sizeof(devices));
    int count = 0;
    if (!SCAM_EnumDevices(devices, 10, &count)) {
        fprintf(stderr, "Error: SCAM_EnumDevices failed: %s\n", SCAM_GetErrorText(SCAM_GetLastError()));
        SCAM_Release();
        return 1;
    }

    g_targetDevice = -1;
    char targetVidPid[32] = {0};   // Added: Store target device VIDPID

    printf("Detected %d devices:\n", count);
    for (int i = 0; i < count; ++i) {
        if (devices[i].isExist) {
            printf("  DeviceLogical ID %d: %s, VIDPID: %s\n", devices[i].number, devices[i].name, devices[i].vidpid);
            if (g_targetDevice == -1) {
                g_targetDevice = devices[i].number;
                // Added: Copy VIDPID
                strncpy(targetVidPid, devices[i].vidpid, sizeof(targetVidPid) - 1);
                targetVidPid[sizeof(targetVidPid) - 1] = '\0';
            }
        }
    }
    if (g_targetDevice == -1) {
        fprintf(stderr, "Error: No available camera found\n");
        SCAM_Release();
        return 1;
    }
    printf("Selected device index: %d\n", g_targetDevice);

    // ===== Added: Set time baseline based on VIDPID =====
    if (strlen(targetVidPid) >= 5) {
        char ch = targetVidPid[4];
        if (ch == 'a' || ch == 'b' || ch == 'c')
            g_useEndTime = false;   // Start time
        else if (ch == 'd' || ch == 'e' || ch == 'f')
            g_useEndTime = true;    // End time
        // Other characters keep default false
    }
    printf("Device VIDPID: %s, Time baseline: %s\n", targetVidPid,
           g_useEndTime ? "Exposure end time" : "Exposure start time");
    // ===== End of Addition =====

    fflush(stdout);

    // 3. Get and set format
    FormatInfo formats[100];
    int formatCount = 0;
    if (!SCAM_GetDeviceFormats(g_targetDevice, formats, 100, &formatCount)) {
        fprintf(stderr, "Error: SCAM_GetDeviceFormats failed: %s\n", SCAM_GetErrorText(SCAM_GetLastError()));
        SCAM_Release();
        return 1;
    }
    printf("Supported formats:\n");
    for (int i = 0; i < formatCount; ++i) {
        printf("  [%d] %s %dx%d %dfps\n", i, format_name(formats[i].fmt),
               formats[i].width, formats[i].height, formats[i].fps);
    }
    if (formatCount == 0) {
        fprintf(stderr, "Error: No available formats\n");
        SCAM_Release();
        return 1;
    }
    // Select first format
    int fmtIdx = 0;
    if (!SCAM_SetDeviceFormat(g_targetDevice, fmtIdx)) {
        fprintf(stderr, "Error: SCAM_SetDeviceFormat failed: %s\n", SCAM_GetErrorText(SCAM_GetLastError()));
        SCAM_Release();
        return 1;
    }
    printf("Format index set %d (%dx%d)\n", fmtIdx, formats[fmtIdx].width, formats[fmtIdx].height);

    // 4. setoutputimageformat(NV12)
    SCAM_SetImageFormat(g_targetDevice, FORMAT_NV12);

    // 5. Open device and register callback
    printf("\033[2J\033[H");
    if (!SCAM_OpenDevice(g_targetDevice, OnFrameCaptured, nullptr)) {
        fprintf(stderr, "Error: SCAM_OpenDevice failed: %s\n", SCAM_GetErrorText(SCAM_GetLastError()));
        SCAM_Release();
        return 1;
    }
    printf("Camera opened, receiving frames...\n");
    fflush(stdout);

    // 6. Start key detection thread
    std::thread keyThread(keypress_thread_func);

    // 7. Main loop waits for exit signal
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 8. Wait for key thread to finish
    keyThread.join();

    // 9. Close device and release resources
    printf("\nClosing device...\n");
    SCAM_CloseDevice(g_targetDevice);
    SCAM_Release();
    printf("Program exited normally.\n");
    return 0;
}
