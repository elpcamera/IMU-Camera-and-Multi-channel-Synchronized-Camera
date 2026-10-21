#include "scamlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <chrono>
#include <thread>
#include <conio.h>
#include <windows.h>
#include <atomic>
#include <mutex>

const int IMU_DATA_COUNT = 11;               //Number of IMU data entries
const int IMU_START_LINE = 3;                    //Starting line for printing IMU data in the text file
const int SAVE_MSG_LINE = 14;                   //Number of text lines printed in the IMU region
const int EXIT_WAIT_MS = 500;                   //Exit wait time
const int CLOSE_DEVICE_WAIT_MS = 300;   //Timeout for closing a device
const int RELEASE_WAIT_MS = 200;             //Timeout for releasing SDK resources
const int POLL_INTERVAL_MS = 10;            //Timeout for waiting on the exit event

std::atomic<bool> g_saveFlag(false);    //Flag to save image
std::atomic<bool> g_running(true);      //Capture started flag
std::atomic<bool> g_paused(false);      //Pause flag
std::mutex g_mutex;                     //Mutex
CamData g_lastCamData;                  //Previous frame image
uint8_t* g_lastImageBuffer = nullptr;   //Previous frame image buffer
size_t g_lastImageBufferSize = 0;       //Previous frame image size
COORD g_imuStartPos = { 0, IMU_START_LINE };    //Console position where IMU printing starts
HANDLE g_exitEvent = nullptr;           //Exit event
HANDLE g_hConsole = nullptr;            //Console handle

//Handle events
BOOL WINAPI ConsoleHandler(DWORD dwCtrlType) {
    if (dwCtrlType == CTRL_CLOSE_EVENT ||
        dwCtrlType == CTRL_LOGOFF_EVENT ||
        dwCtrlType == CTRL_SHUTDOWN_EVENT) {
        if (g_exitEvent) {
            SetEvent(g_exitEvent);
        }
        g_running.store(false);
        return TRUE;
    }
    return TRUE;
}
//Save image
void saveFrameData(const CamData* camData) {
    if (!camData) {
        return;
    }

    auto now = std::chrono::system_clock::now();
    auto sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
    time_t tt = sec.count();
    struct tm localTime;
    localtime_s(&localTime, &tt);

    if (!CreateDirectoryA("pictrues", NULL) && GetLastError() != ERROR_ALREADY_EXISTS) {
        return;
    }

    char timeStr[64];
    sprintf(timeStr, "%04d%02d%02d_%02d%02d%02d",
        localTime.tm_year + 1900, localTime.tm_mon + 1, localTime.tm_mday,
        localTime.tm_hour, localTime.tm_min, localTime.tm_sec);

    char folderName[512];
    sprintf(folderName, "pictrues/%s", timeStr);
    if (!CreateDirectoryA(folderName, NULL) && GetLastError() != ERROR_ALREADY_EXISTS) {
        return;
    }

    char jpgFileName[512];
    sprintf(jpgFileName, "%s/%s.jpg", folderName, timeStr);
    SCAM_SaveToJPG(camData->data, camData->format, camData->width, camData->height, jpgFileName, 95);

    char txtFileName[512];
    sprintf(txtFileName, "%s/%s.txt", folderName, timeStr);
    FILE* txtFile = nullptr;
    errno_t err = fopen_s(&txtFile, txtFileName, "w");
    if (err != 0 || !txtFile) {
        return;
    }

    fprintf(txtFile, "width=%d\n", camData->width);
    fprintf(txtFile, "height=%d\n", camData->height);
    fprintf(txtFile, "bufSize=%d\n", camData->bufSize);
    fprintf(txtFile, "format=NV12\n");
    fprintf(txtFile, "startExpouseTime=%llu us\n", camData->startExpouseTime);
    fprintf(txtFile, "endExpouseTime=%llu us\n", camData->endExpouseTime);
    fprintf(txtFile, "\n");
    fprintf(txtFile, "IMU Data (Acc + Gyro):\n");
    fprintf(txtFile, "| Index | Acc(X) | Acc(Y) | Acc(Z) | Gyro(X) | Gyro(Y) | Gyro(Z) | Timestamp |\n");
    fprintf(txtFile, "|------|-----------|-----------|-----------|-----------|-----------|-----------|--------|\n");
    for (int i = 0; i < IMU_DATA_COUNT; i++) {
        fprintf(txtFile, "| %2d |%12.5f |%12.5f |%12.5f |%10.5f |%10.5f |%10.5f |%llu |\n",
            i + 1,
            camData->imu_data[i].fAccData_X, camData->imu_data[i].fAccData_Y,
            camData->imu_data[i].fAccData_Z, camData->imu_data[i].fGyroData_X,
            camData->imu_data[i].fGyroData_Y, camData->imu_data[i].fGyroData_Z,
            camData->imu_data[i].uTime);
    }
    fprintf(txtFile, "\n");
    fprintf(txtFile, "=========================================\n");
    fclose(txtFile);

    COORD saveMsgPos = { 0, SAVE_MSG_LINE };
    if (g_hConsole && g_hConsole != INVALID_HANDLE_VALUE) {
        SetConsoleCursorPosition(g_hConsole, saveMsgPos);
    }
    fflush(stdout);
}

//Callback function
void onImageReceived(const CamData* image, void* userData) {
    if (!g_running.load()) {
        return;
    }
    if (g_paused.load()) {
        return;
    }
    CamData* camData = const_cast<CamData*>(image);

    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (!g_running.load() || g_paused.load()) {
            return;
        }
        g_lastCamData = *camData;
        if (g_lastImageBufferSize < camData->bufSize) {
            uint8_t* newBuffer = (uint8_t*)realloc(g_lastImageBuffer, camData->bufSize);
            if (newBuffer) {
                g_lastImageBuffer = newBuffer;
                g_lastImageBufferSize = camData->bufSize;
            }
        }
        if (g_lastImageBuffer) {
            memcpy(g_lastImageBuffer, camData->data, camData->bufSize);
        }
    }

    if (g_hConsole && g_hConsole != INVALID_HANDLE_VALUE) {
        SetConsoleCursorPosition(g_hConsole, g_imuStartPos);
    }

    printf("width=%d\n", camData->width);
    printf("height=%d\n", camData->height);
    printf("bufSize=%d\n", camData->bufSize);
    printf("format=NV12\n");
    printf("startExpouseTime=%llu us  endExpouseTime=%llu us\n",
        camData->startExpouseTime, camData->endExpouseTime);
    for (int i = 0; i < IMU_DATA_COUNT; i++) {
        fprintf(stdout,
            "IMU [1/1000(g)]AccData X,Y,Z (%12.5f, %12.5f, %12.5f) GyroData[1/1000(dps)] X,Y,Z(%10.5f, %10.5f, %10.5f) sampling time[%lu us]       \n",
            camData->imu_data[i].fAccData_X,
            camData->imu_data[i].fAccData_Y,
            camData->imu_data[i].fAccData_Z,
            camData->imu_data[i].fGyroData_X,
            camData->imu_data[i].fGyroData_Y,
            camData->imu_data[i].fGyroData_Z,
            camData->imu_data[i].uTime);
    }
    printf("\n");
    fflush(stdout);

    if (g_saveFlag.load() && g_running.load()) {
        saveFrameData(camData);
        g_saveFlag.store(false);
    }
}

int main() {
    g_exitEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (!g_exitEvent) {
        printf("Failed to create exit event\n");
        return -1;
    }

    g_hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    if (g_hConsole == INVALID_HANDLE_VALUE) {
        printf("Failed to get console handle\n");
        CloseHandle(g_exitEvent);
        return -1;
    }

    SetConsoleCtrlHandler(ConsoleHandler, TRUE);
    //1. Initialize SDK internal resources
    int ret = SCAM_Initialize();
    if (ret != 0) {
        printf("Failed to initialize SDK: %d\n", ret);
        CloseHandle(g_exitEvent);
        return -1;
    }

    //2. Enumerate devices
    DeviceInfo devices[4];
    int deviceCount = 0;
    ret = SCAM_EnumDevices(devices, 4, &deviceCount);
    if (ret != 0) {
        printf("Failed to enum devices: %d\n", ret);
        SCAM_Release();
        CloseHandle(g_exitEvent);
        return -1;
    }
    //3. Load the formats supported by the device
    for (int i = 0; i < deviceCount; i++) {
        FormatInfo mainDeviceformat[100];
        int formatCount = 0;
        if (-1 == SCAM_GetDeviceFormats(i, mainDeviceformat, 100, &formatCount)) {
            printf("get device %d formats error!\n", i);
        }
    }
    //4. Set the format of the first device to format 1
    SCAM_SetDeviceFormat(0, 0);
    //5. Set the image format
    SCAM_SetImageFormat(0, CamFormat::FORMAT_NV12);
    //6. Open the first device
    ret = SCAM_OpenDevice(0, onImageReceived);
    if (ret != 0) {
        printf("Failed to open device 0: %d\n", ret);
        SCAM_Release();
        CloseHandle(g_exitEvent);
        return -1;
    }

    printf("Press S/s to save current frame and IMU data, press T/t to pause, press K/k to resume, press ESC to exit\n");
    printf("------------------------------------------------------------\n");

    while (true) {
        if (_kbhit()) {
            int key = _getch();
            if (key == 'S' || key == 's') {//Save the current frame
                g_saveFlag.store(true);
                printf("save pictrues successful.\n");
            }
            else if (key == 'T' || key == 't') {//Pause frame capture
                g_paused.store(true);
                printf("Paused. Press K/k to resume.\n");
            }
            else if (key == 'K' || key == 'k') {//Resume frame capture
                g_paused.store(false);
                printf("Resumed.\n");
            }
            else if (key == 27) {//Exit the program
                g_running.store(false);
                g_paused.store(false);
                std::this_thread::sleep_for(std::chrono::milliseconds(EXIT_WAIT_MS));
                break;
            }
        }

        //Wait for the exit event signal
        DWORD waitResult = WaitForSingleObject(g_exitEvent, POLL_INTERVAL_MS);
        if (waitResult == WAIT_OBJECT_0) {
            g_running.store(false);
            g_paused.store(false);
            std::this_thread::sleep_for(std::chrono::milliseconds(EXIT_WAIT_MS));
            break;
        }
    }

    //7. Close the first device
    SCAM_CloseDevice(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(CLOSE_DEVICE_WAIT_MS));
    //8. Release SDK internal resources
    SCAM_Release();
    std::this_thread::sleep_for(std::chrono::milliseconds(RELEASE_WAIT_MS));
    //Free memory
    if (g_lastImageBuffer) {
        free(g_lastImageBuffer);
        g_lastImageBuffer = nullptr;
        g_lastImageBufferSize = 0;
    }
    if (g_exitEvent) {
        CloseHandle(g_exitEvent);
        g_exitEvent = nullptr;
    }
    SetConsoleCtrlHandler(ConsoleHandler, FALSE);
    return 0;
}
