This demo captures video data from the cameras, prints the related information to the console, and saves one synchronized frame's data to a file when a key is pressed.
Note: the current source code opens the first two devices. If you want to change the number of opened devices, set required_dev_count in main.cpp to the desired number;
If you want to open specific devices, call SYNC_OpenDevice directly to open the corresponding devices

How to compile:
1. Build the project with the VS2022 development tools (refer to "Setting up the development environment" in the synclib2.2.3 documentation)
2. Compile from the command line as follows:
Open the VS terminal tool "x64 Native Tools Command Prompt for VS 2022" and run the following commands in order
1. Go to the directory containing main.cpp
cd C:/Users/Administrator/Desktop/222 (replace with the actual path)
2. Compile main.cpp (/EHsc enables C++ standard exceptions, /std:c++17 uses the C++17 standard, the C++17 standard is required, -I specifies the static library to link)
cl /EHsc /std:c++17 main.cpp ^ -I synccap/include ^ /link ^ synccap/lib/x64/synccap.lib
3. After a successful build, main.exe is generated in the same directory as main.cpp. Copy the dynamic libraries required by the program into this directory, then double-click main.cpp to run it.

The release directory contains pre compiled sample programs. Double click on main.exe to run them

Tip: 
1.If the program prompts for missing VC++libraries during runtime, place the two dynamic libraries in the VC++Library directory in the same directory as the executable program
2.The SDK is compiled using the Visual Studio 2022 (v143) toolset. It is recommended to use the same toolset to compile the program during development when using the SDK.
3.If the program uses this SDK during packaging, please do not use Enigma Virtual Box as a packaging tool