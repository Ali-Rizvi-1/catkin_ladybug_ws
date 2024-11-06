#include <iostream>
#include "ladybug.h"

int main() {
    LadybugContext context;
    LadybugError error;
    
    error = ladybugCreateContext(&context);
    if (error != LADYBUG_OK) {
        std::cout << "Failed to create Ladybug context: " << 
                     ladybugErrorToString(error) << std::endl;
        return -1;
    }

    unsigned int numCameras = 16;
    LadybugCameraInfo camInfo[16];
    error = ladybugBusEnumerateCameras(context, camInfo, &numCameras);
    
    std::cout << "Number of cameras found: " << numCameras << std::endl;
    
    return 0;
}