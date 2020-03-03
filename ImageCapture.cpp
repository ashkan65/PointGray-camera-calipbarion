#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <sstream>
// #include <ncurses>
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;
#ifdef _DEBUG
// Disables heartbeat on GEV cameras so debugging does not incur timeout errors
int DisableHeartbeat(INodeMap& nodeMap, INodeMap& nodeMapTLDevice)
{
    cout << "Checking device type to see if we need to disable the camera's heartbeat..." << endl << endl;
    //
    // Write to boolean node controlling the camera's heartbeat
    //
    // *** NOTES ***
    // This applies only to GEV cameras and only applies when in DEBUG mode.
    // GEV cameras have a heartbeat built in, but when debugging applications the
    // camera may time out due to its heartbeat. Disabling the heartbeat prevents
    // this timeout from occurring, enabling us to continue with any necessary debugging.
    // This procedure does not affect other types of cameras and will prematurely exit
    // if it determines the device in question is not a GEV camera.
    //
    // *** LATER ***
    // Since we only disable the heartbeat on GEV cameras during debug mode, it is better
    // to power cycle the camera after debugging. A power cycle will reset the camera
    // to its default settings.
    //
    CEnumerationPtr ptrDeviceType = nodeMapTLDevice.GetNode("DeviceType");
    if (!IsAvailable(ptrDeviceType) || !IsReadable(ptrDeviceType))
    {
        cout << "Error with reading the device's type. Aborting..." << endl << endl;
        return -1;
    }
    else
    {
        if (ptrDeviceType->GetIntValue() == DeviceType_GEV)
        {
            cout << "Working with a GigE camera. Attempting to disable heartbeat before continuing..." << endl << endl;
            CBooleanPtr ptrDeviceHeartbeat = nodeMap.GetNode("GevGVCPHeartbeatDisable");
            if (!IsAvailable(ptrDeviceHeartbeat) || !IsWritable(ptrDeviceHeartbeat))
            {
                cout << "Unable to disable heartbeat on camera. Continuing with execution as this may be non-fatal..."
                     << endl
                     << endl;
            }
            else
            {
                ptrDeviceHeartbeat->SetValue(true);
                cout << "WARNING: Heartbeat on GigE camera disabled for the rest of Debug Mode." << endl;
                cout << "         Power cycle camera when done debugging to re-enable the heartbeat..." << endl << endl;
            }
        }
        else
        {
            cout << "Camera does not use GigE interface. Resuming normal execution..." << endl << endl;
        }
    }
    return 0;
}
#endif

int AcquireImages(CameraPtr pCam, INodeMap& nodeMap, INodeMap& nodeMapTLDevice, int i)
{
    int result = 0;
    try
    {
        // Errors (in case camera doesn't work)
        CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
        if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
        {
            cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << endl << endl;
            return -1;
        }
        // Retrieve entry node from enumeration node
        CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
        if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
        {
            cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..." << endl << endl;
            return -1;
        }
        // Retrieve integer value from entry node
        const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
        // Set integer value from entry node as new value of enumeration node
        ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
        cout << "Acquisition mode set to continuous..." << endl;
#ifdef _DEBUG
        cout << endl << endl << "*** DEBUG ***" << endl << endl;
        // If using a GEV camera and debugging, should disable heartbeat first to prevent further issues
        if (DisableHeartbeat(nodeMap, nodeMapTLDevice) != 0)
        {
            return -1;
        }
        cout << endl << endl << "*** END OF DEBUG ***" << endl << endl;
#endif
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Begin acquiring images
        // !!!!!!!!!!!!!!!!!!!!!!!!   Image acquisition must be ended when no more images are needed. !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        pCam->BeginAcquisition();
        gcstring deviceSerialNumber("");
        CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
        if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
        {
            deviceSerialNumber = ptrStringSerial->GetValue();
        }
        // Retrieve, convert, and save images
        const unsigned int k_numImages = 10;
        
        try
        {

            ImagePtr pResultImage = pCam->GetNextImage();
            // Something is wrong with the camera
            if (pResultImage->IsIncomplete())
            {
                // Retrieve and print the image status description
                cout << "Image incomplete: " << Image::GetImageStatusDescription(pResultImage->GetImageStatus())
                     << "..." << endl
                     << endl;
            }
            else
            {
                // The camera worked
                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // YOUR CODE GOES HERE!//
                // The captured image is pResultImage, it is in Spinnaker format and yuou have to convert it to mat for open cv!
                const size_t width = pResultImage->GetWidth();
                const size_t height = pResultImage->GetHeight();
                // converting the image
                // ImagePtr convertedImage = pResultImage->Convert(PixelFormat_Mono8, HQ_LINEAR);
                // convert to Opencv format ()
                // These are the options for quality and speed!
                // Return right one
                // ~ 1.3 ms but pixeled
                // return imagePtr->Convert(Spinnaker::PixelFormat_BGR8, Spinnaker::DEFAULT);
                // ~0.5 ms but BW
                // return imagePtr->Convert(Spinnaker::PixelFormat_BGR8, Spinnaker::NO_COLOR_PROCESSING);
                // ~6 ms, looks as good as best
                // return imagePtr->Convert(Spinnaker::PixelFormat_BGR8, Spinnaker::HQ_LINEAR);
                // ~2.2 ms default << edge << best
                // return imagePtr->Convert(Spinnaker::PixelFormat_BGR8, Spinnaker::EDGE_SENSING);
                // ~115, too slow
                // return imagePtr->Convert(Spinnaker::PixelFormat_BGR8, Spinnaker::RIGOROUS);
                // ~1.7 ms, slightly worse than HQ_LINEAR
                // return imagePtr->Convert(Spinnaker::PixelFormat_BGR8, Spinnaker::IPP);
                // ~30 ms, ideally best quality?
                // return imagePtr->Convert(Spinnaker::PixelFormat_BGR8, Spinnaker::DIRECTIONAL_FILTER);

                // ImagePtr convertedImage2 = pResultImage->Convert(PixelFormat_BGR8, NEAREST_NEIGHBOR);

                ImagePtr convertedImage2 = pResultImage->Convert(PixelFormat_BGR8, Spinnaker::IPP);

                unsigned int XPadding = convertedImage2->GetXPadding();
                unsigned int YPadding = convertedImage2->GetYPadding();
                unsigned int rowsize = convertedImage2->GetWidth();
                unsigned int colsize = convertedImage2->GetHeight();
                //image data contains padding. When allocating Mat container size, you need to account for the X,Y image data padding. 
                cv::Mat inputImage = cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC3, convertedImage2->GetData(), convertedImage2->GetStride());
                std::vector<int> markerIds;
                std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
                cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
                cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
                cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
                cv::Mat outputImage = inputImage.clone();
                cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

                cv::Mat cameraMatrix, distCoeffs;
                std::vector<cv::Vec3d> rvecs, tvecs;
                cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

                cv::imwrite(std::to_string(i)+".png", inputImage);
                int V;
                std::cin>>V;

                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                
                //
                // Release image
                //
                // *** NOTES ***
                // Images retrieved directly from the camera (i.e. non-converted
                // images) need to be released in order to keep from filling the
                // buffer.
                //
                pResultImage->Release();
                cout << endl;
            }
        }
        catch (Spinnaker::Exception& e)
        {
            cout << "Error: " << e.what() << endl;
            result = -1;
        }
        
        //
        // End acquisition
        //
        // *** NOTES ***
        // Ending acquisition appropriately helps ensure that devices clean up
        // properly and do not need to be power-cycled to maintain integrity.
        //
        pCam->EndAcquisition();
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return result;
}
// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int PrintDeviceInfo(INodeMap& nodeMap)
{
    cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;
    try
    {
        FeatureList_t features;
        const CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
        if (IsAvailable(category) && IsReadable(category))
        {
            category->GetFeatures(features);
            for (auto it = features.begin(); it != features.end(); ++it)
            {
                const CNodePtr pfeatureNode = *it;
                cout << pfeatureNode->GetName() << " : ";
                CValuePtr pValue = static_cast<CValuePtr>(pfeatureNode);
                cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
                cout << endl;
            }
        }
        else
        {
            cout << "Device control information not available." << endl;
        }
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return 0;
}
// This function acts as the body of the example; please see NodeMapInfo example
// for more in-depth comments on setting up cameras.
int RunSingleCamera(CameraPtr pCam)
{
    int result;
    try
    {
        // Retrieve TL device nodemap and print device information
        INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
        result = PrintDeviceInfo(nodeMapTLDevice);
        // Initialize camera
        pCam->Init();
        // Retrieve GenICam nodemap
        INodeMap& nodeMap = pCam->GetNodeMap();
        // Acquire images
        for (int i = 1; i<40;i++){      //////////////////////////////////////////////////////  i =  How amny images to generate:
            std::cin.ignore();
            result = result | AcquireImages(pCam, nodeMap, nodeMapTLDevice, i);
             // _getch();
        }
        // Deinitialize camera
        pCam->DeInit();
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }
    return result;
}
// Example entry point; please see Enumeration example for more in-depth
// comments on preparing and cleaning up the system.
int main(int /*argc*/, char** /*argv*/)
{
    // Since this application saves images in the current folder
    // we must ensure that we have permission to write to this folder.
    // If we do not have permission, fail right away.
    FILE* tempFile = fopen("test.txt", "w+");
    if (tempFile == nullptr)
    {
        cout << "Failed to create file in current folder.  Please check "
                "permissions."
             << endl;
        cout << "Press Enter to exit..." << endl;
        getchar();
        return -1;
    }
    fclose(tempFile);
    remove("test.txt");
    // Print application build information
    //cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;
    
    // Retrieve singleton reference to system object
    SystemPtr system = System::GetInstance();
    
    // Retrieve list of cameras from the system
    CameraList camList = system->GetCameras();
    const unsigned int numCameras = camList.GetSize();
    cout << "Number of cameras detected: " << numCameras << endl << endl;
    // Finish if there are no cameras
    if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();
        // Release system
        system->ReleaseInstance();
        cout << "Not enough cameras!" << endl;
        cout << "Done! Press Enter to exit..." << endl;
        getchar();
        return -1;
    }
    //
    // Create shared pointer to camera
    //
    // *** NOTES ***
    // The CameraPtr object is a shared pointer, and will generally clean itself
    // up upon exiting its scope. However, if a shared pointer is created in the
    // same scope that a system object is explicitly released (i.e. this scope),
    // the reference to the shared point must be broken manually.
    //
    // *** LATER ***
    // Shared pointers can be terminated manually by assigning them to nullptr.
    // This keeps releasing the system from throwing an exception.
    //
    CameraPtr pCam = nullptr;
    int result = 0;
    // Run example on each camera
    for (unsigned int i = 0; i < numCameras; i++)
    {
        // Select camera
        pCam = camList.GetByIndex(i);
        cout << endl << "Running example for camera " << i << "..." << endl;
        // Run example
        result = result | RunSingleCamera(pCam);
        cout << "Camera " << i << " example complete..." << endl << endl;
    }
    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::drawMarker(dictionary, 23, 200, markerImage, 1);
    // cv::imwrite("marker23.png", markerImage);
    //
    // Release reference to the camera
    //
    // *** NOTES ***
    // Had the CameraPtr object been created within the for-loop, it would not
    // be necessary to manually break the reference because the shared pointer
    // would have automatically cleaned itself up upon exiting the loop.
    //
    pCam = nullptr;
    // Clear camera list before releasing system
    camList.Clear();
    // Release system
    system->ReleaseInstance();
    cout << endl << "Done! Press Enter to exit..." << endl;
    getchar();
    return result;
}