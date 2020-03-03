cmake_minimum_required(VERSION 3.10)
project(Omnimagnet_Project)
find_package( OpenCV REQUIRED )

add_executable(ImageCap ImageCapture.cpp)

# INCLUDE_DIRECTORIES("/home/ashkan/Desktop/naghi_C/build/")
INCLUDE_DIRECTORIES(/home/ashkan/eigen/source , /usr/local/lib, /usr/include/spinnaker)

# target_link_libraries(pigi PRIVATE "/home/ashkan/Desktop/naghi_C/build/libghazam.a" )
target_link_libraries(ImageCap "/usr/lib/libSpinnaker.so"  ${OpenCV_LIBS})
# target_link_libraries(SystemRun PRIVATE omnimagnet type "/usr/local/lib/libcomedi.a")

add_executable(CameraCalibration camera_calibration.cpp)

target_link_libraries(CameraCalibration "/usr/lib/libSpinnaker.so"  ${OpenCV_LIBS})
