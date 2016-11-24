add_executable(ass2_ar ARDrawingContext.cpp
ARDrawingContext.hpp
CameraInfo.cpp
CameraInfo.hpp
GeometryTypes.cpp
GeometryTypes.hpp
main.cpp
ARPipeline.hpp
ARPipeline.cpp
Pattern.cpp
Pattern.hpp
PatternDetector.cpp
PatternDetector.hpp
DebugHelpers.hpp
        macro.h)

target_link_libraries( ass2_ar ${OpenCV_LIBRARIES} )
target_link_libraries( ass2_ar ${OPENGL_LIBRARIES} )
     
install (TARGETS ass2_ar DESTINATION bin)
