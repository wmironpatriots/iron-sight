#include "src/localization/AprilTagDetector.h"
#include <apriltag/apriltag.h>
#include "tag36h11.h"
#include "src/utils/PCH.h"

namespace localization {
    auto AprilTagDetector::detectTags(camera::TimestampedFrame& tframe){
        apriltag_family_t *tf = tag36h11_create();
        apriltag_detector_t *td = apriltag_detector_create();
        apriltag_detector_add_family(td, tf);
        td->quad_decimate = 2.0;  
        td->quad_sigma = 0.0;
        td->nthreads = 4;
        cv::Mat frame = tframe->frame;
        image_u8_t image = {frame.cols, frame.rows, frame.cols, frame.data};
        zarray_t *detections = apriltag_detector_detect(td, &image);


    }

}