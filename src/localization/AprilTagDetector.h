#include <apriltag/apriltag.h>
#include "src/utils/PCH.h"

namespace localization {
    class AprilTagDetector {
        public:
            virtual ~AprilTagDetector() = default;
            virtual auto getDetectedTags() -> std::vector<apriltag_detection_t*>;
    };
}