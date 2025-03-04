#include <librealsense2/rs.hpp>
#include <iostream>

int main(int argc, char *argv[]) try {
    // Create pipeline - top-level API for streaming and processing frames
    rs2::pipeline p;

    // Configure and start the pipeline
    p.start();

    while (true) {
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        // Get a frame of depth image
        rs2::depth_frame depth = frames.get_depth_frame();

        // Get depth frame's dimension
        auto width = depth.get_width();
        auto height = depth.get_height();

        // Get distance from a pixel
        float dist_to_center = depth.get_distance(width / 2, height / 2);

        // Print the distance
        std::cout << "Frame size:" << width << " * " << height << std::endl << "The camera is facing an object " << dist_to_center << "meters away \r";

    }

    return EXIT_SUCCESS;
}

catch (const rs2::error &e) {
    std::cerr << "RealSense error calling" << e.get_failed_function() << "(" << e.get_failed_args() << "):\n" << e.what() << std::endl;
    return EXIT_FAILURE;
}
// catch (const std::exception& e) {
//     std::cerr << e.what() << std::endl;
//     return EXIT_FAILURE;
// }