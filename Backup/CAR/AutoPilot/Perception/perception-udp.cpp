#include <librealsense2/rs.hpp>
//#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <thread>

using namespace rs2;

const char* ip = "127.0.0.1";
//const char* ip = "192.168.0.112";
int port = 5002;
struct sockaddr_in serverAddr;
int sockfd;

void initialize_udp() {
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Error creating socket" << std::endl;
        exit(EXIT_FAILURE);
    }

    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = inet_addr(ip);
}

void send_udp_message(const char* message) {
    sendto(sockfd, message, strlen(message), 0, reinterpret_cast<const sockaddr*>(&serverAddr), sizeof(serverAddr));
}

bool is_object_detected(rs2::depth_frame frame, float min_distance, int width, int height) {
    const uint16_t* depth_data = reinterpret_cast<const uint16_t*>(frame.get_data());
    // int width = frame.get_width();
    // int height = frame.get_height();

    float dist = frame.get_distance(width, height);
    if (dist < min_distance) return true;
    else return false;
}

int main() try {
    // Create a RealSense pipeline. Encapsulation the actual device and sensors
    rs2::pipeline pipe;

    // Start straming with default configuration
    pipe.start();

    // Create an OpenCV window
    // cv::namedWindow("RGB Stream", cv::WINDOW_AUTOSIZE);

    // Initialize UDP sending
    initialize_udp();

    while (true) {
        // Block program until frames arrive
        rs2::frameset frames = pipe.wait_for_frames();

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame();

        // Get a frame of color image
        rs2::frame color = frames.get_color_frame();

        // Convert RealSense frame to OpenCV matrix
        // cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color,get_data(), cv::Mat::AUTO_STEP);

        // Display the RGB image
        // cv::imshow("RGB Stream", color_image);

        // Get the depth frame's dimensions
        const int w = depth.get_width();
        const int h = depth.get_height();

	const float dist = depth.get_distance(w / 2, h / 2);
	const float left_dist = depth.get_distance(w / 4, h / 2);
	const float right_dist = depth.get_distance(3 * w / 4, h / 2);

        if (is_object_detected(depth, 2.3, w / 2, h / 2)) {  // Previous: 1.5
            // Object detected, start to verify the target point
            if (is_object_detected(depth, 2.3, w / 4, h / 2)) {  // Previous: 1.8
		if (is_object_detected(depth, 2.3, 3 * w / 4, h / 2)) {  // Previous: 1.8
		        send_udp_message("2"); // Stop the car immediately
		        std::cout << "Dist: " << dist << "    Stop the Car! Stop_2 \r";
            	} else {
		        send_udp_message("12"); // Start lane-changing, right-side empty
		        std::cout << "X: -250; Y: 900;  Lane-changing_12 \r";
            	}
            } else {
		if (is_object_detected(depth, 2.3, 3 * w / 4, h / 2)) {  // Previous: 1.8
			send_udp_message("11"); // Start lane-changing, left-side empty
		        std::cout << "Dist: " << right_dist << "    X: 250; Y: 900;  Lane-changing_11 \r";
		} else {
			send_udp_message("13"); // Start lane-changing, both sides empty
			std::cout << "Dist: " << dist << "    Lane-changing: Both sides empty \r";
        	}
	    }
        } else {
		send_udp_message("0"); // Following the lane
		std::cout << "Dist: " << dist << "    Following the lane Following_0 \r";
	}
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    close(sockfd);
}

catch (const rs2::error&e) {
    std::cerr << "RealSense error calling:" << e.get_failed_function() << "(" << e.get_failed_args() << "):\n" << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception&e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
