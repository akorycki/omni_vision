#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <opencv2/ccalib/omnidir.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

class OmniRect {
public:
    cv::Matx33f K;
    cv::Matx41f D;
    double xi;

    cv::Size new_size;

    int rect_type;    // cv::omnidir rectification type
    cv::Matx33f Knew; // Camera matrix of the distorted image

    OmniRect(ros::NodeHandle& nh)
    : it_(nh) {
        // Load parameters from the YAML file
        string yaml_file;
        nh.param("yaml_file", yaml_file, string(""));

        if (yaml_file.empty()) {
            ROS_ERROR("YAML file not provided");
            ros::shutdown();
            return;
        }

        // load calibration parameters from yaml file
        YAML::Node config = YAML::LoadFile(yaml_file);
        string input_topic = config["cam0"]["rostopic"].as<string>();
        string output_topic = generateOutputTopic(input_topic);
        vector<double> D_param = config["cam0"]["distortion_coeffs"].as<vector<double>>();
        vector<double> K_param = config["cam0"]["intrinsics"].as<vector<double>>();
        vector<int> res = config["cam0"]["resolution"].as<vector<int>>();

        // get height and width of rectified image
        if (nh.getParam("new_height", new_size.height)) {
        } else {
            new_size.height = res[1];
        }
        ROS_INFO("Rectified image height: %d", new_size.height);
        if (nh.getParam("new_width", new_size.width)) {
        } else {
            new_size.width = res[0];
        }
        ROS_INFO("Rectified image width: %d", new_size.width);

        // get rectification type, perspective by default
        string rect_type_str;
        if (nh.getParam("rectification_type", rect_type_str)) {
            if (rect_type_str == "perspective")
                rect_type = cv::omnidir::RECTIFY_PERSPECTIVE;
            if (rect_type_str == "cylindrical")
                rect_type = cv::omnidir::RECTIFY_CYLINDRICAL;
            if (rect_type_str == "stereographic")
                rect_type = cv::omnidir::RECTIFY_STEREOGRAPHIC;
            if (rect_type_str == "longlat")
                rect_type = cv::omnidir::RECTIFY_LONGLATI;
        } else {
            rect_type = cv::omnidir::RECTIFY_PERSPECTIVE;
        }
        ROS_INFO("Rectified type: %s", rect_type_str.c_str());
        ROS_INFO("Input topic: %s", input_topic.c_str());
        ROS_INFO("Output topic: %s", output_topic.c_str());

        // load intrinsic and distortion parameters into OpenCV types
        K = cv::Matx33f(K_param[1], 0.0, K_param[3],
                        0.0, K_param[2], K_param[4],
                        0.0, 0.0, 1.0);
        xi = K_param[0];
        D = cv::Mat(D_param);

        // create new camera matrix for distorted image
        if (rect_type == cv::omnidir::RECTIFY_PERSPECTIVE) {
            Knew = cv::Matx33f(new_size.width/4, 0, new_size.width/2,
                               0, new_size.height/4, new_size.height/2,
                               0, 0, 1);
        } else {
            Knew = cv::Matx33f(new_size.width/3.1415, 0, 0,
                               0, new_size.height/3.1415, 0,
                               0, 0, 1);
        }

        // Subscribe to the input image topic
        image_sub_ = it_.subscribe(input_topic, 1, &OmniRect::imageCallback, this);

        // Advertise the output image topic
        image_pub_ = it_.advertise(output_topic, 1);
    }

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv::Mat distorted, undistorted;
        distorted = cv_bridge::toCvCopy(msg)->image;
        cv::omnidir::undistortImage(distorted, undistorted, K, D, xi, rect_type, Knew, new_size);
        sensor_msgs::ImagePtr rect_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistorted).toImageMsg();
        image_pub_.publish(rect_msg);
    }

    string generateOutputTopic(const string& input_topic) {
        // Find the position of the last '/'
        size_t last_slash_pos = input_topic.find_last_of("/");

        // Replace the last part with 'image_rect'
        string output_topic = input_topic.substr(0, last_slash_pos) + "/image_rect";
        return output_topic;
    }

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "omni_rect");
    ros::NodeHandle nh;

    OmniRect omniRect(nh);

    ros::spin();
    return 0;
}
