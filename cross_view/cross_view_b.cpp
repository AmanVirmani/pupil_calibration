#include <vector>
#include <valarray>
#include <algorithm>
#include <type_traits>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <mutex>

// #include <X11/Xlib.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

// Local includes
#include <common.h>
#include <event.h>
#include <event_vis.h>

// DVS / DAVIS
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <pupil_ros/gaze_positions.h>
#include <pupil_ros/gaze.h>



class Visualizer {
protected:
    std::mutex mutex;
    std::string name, window_name, ctopic, etopic, gaze_topic;
    ros::Subscriber e_sub, c_sub, gaze_info;

    uint32_t c_res_x, c_res_y, e_res_x, e_res_y;
    //uint32_t gaze_px, gaze_py, gaze_match_px, gaze_match_py;
    float gaze_px, gaze_py, gaze_match_px, gaze_match_py;
    float gaze_Z;

    // Buffer for incoming events (aka 'slice')
    // 1e8 == 0.1 sec
    CircularArray<Event, 1000000, 50000000> ev_buffer;
    cv::Mat c_image;

    cv::Mat c_K, c_D, e_K, e_D;
    std::string c_model, e_model;
    //pupil_ros::gaze_positions gaze_point; 

public:
    float point_x = 600, point_y = 400, point_Z = 500;
    Visualizer(ros::NodeHandle &nh, std::string ctopic_, std::string etopic_, std::string gaze_topic_ , std::string name_="visualizer")
        : name(name_), ctopic(ctopic_), etopic(etopic_), gaze_topic(gaze_topic_), c_res_x(0), c_res_y(0), e_res_x(0), e_res_y(0)
        , gaze_px(100), gaze_py(100), gaze_Z(1.0) {
        this->window_name = this->name;
        cv::namedWindow(this->window_name, cv::WINDOW_NORMAL);

        // Set camera intrinsics
        this->c_K = Visualizer::get_K(801.6947924812483, 802.6485823902558, 620.2055872431376, 354.3864350612314);
        this->c_D = (cv::Mat1d(1, 4) << -0.13229634176002464, -0.023644289230288832, 0.019704866760515385, -0.0065800357834113);
        this->c_model = "equidistant";

        this->e_K = Visualizer::get_K(506.4692274538, 504.1712838890729, 325.79676532421803, 238.70837103100686);
        this->e_D = (cv::Mat1d(1, 4) << 0.13140489691973775, -0.17447930179658452, 0.00044504317597171593, 0.004727272184495526);
        this->e_model = "radtan";

        //this->gaze_info = nh.subscribe(this->gaze_topic, 10, &Visualizer::callback_gaze, this);
        this->gaze_info = nh.subscribe(this->gaze_topic, 10, &Visualizer::callback_gaze, this);
        this->c_sub = nh.subscribe(this->ctopic, 1, &Visualizer::c_cb, this);
        this->e_sub = nh.subscribe(this->etopic, 1, &Visualizer::e_cb<dvs_msgs::EventArray::ConstPtr>, this);

    }

    std::string get_window_name() {return this->window_name; }

    // Callbacks
    void c_cb(const sensor_msgs::ImageConstPtr& msg) {
        const std::lock_guard<std::mutex> lock(this->mutex);

        cv::Mat img;
        if (msg->encoding == "8UC1") {
            sensor_msgs::Image simg = *msg;
            simg.encoding = "mono8";
            img = cv_bridge::toCvCopy(simg, "bgr8")->image;
        } else {
            img = cv_bridge::toCvShare(msg, "bgr8")->image;
        }
        this->c_res_y = msg->height;
        this->c_res_x = msg->width;
        this->c_image = img.clone();
    }

    template<class T>
    void e_cb(const T& msg) {
        const std::lock_guard<std::mutex> lock(this->mutex);
        for (uint i = 0; i < msg->events.size(); ++i) {
            ull time = msg->events[i].ts.toNSec();
            Event e(msg->events[i].y, msg->events[i].x, time);
            this->ev_buffer.push_back(e);
        }
        this->e_res_y = msg->height;
        this->e_res_x = msg->width;
    }

    void set_gaze_debug(float px, float py, float Z) {
        if ((px >= this->c_res_x) || (py >= this->c_res_y) || (Z <= 0)) return;
        this->gaze_px = px; this->gaze_py = py; this->gaze_Z = Z;

        cv::Mat T = (cv::Mat1f(1, 3) << -0.019612239964863745, 0.017638802911241643, 0.010431131020442333);
        cv::Matx33f R(0.9996530617336309, -0.01839391872107208, 0.01885258392549504,
                      0.01873644874065071, 0.9996595912024482, -0.018156189180298512,
                      -0.01851220287200333, 0.01850312057584921, 0.9996574077521704);

        cv::Mat R_rodrig;
        cv::Rodrigues(R, R_rodrig);

        cv::Mat dst;
        if (this->c_model == "equidistant") {
            cv::Mat src = cv::Mat(1, 1, CV_32FC2, cv::Scalar(px, py));
            cv::fisheye::undistortPoints(src, dst, this->c_K, this->c_D);
        }
        if (this->c_model == "radtan") {
            cv::Mat src = cv::Mat(1, 1, CV_32FC2, cv::Scalar(px, py));
            cv::undistortPoints(src, dst, this->c_K, this->c_D);
        }
        auto p = cv::Point3f(dst.at<cv::Vec2f>(0, 0)[0], dst.at<cv::Vec2f>(0, 0)[1], 1) * Z;
        //this->gaze_px = p.x; this->gaze_py = p.y; this->gaze_Z = p.z;

        
        float fx = 801.6947924812483,fy = 802.6485823902558, cx = 620.2055872431376, cy = 354.3864350612314;
                                                                                                    
        this->gaze_match_px = fx*px + cy; this->gaze_match_py = fy*py + cy;
        std::cout<<"gaze_pupil: " << this->gaze_px << ", " << this->gaze_py << std::endl;
        std::cout<<"gaze_dvs: " << this->gaze_match_px << ", " << this->gaze_match_py << std::endl;
        cv::Mat c_image_ = this->c_image; //.clone();
        cv::Mat c_undistorted = Visualizer::undistort(c_image_, this->c_K, this->c_D, this->c_model);
        cv::circle(c_undistorted, cv::Point(this->gaze_px, this->gaze_py), 10, cv::Scalar(0,0,255), -1);
        cv::circle(c_undistorted, cv::Point(this->gaze_match_px, this->gaze_match_py), 10, cv::Scalar(0,255,0), -1);
        //cv::circle(c_image_, cv::Point(50, 50), 10, cv::Scalar(0,255,0), -1);
        //cv::hconcat(e_resized, c_resized, img);
        cv::imshow(this->window_name, c_undistorted);
    }
    
    
    void set_gaze(uint32_t px, uint32_t py, float Z) {
        if ((px >= this->c_res_x) || (py >= this->c_res_y) || (Z <= 0)) return;
        this->gaze_px = px; this->gaze_py = py; this->gaze_Z = Z;

        cv::Mat T = (cv::Mat1f(1, 3) << -0.019612239964863745, 0.017638802911241643, 0.010431131020442333);
        cv::Matx33f R(0.9996530617336309, -0.01839391872107208, 0.01885258392549504,
                      0.01873644874065071, 0.9996595912024482, -0.018156189180298512,
                      -0.01851220287200333, 0.01850312057584921, 0.9996574077521704);

        cv::Mat R_rodrig;
        cv::Rodrigues(R, R_rodrig);

        cv::Mat dst;
        if (this->c_model == "equidistant") {
            cv::Mat src = cv::Mat(1, 1, CV_32FC2, cv::Scalar(px, py));
            cv::fisheye::undistortPoints(src, dst, this->c_K, this->c_D);
        }
        if (this->c_model == "radtan") {
            cv::Mat src = cv::Mat(1, 1, CV_32FC2, cv::Scalar(px, py));
            cv::undistortPoints(src, dst, this->c_K, this->c_D);
        }
        auto p = cv::Point3f(dst.at<cv::Vec2f>(0, 0)[0], dst.at<cv::Vec2f>(0, 0)[1], 1) * Z;
        //this->gaze_px = p.x; this->gaze_py = p.y; this->gaze_Z = p.z;
        std::vector<cv::Point3f> p_vec = {p};

        cv::Mat pix;
        if (this->e_model == "equidistant") {
            cv::fisheye::projectPoints(p_vec, pix, R_rodrig, T, e_K, e_D);
        }
        if (this->e_model == "radtan") {
            cv::projectPoints(p_vec, R_rodrig, T, e_K, e_D, pix);
        }

        this->gaze_match_px = pix.at<cv::Vec2f>(0, 0)[0]; this->gaze_match_py = pix.at<cv::Vec2f>(0, 0)[1];
        std::cout<<"gaze_pupil: " << this->gaze_px << ", " << this->gaze_py << std::endl;
        std::cout<<"gaze_dvs: " << this->gaze_match_px << ", " << this->gaze_match_py << std::endl;
    }

    void visualize(int e_mode=0) {
        const std::lock_guard<std::mutex> lock(this->mutex);
        cv::Mat e_image;
        if (e_mode % 2 == 0) {
            e_image = EventFile::color_time_img(&ev_buffer, 1, this->e_res_y, this->e_res_x);
        } else {
            e_image = EventFile::projection_img(&ev_buffer, 1, this->e_res_y, this->e_res_x);
            cv::cvtColor(e_image, e_image, CV_GRAY2RGB);
        }
        
        uint32_t height = std::min(this->c_image.rows, e_image.rows);
        if ((this->c_image.rows < 1) || (e_image.rows < 1)) return;

        // Draw on source frame
        cv::Mat c_image_ = this->c_image.clone();
        cv::circle(c_image_, cv::Point(this->gaze_px, this->gaze_py), 10, cv::Scalar(0,0,255), -1);
        cv::circle(e_image,  cv::Point(this->gaze_match_px, this->gaze_match_py), 5, cv::Scalar(0,0,255), -1);

        // Resize, undistort
        cv::Mat c_undistorted = Visualizer::undistort(c_image_, this->c_K, this->c_D, this->c_model);
        cv::Mat e_undistorted = Visualizer::undistort(e_image,  this->e_K, this->e_D, this->e_model);
	
	std::cout << "RGB image size: " << c_undistorted.rows << ", " << c_undistorted.cols << std::endl;
	std::cout << "DVS image size: " << e_undistorted.rows << ", " << e_undistorted.cols << std::endl;
        uint32_t e_width = (e_undistorted.cols * height) / e_undistorted.rows;
        uint32_t c_width = (c_undistorted.cols * height) / c_undistorted.rows;

        cv::Mat e_resized, c_resized, img;
        cv::resize(c_undistorted, c_resized, cv::Size(c_width, height), 0, 0);
        cv::resize(e_undistorted, e_resized, cv::Size(e_width, height), 0, 0);

        cv::hconcat(e_resized, c_resized, img);
        cv::imshow(this->window_name, img);
    }

    static cv::Mat get_K(float fx, float fy, float cx, float cy) {
        cv::Mat K = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
        return K;
    }

    void callback_gaze(const pupil_ros::gaze_positions &info) {
        int resx = 1280, resy = 720;
        //float Z = info.gazes[0].gaze_point_3d.z;//1000; // in m
        //float X = info.gazes[0].gaze_point_3d.x;//1000;
        //float Y = info.gazes[0].gaze_point_3d.y;//1000;
    
        //cv::Mat worldP=  (cv::Mat1d(3,1) << (X, Y, Z));
        //cv::Mat dst = this->c_K * worldP;
        //this->point_Z = dst.at<cv::Vec2f>(0, 0)[2];//dst.at<cv::Vec2f>(0, 0)[2]; //projectedP[2];
        //this->point_y = dst.at<cv::Vec2f>(0, 0)[1];//dst.at<cv::Vec2f>(0, 0)[2];  //projectedP[1];
        //this->point_x = dst.at<cv::Vec2f>(0, 0)[0];//dst.at<cv::Vec2f>(0, 0)[2];  //projectedP.at(0,0);
        this->point_Z = info.gazes[0].gaze_point_3d.z; // in m
        this->point_x = info.gazes[0].norm_pos.x * resx;
        this->point_y = (1- info.gazes[0].norm_pos.y) * resy;
    }

protected:
    static cv::Mat undistort(cv::Mat &img, cv::Mat K, cv::Mat D, std::string dist_model) {
        cv::Mat ret;
        if (dist_model == "radtan") {
            cv::undistort(img, ret, K, D, 1.0 * K);
        } else if (dist_model == "equidistant") {
            cv::fisheye::undistortImage(img, ret, K, D, 1.0 * K);
        } else {
            std::cout << "Unknown distortion model! " << dist_model << std::endl;
        }

        return ret;
    }
};


void on_trackbar(int, void*) {}

//int point_x = 600, point_y = 400, point_Z = 500;

int main (int argc, char** argv) {
    std::string node_name = "cross_view";
    ros::init (argc, argv, node_name);
    ros::NodeHandle nh("~");

        
    Visualizer vis(nh, "/pupil_capture/world", "/samsung/camera/events", "/pupil_capture/gaze");
    //ros::Subscriber gaze_info = nh.subscribe("pupil_capture/gaze", 10, vis.callback_gaze());

    //global int point_x = 600, point_y = 400, point_Z = 500;
    // cv::createTrackbar("x", vis.get_window_name(), &point_x, 1200, on_trackbar);
    // cv::createTrackbar("y", vis.get_window_name(), &point_y, 1200, on_trackbar);
    // cv::createTrackbar("Z", vis.get_window_name(), &point_Z, 1000, on_trackbar);


    int e_mode = 0;
    int code = 0; // Key code
    while ((code != 27) && ros::ok()) {
        code = cv::waitKey(20);
        if (code == 32) e_mode ++;

        //vis.set_gaze_debug(vis.point_x, vis.point_y, float(vis.point_Z + 1) / 100.0);
        vis.set_gaze(vis.point_x, vis.point_y, float(vis.point_Z + 1) / 100.0);
        vis.visualize(e_mode);

        ros::spinOnce();
    }

    ros::shutdown();
    return 0;
}
