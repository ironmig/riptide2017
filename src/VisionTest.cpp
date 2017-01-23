#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;
class RotorFinder
{
private:
  image_transport::ImageTransport image_transport;
  image_transport::Subscriber image_sub;
  private:
    Mat targetMat()
    {
        //Represents the optimal target, each pixel = 1 mm
        Mat target(127, 260, CV_8U);
        rectangle(target, Point(0, 0), Point (51,127), Scalar(255), CV_FILLED);
        rectangle(target, Point(210, 0), Point (260,127), Scalar(255), CV_FILLED);
        return target;
    }
    void image_cb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }
        Mat frame = cv_ptr->image;
        
        
        Mat blured;
        bilateralFilter(frame, blured, 5, 100, 100);
        Mat hsv;
        cvtColor(blured, hsv, CV_BGR2HSV);
        
        Mat thresholded;
        inRange(hsv, Scalar(35.0,115.0,0), Scalar(90.0,255,255), thresholded);
        
        Mat target = targetMat();
        
        
        imshow("test", frame);
        imshow("blur", blured);
        imshow("hsv", hsv);
        imshow("thresh", thresholded);
    }
public:
    RotorFinder(ros::NodeHandle& nh) : image_transport(nh)
    {
        image_sub = image_transport.subscribe("/usb_cam/image_raw", 1, &RotorFinder::image_cb, this);
        imshow("Thing", targetMat());
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv,"rotor_vision");
    ros::NodeHandle nh;
    RotorFinder vision(nh);
    while (ros::ok())
    {
        ros::spinOnce();
        waitKey(50);
    }
    return 0;
}
