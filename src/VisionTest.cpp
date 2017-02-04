#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <image_geometry/pinhole_camera_model.h>

using namespace std;
using namespace cv;
class LiftFinder
{
private:
  image_geometry::PinholeCameraModel cameraModel;
  image_transport::ImageTransport image_transport;
  image_transport::Subscriber image_sub;
  ros::Subscriber cameraInfoSub;
  private:
    void cameraInfoCallback(sensor_msgs::CameraInfo info)
    {
        cameraModel.fromCameraInfo(info);
    }
    Mat targetMat()
    {
        //Represents the optimal target, each pixel = 1 mm
        Mat target(127, 260, CV_8U);
        size_t top1, top2, bot1, bot2 = 0;
        
        rectangle(target, Point(0, 0), Point (51,127), Scalar(255), CV_FILLED);
        rectangle(target, Point(210, 0), Point (260,127), Scalar(255), CV_FILLED);
        return target;
    }
    static vector<Point> getOrderedRectPoints(vector<Point>& rpoints)
    {
        vector<Point2f> rPointsF(4);
        rPointsF[0] = Point2f(rpoints[0].x, rpoints[0].y);
        rPointsF[1] = Point2f(rpoints[1].x, rpoints[1].y);
        rPointsF[2] = Point2f(rpoints[2].x, rpoints[2].y);
        rPointsF[3] = Point2f(rpoints[3].x, rpoints[3].y);
        return getOrderedRectPoints(rPointsF);
    }
    static vector<Point> getOrderedRectPoints(vector<Point2f>& rPoints)
    {
        Point2f points[4];
        points[0] = Point2f(0, 100000);
        points[1] = Point2f(0, 100000);
        points[2] = Point2f(0, 0);
        points[3] = Point2f(0, 0);
        for(size_t i = 0; i < 4; i++)
        {
            if ( rPoints[i].y < points[0].y) 
            {
                points[1] = points[0];
                points[0] = rPoints[i];
            }
            else if ( rPoints[i].y < points[1].y)
            {
                points[1] = rPoints[i];
            }
            if ( rPoints[i].y > points[2].y)
            {
                points[3] = points[2];
                points[2] =  rPoints[i];
            }
            else if ( rPoints[i].y > points[3].y)
            {
                points[3] =  rPoints[i];
            }
        }
        if (points[0].x > points[1].x)
        {
            Point2f temp = points[0];
            points[0] = points[1];
            points[1] = temp;
        }
        if (points[3].x > points[2].x)
        {
            Point2f temp = points[2];
            points[2] = points[3];
            points[3] = temp;
        }
        vector<Point> points_vec(4);
        points_vec[0] = Point(points[0].x, points[0].y);
        points_vec[1] = Point(points[1].x, points[1].y);
        points_vec[2] = Point(points[2].x, points[2].y);
        points_vec[3] = Point(points[3].x, points[3].y);
        return points_vec; 
    }
    static vector<Point> getOrderedRectPoints(RotatedRect& r)
    {
        vector<Point2f> rPoints(4);
        r.points(&rPoints[0]);
        return getOrderedRectPoints(rPoints);

    }
    static bool ApproxPolygonTest(vector<Point>& points)
    {
        vector<Point> polygon;
        approxPolyDP(points, polygon, arcLength(points, true) * 0.05, true);
        printf("polygon test sides = %d\n", polygon.size());
        return polygon.size() == 4; 
    }
    static double RRAreaTest(vector<Point>& points)
    {
        double area = contourArea(points);
        Point2f rr_points[4];
        RotatedRect rr = minAreaRect(points);
        rr.points(rr_points);
        vector<Point> rr_points_vec(4);
        rr_points_vec[0] = rr_points[0];
        rr_points_vec[1] = rr_points[1];
        rr_points_vec[2] = rr_points[2];
        rr_points_vec[3] = rr_points[3];
        double rr_area = contourArea(rr_points_vec);
        return fabs(rr_area-area)/area;
    }
    static double AreaPerimeterTest(vector<Point>& points)
    {
        double area = contourArea(points);
        double perimeter = arcLength(points, true);
        double predicted_area = (perimeter / 14.0 * 5.0) * (perimeter / 14.0 * 2.0);
        return fabs(predicted_area - area)/predicted_area; 
    }
    static bool ValidRect(vector<Point>& points)
    {
        double area = contourArea(points);
        double rect_area_error = RRAreaTest(points);
        bool approx_poly_test = ApproxPolygonTest(points);
        printf("Valid Rect Test Area=%f RectAreaError=%f ApproxPoly=%d\n", area, rect_area_error, approx_poly_test);
        return area > 100 && rect_area_error < 1 && approx_poly_test;
    }
    static bool ValidContour(vector<Point>& points)
    {
        double area_perimeter_error = AreaPerimeterTest(points);
        return ValidRect(points) && area_perimeter_error < 0.2;
    }
    /* Returns
     * TL, TR, BL, BR
     * 
     */
    static Point2f* combineTwoRects(RotatedRect& r1, RotatedRect& r2)
    {
        //First 2 lowest Y values (top), Second 2 highest y value (bottom)
        Point2f r1Points[4];
        Point2f r2Points[4];
        r1.points(r1Points);
        r2.points(r2Points);
        Point2f* points = new Point2f[4];
        points[0] = Point2f(0, 1000000);
        points[1] = Point2f(0, 1000000);
        points[2] = Point2f(0, 0);
        points[3] = Point2f(0, 0);
        for(size_t i = 0; i < 4; i++)
        {
            if (r1Points[i].y < points[0].y) 
            {
                points[1] = points[0];
                points[0] = r1Points[i];
            }
            else if (r1Points[i].y < points[1].y)
            {
                points[1] = r1Points[i];
            }
            if (r1Points[i].y > points[2].y)
            {
                points[3] = points[2];
                points[2] = r1Points[i];
            }
            else if (r1Points[i].y > points[3].y)
            {
                points[3] = r1Points[i];
            }
            
            if (r2Points[i].y < points[0].y) 
            {
                points[1] = points[0];
                points[0] = r2Points[i];
            }
            else if (r2Points[i].y < points[1].y)
            {
                points[1] = r2Points[i];
            }
            if (r2Points[i].y > points[2].y)
            {
                points[3] = points[2];
                points[2] = r2Points[i];
            }
            else if (r2Points[i].y > points[3].y)
            {
                points[3] = r2Points[i];
            }
        }
        return points;
    }
    void get3DPose(vector<vector<Point> >& contours)
    {
        if (contours.size() != 2) printf("Contours not size 2\n");
        if (contours[0].size() != 4 || contours[1].size() != 4) printf("Contours not rectangles\n");
        vector<Point> left, right;
        if(contours[0][0].x > contours[1][0].x)
        {
            left = contours[1];
            right = contours[0];
        } else {
            left = contours[0];
            right = contours[1];
        }
        vector<Point> combined(8);
        for (size_t i = 0; i < 4; i++)
            combined[i]= left[i];
        for (size_t i = 0; i < 4; i++)
            combined[4+i] = right[i];
        
        //Mat rvec, tvec;
        //cv::solvePnP(, scenePoints, cameraModel.intrinsicMatrix(),
        //    cameraModel.distortionCoeffs(), rvec, tvec, false);
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
        imshow("Original Image", frame);
        
        int y_offset = frame.rows/2.0;
        Rect rect(0, y_offset, frame.cols, frame.rows - y_offset);
        
        Mat roied = frame(rect);
        Mat finished = roied.clone();

        
        Mat blurred;
        bilateralFilter(roied, blurred, 5, 100, 100);
        
        Mat hsv;
        cvtColor(blurred, hsv, CV_BGR2HSV);
        imshow("HSV", hsv);
        
        Mat thresholded;
        inRange(hsv, Scalar(47, 0.0, 73), Scalar(75.0, 255.0, 255), thresholded);
        imshow("Thresh", thresholded);
        
        vector<vector<Point> > contours;
        vector<vector<Point> > filteredContours;
        vector<Vec4i> hierarchy;
        //~ vector<Moments> moments(contours.size());
        
        findContours(thresholded, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE,  Point(0, 0));
        hierarchy.clear();
        Scalar colors[4] = {
            Scalar(0,0,255),
            Scalar(0,255,0),
            Scalar(255,0,0),
            Scalar(255,255,255)
        };
        printf("BEFORE Contours=%lu Hierachy=%lu\n", contours.size(), hierarchy.size());
        outer: for (vector<vector<Point> >::iterator c = contours.begin(); c != contours.end(); c++)
        {
            bool added = false;
            Moments mu = moments(*c);
            Point2f center( mu.m10/mu.m00 , mu.m01/mu.m00 );
            for (vector<vector<Point> >::iterator c2 = c+1; c2 != contours.end(); c2++)
            {
                if(c2->size() > 200 || c2->size() < 4) continue;
                Moments mu2 = moments(*c2, false);
                Point2f center2( mu2.m10/mu2.m00 , mu2.m01/mu2.m00 );
                if (abs(center2.x-center.x) < 20)
                {
                    line(roied, center, center2, Scalar(255,255,255));
                    RotatedRect cRect = minAreaRect(*c);
                    Point2f cPoints[4];
                    cRect.points(cPoints);
                    RotatedRect c2Rect = minAreaRect(*c2);
                    Point2f c2Points[4];
                    c2Rect.points(c2Points);
                    Point2f* pointsCombined = combineTwoRects(cRect, c2Rect);
                    for( int j = 0; j < 4; j++ )
                    {
                        line(roied, cPoints[j], cPoints[(j+1)%4], Scalar(255,0,0), 5, 8 );
                        circle(roied, pointsCombined[j], 10, colors[j], -1);
                        line(roied, c2Points[j], c2Points[(j+1)%4], Scalar(0,255,0), 5, 8 );
                    }
                    vector<Point> combinedContour(4);
                    combinedContour[0] = pointsCombined[0];
                    combinedContour[1] = pointsCombined[1];
                    combinedContour[2] = pointsCombined[2];
                    combinedContour[3] = pointsCombined[3];
                    if (ValidRect(*c) && ValidRect(*c2) && ValidContour(combinedContour))
                    {
                        filteredContours.push_back(getOrderedRectPoints(combinedContour));
                        contours.erase(c2);
                        added = true;
                        break;
                    }
                    
                    
                    
                    /*
                    vector<Point> combined(c2->size() + c->size());
                    combined.insert(combined.end(),c->begin(), c->end());
                    combined.insert(combined.end(), c2->begin(), c2->end());
                    Rect r = boundingRect(combined);
                    circle(roied, r.br(), 5, Scalar(0,255,0), -1);
                    circle(roied, r.tl(), 5, Scalar(0,255,0), -1);
                    Mat mask = roied(r);
                    imshow("OH BOY", mask);
                    */
                }
            }
            //circle(roied, center, 5, Scalar(255,0,0), -1);
            if (!added && ValidContour(*c))
            {
                RotatedRect rr = minAreaRect(*c);
                vector<Point> temp  = getOrderedRectPoints(rr);
                filteredContours.push_back(temp);
            }
        }

        
        RNG rng(12345);
        for( int i = 0; i< contours.size(); i++ )
        {
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours(roied, contours, i, color, 2, 8, hierarchy, 0, Point() );
        }
        for( int i = 0; i< filteredContours.size(); i++ )
        {
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours(finished, filteredContours, i, color, 2, 8, hierarchy, 0, Point() );
            if(filteredContours[i].size() == 4) 
                for (size_t j = 0; j < 4; j++)
                    circle(finished,filteredContours[i][j], 5, colors[j], -1);
        }
        printf("AFTER Contours=%lu \n", filteredContours.size(), hierarchy.size());
        imshow("ROIEDDD", roied);
        imshow("DONE", finished);

    }
public:
    LiftFinder(ros::NodeHandle& nh) : image_transport(nh)
    {
        image_sub = image_transport.subscribe("/usb_cam/image_raw", 1, &LiftFinder::image_cb, this);
        cameraInfoSub = nh.subscribe("/usb_cam/camera_info", 1, &LiftFinder::cameraInfoCallback, this);
        imshow("Thing", targetMat());
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv,"rotor_vision");
    ros::NodeHandle nh;
    LiftFinder vision(nh);
    while (ros::ok())
    {
        ros::spinOnce();
        waitKey(50);
    }
    return 0;
}
