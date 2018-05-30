// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include "cv_helpers.hpp"

#include <iostream>
#include <stdio.h>
#include <cmath>

#define BUFFER_SIZE 16

using namespace std;
using namespace cv;

vector<Rect> detectPutter(Mat frame);
Ptr<Tracker> createTracker(string track_type);
Vec3f getVector(const rs2_intrinsics& intrinsics, const rs2::depth_frame& depth_frame, Point& left_pixel, Point& right_pixel);
float computeAngle(Vec3f a, Vec3f b);
void update_result(Rect bbox, Mat& color_mat, Vec3f vec_calibrator, const rs2_intrinsics& intrinsics, const rs2::depth_frame& depth_frame);

/** Global variables */
String putter_cascade_name = "cascade.xml";
CascadeClassifier putter_cascade;
string window_name = "Putter Detection";
string track_types[6]= {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW", "GOTURN"};
deque<Point> pts(BUFFER_SIZE, Point(0, 0));
int counter = 0;

int main(int argc, char * argv[]) try
{
//    rs2::decimation_filter dec;
//    dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 3);
//    rs2::disparity_transform depth2disparity;
//    rs2::disparity_transform disparity2depth(false);
//    rs2::spatial_filter spat;
//    spat.set_option(RS2_OPTION_HOLES_FILL, 5);
//    rs2::temporal_filter temp;
    rs2::align align_to(RS2_STREAM_COLOR);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Stream configurations
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30); //1280, 720; 960, 540; 848, 480; 640, 480; 640, 360; 480, 270
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);  // 30, 15, 6
    // Start streaming with default recommended configuration
    auto profile = pipe.start(cfg);

//    auto sensor = profile.get_device().first<rs2::depth_sensor>();
//    auto range = sensor.get_option_range(RS2_OPTION_VISUAL_PRESET);
//    for (auto i = range.min; i < range.max; i+= range.step)
//        if (string(sensor.get_option_value_description(RS2_OPTION_VISUAL_PRESET, i)) == "High Density")
//            sensor.set_option(RS2_OPTION_VISUAL_PRESET, i);

    auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto intrinsics = stream.get_intrinsics();

    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);
    string track_type = track_types[3];
    int count = 0;
    bool detect_flag = true;
    Rect2d bbox;
    Ptr<Tracker> tracker;
    rs2::colorizer color_map;
    Point calibrator_left(120, 320);
    Point calibrator_right(180, 320);
    VideoWriter video("demo.avi", CV_FOURCC('M', 'J', 'P', 'G'), 2, Size(640, 480));


    // Load cascade file
    if( !putter_cascade.load( putter_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };

    while (waitKey(1) < 0 && cvGetWindowHandle(window_name))
    {
        // Wait for the next set of frames
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        // Make sure the frames are spatially aligned
        data = align_to.process(data);

        auto color_frame = data.get_color_frame();
        auto depth_frame = data.get_depth_frame();

//        depth_frame = dec.process(depth_frame);
//        depth_frame = depth2disparity.process(depth_frame);
//        depth_frame = spat.process(depth_frame);
//        depth_frame = temp.process(depth_frame);
//        depth_frame = disparity2depth.process(depth_frame);

//        auto color_depth = color_map(depth_frame);
//        int w = color_depth.as<rs2::video_frame>().get_width();
//        int h = color_depth.as<rs2::video_frame>().get_height();
//        Mat image(Size(w,h), CV_8UC3, (void*)color_depth.get_data(), Mat::AUTO_STEP);
//        imshow("Depth image", image);

        //Get the 3d vector of calibrator: ground truth
        Vec3f vec_calibrator = getVector(intrinsics, depth_frame, calibrator_left, calibrator_right);

        // If we only receieved new depth frame, but the color did not capture, continue
        static int last_frame_number = 0;
        if (color_frame.get_frame_number() == last_frame_number) continue;
        last_frame_number = color_frame.get_frame_number();

        // Convert realsense frmae to opencv matrix
        auto color_mat = frame_to_mat(color_frame);
        auto depth_mat = depth_frame_to_meters(pipe, depth_frame);

        // Re-detect putter after every 15 frames
        count = count + 1;
        if ((count-1)%15 == 0)
        {
            detect_flag = true;
        }

        detect_flag = true;
        if (detect_flag)
        {
            vector<Rect> putters = detectPutter(color_mat);

            if (putters.size() == 1)
            {
                // Define initial bounding box
                bbox.x = putters[0].x;
                bbox.y = putters[0].y;
                bbox.width = putters[0].width;
                bbox.height = putters[0].height;

                // Create tracker and initilize with bounding box
                tracker = createTracker(track_type);
                bool ok = tracker->init(color_mat, bbox);
//                bool ok = false;
//                if (tracker->init(color_mat, bbox) || tracker->update(color_mat, bbox))
//                    ok = true;

                detect_flag = false;

                if (!ok)
                {
                    printf("tracker initilize failure");
                    detect_flag = true;
                }

                // Update result: compute putter vector and calculate angle between putter and ground truth
                update_result(putters[0], color_mat, vec_calibrator, intrinsics, depth_frame);
            }
            else
            {
                detect_flag = true;
            }

            for (size_t i = 0; i < putters.size(); i++)
            {
               // Calculate depth data
               Scalar m = mean(depth_mat(putters[i]));
               ostringstream ss;
               ss << setprecision(4) << m[0] << "meters away";
               String conf(ss.str());

               // Draw the detected objects
               rectangle(color_mat, putters[i], Scalar( 255, 0, 255 ), 2);
               putText(color_mat, ss.str(), Point(putters[0].x, putters[0].y-putters[0].height/2.0), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0));
            }
        }
        else
        {
            // Update the tracking result
            bool ok = tracker->update(color_mat, bbox);

            if (ok)
            {
                // Calculate depth data
                Scalar m = mean(depth_mat(bbox));
                ostringstream ss1;
                ss1 << setprecision(4) << m[0] << "meters away";
                String conf1(ss1.str());

                // Tracking sucess: Draw track result
                rectangle(color_mat, bbox, Scalar( 0, 255, 0 ), 2);
                putText(color_mat, ss1.str(), Point(bbox.x, bbox.y-bbox.height/2.0), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0));

                // Update result: compute putter vector and calculate angle between putter and ground truth
                update_result(bbox, color_mat, vec_calibrator, intrinsics, depth_frame);
            }
            else
            {
                // Tracking failure
                putText(color_mat, "Tracker failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
                detect_flag = true;
            }
        }

        circle(color_mat, calibrator_left, 5, Scalar( 0, 191, 255 ), 2);
        circle(color_mat, calibrator_right, 5, Scalar( 0, 191, 255 ), 2);
        putText(color_mat, "Please put the calibrator here", Point(calibrator_left.x,calibrator_left.y-5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 191, 255),1);
        // Show the set of tracked points
        for (int i = 1; i < BUFFER_SIZE; i++)
        {
            if ((pts[i-1].x == 0 && pts[i-1].y == 0) || (pts[i].x == 0 && pts[i].y == 0))
                continue;

            int thickness = (int)(sqrt((32/(float)(i+1)))*2.5);
            line(color_mat, pts[i-1], pts[i], Scalar(255, 255, 0), thickness);
        }
        // Update the window with new data
        imshow(window_name, color_mat);

        //video.write(color_mat);

    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

vector<Rect> detectPutter(Mat frame)
{
    vector<Rect> putters;
    Mat frame_gray;
    cvtColor( frame, frame_gray, CV_BGR2GRAY );
    //equalizeHist( frame_gray, frame_gray );
    putter_cascade.detectMultiScale(frame_gray, putters, 1.1, 30, 0, Size(60,15), Size(240, 60));

    return putters;

}

Ptr<Tracker> createTracker(string track_type)
{
    Ptr<Tracker> res;
    if (track_type == "BOOSTING")
        res = TrackerBoosting::create();
    else if (track_type == "MIL")
        res = TrackerMIL::create();
    else if (track_type == "KCF")
        res = TrackerKCF::create();
    else if (track_type == "TLD")
        res = TrackerTLD::create();
    else if (track_type == "MEDIANFLOW")
        res = TrackerMedianFlow::create();
    else if (track_type == "GOTURN")
        res = TrackerGOTURN::create();
    else
        printf("--(!)Error track_type\n");

    return res;
}

Vec3f getVector(const rs2_intrinsics& intrinsics, const rs2::depth_frame& depth_frame, Point& left, Point& right)
{
    float distance_left = depth_frame.get_distance(left.x, left.y);
    float distance_right = depth_frame.get_distance(right.x, right.y);

    float left_pixel[2];
    float right_pixel[2];
    float left_point[3];
    float right_point[3];

    left_pixel[0] = left.x;
    left_pixel[1] = left.y;
    right_pixel[0] = right.x;
    right_pixel[1] = right.y;

    rs2_deproject_pixel_to_point(left_point, &intrinsics, left_pixel, distance_left);
    rs2_deproject_pixel_to_point(right_point, &intrinsics, right_pixel, distance_right);

    Vec3f vec(left_point[0]-right_point[0], left_point[1]-right_point[1], left_point[2]-right_point[2]);

    return vec;
}

float computeAngle(Vec3f a, Vec3f b)
{
    float dot = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    float mag_a = sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    float mag_b = sqrt(b[0]*b[0] + b[1]*b[1] + b[2]*b[2]);

    return acos(dot/(mag_a*mag_b)) * 180 / 3.14159265;
}

void update_result(Rect bbox, Mat& color_mat, Vec3f vec_calibrator, const rs2_intrinsics& intrinsics, const rs2::depth_frame& depth_frame)
{
    // Get the center of the detection rectange and draw a small circle
    Point center_putter(bbox.x+0.5*bbox.width, bbox.y+0.5*bbox.height);
    circle(color_mat, center_putter, 5, Scalar( 255, 0, 0 ), 2);

    // update the points queque
    counter++;
    if ((pts[0].x == 0 && pts[0].y == 0) || counter > 10)
    {
        pts.push_front(center_putter);
        pts.pop_back();

        if (counter > 10)
            counter = 0;
    }
    else
    {
        double dis = sqrt(pow((pts[0].x - center_putter.x), 2) + pow((pts[0].y - center_putter.y), 2));
        if (dis <= 100)
        {
            pts.push_front(center_putter);
            pts.pop_back();
        }
    }

    // Set the left point and right point, which are used to calculate vectors later
    Point left_putter(center_putter.x-15, center_putter.y);
    Point right_putter(center_putter.x+15, center_putter.y);

    //Get the 3d vector of putter
    Vec3f vec_putter = getVector(intrinsics, depth_frame, left_putter, right_putter);

    // Compute angle between calibrator (ground truth) and putter
    float angle = computeAngle(vec_calibrator, vec_putter);
    angle = angle > 90 ? 180-angle : angle;

    ostringstream ss;
    ss << "The angle is " << setprecision(2) << angle;
    putText(color_mat, ss.str(), Point(bbox.x, bbox.y+bbox.height*1.5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,0));
}
