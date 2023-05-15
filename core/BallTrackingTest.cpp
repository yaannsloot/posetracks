// BallTrackingTest.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include "chessboard_compute.hpp"
#include <chrono>

using namespace std;
using namespace cv;

int k_size = 4;
int sigma_color = 75;
int sigma_spatial = 75;
int iter = 1;

void HoughCirclesCUDA(InputArray image, OutputArray circles,
    double dp, double minDist,
    double param1 = 100, double param2 = 100,
    int minRadius = 0, int maxRadius = 0);

void medianBlurCUDA(InputArray src, OutputArray dst, int ksize);

Mat imt_ball1;
Mat dest;

static void on_trackbar_a(int, void*) {
    cuda::GpuMat img;
    img.upload(imt_ball1);
    cuda::GpuMat blur;
    cuda::bilateralFilter(img, blur, k_size, 75, 75);
    cuda::bilateralFilter(img, blur, 9, sigma_color, 75);
    cuda::bilateralFilter(img, blur, 9, 75, sigma_spatial);
    blur.download(dest);
    imshow("Image", dest);
}

int main()
{

   try {
        bool cuda = true;
        std::cout << "Hello World!\n";
        imt_ball1 = imread("balls2.jpg");
        Mat imt_ball2 = imread("balls1.jpg");

        namedWindow("Image", WINDOW_NORMAL);

        

        string name_a = "k_size";
        string name_b = "sigma_color";
        string name_c = "sigma_spatial";
        string name_d = "iterations";

        namedWindow("Filter", WINDOW_AUTOSIZE);

        createTrackbar(name_a, "Filter", &k_size, 1000, on_trackbar_a);
        createTrackbar(name_b, "Filter", &sigma_color, 1000, on_trackbar_a);
        createTrackbar(name_c, "Filter", &sigma_spatial, 1000, on_trackbar_a);
        createTrackbar(name_d, "Filter", &iter, 100, on_trackbar_a);

        //bilateralFilter(imt_ball1, blur, 90, 75, 75);
        //boxFilter(imt_ball1, blur, -1, Size(10,10));
        Mat eq;
        imshow("Image", imt_ball1);
        waitKey(0);
        Mat gray_ball1;
        cvtColor(imt_ball1, gray_ball1, COLOR_BGR2GRAY);
        imshow("Image", gray_ball1);
        waitKey(0);
        
        /*
        if (!cuda)
            medianBlur(gray_ball1, gray_ball1, 151);
        else
            medianBlurCUDA(gray_ball1, gray_ball1, 151);
        */
        

        equalizeHist(gray_ball1, gray_ball1);
        bitwise_not(gray_ball1, gray_ball1);

        bilateralFilter(gray_ball1, eq, 9, 75, 75);
        gray_ball1 = eq;
        imshow("Image", gray_ball1);
        waitKey(0);

        imshow("Image", gray_ball1);
        waitKey(0);

        SimpleBlobDetector::Params params;

        params.minThreshold = 0;
        params.maxThreshold = 100;

        params.filterByArea = true;
        params.minArea = 1500;
        params.maxArea = 500000;

        params.filterByCircularity = true;
        params.minCircularity = 0.8;

        params.filterByConvexity = true;
        params.minConvexity = 0.95;

        params.filterByInertia = true;
        params.minInertiaRatio = 0.1;

        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

        std::vector<KeyPoint> keypoints;
        detector->detect(gray_ball1, keypoints);

        Mat im_keypoints = gray_ball1;
        for (int i = 0; i < keypoints.size(); i++) {
            Point center = keypoints[i].pt;
            circle(im_keypoints, center, 1, Scalar(0, 100, 100), 3, LINE_AA);
            int radius = keypoints[i].size;
            circle(im_keypoints, center, radius, Scalar(255, 0, 255), 3, LINE_AA);
        }

        cout << keypoints.size() << endl;

        imshow("Image", im_keypoints);
        waitKey(0);

        vector<Vec3f> circles;
        if(!cuda)
            HoughCircles(gray_ball1, circles, HOUGH_GRADIENT, 1, gray_ball1.rows / 16, 100, 30, 10, 500);
        else
            HoughCirclesCUDA(gray_ball1, circles, 1, gray_ball1.rows / 16, 50, 100, 70, 500);

        for (int i = 0; i < circles.size(); i++) {
            Vec3i c = circles[i];
            Point center = Point(c[0], c[1]);
            circle(imt_ball1, center, 1, Scalar(0, 100, 100), 3, LINE_AA);
            int radius = c[2];
            circle(imt_ball1, center, radius, Scalar(255, 0, 255), 3, LINE_AA);
        }
        
        

        imshow("Image", imt_ball1);
        waitKey(0);

        imshow("Image", imt_ball2);
        waitKey(0);

        Mat gray_ball2;
        cvtColor(imt_ball2, gray_ball2, COLOR_BGR2GRAY);
        if(!cuda)
            medianBlur(gray_ball2, gray_ball2, 21);
        else
            medianBlurCUDA(gray_ball2, gray_ball2, 21);
        equalizeHist(gray_ball2, gray_ball2);
        bitwise_not(gray_ball2, gray_ball2);
        imshow("Image", gray_ball2);
        waitKey(0);

        

        vector<Vec3f> circles2;

        if(!cuda)
            HoughCircles(gray_ball2, circles2, HOUGH_GRADIENT, 1, gray_ball2.rows / 16, 100, 30, 10, 500);
        else {
            HoughCirclesCUDA(gray_ball2, circles2, 1, gray_ball2.rows / 16, 100, 30, 10, 500);
        }

        for (int i = 0; i < circles2.size(); i++) {
            Vec3i c = circles2[i];
            Point center = Point(c[0], c[1]);
            circle(imt_ball2, center, 1, Scalar(0, 100, 100), 3, LINE_AA);
            int radius = c[2];
            circle(imt_ball2, center, radius, Scalar(255, 0, 255), 3, LINE_AA);
        }

        imshow("Image", imt_ball2);
        waitKey(0);
        
        namedWindow("Video", WINDOW_NORMAL);
        
        VideoCapture video("balls.MOV");
        if (!video.isOpened())
            cout << "Error opening video file" << endl;
        else {
            int frame_time = 1000 / video.get(CAP_PROP_FPS);
            chrono::steady_clock::time_point last_time = chrono::steady_clock::now();
            VideoWriter out_vid("balls_out.mp4", VideoWriter::fourcc('m', 'p', '4', 'v'), video.get(CAP_PROP_FPS), Size(video.get(CAP_PROP_FRAME_WIDTH), video.get(CAP_PROP_FRAME_HEIGHT)));
            bool success = true;
            while (success) {
                Mat frame;
                success = video.read(frame);
                if (success) {
                    vector<Vec3f> circles_v;
                    Mat gray_frame;
                    cvtColor(frame, gray_frame, COLOR_BGR2GRAY);
                    equalizeHist(gray_frame, gray_frame);
                    bitwise_not(gray_frame, gray_frame);
                    if (!cuda) {
                        medianBlur(gray_frame, gray_frame, 21);
                        HoughCircles(gray_frame, circles_v, HOUGH_GRADIENT, 1, gray_frame.rows / 16, 100, 30, 10, 500);
                    }
                    else {
                        medianBlurCUDA(gray_frame, gray_frame, 21);
                        //HoughCirclesCUDA(gray_frame, circles_v, 1, gray_frame.rows / 16, 100, 30, 10, 500);
                    }
                    std::vector<KeyPoint> keypoints;
                    detector->detect(gray_frame, keypoints);
                    for (int i = 0; i < keypoints.size(); i++) {
                        Point center = keypoints[i].pt;
                        circle(frame, center, 1, Scalar(100, 0, 100), 3, LINE_AA);
                        int radius = keypoints[i].size;
                        circle(frame, center, radius, Scalar(0, 255, 255), 3, LINE_AA);
                    }
                    for (int i = 0; i < circles_v.size(); i++) {
                        Vec3i c = circles_v[i];
                        Point center = Point(c[0], c[1]);
                        circle(frame, center, 1, Scalar(0, 100, 100), 3, LINE_AA);
                        int radius = c[2];
                        circle(frame, center, radius, Scalar(255, 0, 255), 3, LINE_AA);
                    }
                    out_vid.write(frame);
                    imshow("Video", frame);
                    chrono::steady_clock::time_point current_time = chrono::steady_clock::now();
                    int diff = chrono::duration_cast<chrono::milliseconds>(current_time - last_time).count();
                    last_time = current_time;
                    int wait_time = frame_time - diff;
                    if (wait_time < 1)
                        wait_time = 1;
                    waitKey(wait_time);
                }
            }
            out_vid.release();
            video.release();
        }

    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
    }

    destroyAllWindows();

}

Ptr<cuda::HoughCirclesDetector> houghCircles = nullptr;
Ptr<cuda::Filter> filter = nullptr;


void HoughCirclesCUDA(InputArray image, OutputArray circles,
    double dp, double minDist,
    double param1, double param2,
    int minRadius, int maxRadius) {
    vector<Vec3f> circles_;
    cuda::GpuMat gray_gpu;
    cuda::GpuMat gpu_circles;
    gray_gpu.upload(image.getMat());
    if(houghCircles == nullptr)
        houghCircles = cuda::createHoughCirclesDetector(dp, minDist, param1, param2, minRadius, maxRadius);
    houghCircles->detect(gray_gpu, gpu_circles);
    if (!gpu_circles.empty()) {
        gpu_circles.download(circles_);
        map<pair<int, int>, vector<Vec3f>> point_map;
        for (int i = 0; i < circles_.size(); i++) {
            Vec3i c = circles_[i];
            pair<int, int> center(c[0], c[1]);
            auto it = point_map.find(center);
            if (it != point_map.end())
                it->second.push_back(circles_[i]);
            else {
                vector<Vec3f> v;
                v.push_back(circles_[i]);
                point_map.emplace(center, v);
            }
        }
        circles_.clear();
        for (auto it = point_map.begin(); it != point_map.end(); ++it) {
            float x = 0;
            float y = 0;
            float z = 0;
            for (auto itt = it->second.begin(); itt != it->second.end(); ++itt) {
                x += (*itt)[0];
                y += (*itt)[1];
                z += (*itt)[2];
            }
            int size = it->second.size();
            circles_.push_back(Vec3f(x / size, y / size, z / size));
        }
        Mat _circles(circles_);
        _circles.copyTo(circles);
    }
}

void medianBlurCUDA(InputArray src, OutputArray dst, int ksize) {
    cuda::GpuMat gpu_src;
    cuda::GpuMat gpu_dst;
    gpu_src.upload(src.getMat());
    if(filter == nullptr)
        filter = cuda::createMedianFilter(gpu_src.type(), ksize);
    filter->apply(gpu_src, gpu_dst);
    gpu_dst.download(dst);
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
