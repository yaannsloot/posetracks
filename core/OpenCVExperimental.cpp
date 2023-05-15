// OpenCVExperimental.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <iostream>
#include <fstream>
#include <math.h>
#include <thread>
#include <future>
#include "chessboard_compute.hpp"

using namespace std;
using namespace cv;
using namespace dnn;

void getQuaternion(Mat R, double Q[]);

int main()
{
    try {
        std::cout << "Hello World!\n";

        cout << getBuildInformation() << endl;

        VideoCapture video("trim.0433A616-93D3-4630-9482-5420CC040FF9.MOV");
        if (!video.isOpened())
            cout << "Error opening video file" << endl;
        Mat imt_color = imread("gull.png");
        Mat imt_grayscale = imread("gull.png", IMREAD_GRAYSCALE);

        SimpleBlobDetector::Params params;

        params.minThreshold = 10;
        params.maxThreshold = 200;

        params.filterByArea = true;
        params.minArea = 1500;

        params.filterByCircularity = true;
        params.minCircularity = 0.1;

        params.filterByConvexity = true;
        params.minConvexity = 0.5;

        params.filterByInertia = true;
        params.minInertiaRatio = 0.01;

        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

        std::vector<KeyPoint> keypoints;
        detector->detect(imt_grayscale, keypoints);

        Mat im_keypoints;
        drawKeypoints(imt_grayscale, keypoints, im_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        //if (keypoints.size() > 0) {
        //    KeyPoint main_blob = keypoints.at(0);
        //    int width = (main_blob.pt.x < imt_color.cols - main_blob.pt.x) ? main_blob.pt.x : imt_color.cols - main_blob.pt.x;
        //    int height = (main_blob.pt.y < imt_color.rows - main_blob.pt.y) ? main_blob.pt.y : imt_color.rows - main_blob.pt.y;
        //    width = (width < height) ? width : height;
        //    imt_color = imt_color(Range(main_blob.pt.y - width, main_blob.pt.y + width), Range(main_blob.pt.x - width, main_blob.pt.x + width));
        //}



        std::vector<std::string> class_names;
        std::ifstream ifs(std::string("models/class_names.txt").c_str());
        std::string line;
        while (std::getline(ifs, line))
        {
            class_names.push_back(line);
        }
        ifs.close();

        auto model = readNet("models/DenseNet_161.caffemodel", "models/DenseNet_161.prototxt", "Caffe");

        Mat blob = blobFromImage(imt_color, 0.017, Size(224, 224), Scalar(103.94, 116.78, 123.68));
        model.setInput(blob);
        Mat outputs = model.forward();

        Point classIdPoint;
        double final_prob;
        cout << "xTrainData (python)  = " << endl << format(outputs.reshape(1, 1), Formatter::FMT_PYTHON) << endl << endl;
        Mat scores = outputs.reshape(1, 1);
        cout << scores.cols << " matches";
        minMaxLoc(scores, 0, &final_prob, 0, &classIdPoint);
        final_prob = exp(final_prob) / sum(exp(final_prob) * scores)[0] * -1;
        int label_id = classIdPoint.x;
        std::string out_text = format("%.3f", final_prob);
        out_text += '%';
        int scale = imt_color.rows / 500;
        if (scale < 1) scale = 1;
        putText(imt_color, class_names[label_id].c_str(), Point(7 * scale, 50 * scale), FONT_HERSHEY_SIMPLEX, scale, Scalar(0, 255, 0), 2);
        putText(imt_color, out_text, Point(7 * scale, 84 * scale), FONT_HERSHEY_SIMPLEX, scale, Scalar(0, 255, 0), 2);

        namedWindow("Image", WINDOW_NORMAL);
        namedWindow("Video", WINDOW_NORMAL);
        imshow("Image", imt_color);
        waitKey(0);
        imshow("Image", imt_grayscale);
        waitKey(0);
        imshow("Image", im_keypoints);
        waitKey(0);

        vector<vector<Point3f>> objpoints;
        vector<vector<Point2f>> imgpoints;
        vector<Point3f> objp;
        for (int i = 0; i < 14; i++) {
            for (int j = 0; j < 9; j++) {
                objp.push_back(Point3f(j, i, 0));
            }
        }
        vector<Point2f> corner_pts;
        unsigned int thread_count = 1;// thread::hardware_concurrency();
        cout << endl << "Obtaining checkerboard points... (0%)";
        int frame_count = 0;
        int total_frames = video.get(CAP_PROP_FRAME_COUNT);
        Size vid_size(video.get(CAP_PROP_FRAME_WIDTH), video.get(CAP_PROP_FRAME_HEIGHT));
        while (video.isOpened()) {
            vector<Mat> frames;
            vector<Mat> frames_resize;
            vector<vector<Point2f>> batch_points;
            bool success = true;
            for (int i = 0; i < thread_count; i++) {
                Mat f;
                success = video.read(f);
                if (success) frames.push_back(f);
                else break;
            }
            batch_points.resize(frames.size());
            for (Mat f : frames) {
                Mat f2;
                resize(f, f2, Size(1280, 720));
                frames_resize.push_back(f2);
            }
            double scale_factor = 1;// frames[0].cols / frames_resize[0].cols;
            processFramesMT(frames_resize, Size(9, 14), batch_points, CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_EXHAUSTIVE, true);
            frame_count += frames.size();
            cout << "\rObtaining checkerboard points... (" << format("%.2f", ((double)frame_count / (double)total_frames) * 100) << "%)                                  ";
            for (auto pv = batch_points.begin(); pv != batch_points.end(); ++pv) {
                for (auto ip = pv->begin(); ip != pv->end(); ++ip) {
                    ip->x *= scale_factor;
                    ip->y *= scale_factor;
                }
                objpoints.push_back(objp);
            }
            for (int i = 0; i < frames.size(); i++) {
                imgpoints.push_back(batch_points[i]);
            }
            if (!success)
                break;
        }
        cout << "\rObtaining checkerboard points... (100%)                                  " << endl;

        /*
        vector<vector<Point2f>> imgpoints_cal;
        int calibration_frames = 10;
        int calibration_offset = imgpoints.size() / calibration_frames;
        for (int i = 0; i < calibration_frames; i++) {
            imgpoints_cal.push_back(imgpoints.at(i * calibration_offset));
        }
        video.release();
        Mat mtx, dist, rvecs, tvecs;
        double ret = calibrateCamera(objpoints, imgpoints_cal, vid_size, mtx, dist, rvecs, tvecs);
        Mat cam_mtx = getOptimalNewCameraMatrix(mtx, dist, vid_size, 1, vid_size);

        double sensor_width = 7.6608;
        double sensor_height = 5.7456;
        double fx = mtx.ptr<double>(0)[0];
        double fy = mtx.ptr<double>(1)[1];
        fx = fx * sensor_width / vid_size.width;
        fy = fy * sensor_height / vid_size.height;
        */

        video.open("trim.0433A616-93D3-4630-9482-5420CC040FF9.MOV");
        VideoWriter out_vid("out.mp4", VideoWriter::fourcc('m', 'p', '4', 'v'), video.get(CAP_PROP_FPS), Size(video.get(CAP_PROP_FRAME_WIDTH), video.get(CAP_PROP_FRAME_HEIGHT)));
        frame_count = 0;
        int key = -1;
        while (video.isOpened()) {
            Mat frame;
            bool success = video.read(frame);
            if (success) {
                vector<Point2f> pts;
                vector<Point3f> cube_pts;
                vector<Point2f> cube_pts_img;
                for (int i = 0; i < 13; i++) {
                    for (int j = 0; j < 13; j++) {
                        for (int k = 0; k < 13; k++) {
                            cube_pts.push_back(Point3f((float)i / 2, (float)j / 2, (float)k / 2 * -1));
                        }
                    }
                }
                cube_pts_img.resize(cube_pts.size());

                
                Mat rvec_c, tvec_c;
                double retc = solvePnP(objp, imgpoints[frame_count], mtx, dist, rvec_c, tvec_c);
                projectPoints(objp, rvec_c, tvec_c, mtx, dist, pts);
                projectPoints(cube_pts, rvec_c, tvec_c, mtx, dist, cube_pts_img);

                // Getting camera coordinates
                Mat R;
                Rodrigues(rvec_c, R);
                R = R.t();
                Mat tvec_cam = -R * tvec_c;

                // R is a 3x3 Rotation matrix of the camera
                // tvec_cam is a 3x1 matrix of the camera position

                std::vector<double> tvec_vector;
                for (int i = 0; i < tvec_cam.rows; i++) {
                    tvec_vector.push_back(tvec_cam.ptr<double>(i)[0]);
                }
                stringstream ss;
                ss << "position(XYZ)=[ " << tvec_vector.at(0) * -1 << ", " << tvec_vector.at(1) << ", " << tvec_vector.at(2) * -1 << " ]";
                string pos_vec = ss.str();
                ss.clear();
                ss.str("");
                double Q[4];
                getQuaternion(R, Q);
                ss << "rotation(WXYZ)=[ " << Q[2] << ", " << Q[1] << ", " << Q[0] << ", " << Q[3] << " ]";
                string rot_vec = ss.str();
                


                //frame.setTo(Scalar(0, 0, 0));
                Mat frame_fix;
                undistort(frame, frame_fix, mtx, dist);
                frame = frame_fix;
                

                //pts.resize(imgpoints[frame_count].size());


                vector<Point2f> points;
                points.resize(imgpoints[frame_count].size());
                //undistortImagePoints(imgpoints[frame_count], points, mtx, dist, TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.001));
                //imgpoints[frame_count] = points;

                drawChessboardCorners(frame, Size(9, 14), imgpoints[frame_count], true);

                /*
                for (int i = 0; i < cube_pts_img.size(); i++) {
                    drawMarker(frame, cube_pts_img[i], Scalar(0, 255, 0));
                }
                */

                int display_scale = frame.rows / 500;
                string frame_text = format("current_frame: %d", frame_count);
                putText(frame, frame_text, Point(4 * display_scale, 50 * display_scale), FONT_HERSHEY_SIMPLEX, display_scale, Scalar(0, 255, 0), 2);

                /*
                putText(frame, pos_vec, Point(4 * display_scale, 100 * display_scale), FONT_HERSHEY_SIMPLEX, display_scale, Scalar(255, 0, 0), 2);
                putText(frame, rot_vec, Point(4 * display_scale, 150 * display_scale), FONT_HERSHEY_SIMPLEX, display_scale, Scalar(0, 0, 255), 2);
                */

                //putText(frame, format("fx: %.2fmm", fx), Point(4 * display_scale, 200 * display_scale), FONT_HERSHEY_SIMPLEX, display_scale, Scalar(0, 255, 255), 2);
                //putText(frame, format("fy: %.2fmm", fy), Point(4 * display_scale, 250 * display_scale), FONT_HERSHEY_SIMPLEX, display_scale, Scalar(255, 0, 255), 2);

                imshow("Video", frame);
                out_vid.write(frame);
                key = waitKey(1000 / video.get(CAP_PROP_FPS));
                if (key > -1)
                    break;
                frame_count++;
            }
            if (!success)
                break;
        }
        video.release();
        out_vid.release();

        destroyAllWindows();
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
    }
}

void getQuaternion(Mat R, double Q[])
{
    double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);

    if (trace > 0.0)
    {
        double s = sqrt(trace + 1.0);
        Q[3] = (s * 0.5);
        s = 0.5 / s;
        Q[0] = ((R.at<double>(2, 1) - R.at<double>(1, 2)) * s);
        Q[1] = ((R.at<double>(0, 2) - R.at<double>(2, 0)) * s);
        Q[2] = ((R.at<double>(1, 0) - R.at<double>(0, 1)) * s);
    }

    else
    {
        int i = R.at<double>(0, 0) < R.at<double>(1, 1) ? (R.at<double>(1, 1) < R.at<double>(2, 2) ? 2 : 1) : (R.at<double>(0, 0) < R.at<double>(2, 2) ? 2 : 0);
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;

        double s = sqrt(R.at<double>(i, i) - R.at<double>(j, j) - R.at<double>(k, k) + 1.0);
        Q[i] = s * 0.5;
        s = 0.5 / s;

        Q[3] = (R.at<double>(k, j) - R.at<double>(j, k)) * s;
        Q[j] = (R.at<double>(j, i) + R.at<double>(i, j)) * s;
        Q[k] = (R.at<double>(k, i) + R.at<double>(i, k)) * s;
    }
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
