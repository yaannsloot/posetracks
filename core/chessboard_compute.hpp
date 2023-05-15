#ifndef CHESSBOARD_COMPUTE_HPP
#define CHESSBOARD_COMPUTE_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <future>
#include <thread>
#include <atomic>

namespace cv {
    
    namespace aruco {

        void detectMarkersCompute(InputArray image, const Ptr<Dictionary>& dictionary, OutputArrayOfArrays corners,
            OutputArray ids, const Ptr<DetectorParameters>& parameters = DetectorParameters::create(),
            OutputArrayOfArrays rejectedImgPoints = noArray());

    }

    bool findChessboardCornersCUDA(cv::InputArray image_, cv::Size pattern_size,
        cv::OutputArray corners_, int flags);

    void processFramesMT(std::vector<Mat>& images, cv::Size pattern_size,
        std::vector<std::vector<Point2f>>& image_corners, std::vector<bool>& success, int flags, bool useGPU = false);

    void processFramesMT(std::vector<Mat> &images, cv::Size pattern_size,
        std::vector<std::vector<Point2f>> &image_corners, int flags, bool useGPU = false);

    void processFramesMT(std::vector<Mat>& images, const cv::Ptr<cv::aruco::CharucoBoard>& board, std::vector<std::vector<cv::Point2f>>& charucoCorners,
        std::vector<std::vector<int>>& charucoIds, std::vector<bool>& success, const cv::Ptr<cv::aruco::DetectorParameters>& parameters = cv::aruco::DetectorParameters::create(), bool useGPU = false);

    void processFramesMT(std::vector<Mat>& images, const cv::Ptr<cv::aruco::CharucoBoard>& board, std::vector<std::vector<cv::Point2f>>& charucoCorners,
        std::vector<std::vector<int>>& charucoIds, const cv::Ptr<cv::aruco::DetectorParameters>& parameters = cv::aruco::DetectorParameters::create(), bool useGPU = false);

    void detectMarkersMT(std::vector<Mat>& images, const cv::Ptr<cv::aruco::Dictionary>& dictionary, std::vector<std::vector<std::vector<cv::Point2f>>>& markerCorners,
        std::vector<std::vector<int>>& markerIds, std::vector<bool>& success, const cv::Ptr<cv::aruco::DetectorParameters>& parameters = cv::aruco::DetectorParameters::create(), bool useGPU = false);

    void detectMarkersMT(std::vector<Mat>& images, const cv::Ptr<cv::aruco::Dictionary>& dictionary, std::vector<std::vector<std::vector<cv::Point2f>>>& markerCorners,
        std::vector<std::vector<int>>& markerIds, const cv::Ptr<cv::aruco::DetectorParameters>& parameters = cv::aruco::DetectorParameters::create(), bool useGPU = false);

}

namespace me {

    struct ComputeTaskParams {
        cv::Ptr<cv::aruco::CharucoBoard> board;
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        std::vector<std::vector<cv::Point2f>>* board_corners;
        std::vector<std::vector<int>>* board_ids;
        std::vector<bool>* board_success;
        std::vector<std::vector<std::vector<cv::Point2f>>>* marker_corners;
        std::vector<std::vector<int>>* marker_ids;
        std::vector<bool>* marker_success;
        cv::Ptr<cv::aruco::DetectorParameters> marker_parameters;
        std::string filepath;

        ComputeTaskParams() :
            board(nullptr),
            dictionary(nullptr),
            board_corners(nullptr),
            board_ids(nullptr),
            board_success(nullptr),
            marker_corners(nullptr),
            marker_ids(nullptr),
            marker_success(nullptr),
            marker_parameters(cv::aruco::DetectorParameters::create())
        {}
    };

    class ARComputeTask {
    public:
        ARComputeTask(ComputeTaskParams& params);
        bool start(const uint32_t numthreads = std::thread::hardware_concurrency());
        void stop();
        bool busy();
        void wait();
        int get_total();
        int get_remaining();
        int get_complete();
        ~ARComputeTask();
    private:
        cv::VideoCapture video;
        ComputeTaskParams params;
        int total_frames = 0;
        std::atomic<int> counter = 0;
        bool should_terminate = false;           // Tells threads to stop looking for jobs
        std::mutex count_mutex;                  // Prevents data races to the job queue
        std::mutex reader_mutex;
        std::vector<std::thread> threads;
    };

}

#endif

