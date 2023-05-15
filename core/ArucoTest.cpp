#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include <iostream>
#include <hpdf.h>
#include <cstring>
#include <chessboard_compute.hpp>
#include <MEComputeLib.h>
#include <windows.h>
#include <fstream>
#include <nlohmann/json.hpp>
#include <MECore/motion_engine.hpp>
#include <MECCTag/me_cctag.hpp>
#include <MECore/me_utils.hpp>
#include <MECore/me_crypto.hpp>
#include <random>
#include <bezier.h>
#include <opencv2/plot.hpp>
#include <opencv2/imgproc/types_c.h>
#include <boost/program_options.hpp>
#include <torch/script.h>
#include <me_dnn_module.hpp>

using namespace std;
using namespace cv;
using namespace boost::program_options;
using namespace std;
using namespace cv::dnn;
using json = nlohmann::json;

jmp_buf env;

void error_handler(HPDF_STATUS   error_no,
    HPDF_STATUS   detail_no,
    void* user_data)
{
    printf("ERROR: error_no=%04X, detail_no=%u\n", (HPDF_UINT)error_no,
        (HPDF_UINT)detail_no);
    longjmp(env, 1);
}

bool HPDF_Draw_Marker(HPDF_Page page, Mat m, float loc_x, float loc_y, float width);

void Generate_Synchronization_Video(std::string out_file, cv::Ptr<cv::aruco::Dictionary>& dictionary, int marker_id, float size_ratio, int size, float delay_interval, int start_delays, int end_delays);

int export_marker_pdf(std::string of);

int old_test();

int export_marker_pdf(std::string of) {
    cv::Mat markerImage;
    int dict_predef = aruco::DICT_7X7_1000;
    const string dict_names[16] = { "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000",
        "DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250", "DICT_5X5_1000",
        "DICT_6X6_50", "DICT_6X6_100", "DICT_6X6_250", "DICT_6X6_1000",
        "DICT_7X7_50", "DICT_7X7_100", "DICT_7X7_250", "DICT_7X7_1000", };
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    dictionary = cv::aruco::getPredefinedDictionary(dict_predef);
    map<string, Mat> aruco_markers;
    for (int i = 0; i < 128; i++) {
        cv::aruco::drawMarker(dictionary, i, 200, markerImage, 1);
        stringstream ss;
        ss << "markers/" << "marker";
        if (i < 10)
            ss << '0';
        ss << i << ".png";
        cv::imwrite(ss.str(), markerImage);
        cv::aruco::drawMarker(dictionary, i, dictionary.get()->markerSize + 2, markerImage, 1);
        Mat m(markerImage);
        aruco_markers.emplace(ss.str(), m);
    }

    // Marker PDF generator parameters
    int pdf_dpi = 72;
    float pdf_dim_x = 8.5;
    float pdf_dim_y = 11;
    float marker_dim = 2;
    float corner_offset = 0.3;
    float min_distance_x = 0.3;
    float min_distance_y = 0.3;
    float max_horiz_label_scale = 0.8;
    float max_vert_label_scale = 0.8;
    float label_height = 0.3;
    float title_scale = 0.6;

    // Generator computed values
    int pdf_coords_x = pdf_dim_x * pdf_dpi;
    int pdf_coords_y = pdf_dim_y * pdf_dpi;
    float marker_width = marker_dim * pdf_dpi;
    float corner_width = corner_offset * pdf_dpi;
    float fillable_space_x = pdf_coords_x - corner_width * 2;
    float fillable_space_y = pdf_coords_y - corner_width * 2;
    float marker_spacing_x = fillable_space_x;
    float marker_spacing_y = fillable_space_y;
    float min_coord_x = min_distance_x * pdf_dpi;
    float min_coord_y = min_distance_y * pdf_dpi;
    int num_marker_x = fillable_space_x / marker_width;
    int num_marker_y = fillable_space_y / marker_width;
    cout << num_marker_x << ' ' << num_marker_y << endl;
    if (num_marker_x > 1)
        marker_spacing_x = (fillable_space_x - marker_width * num_marker_x) / (num_marker_x + 1);
    else if (num_marker_x == 1)
        marker_spacing_x = (fillable_space_x - marker_width) / 2;
    else
        return 1;
    if (num_marker_y > 1)
        marker_spacing_y = (fillable_space_y - marker_width * num_marker_y) / (num_marker_y + 1);
    else if (num_marker_y == 1)
        marker_spacing_y = (fillable_space_y - marker_width) / 2;
    else
        return 1;
    while (num_marker_x > 1) {
        if (marker_spacing_x >= min_coord_x)
            break;
        num_marker_x--;
        if (num_marker_x == 1)
            break;
        marker_spacing_x = (fillable_space_x - marker_width * num_marker_x) / (num_marker_x + 1);
    }
    if (num_marker_x == 1)
        marker_spacing_x = (fillable_space_x - marker_width) / 2;
    while (num_marker_y > 1) {
        if (marker_spacing_y >= min_coord_y)
            break;
        num_marker_y--;
        if (num_marker_y == 1)
            break;
        marker_spacing_y = (fillable_space_y - marker_width * num_marker_y) / (num_marker_y + 1);
    }
    if (num_marker_y == 1)
        marker_spacing_y = (fillable_space_y - marker_width) / 2;
    float font_size = label_height * pdf_dpi;
    if (font_size > marker_spacing_y * max_vert_label_scale)
        font_size = marker_spacing_y * max_vert_label_scale;
    int page_count = aruco_markers.size() / (num_marker_x * num_marker_y);
    if (aruco_markers.size() > page_count * num_marker_x * num_marker_y)
        page_count += 1;

    // PDF creation
    HPDF_Doc pdf = HPDF_New(error_handler, NULL);
    HPDF_SetCompressionMode(pdf, 0x0F);
    HPDF_SetPageMode(pdf, HPDF_PAGE_MODE_USE_OUTLINE);


    auto it = aruco_markers.begin();
    for (int p = 1; p <= page_count; p++) {
        HPDF_Page page = HPDF_AddPage(pdf);
        HPDF_Page_SetSize(page, HPDF_PAGE_SIZE_LETTER, HPDF_PAGE_PORTRAIT);
        HPDF_Page_BeginText(page);


        // Side title drawing
        stringstream title;
        title << "aruco::" << dict_names[dict_predef] << ", Page " << p << " of " << page_count;
        HPDF_Page_SetFontAndSize(page, HPDF_GetFont(pdf, "Helvetica", NULL), corner_width * title_scale);
        HPDF_Page_TextOut(page, pdf_coords_x / 2 - HPDF_Page_TextWidth(page, title.str().c_str()) / 2, pdf_coords_y - corner_width, title.str().c_str());

        // Finish page title writing
        HPDF_Page_EndText(page);

        // Corner box drawing
        HPDF_Page_SetRGBFill(page, 0, 0, 0);
        HPDF_Page_Rectangle(page, 0, 0, corner_width, corner_width);
        HPDF_Page_Rectangle(page, pdf_coords_x - corner_width, 0, corner_width, corner_width);
        HPDF_Page_Rectangle(page, 0, pdf_coords_y - corner_width, corner_width, corner_width);
        HPDF_Page_Rectangle(page, pdf_coords_x - corner_width, pdf_coords_y - corner_width, corner_width, corner_width);
        HPDF_Page_FillStroke(page);

        // Marker drawing
        cout << num_marker_x << ' ' << num_marker_y << endl;
        for (int y = 0; y < num_marker_y && it != aruco_markers.end(); y++) {
            for (int x = 0; x < num_marker_x && it != aruco_markers.end(); x++) {
                float pos_x = marker_spacing_x + corner_width + x * (marker_width + marker_spacing_x);
                float pos_y = marker_spacing_y + corner_width + (num_marker_y - y - 1) * (marker_width + marker_spacing_y);
                if (num_marker_x == 1)
                    pos_x = corner_width + marker_spacing_x;
                if (num_marker_y == 1)
                    pos_y = corner_width + marker_spacing_y;
                HPDF_Draw_Marker(page, it->second, pos_x, pos_y, marker_width);
                ++it;
            }
        }
    }

    // Save to test.pdf
    HPDF_SaveToFile(pdf, "of");

    // END ARUCO PDF GENERATION
}

// YOLOv5 Constants
// Constants.
const float INPUT_WIDTH = 640.0;
const float INPUT_HEIGHT = 640.0;
const float SCORE_THRESHOLD = 0.5;
const float NMS_THRESHOLD = 0.45;
const float CONFIDENCE_THRESHOLD = 0.45;

// Text parameters.
const float FONT_SCALE = 0.7;
const int FONT_FACE = FONT_HERSHEY_SIMPLEX;
const int THICKNESS = 1;

// Colors.
Scalar BLACK = Scalar(0, 0, 0);
Scalar BLUE = Scalar(255, 178, 50);
Scalar YELLOW = Scalar(0, 255, 255);
Scalar RED = Scalar(0, 0, 255);

// YOLOv5 functions

std::vector<float> LetterboxImage(const cv::Mat& src, cv::Mat& dst, const cv::Size& out_size) {
    auto in_h = static_cast<float>(src.rows);
    auto in_w = static_cast<float>(src.cols);
    float out_h = out_size.height;
    float out_w = out_size.width;

    float scale = std::min(out_w / in_w, out_h / in_h);

    int mid_h = static_cast<int>(in_h * scale);
    int mid_w = static_cast<int>(in_w * scale);

    cuda::GpuMat gpuIn;
    cuda::GpuMat gpuOut;

    gpuIn.upload(src);

    cuda::resize(gpuIn, gpuOut, cv::Size(mid_w, mid_h));

    int top = (static_cast<int>(out_h) - mid_h) / 2;
    int down = (static_cast<int>(out_h) - mid_h + 1) / 2;
    int left = (static_cast<int>(out_w) - mid_w) / 2;
    int right = (static_cast<int>(out_w) - mid_w + 1) / 2;

    cuda::copyMakeBorder(gpuOut, gpuOut, top, down, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

    gpuOut.download(dst);

    std::vector<float> pad_info{ static_cast<float>(left), static_cast<float>(top), scale };
    return pad_info;
}

enum Det {
    tl_x = 0,
    tl_y = 1,
    br_x = 2,
    br_y = 3,
    score = 4,
    class_idx = 5
};

struct Detection {
    cv::Rect bbox;
    float score;
    int class_idx;
};

void Tensor2Detection(const at::TensorAccessor<float, 2>& offset_boxes,
    const at::TensorAccessor<float, 2>& det,
    std::vector<cv::Rect>& offset_box_vec,
    std::vector<float>& score_vec) {

    for (int i = 0; i < offset_boxes.size(0); i++) {
        offset_box_vec.emplace_back(
            cv::Rect(cv::Point(offset_boxes[i][Det::tl_x], offset_boxes[i][Det::tl_y]),
                cv::Point(offset_boxes[i][Det::br_x], offset_boxes[i][Det::br_y]))
        );
        score_vec.emplace_back(det[i][Det::score]);
    }
}

torch::Tensor xywh2xyxy(const torch::Tensor& x) {
    auto y = torch::zeros_like(x);
    // convert bounding box format from (center x, center y, width, height) to (x1, y1, x2, y2)
    y.select(1, Det::tl_x) = x.select(1, 0) - x.select(1, 2).div(2);
    y.select(1, Det::tl_y) = x.select(1, 1) - x.select(1, 3).div(2);
    y.select(1, Det::br_x) = x.select(1, 0) + x.select(1, 2).div(2);
    y.select(1, Det::br_y) = x.select(1, 1) + x.select(1, 3).div(2);
    return y;
}

void ScaleCoordinates(std::pair<Point2f, Point2f>& box, float pad_w, float pad_h,
    float scale, const cv::Size& img_shape) {
    auto clip = [](float n, float lower, float upper) {
        return std::max(lower, std::min(n, upper));
    };

    float x1 = (box.first.x - pad_w) / scale;  // x padding
    float y1 = (box.first.y - pad_h) / scale;  // y padding
    float x2 = (box.second.x - pad_w) / scale;  // x padding
    float y2 = (box.second.y - pad_h) / scale;  // y padding

    x1 = clip(x1, 0, img_shape.width);
    y1 = clip(y1, 0, img_shape.height);
    x2 = clip(x2, 0, img_shape.width);
    y2 = clip(y2, 0, img_shape.height);

    box.first.x = x1;
    box.first.y = y1;
    box.second.x = x2;
    box.second.y = y2;
}

std::vector<std::vector<Detection>> PostProcessing(const torch::Tensor& detections,
    float pad_w, float pad_h, float scale, const cv::Size& img_shape,
    float conf_thres, float iou_thres) {
    constexpr int item_attr_size = 5;
    int batch_size = detections.size(0);
    // number of classes, e.g. 80 for coco dataset
    auto num_classes = detections.size(2) - item_attr_size;

    // get candidates which object confidence > threshold
    auto conf_mask = detections.select(2, 4).ge(conf_thres).unsqueeze(2);

    std::vector<std::vector<Detection>> output;
    output.reserve(batch_size);

    // iterating all images in the batch
    for (int batch_i = 0; batch_i < batch_size; batch_i++) {
        // apply constrains to get filtered detections for current image
        auto det = torch::masked_select(detections[batch_i], conf_mask[batch_i]).view({ -1, num_classes + item_attr_size });

        // if none detections remain then skip and start to process next image
        if (0 == det.size(0)) {
            continue;
        }

        // compute overall score = obj_conf * cls_conf, similar to x[:, 5:] *= x[:, 4:5]
        det.slice(1, item_attr_size, item_attr_size + num_classes) *= det.select(1, 4).unsqueeze(1);

        // box (center x, center y, width, height) to (x1, y1, x2, y2)
        torch::Tensor box = xywh2xyxy(det.slice(1, 0, 4));

        // [best class only] get the max classes score at each result (e.g. elements 5-84)
        std::tuple<torch::Tensor, torch::Tensor> max_classes = torch::max(det.slice(1, item_attr_size, item_attr_size + num_classes), 1);

        // class score
        auto max_conf_score = std::get<0>(max_classes);
        // index
        auto max_conf_index = std::get<1>(max_classes);

        max_conf_score = max_conf_score.to(torch::kFloat).unsqueeze(1);
        max_conf_index = max_conf_index.to(torch::kFloat).unsqueeze(1);

        // shape: n * 6, top-left x/y (0,1), bottom-right x/y (2,3), score(4), class index(5)
        det = torch::cat({ box.slice(1, 0, 4), max_conf_score, max_conf_index }, 1);

        // for batched NMS
        constexpr int max_wh = 4096;
        auto c = det.slice(1, item_attr_size, item_attr_size + 1) * max_wh;
        auto offset_box = det.slice(1, 0, 4) + c;

        std::vector<cv::Rect> offset_box_vec;
        std::vector<float> score_vec;

        // copy data back to cpu
        auto offset_boxes_cpu = offset_box.cpu();
        auto det_cpu = det.cpu();
        const auto& det_cpu_array = det_cpu.accessor<float, 2>();

        // use accessor to access tensor elements efficiently
        Tensor2Detection(offset_boxes_cpu.accessor<float, 2>(), det_cpu_array, offset_box_vec, score_vec);

        // run NMS
        std::vector<int> nms_indices;
        cv::dnn::NMSBoxes(offset_box_vec, score_vec, conf_thres, iou_thres, nms_indices);

        std::vector<Detection> det_vec;
        for (int index : nms_indices) {
            Detection t;
            const auto& b = det_cpu_array[index];
            t.bbox =
                cv::Rect(cv::Point(b[Det::tl_x], b[Det::tl_y]),
                    cv::Point(b[Det::br_x], b[Det::br_y]));
            t.score = det_cpu_array[index][Det::score];
            t.class_idx = det_cpu_array[index][Det::class_idx];
            det_vec.emplace_back(t);
        }

        //ScaleCoordinates(det_vec, pad_w, pad_h, scale, img_shape);

        // save final detection for the current image
        output.emplace_back(det_vec);
    } // end of batch iterating

    return output;
}

void Demo(cv::Mat& img,
    const std::vector<std::vector<Detection>>& detections,
    const std::vector<std::string>& class_names,
    bool label = true, bool draw_marker_labels = true) {

    if (!detections.empty()) {
        for (const auto& detection : detections[0]) {
            const auto& box = detection.bbox;
            float score = detection.score;
            int class_idx = detection.class_idx;

            cv::rectangle(img, box, cv::Scalar(0, 0, 255), 2);

            if (label && !(!draw_marker_labels && class_names[class_idx].compare("marker") == 0)) {
                std::stringstream ss;
                ss << std::fixed << std::setprecision(2) << score;
                std::string s = class_names[class_idx] + " " + ss.str();

                auto font_face = cv::FONT_HERSHEY_DUPLEX;
                auto font_scale = 1.0;
                int thickness = 1;
                int baseline = 0;
                auto s_size = cv::getTextSize(s, font_face, font_scale, thickness, &baseline);
                cv::rectangle(img,
                    cv::Point(box.tl().x, box.tl().y - s_size.height - 5),
                    cv::Point(box.tl().x + s_size.width, box.tl().y),
                    cv::Scalar(0, 0, 255), -1);
                cv::putText(img, s, cv::Point(box.tl().x, box.tl().y - 5),
                    font_face, font_scale, cv::Scalar(255, 255, 255), thickness);
            }
        }
    }
}

std::pair<cv::Point2f, cv::Point2f> extractRegionWithPadding(const cv::Mat& input, cv::Mat& output, cv::Point2f center, int region_width) {
    // Calculate the top-left corner of the region
    cv::Point2f topLeft(center.x - region_width / 2.0f, center.y - region_width / 2.0f);
    cv::Point2f bottomRight(topLeft.x + region_width, topLeft.y + region_width);

    // Create an empty output image with a gray background
    output = cv::Mat(region_width, region_width, input.type(), cv::Scalar(114, 114, 114));

    // Calculate the overlap between the input image and the region
    cv::Rect inputRect(0, 0, input.cols, input.rows);
    cv::Rect regionRect(topLeft, cv::Size(region_width, region_width));
    cv::Rect overlapRect = inputRect & regionRect;

    // Check if the region is entirely outside the input image
    if (overlapRect.empty()) {
        return std::make_pair(topLeft, bottomRight);
    }

    // Copy the overlapping part of the input image to the output image
    cv::Mat inputROI = input(overlapRect);
    cv::Rect outputROI(overlapRect.x - regionRect.x, overlapRect.y - regionRect.y, overlapRect.width, overlapRect.height);
    inputROI.copyTo(output(outputROI));

    return std::make_pair(topLeft, bottomRight);
}

class MovingAverage {
public:
    MovingAverage(size_t size) : maxSize(size), buffer(size), count(0), start(0), sum(0) {}

    void add(double value) {
        if (count < maxSize) {
            count++;
        }
        else {
            sum -= buffer[start];
            start = (start + 1) % maxSize;
        }
        buffer[(start + count - 1) % maxSize] = value;
        sum += value;
    }

    double getAverage() const {
        if (count == 0) {
            throw std::runtime_error("Cannot calculate average of an empty buffer.");
        }
        return sum / count;
    }

private:
    size_t maxSize;
    std::vector<double> buffer;
    size_t count;
    size_t start;
    double sum;
};

class AveragingTracker {
public:
    AveragingTracker() : count(0), sum(0), lowest(std::numeric_limits<double>::max()), highest(std::numeric_limits<double>::min()) {}

    void add(double value) {
        count++;
        sum += value;

        if (value < lowest) {
            lowest = value;
        }

        if (value > highest) {
            highest = value;
        }
    }

    double getMean() const {
        if (count == 0) {
            throw std::runtime_error("Cannot calculate the mean of an empty set.");
        }
        return sum / count;
    }

    double getLowest() const {
        if (count == 0) {
            throw std::runtime_error("Cannot get the lowest value of an empty set.");
        }
        return lowest;
    }

    double getHighest() const {
        if (count == 0) {
            throw std::runtime_error("Cannot get the highest value of an empty set.");
        }
        return highest;
    }

private:
    size_t count;
    double sum;
    double lowest;
    double highest;
};

int main(int argc, const char* argv[])
{
    try
    {
        options_description desc{"Options"};
        desc.add_options()
            ("help", "Display help screen")
            ("video", value<string>(), "Set video input file path")
            ("output", value<string>(), "Set video output file path")
            ("weights", value<string>()->default_value("MEMarkerDetectYOLOv5.torchscript"), "Set neural net onnx weights file path")
            ("imgsz", value<int>()->default_value(640), "Set model input dimensions")
            ("conf", value<float>()->default_value(0.66), "Set confindence threshold")
            ("iou", value<float>()->default_value(0.2), "Set IOU threshold")
            ("draw-m-labels", value<bool>()->default_value(true), "Draw marker labels")
            ("half", value<bool>()->default_value(false), "Set input to half precision for use with FP16 models");
        variables_map vm;
        store(parse_command_line(argc, argv, desc), vm);
        notify(vm);
    
        if (vm.count("help"))
            std::cout << desc << '\n';
        else if (vm.count("video") && vm.count("output"))
        {
            cout << "Input: " << vm["video"].as<string>() << endl;
            cout << "Output: " << vm["output"].as<string>() << endl;
            cout << "Model weights: " << vm["weights"].as<string>() << endl;

            me::dnn::MEDNNModuleInstance darknet_module("MEDNNDarknet.dll");

            std::unordered_map<std::string, std::any> d = std::any_cast<std::unordered_map<std::string, std::any>>(darknet_module.GetModuleDetails());
            std::cout << std::any_cast<const char*>(d["framework"]) << std::endl << std::any_cast<const char*>(d["module_version"]) << std::endl;
            
            std::unordered_map<std::string, std::any> d_conf;
            d_conf["cfg_file"] = std::string("dataset_tracking_target_all.cfg");
            d_conf["weights_file"] = std::string("best_pre_tuning.weights");
            d_conf["thresh"] = 0.5f;
            d_conf["hier_thresh"] = 0.5f;
            d_conf["nms"] = 0.2f;
            std::any dc = d_conf;
            darknet_module.LoadModel(dc);
            darknet_module.SetForwardRules(dc);

            float scale_bounds = 0.1f;

            vector<string> class_list;
            class_list.push_back("tracking_target");
            class_list.push_back("pole_target");

            MovingAverage ma(12);
            MovingAverage ma_center_x(6);
            MovingAverage ma_center_y(6);
            AveragingTracker t_pre;
            AveragingTracker t_inf;
            AveragingTracker t_forward;
            AveragingTracker confidence;
            std::ofstream outputFile("report.csv");
            outputFile << vm["video"].as<string>() << ",t.Pre (ms),t.Inf (ms),t.Forward (ms),C-Score Avg%" << std::endl;

#define NO_TORCH
#ifndef NO_TORCH

            // Load and prepare model
            /*
            YOLODetector detector{ nullptr };
            detector = YOLODetector(vm["weights"].as<string>(), true, cv::Size(1280, 1280));
            */

            
            torch::jit::Module module;
            c10::DeviceType torch_device;
            if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
                torch_device = torch::kCUDA;
                module = torch::jit::load(vm["weights"].as<string>(), torch_device);
                std::cout << "Using libtorch GPU device" << std::endl;
            }
            else {
                torch_device = torch::kCPU;
                module = torch::jit::load(vm["weights"].as<string>(), torch_device);
                std::cout << "Using libtorch CPU device" << std::endl;
            }
#endif

            bool success = true;
            VideoCapture input(vm["video"].as<string>(), cv::CAP_FFMPEG, { cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY });
            VideoWriter output(vm["output"].as<string>(), cv::CAP_FFMPEG,
                VideoWriter::fourcc('m', 'p', '4', 'v'), input.get(CAP_PROP_FPS), Size(input.get(CAP_PROP_FRAME_WIDTH), input.get(CAP_PROP_FRAME_HEIGHT)), { cv::VIDEOWRITER_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY });
            Size frame_size(input.get(CAP_PROP_FRAME_WIDTH), input.get(CAP_PROP_FRAME_HEIGHT));
            Size model_size(416, 416);
            VideoWriter output2("NetTester/10mm_CROP_2.mp4", cv::CAP_FFMPEG,
                VideoWriter::fourcc('m', 'p', '4', 'v'), input.get(CAP_PROP_FPS), model_size, {cv::VIDEOWRITER_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY});
            string padding = "";
            int frame_num = 0;
            while (success) {
                Mat frame;
                success = input.read(frame);
                if (success) {

                    // Neural net processing
                    /*
                    std::vector<Detection> result;
                    result = detector.detect(frame, 0.3, 0.4);
                    detectutils::visualizeDetection(frame, result, class_list);
                    */

                    auto start = std::chrono::high_resolution_clock::now();
#ifndef NO_TORCH
                    
                    Mat frame_rgb;
                    cv::cvtColor(frame, frame_rgb, cv::COLOR_BGR2RGB);
                    Mat img;
                    std::vector<float> pad_info = LetterboxImage(frame_rgb, img, Size(vm["imgsz"].as<int>(), vm["imgsz"].as<int>()));
                    const float pad_w = pad_info[0];
                    const float pad_h = pad_info[1];
                    const float scale = pad_info[2];
                    img.convertTo(img, CV_32FC3, 1.0f / 255.0f);
                    torch::Tensor input_tensor = torch::from_blob(img.data, { 1, vm["imgsz"].as<int>(), vm["imgsz"].as<int>(), 3 });
                    if (vm["half"].as<bool>())
                        input_tensor = input_tensor.to(torch::kHalf);
                    input_tensor = input_tensor.to(torch_device);
                    input_tensor = input_tensor.permute({ 0, 3, 1, 2 }).contiguous();
                    torch::jit::Stack inputs;
                    inputs.emplace_back(input_tensor); // this is where you would batch inputs. Needs to be modified so that all gpu memory is taken advantage of
#endif
                    cv::Mat proc;
                    auto pad_img = LetterboxImage(frame, proc, model_size);



                    auto end = std::chrono::high_resolution_clock::now();
                    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                    // It should be known that it takes longer time at first time
                    
                    

                    start = std::chrono::high_resolution_clock::now();

#ifndef NO_TORCH
                    torch::jit::IValue output_pt = module.forward(inputs);
#endif

                    std::any f_in = proc;
                    std::any d_out = darknet_module.Forward(f_in);
                    std::unordered_map<std::string, std::any> d_dt = std::any_cast<std::unordered_map<std::string, std::any>>(d_out);
                    std::unordered_map<int, std::vector<std::pair<cv::Rect2f, float>>> d_det = std::any_cast<std::unordered_map<int, std::vector<std::pair<cv::Rect2f, float>>>>(d_dt["detections"]);


                    end = std::chrono::high_resolution_clock::now();
                    auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

                    for (auto class_pair : d_det) {
                        std::string name = class_list[class_pair.first];
                        for (auto box : class_pair.second) {
                            cv::rectangle(proc, box.first.tl(), box.first.br(), Scalar(0, 255, 0), 2);
                        }
                    }

                    AveragingTracker c_score;

                    for (auto class_pair : d_det) {
                        std::string name = class_list[class_pair.first];
                        for (auto box_r : class_pair.second) {
                            if (name == "tracking_target") {
                                std::pair<cv::Point2f, cv::Point2f> box(box_r.first.tl(), box_r.first.br());
                                ScaleCoordinates(box, pad_img[0], pad_img[1], pad_img[2], frame_size);
                                auto center = (box.first + box.second) / 2;
                                ma_center_x.add(center.x);
                                ma_center_y.add(center.y);
                                center.x = ma_center_x.getAverage();
                                center.y = ma_center_y.getAverage();
                                int region_width = max(abs(box.first.x - box.second.x), abs(box.first.y - box.second.y));
                                region_width = region_width + (region_width * scale_bounds);
                                ma.add(region_width);
                                region_width = ma.getAverage();
                                Mat extracted_box;
                                auto region_bounds = extractRegionWithPadding(frame, extracted_box, center, region_width);
                                
                                cuda::GpuMat gpuIn;
                                cuda::GpuMat gpuOut;

                                gpuIn.upload(extracted_box);

                                cuda::resize(gpuIn, gpuOut, model_size);

                                gpuOut.download(extracted_box);

                                output2.write(extracted_box);

                                cv::rectangle(frame, region_bounds.first, region_bounds.second, Scalar(255, 100, 100), 2);
                                cv::drawMarker(frame, center, Scalar(255, 255, 0), 0);

                                c_score.add(box_r.second);

                                stringstream ss;
                                ss << std::fixed << std::setprecision(2) << box_r.second;
                                string s = ss.str();

                                cv::putText(frame, s, cv::Point(center.x, center.y - 5),
                                    cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0, 255, 0), 1);

                                cv::Rect roi(frame_size.width - model_size.width, 0, proc.cols, proc.rows);
                                cv::Mat baseImageROI = frame(roi);
                                cv::Mat mask;
                                if (proc.channels() == 4)
                                {
                                    // Split the overlay image into separate channels
                                    std::vector<cv::Mat> channels;
                                    cv::split(proc, channels);

                                    // Use the alpha channel as the mask
                                    mask = channels[3];
                                }
                                extracted_box.copyTo(baseImageROI, mask);
                            }
                        }
                    }

                    cv::Rect roi(0, 0, proc.cols, proc.rows);
                    cv::Mat baseImageROI = frame(roi);
                    cv::Mat mask;
                    if (proc.channels() == 4)
                    {
                        // Split the overlay image into separate channels
                        std::vector<cv::Mat> channels;
                        cv::split(proc, channels);

                        // Use the alpha channel as the mask
                        mask = channels[3];
                    }
                    proc.copyTo(baseImageROI, mask);

                    
                    stringstream ss;
                    ss << "Processing stats-> pre-process: " << duration1.count() << " ms, inference: " << duration2.count() << " ms, total: " << duration1.count() + duration2.count() << " ms";

                    if (frame_num > 0 && d_det.count(0) > 0) {
                        outputFile << frame_num << "," << duration1.count() << "," << duration2.count() << "," << duration1.count() + duration2.count() << "," << c_score.getMean() << std::endl;

                        t_pre.add(duration1.count());
                        t_inf.add(duration2.count());
                        t_forward.add(duration1.count() + duration2.count());
                        confidence.add(c_score.getMean());
                    }

                    string stats = ss.str();
                    if (stats.size() >= padding.size()) {
                        std::cout << stats << '\r';
                        for (int i = 0; i < stats.size(); i++) {
                            stats[i] = ' ';
                        }
                        padding = stats;
                    }
                    else {
                        int len = stats.size();
                        string padding_old = padding;
                        for (int i = 0; i < stats.size(); i++) {
                            padding_old[i] = stats[i];
                            stats[i] = ' ';
                        }
                        padding = stats;
                        std::cout << padding_old << '\r';
                    }

#ifndef NO_TORCH
                    auto detections = output_pt.toTuple()->elements()[0].toTensor();
                    auto result = PostProcessing(detections, pad_w, pad_h, scale, frame.size(), vm["conf"].as<float>(), vm["iou"].as<float>());

                    Demo(frame, result, class_list, true, vm["draw-m-labels"].as<bool>());
#endif

                    imshow("Output", frame);
                    int ret = waitKey(1);
                    output.write(frame);
                    if (ret == 'r')
                        break;

                    frame_num++;
                }
            }
            outputFile << "avg," << t_pre.getMean() << "," << t_inf.getMean() << "," << t_forward.getMean() << "," << confidence.getMean() << std::endl;
            outputFile << "worst," << t_pre.getHighest() << "," << t_inf.getHighest() << "," << t_forward.getHighest() << "," << confidence.getLowest() << std::endl;
            outputFile << "best," << t_pre.getLowest() << "," << t_inf.getLowest() << "," << t_forward.getLowest() << "," << confidence.getHighest() << std::endl;

            input.release();
            output.release();
            output2.release();
            
        }
        else {
            std::cout << "ERROR: A video path must be provided" << std::endl;
        }

    }
    catch (const std::exception& ex)
    {
        std::cerr << ex.what() << '\n';
    }

    /*
    export_marker_pdf("test.pdf");
    
    me::cc::CCTagDetector detector;

    bool success = true;
    VideoCapture input("cctag3.mp4", cv::CAP_FFMPEG, { cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY });
    VideoWriter output("cctag3out.mp4", cv::CAP_FFMPEG,
        VideoWriter::fourcc('m', 'p', '4', 'v'), input.get(CAP_PROP_FPS), Size(input.get(CAP_PROP_FRAME_WIDTH), input.get(CAP_PROP_FRAME_HEIGHT)), { cv::VIDEOWRITER_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY });
    //int frame_id = 0;
    while (success) {
        Mat frame;
        success = input.read(frame);
        if (success) {
            
            auto detections = detector.detect(frame);

            me::cc::drawMarkers(frame, detections);

            cv::imshow("Result", frame);

            waitKey(1);

            output.write(frame);

            //frame_id++;

        }
    }

    input.release();
    output.release();
    
        */
    /*
    // load the image e.g. from file
    cv::Mat src = cv::imread("02.png");
    cv::Mat graySrc;
    cv::cvtColor(src, graySrc, CV_BGR2GRAY);

        

    // process the image
    boost::ptr_list<cctag::ICCTag> markers{};
    cctagDetection(markers, pipeId, frameId, graySrc, params);


    drawMarkers(markers, src);

    cv::imshow("Result", src);

    waitKey(0);

        

    // load the image e.g. from file
    cv::Mat src2 = cv::imread("cctag-lowres-highblur.jpg");
    cv::Mat graySrc2;
    cv::cvtColor(src2, graySrc2, CV_BGR2GRAY);

    // choose a cuda pipe
    const int pipeId2{ 0 };

    // an arbitrary id for the frame
    const int frameId2{ 0 };

    // process the image
    boost::ptr_list<cctag::ICCTag> markers2{};
    cctagDetection(markers2, pipeId2, frameId2, graySrc2, params);


    drawMarkers(markers2, src2);

    cv::imshow("Result", src2);

    waitKey(0);
    */

    


}



int old_test() {

    try {
        
        cv::Point3d world_pos(0, 0, 0);
        cv::Point3d world_rot(0, 0, 0);
        cv::Point3d cam_pos(1, 45, 32);
        cv::Point3d cam_rot(0.534, 0.123, 1.324);
        std::cout << "World pos:" << world_pos.x << ", " << world_pos.y << ", " << world_pos.z << std::endl;
        std::cout << "World rot:" << world_rot.x << ", " << world_rot.y << ", " << world_rot.z << std::endl;
        std::cout << "Cam pos:" << cam_pos.x << ", " << cam_pos.y << ", " << cam_pos.z << std::endl;
        std::cout << "Cam rot:" << cam_rot.x << ", " << cam_rot.y << ", " << cam_rot.z << std::endl;
        Mat R;
        Mat R2;
        cv::Rodrigues((cv::Mat)cam_rot, R);
        cv::Rodrigues((cv::Mat)world_rot, R2);
        cv::Point3d world_pos_to_cam = world_pos + cam_pos;
        cv::Mat_<double> rotm = R2 * R;
        cv::Point3d world_rot_to_cam = Point3d(rotm.at<double>(0));
        std::cout << "World pos:" << world_pos_to_cam.x << ", " << world_pos_to_cam.y << ", " << world_pos_to_cam.z << std::endl;
        std::cout << "World rot:" << world_rot_to_cam.x << ", " << world_rot_to_cam.y << ", " << world_rot_to_cam.z << std::endl;


        cv::Mat mat1(3, 4, CV_64FC1);
        std::cout << "Mat1: " << mat1.rows << ", " << mat1.cols << std::endl;
        cv::Mat m1;
        mat1.row(0).copyTo(m1);
        std::cout << "Mat2: " << m1.rows << ", " << m1.cols << std::endl;
        cv::Mat m2 = (cv::Mat)cv::Point3f(1, 3, 5);
        std::cout << "Mat3: " << m2.rows << ", " << m2.cols << std::endl;

        me::utility::Graph<int> graph;
        std::cout << (me::utility::Edge<int>(0, 2) == me::utility::Edge<int>(0, 3)) << std::endl;
        graph.addEdge(0, 2);
        graph.addEdge(0, 4);
        graph.addEdge(0, 5);
        graph.addEdge(1, 4);
        graph.addEdge(1, 5);
        graph.addEdge(2, 3);
        graph.addEdge(2, 4);
        graph.addEdge(4, 5);
        graph.addEdge(7, 6);
        std::cout << graph.checkContinuity() << std::endl;
        std::cout << graph.getEdges().size() << std::endl;
        std::set<int> verts = graph.getVerticies();
        for (auto it = verts.begin(); it != verts.end(); it++) {
            std::cout << *it << ' ';
        }
        std::cout << std::endl;
        std::set<int> adj = graph.getAdjacentVerticies(2);
        for (auto it = adj.begin(); it != adj.end(); it++) {
            std::cout << *it << ' ';
        }
        std::cout << std::endl;
        std::stack<int> path = graph.getVertexPathDFS(3, 1);
        while(!path.empty()) {
            std::cout << path.top() << ' ';
            path.pop();
        }
        std::cout << std::endl;

        me::utility::Range range1(-5, 10);
        me::utility::Range range2(-40, 40);
        std::cout << range1.solve(0) << ", " << range1.solve(1) << endl;
        std::cout << range1.solve(0.5) << endl;
        std::cout << range2.solve(0) << ", " << range2.solve(1) << endl;
        std::cout << range2.solve(0.5) << endl;
        std::cout << range1.getOverlapRange(range2).solve(0) << ", " << range1.getOverlapRange(range2).solve(1) << endl;
        std::cout << range1.getOverlapRange(range2).solve(0.5) << endl;

        cv::Ptr<cv::aruco::Dictionary> dict1 = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        cv::Ptr<cv::aruco::Dictionary> dict2 = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        std::cout << (dict1 == dict2) << endl;

        std::vector<std::string> in_files;
        in_files.push_back("calibration_config.bson");
        in_files.push_back("calibration_config.json");
        in_files.push_back("out.mp4");
        in_files.push_back("out2.mp4");

        me::crypto::createParityFile(in_files, "parity.bin");

        std::stringstream ss("v9HlACe6ULgB5bge9KLQkEF6Wgyvcnq8VpmMByq2dD2robpa9RpD02yi4SqHchDY");
        std::string hash = me::crypto::hashStreamSHA1(ss).to_string();
        std::cout << hash << std::endl;

        std::ifstream parity("parity.bin", std::ifstream::in | std::ifstream::binary);
        std::cout << me::crypto::hashStreamSHA1(parity).to_string() << std::endl;
        std::stringstream key("dudebro");
        std::cout << me::crypto::StreamToHMAC_SHA1(parity, key).to_string() << std::endl;
        std::cout << std::hex << (uint32_t)0xFFFFFFFFFFFFFFFF << std::dec << std::endl;
        std::stringstream aa("v9HlACe6ULgB5bge9KLQkEF6Wgyvcnq8VpmMByq2dD2robpa9RpD02yi4SqHchDY");
        const uint8_t* bytes = me::crypto::hashStreamSHA1(aa).to_bytes();
        for (int i = 0; i < 20; i++) {
            std::cout << std::hex << +bytes[i] << std::dec << ' ';
        }
        std::cout << std::endl;

        random_device rd;
        mt19937 gen(rd());

        std::stringstream init_stream;
        for (int i = 0; i < 1000; i++) {
            init_stream << gen();
        }

        std::ofstream keyout("keyfile.bin", std::ofstream::out | std::ofstream::binary | std::ofstream::trunc);
        me::crypto::generateSHA1PRNGKey(keyout, init_stream);
        keyout.close();

        std::cout << "WAAAAAAA" << std::endl;

        // Curve test
        me::utility::curves::CardinalCubicHermiteSpline spline;
        me::utility::curves::Curve *curve = &spline;
        curve->addPoint(0, 0);
        curve->addPoint(1, 0);
        curve->addPoint(2, 1);
        curve->addPoint(4, -1);
        curve->addPoint(3, 2);
        curve->addPoint(5, 1);
        curve->addPoint(6, 1);
        curve->addPoint(7, 0);

        curve->getPoints().at(3)->y = -2;

        std::vector<double> pts_x;
        std::vector<double> pts_y;

        double minX = 0;
        double minY = 0;
        double maxX = 0;
        double maxY = 0;
        for (double i = 0; i <= 7; i += 0.01) {
            if (curve->solve(i).x < minX)
                minX = curve->solve(i).x;
            if (curve->solve(i).x > maxX)
                maxX = curve->solve(i).x;
            if (curve->solve(i).y < minY)
                minY = curve->solve(i).y;
            if (curve->solve(i).y > maxY)
                maxY = curve->solve(i).y;
            pts_x.push_back(curve->solve(i).x);
            pts_y.push_back(curve->solve(i).y);
            std::cout << curve->solve(i).x << ", " << curve->solve(i).y << std::endl;
        }

        cv::Ptr<cv::plot::Plot2d> plot = cv::plot::Plot2d::create(pts_x, pts_y);
        plot->setMinX(minX);
        plot->setMinY(minY);
        plot->setMaxX(maxX);
        plot->setMaxY(maxY);
        plot->setInvertOrientation(true);
        plot->setGridLinesNumber(7);

        cv::Mat render;
        plot->render(render);
        cv::imshow("Plot", render);

        curve->clearPoints();
        me::utility::curves::BezierCurve bezier;
        curve = &bezier;
        curve->addPoint(1.574, 1.45);
        curve->addPoint(4.62, 5.8);
        curve->addPoint(5.9, 1.65);
        curve->addPoint(7.43, 6.66);
        pts_x.clear();
        pts_y.clear();
        minX = curve->solve(0).x;
        minY = curve->solve(0).y;
        maxX = curve->solve(0).x;
        maxY = curve->solve(0).y;
        for (double i = 0; i <= 1; i += 0.01) {
            if (curve->solve(i).x < minX)
                minX = curve->solve(i).x;
            if (curve->solve(i).x > maxX)
                maxX = curve->solve(i).x;
            if (curve->solve(i).y < minY)
                minY = curve->solve(i).y;
            if (curve->solve(i).y > maxY)
                maxY = curve->solve(i).y;
            pts_x.push_back(curve->solve(i).x);
            pts_y.push_back(curve->solve(i).y);
            std::cout << curve->solve(i).x << ", " << curve->solve(i).y << std::endl;
        }
        cv::Ptr<cv::plot::Plot2d> plot2 = cv::plot::Plot2d::create(pts_x, pts_y);
        plot2->setMinX(minX);
        plot2->setMinY(minY);
        plot2->setMaxX(maxX);
        plot2->setMaxY(maxY);
        plot2->setInvertOrientation(true);
        plot2->render(render);
        cv::imshow("Plot2", render);
        cv::Point2d pt = curve->solve(me::utility::curves::estimateT(bezier, 4));
        std::cout << pt.x << ", " << pt.y << std::endl;

        double xd = 9;
        double yd = 5;
        cv::Point2d ptd(xd, yd);
        std::cout << ptd.x << ", " << ptd.y << std::endl;
        xd = 6.7;
        yd = 4.3;
        std::cout << ptd.x << ", " << ptd.y << std::endl;
        
        // START ARUCO PDF GENERATION

        // ArUco marker generation





        cv::Mat markerImage;
        int dict_predef = aruco::DICT_4X4_1000;
        const string dict_names[16] = { "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000",
            "DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250", "DICT_5X5_1000",
            "DICT_6X6_50", "DICT_6X6_100", "DICT_6X6_250", "DICT_6X6_1000",
            "DICT_7X7_50", "DICT_7X7_100", "DICT_7X7_250", "DICT_7X7_1000", };
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        dictionary = cv::aruco::getPredefinedDictionary(dict_predef);
        map<string, Mat> aruco_markers;
        for (int i = 0; i < 128; i++) {
            //cv::aruco::drawMarker(dictionary, i, 200, markerImage, 1);
            stringstream ss;
            ss << "markers/" << "marker";
            if (i < 10)
                ss << '0';
            ss << i << ".png";
            cv::imwrite(ss.str(), markerImage);
            //cv::aruco::drawMarker(dictionary, i, dictionary.get()->markerSize + 2, markerImage, 1);
            Mat m(markerImage);
            aruco_markers.emplace(ss.str(), m);
        }

        Generate_Synchronization_Video("sync.mp4", dictionary, 999, 0.6f, 1000, 2, 2, 2);

        // Marker PDF generator parameters
        int pdf_dpi = 72;
        float pdf_dim_x = 8.5;
        float pdf_dim_y = 11;
        float marker_dim = 2;
        float corner_offset = 0.3;
        float min_distance_x = 0.3;
        float min_distance_y = 0.3;
        float max_horiz_label_scale = 0.8;
        float max_vert_label_scale = 0.8;
        float label_height = 0.3;
        float title_scale = 0.6;

        // Generator computed values
        int pdf_coords_x = pdf_dim_x * pdf_dpi;
        int pdf_coords_y = pdf_dim_y * pdf_dpi;
        float marker_width = marker_dim * pdf_dpi;
        float corner_width = corner_offset * pdf_dpi;
        float fillable_space_x = pdf_coords_x - corner_width * 2;
        float fillable_space_y = pdf_coords_y - corner_width * 2;
        float marker_spacing_x = fillable_space_x;
        float marker_spacing_y = fillable_space_y;
        float min_coord_x = min_distance_x * pdf_dpi;
        float min_coord_y = min_distance_y * pdf_dpi;
        int num_marker_x = fillable_space_x / marker_width;
        int num_marker_y = fillable_space_y / marker_width;
        cout << num_marker_x << ' ' << num_marker_y << endl;
        if (num_marker_x > 1)
            marker_spacing_x = (fillable_space_x - marker_width * num_marker_x) / (num_marker_x + 1);
        else if (num_marker_x == 1)
            marker_spacing_x = (fillable_space_x - marker_width) / 2;
        else
            return 1;
        if (num_marker_y > 1)
            marker_spacing_y = (fillable_space_y - marker_width * num_marker_y) / (num_marker_y + 1);
        else if (num_marker_y == 1)
            marker_spacing_y = (fillable_space_y - marker_width) / 2;
        else
            return 1;
        while (num_marker_x > 1) {
            if (marker_spacing_x >= min_coord_x)
                break;
            num_marker_x--;
            if (num_marker_x == 1)
                break;
            marker_spacing_x = (fillable_space_x - marker_width * num_marker_x) / (num_marker_x + 1);
        }
        if (num_marker_x == 1)
            marker_spacing_x = (fillable_space_x - marker_width) / 2;
        while (num_marker_y > 1) {
            if (marker_spacing_y >= min_coord_y)
                break;
            num_marker_y--;
            if (num_marker_y == 1)
                break;
            marker_spacing_y = (fillable_space_y - marker_width * num_marker_y) / (num_marker_y + 1);
        }
        if (num_marker_y == 1)
            marker_spacing_y = (fillable_space_y - marker_width) / 2;
        float font_size = label_height * pdf_dpi;
        if (font_size > marker_spacing_y * max_vert_label_scale)
            font_size = marker_spacing_y * max_vert_label_scale;
        int page_count = aruco_markers.size() / (num_marker_x * num_marker_y);
        if (aruco_markers.size() > page_count * num_marker_x * num_marker_y)
            page_count += 1;


        /*

        // PDF creation
        HPDF_Doc pdf = HPDF_New(error_handler, NULL);
        HPDF_SetCompressionMode(pdf, 0x0F);
        HPDF_SetPageMode(pdf, HPDF_PAGE_MODE_USE_OUTLINE);

        auto it = aruco_markers.begin();
        for (int p = 1; p <= page_count; p++) {
            HPDF_Page page = HPDF_AddPage(pdf);
            HPDF_Page_SetSize(page, HPDF_PAGE_SIZE_LETTER, HPDF_PAGE_PORTRAIT);
            HPDF_Page_BeginText(page);


            // Side title drawing
            stringstream title;
            title << "aruco::" << dict_names[dict_predef] << ", Page " << p << " of " << page_count;
            HPDF_Page_SetFontAndSize(page, HPDF_GetFont(pdf, "Helvetica", NULL), corner_width * title_scale);
            HPDF_Page_TextOut(page, pdf_coords_x / 2 - HPDF_Page_TextWidth(page, title.str().c_str()) / 2, pdf_coords_y - corner_width, title.str().c_str());

            // Finish page title writing
            HPDF_Page_EndText(page);

            // Corner box drawing
            HPDF_Page_SetRGBFill(page, 0, 0, 0);
            HPDF_Page_Rectangle(page, 0, 0, corner_width, corner_width);
            HPDF_Page_Rectangle(page, pdf_coords_x - corner_width, 0, corner_width, corner_width);
            HPDF_Page_Rectangle(page, 0, pdf_coords_y - corner_width, corner_width, corner_width);
            HPDF_Page_Rectangle(page, pdf_coords_x - corner_width, pdf_coords_y - corner_width, corner_width, corner_width);
            HPDF_Page_FillStroke(page);

            // Marker drawing
            cout << num_marker_x << ' ' << num_marker_y << endl;
            for (int y = 0; y < num_marker_y && it != aruco_markers.end(); y++) {
                for (int x = 0; x < num_marker_x && it != aruco_markers.end(); x++) {
                    float pos_x = marker_spacing_x + corner_width + x * (marker_width + marker_spacing_x);
                    float pos_y = marker_spacing_y + corner_width + (num_marker_y - y - 1) * (marker_width + marker_spacing_y);
                    if (num_marker_x == 1)
                        pos_x = corner_width + marker_spacing_x;
                    if (num_marker_y == 1)
                        pos_y = corner_width + marker_spacing_y;
                    HPDF_Draw_Marker(page, it->second, pos_x, pos_y, marker_width);
                    ++it;
                }
            }
        }

        // Save to test.pdf
        HPDF_SaveToFile(pdf, "test.pdf");

        // END ARUCO PDF GENERATION

        // START ARUCO DETECTION TEST

        */

        // ChArUco camera calibration

        // Setup
        VideoCapture calibration("F:/programming_projects/opencv-experimental/tests/triangulation/view2.MP4");
        cv::Ptr<cv::aruco::CharucoBoard> board;
        board = cv::aruco::CharucoBoard::create(8, 8, 0.02f, 0.016f, dictionary);
        Size img_size(calibration.get(CAP_PROP_FRAME_WIDTH), calibration.get(CAP_PROP_FRAME_HEIGHT));
        int frame_count = calibration.get(CAP_PROP_FRAME_COUNT);
        std::vector<std::vector<cv::Point2f>> allCharucoCorners;
        std::vector<std::vector<int>> allCharucoIds;
        unsigned int thread_count = 1;//thread::hardware_concurrency();

        // Threshold test
        namedWindow("Thresh test", WINDOW_NORMAL);
        calibration.set(cv::CAP_PROP_POS_FRAMES, frame_count / 2);
        Mat thresh_in;
        calibration.read(thresh_in);
        calibration.set(cv::CAP_PROP_POS_FRAMES, 0);
        Mat thresh;
        cvtColor(thresh_in, thresh, COLOR_BGR2GRAY);
        int size_min = 5;
        int size_max = 51;
        int step = 14;
        double constant = 7;
        for (int i = size_min; i <= size_max; i += step) {
            Mat thresh_out;
            cv::adaptiveThreshold(thresh, thresh_out, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, i, constant);
            imshow("Thresh test", thresh_out);
            waitKey(0);
        }
        Mat blur;
        Mat thresh_out;
        Mat weighted;
        //cv::addWeighted(thresh, 65.5, thresh, 0, -8191.5, weighted);
        //imshow("Thresh test", weighted);
        waitKey(0);
        threshold(thresh, thresh_out, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);
        imshow("Thresh test", thresh_out);
        waitKey(0);
        std::vector<std::vector<Point>> contours;
        Mat contour;
        cvtColor(thresh_out, contour, COLOR_GRAY2BGR);
        findContours(thresh_out, contours, RETR_LIST, CHAIN_APPROX_NONE);
        drawContours(contour, contours, -1, Scalar(0, 255, 0), 2);
        imshow("Thresh test", contour);
        waitKey(0);

        //  ChArUco Detection
        calibration.release();

        std::vector<std::vector<std::vector<cv::Point2f>>> markerCorners;
        std::vector<std::vector<int>> markerIds;
        std::vector<bool> boardSuccess;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        parameters->adaptiveThreshWinSizeMin = size_min;
        parameters->adaptiveThreshWinSizeMax = size_max;
        parameters->adaptiveThreshWinSizeStep = step;
        parameters->errorCorrectionRate = 0.8;
        parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

        me::ComputeTaskParams calib_params;
        calib_params.filepath = "F:/programming_projects/opencv-experimental/tests/triangulation/view2.MP4";
        calib_params.board = board;
        calib_params.board_corners = &allCharucoCorners;
        calib_params.board_ids = &allCharucoIds;
        calib_params.dictionary = dictionary;
        calib_params.marker_corners = &markerCorners;
        calib_params.marker_ids = &markerIds;
        calib_params.marker_parameters = parameters;
        calib_params.board_success = &boardSuccess;

        me::ARComputeTask calib_task(calib_params);
        calib_task.start();

        std::vector<int> calib_counts;
        cout << endl << "Detecting markers...";

        calib_task.wait();
        /*
        while (calib_task.busy()) {
            cout << "\rDetecting markers... (" << calib_task.get_complete() << '/' << calib_task.get_total() << ')';
            int last = calib_task.get_complete();
            Sleep(1000);
            int current = calib_task.get_complete();
            if (calib_counts.size() > 10)
                calib_counts.erase(calib_counts.begin());
            calib_counts.push_back(current - last);
            double total = 0;
            for (int i = 0; i < calib_counts.size(); i++) {
                total += calib_counts[i];
            }
            cout << " avg. " << format("%.2f", total / calib_counts.size()) << "f/sec               ";
        }
        */

        // Calibration
        cout << endl << "Calibrating..." << endl;
        std::vector<std::vector<cv::Point2f>> cCharucoCorners;
        std::vector<std::vector<int>> cCharucoIds;
        for (int i = 0; i < allCharucoIds.size(); i++) {
            if (boardSuccess[i] == true) {
                cCharucoCorners.push_back(allCharucoCorners[i]);
                cCharucoIds.push_back(allCharucoIds[i]);
            }
        }
        int frames = 10;
        int offset = cCharucoIds.size() / frames;
        std::vector<std::vector<cv::Point2f>> calCharucoCorners;
        std::vector<std::vector<int>> calCharucoIds;
        for (int i = 0; i < frames; i++) {
            calCharucoCorners.push_back(cCharucoCorners[i * offset]);
            calCharucoIds.push_back(cCharucoIds[i * offset]);
        }
        cv::Mat cameraMatrix, distCoeffs;
        std::vector<cv::Mat> rvecs, tvecs;
        double repError = cv::aruco::calibrateCameraCharuco(calCharucoCorners, calCharucoIds, board, img_size, cameraMatrix, distCoeffs, rvecs, tvecs);

        allCharucoCorners.clear();
        allCharucoIds.clear();
        markerCorners.clear();
        markerIds.clear();
        boardSuccess.clear();

        me::ComputeTaskParams compute_params;
        compute_params.filepath = "F:/programming_projects/opencv-experimental/tests/triangulation/view2.MP4";
        compute_params.board = board;
        compute_params.board_corners = &allCharucoCorners;
        compute_params.board_ids = &allCharucoIds;
        compute_params.dictionary = dictionary;
        compute_params.marker_corners = &markerCorners;
        compute_params.marker_ids = &markerIds;
        compute_params.marker_parameters = parameters;
        compute_params.board_success = &boardSuccess;

        me::ARComputeTask compute_task(compute_params);
        compute_task.start();

        std::vector<int> frame_counts;
        cout << endl << "Detecting markers...";
        while (compute_task.busy()) {
            cout << "\rDetecting markers... (" << compute_task.get_complete() << '/' << compute_task.get_total() << ')';
            int last = compute_task.get_complete();
            Sleep(1000);
            int current = compute_task.get_complete();
            if (frame_counts.size() > 10)
                frame_counts.erase(frame_counts.begin());
            frame_counts.push_back(current - last);
            double total = 0;
            for (int i = 0; i < frame_counts.size(); i++) {
                total += frame_counts[i];
            }
            cout << " avg. " << format("%.2f", total / frame_counts.size()) << "f/sec               ";
        }

        // Detection
        cout << "Detecting..." << endl;
        VideoCapture input("F:/programming_projects/opencv-experimental/tests/triangulation/view2.MP4", cv::CAP_FFMPEG, { cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY });
        VideoWriter output("F:/programming_projects/opencv-experimental/tests/triangulation/view2coords.MP4", cv::CAP_FFMPEG,
            VideoWriter::fourcc('m', 'p', '4', 'v'), input.get(CAP_PROP_FPS), Size(input.get(CAP_PROP_FRAME_WIDTH), input.get(CAP_PROP_FRAME_HEIGHT)), { cv::VIDEOWRITER_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY });
        bool success = true;
        int frame_num = 0;
        while (success) {
            auto start = std::chrono::high_resolution_clock::now();
            Mat frame;
            success = input.read(frame);
            if (success) {
                bool charuco_found = false;
                std::vector<cv::Vec3d> rvec_m, tvec_m;
                cv::Mat outputImage = frame.clone();
                if (markerIds[frame_num].size() > 0) {
                    charuco_found = true;
                    cv::aruco::drawDetectedMarkers(outputImage, markerCorners[frame_num], markerIds[frame_num]);
                    std::vector<cv::Vec3d> rvecs_m, tvecs_m;
                    cv::aruco::estimatePoseSingleMarkers(markerCorners[frame_num], 0.0508f, cameraMatrix, distCoeffs, rvecs_m, tvecs_m);
                    for (int i = 0; i < rvecs_m.size(); ++i) {
                        if (markerIds[frame_num][i] > 31) {
                            auto rvec = rvecs_m[i];
                            auto tvec = tvecs_m[i];
                            rvec_m.push_back(rvecs_m[i]);
                            tvec_m.push_back(tvecs_m[i]);
                            cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvec, tvec, 0.0254f);
                        }
                    }
                }
                int display_scale = frame.rows / 500;
                Mat rvec, tvec;
                if (charuco_found) {
                    bool valid = cv::aruco::estimatePoseCharucoBoard(allCharucoCorners[frame_num], allCharucoIds[frame_num], board, cameraMatrix, distCoeffs, rvec, tvec);
                    if (valid) {
                        drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvec, tvec, 0.0762f);
                        std::vector<double> tvec_vector;
                        for (int i = 0; i < tvec.rows; i++) {
                            tvec_vector.push_back(tvec.ptr<double>(i)[0]);
                        }

                        vector<Point3f> cube_pts;
                        vector<Point2f> cube_pts_img;
                        for (int i = 0; i < 13; i++) {
                            for (int j = 0; j < 13; j++) {
                                for (int k = 0; k < 13; k++) {
                                    cube_pts.push_back(Point3f((float)i / 50, (float)j / 50, (float)k / 50 * -1));
                                }
                            }
                        }
                        cube_pts_img.resize(cube_pts.size());

                        projectPoints(cube_pts, rvec, tvec, cameraMatrix, distCoeffs, cube_pts_img);

                        for (int i = 0; i < cube_pts_img.size(); i++) {
                            drawMarker(outputImage, cube_pts_img[i], Scalar(0, 255, 0));
                        }

                        // Getting camera coordinates
                        Mat R;
                        Rodrigues(rvec, R);
                        R = R.t();
                        Mat tvec_cam = -R * tvec;
                        Mat rvec_cam;
                        Rodrigues(R, rvec_cam);

                        stringstream ss;
                        ss << "position(XYZ)=[ " << tvec_cam.ptr<double>(0)[0] << ", " << tvec_cam.ptr<double>(1)[0] << ", " << tvec_cam.ptr<double>(2)[0] << " ]";
                        string pos_vec = ss.str();
                        putText(outputImage, pos_vec, Point(4 * display_scale, 50 * display_scale), FONT_HERSHEY_SIMPLEX, display_scale, Scalar(255, 0, 0), 2);
                        ss.clear();
                        ss.str("");
                        ss << "rotation(XYZ)=[ " << rvec_cam.ptr<double>(0)[0] << ", " << rvec_cam.ptr<double>(1)[0] << ", " << rvec_cam.ptr<double>(2)[0] << " ]";
                        putText(outputImage, ss.str(), Point(4 * display_scale, 100 * display_scale), FONT_HERSHEY_SIMPLEX, display_scale, Scalar(255, 0, 0), 2);
                        ss.clear();
                        ss.str("");

                        // R is a 3x3 Rotation matrix of the camera
                        // tvec_cam is a 3x1 matrix of the camera position
                        // Q is the quaternion rotation of the camera, in 

                        if (rvec_m.size() > 0) {
                            for (int i = 0; i < rvec_m.size(); i++) {
                                Mat R_char;
                                Rodrigues(rvec, R_char);
                                Mat tvec_m_world = R_char.t() * (tvec_m[i] - tvec);
                                ss << "position(XYZ)=[ " << tvec_m_world.ptr<double>(0)[0] << ", " << tvec_m_world.ptr<double>(1)[0] << ", " << tvec_m_world.ptr<double>(2)[0] << " ]";
                                string pos_vec = ss.str();
                                vector<Point3f> pts;
                                vector<Point2f> pts_img;
                                pts.push_back(Point3f(tvec_m_world.ptr<double>(0)[0], tvec_m_world.ptr<double>(1)[0], tvec_m_world.ptr<double>(2)[0]));
                                pts_img.resize(1);
                                projectPoints(pts, rvec, tvec, cameraMatrix, distCoeffs, pts_img);
                                putText(outputImage, pos_vec, Point(5 * display_scale + pts_img[0].x, 5 * display_scale + pts_img[0].y), FONT_HERSHEY_SIMPLEX, display_scale / 4, Scalar(255, 0, 0), 2);
                                ss.clear();
                                ss.str("");
                                Mat R_m;
                                Rodrigues(rvec_m[i], R_m);
                                Mat R_mark = R_char.t() * R_m;
                                Mat rvec_m_world;
                                Rodrigues(R_mark, rvec_m_world);
                                ss << "rotation(XYZ)=[ " << rvec_m_world.ptr<double>(0)[0] << ", " << rvec_m_world.ptr<double>(1)[0] << ", " << rvec_m_world.ptr<double>(2)[0] << " ]";
                                putText(outputImage, ss.str(), Point(5 * display_scale + pts_img[0].x, 13 * display_scale + 5 * display_scale + pts_img[0].y), FONT_HERSHEY_SIMPLEX, display_scale / 4, Scalar(255, 0, 0), 2);
                                ss.clear();
                                ss.str("");
                            }
                        }

                    }
                }
                imshow("Detection", outputImage);
                output.write(outputImage);
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
                double frame_time = 1000 / input.get(CAP_PROP_FPS);
                double wait_time = frame_time - duration.count();
                cout << wait_time << endl;
                if ((int)wait_time <= 0)
                    wait_time = 1;
                waitKey(wait_time);
            }
            frame_num++;
        }

        // END ARUCO DETECTION TEST

    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
    }

}

bool HPDF_Draw_Marker(HPDF_Page page, Mat m, float loc_x, float loc_y, float width)
{
    if (page != NULL && !m.empty() && m.rows == m.cols) {
        float pixel_offset = width / m.rows;
        for (int x = 0; x < m.rows; x++) {
            for (int y = 0; y < m.rows; y++) {
                bool blk = true;
                if (m.ptr(m.rows - y - 1, x)[0] > 100)
                    blk = false;
                HPDF_Page_SetLineWidth(page, 0);
                HPDF_Page_SetRGBStroke(page, 0, 0, 0);
                if (blk) {
                    HPDF_Page_Rectangle(page, loc_x + pixel_offset * x, loc_y + pixel_offset * y, pixel_offset, pixel_offset);
                    HPDF_Page_FillStroke(page);
                }
            }
        }
        return true;
    }
    return false;
}

void Generate_Synchronization_Video(std::string out_file, cv::Ptr<cv::aruco::Dictionary>& dictionary, int marker_id, float size_ratio, int size, float delay_interval, int start_delays, int end_delays) {
    VideoWriter output(out_file, VideoWriter::fourcc('m', 'p', '4', 'v'), 60, Size(size, size));
    if (output.isOpened()) {
        if (size_ratio > 1)
            size_ratio = 1 / size_ratio;
        int marker_size = size * size_ratio;
        Mat mkr;
        int delay_frames = 60 * delay_interval;
        //cv::aruco::drawMarker(dictionary, marker_id, marker_size, mkr, 1);
        Mat marker;
        cvtColor(mkr, marker, COLOR_GRAY2BGR);
        Mat begin_frame(size, size, CV_8UC3, Scalar(0, 0, 0));
        Mat end_frame(size, size, CV_8UC3, Scalar(0, 0, 0));
        Mat marker_frame(size, size, CV_8UC3, Scalar(255, 255, 255));
        int offset = (size - marker_size) / 2;
        cv::Rect roi(cv::Point(offset, offset), marker.size());
        marker.copyTo(marker_frame(roi));
        float display_scale = size / 100;
        Size clear_size = getTextSize("STAND CLEAR", FONT_HERSHEY_SIMPLEX, display_scale / 4, 1, 0);
        Size sync_size = getTextSize("Synchronization in progress", FONT_HERSHEY_SIMPLEX, display_scale / 8, 1, 0);
        Size complete_size = getTextSize("SYNC COMPLETE", FONT_HERSHEY_SIMPLEX, display_scale / 4, 1, 0);
        Size remove_size = getTextSize("This marker can now be removed", FONT_HERSHEY_SIMPLEX, display_scale / 8, 1, 0);
        putText(begin_frame, "STAND CLEAR", Point((size - clear_size.width) / 2, (size / 2) - 1 * display_scale), FONT_HERSHEY_SIMPLEX, display_scale / 4, Scalar(0, 255, 0), 2);
        putText(begin_frame, "Synchronization in progress", Point((size - sync_size.width) / 2, (size / 2) + 1 * display_scale + sync_size.height), FONT_HERSHEY_SIMPLEX, display_scale / 8, Scalar(0, 255, 0), 2);
        putText(end_frame, "SYNC COMPLETE", Point((size - complete_size.width) / 2, (size / 2) - 1 * display_scale), FONT_HERSHEY_SIMPLEX, display_scale / 4, Scalar(0, 255, 0), 2);
        putText(end_frame, "This marker can now be removed", Point((size - remove_size.width) / 2, (size / 2) + 1 * display_scale + remove_size.height), FONT_HERSHEY_SIMPLEX, display_scale / 8, Scalar(0, 255, 0), 2);
        for (int i = 0; i < delay_frames * start_delays; i++) {
            output.write(begin_frame);
        }
        for (int i = 0; i < delay_frames; i++) {
            output.write(marker_frame);
        }
        for (int i = 0; i < delay_frames * end_delays; i++) {
            output.write(end_frame);
        }
    }
    output.release();
}
