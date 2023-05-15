#include <pch.h>
#include <me_cctag.hpp>
#include <cctag/CCTag.hpp>
#include <cctag/ICCTag.hpp>
#include <cctag/Detection.hpp>
#include <boost/smart_ptr.hpp>
#include <opencv2/imgproc/types_c.h>


namespace me {
	namespace cc {
		
		CCTagMarker::CCTagMarker(int id, cv::Point2f center, DETECTION_STATUS status, cv::Size2f ellipse_size, double angle) : id(id), center(center), status(status), ellipse_size(ellipse_size), angle(angle) {}

		CCTagDetector::CCTagDetector(int dictionary, bool use_cuda) : dictionary(dictionary), use_cuda(use_cuda) {}

		std::vector<CCTagMarker> CCTagDetector::detect(cv::Mat &input, int frame_id, int pipe_id) {
			std::vector<CCTagMarker> result;
			const std::size_t nCrowns = (dictionary == DICT_CCTAG_3CROWNS) ? 3 : 4;

			cctag::Parameters params(nCrowns);
			// if you want to use GPU
			params.setUseCuda(use_cuda);

			// choose a cuda pipe
			const int pipeId = pipe_id;

			// an arbitrary id for the frame
			int frameId = frame_id;

			cv::Mat graySrc;
			cv::cvtColor(input, graySrc, CV_BGR2GRAY);

			boost::ptr_list<cctag::ICCTag> markers{};
			cctagDetection(markers, pipeId, frameId, graySrc, params);
			
			for (const auto& marker : markers) {
				const cv::Point center = cv::Point(marker.x(), marker.y());
				const auto rescaledOuterEllipse = marker.rescaledOuterEllipse();
				const cv::Size size = cv::Size(rescaledOuterEllipse.a(), rescaledOuterEllipse.b());
				const double angle = rescaledOuterEllipse.angle() * 180 / boost::math::constants::pi<double>();
				DETECTION_STATUS status = (marker.getStatus() == cctag::status::id_reliable) ? CCTAG_RELIABLE : CCTAG_UNRELIABLE;
				CCTagMarker marker_out(marker.id(),center, status, size, angle);
				result.push_back(marker_out);
			}

			return result;
		}

		void drawMarkers(cv::Mat& input, std::vector<CCTagMarker>& markers) {
			// drawing settings
			const int radius{ 10 };
			const int fontSize{ 3 };
			const int thickness{ 2 };
			const int fontFace{ cv::FONT_HERSHEY_SIMPLEX };
			for (const auto& marker : markers)
			{
				if (marker.status == CCTAG_RELIABLE) {
					const cv::Scalar color = cv::Scalar(0, 255, 0, 255);
					cv::circle(input, marker.center, radius, color, thickness);
					cv::putText(input, std::to_string(marker.id), marker.center, fontFace, fontSize, color, thickness);
					cv::ellipse(input, marker.center, marker.ellipse_size, marker.angle, 0, 360, color, thickness);
				}
				else {
					// the same for invalid markers but in red
					const cv::Scalar color = cv::Scalar(0, 0, 255, 255);
					cv::circle(input, marker.center, radius, color, thickness);
					cv::putText(input, std::to_string(marker.id), marker.center, fontFace, fontSize, color, thickness);
				}
			}
		}

	}
}

void cctag_analyse_image(char* in_file, char* out_file) {

	try {
		
		cv::Mat frame = cv::imread(in_file);
		me::cc::CCTagDetector detector;
		auto markers = detector.detect(frame);

		std::ofstream fout;
		fout.open(out_file);

		for (const auto& marker : markers)
		{
			// check the status and draw accordingly, green for valid, red otherwise
			if (marker.status == me::cc::CCTAG_RELIABLE)
			{
				std::stringstream ss;
				ss << std::to_string(marker.id) << ' ' << marker.center.x << ' ' << marker.center.y << ' ';
				ss << marker.ellipse_size.width << ' ' << marker.ellipse_size.height << ' ' << marker.angle;
				fout << ss.str() << std::endl;
			}
		}

		fout.close();
		me::cc::drawMarkers(frame, markers);
		cv::imwrite(std::string(in_file) + ".cctag.png", frame);


	}
	catch (const std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
}