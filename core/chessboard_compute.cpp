#include <pch.h>
#include <chessboard_compute.hpp>
#include <math.h>
#include <thread>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudacodec.hpp>
#include <chrono>

// CONTAINS CODE COPIED DIRECTLY FROM CHESSBOARD.CPP (opencv/sources/modules/calib3d/src)
// AND ARUCO.CPP (opencv-contrib/modules/aruco/src)
// MODIFIED TO WORK WITH MULTITHREADING AND CUDA
// ALSO CONTAINS TWEAKS MEANT TO PROVIDE SLIGHT OPTIMIZATIONS IN DETECTION ALGORITHMS

using namespace std::chrono_literals;

namespace cv {

    namespace aruco {

        /**
         * @brief Threshold input image using adaptive thresholding
         */
        static void _threshold(InputArray _in, OutputArray _out, int winSize, double constant) {

            CV_Assert(winSize >= 3);
            if (winSize % 2 == 0) winSize++; // win size must be odd
            adaptiveThreshold(_in, _out, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, winSize, constant);
        }

        /**
         * @brief Given a tresholded image, find the contours, calculate their polygonal approximation
         * and take those that accomplish some conditions
         */
        static void _findMarkerContours(InputArray _in, std::vector<std::vector<Point2f>>& candidates,
            std::vector<std::vector<Point>>& contoursOut, double minPerimeterRate,
            double maxPerimeterRate, double accuracyRate,
            double minCornerDistanceRate, int minDistanceToBorder, int minSize) {

            CV_Assert(minPerimeterRate > 0 && maxPerimeterRate > 0 && accuracyRate > 0 &&
                minCornerDistanceRate >= 0 && minDistanceToBorder >= 0);

            // calculate maximum and minimum sizes in pixels
            unsigned int minPerimeterPixels =
                (unsigned int)(minPerimeterRate * max(_in.getUMat().cols, _in.getUMat().rows));
            unsigned int maxPerimeterPixels =
                (unsigned int)(maxPerimeterRate * max(_in.getUMat().cols, _in.getUMat().rows));

            // for aruco3 functionality
            if (minSize != 0) {
                minPerimeterPixels = 4 * minSize;
            }

            UMat contoursImg;
            _in.getUMat().copyTo(contoursImg);
            std::vector<std::vector<Point>> contours;
            findContours(contoursImg, contours, RETR_LIST, CHAIN_APPROX_NONE);
            // now filter list of contours
            for (unsigned int i = 0; i < contours.size(); i++) {
                // check perimeter
                if (contours[i].size() < minPerimeterPixels || contours[i].size() > maxPerimeterPixels)
                    continue;

                // check is square and is convex
                std::vector<Point> approxCurve;
                approxPolyDP(contours[i], approxCurve, double(contours[i].size()) * accuracyRate, true);
                if (approxCurve.size() != 4 || !isContourConvex(approxCurve)) continue;

                // check min distance between corners
                double minDistSq =
                    max(contoursImg.cols, contoursImg.rows) * max(contoursImg.cols, contoursImg.rows);
                for (int j = 0; j < 4; j++) {
                    double d = (double)(approxCurve[j].x - approxCurve[(j + 1) % 4].x) *
                        (double)(approxCurve[j].x - approxCurve[(j + 1) % 4].x) +
                        (double)(approxCurve[j].y - approxCurve[(j + 1) % 4].y) *
                        (double)(approxCurve[j].y - approxCurve[(j + 1) % 4].y);
                    minDistSq = min(minDistSq, d);
                }
                double minCornerDistancePixels = double(contours[i].size()) * minCornerDistanceRate;
                if (minDistSq < minCornerDistancePixels * minCornerDistancePixels) continue;

                // check if it is too near to the image border
                bool tooNearBorder = false;
                for (int j = 0; j < 4; j++) {
                    if (approxCurve[j].x < minDistanceToBorder || approxCurve[j].y < minDistanceToBorder ||
                        approxCurve[j].x > contoursImg.cols - 1 - minDistanceToBorder ||
                        approxCurve[j].y > contoursImg.rows - 1 - minDistanceToBorder)
                        tooNearBorder = true;
                }
                if (tooNearBorder) continue;

                // if it passes all the test, add to candidates vector
                std::vector<Point2f> currentCandidate;
                currentCandidate.resize(4);
                for (int j = 0; j < 4; j++) {
                    currentCandidate[j] = Point2f((float)approxCurve[j].x, (float)approxCurve[j].y);
                }
                candidates.push_back(currentCandidate);
                contoursOut.push_back(contours[i]);
            }
        }

        /**
         * @brief Initial steps on finding square candidates (modified to use otsu's thresholding)
         */
        static void _detectInitialCandidates(const UMat& grey, std::vector<std::vector<Point2f>>& candidates,
            std::vector<std::vector<Point>>& contours,
            const Ptr<DetectorParameters>& params) {

            CV_Assert(params->adaptiveThreshWinSizeMin >= 3 && params->adaptiveThreshWinSizeMax >= 3);
            CV_Assert(params->adaptiveThreshWinSizeMax >= params->adaptiveThreshWinSizeMin);
            CV_Assert(params->adaptiveThreshWinSizeStep > 0);

            // number of window sizes (scales) to apply adaptive thresholding
            int nScales = (params->adaptiveThreshWinSizeMax - params->adaptiveThreshWinSizeMin) /
                params->adaptiveThreshWinSizeStep + 1;

            std::vector<std::vector<std::vector<Point2f>>> candidatesArrays((size_t)nScales);
            std::vector<std::vector<std::vector<Point>>> contoursArrays((size_t)nScales);

            ////for each value in the interval of thresholding window sizes
            parallel_for_(Range(0, nScales), [&](const Range& range) {
                const int begin = range.start;
                const int end = range.end;

                for (int i = begin; i < end; i++) {
                    int currScale = params->adaptiveThreshWinSizeMin + i * params->adaptiveThreshWinSizeStep;
                    // threshold
                    UMat thresh;
                    _threshold(grey, thresh, currScale, params->adaptiveThreshConstant);

                    // detect rectangles
                    _findMarkerContours(thresh, candidatesArrays[i], contoursArrays[i],
                        params->minMarkerPerimeterRate, params->maxMarkerPerimeterRate,
                        params->polygonalApproxAccuracyRate, params->minCornerDistanceRate,
                        params->minDistanceToBorder, params->minSideLengthCanonicalImg);
                }
            });
            // join candidates
            for (int i = 0; i < nScales; i++) {
                for (unsigned int j = 0; j < candidatesArrays[i].size(); j++) {
                    candidates.push_back(candidatesArrays[i][j]);
                    contours.push_back(contoursArrays[i][j]);
                }
            }
        }

        /**
         * @brief Convert input image to gray if it is a 3-channels image
         */
        static void _convertToGrey(InputArray _in, OutputArray _out) {

            CV_Assert(_in.type() == CV_8UC1 || _in.type() == CV_8UC3);

            if (_in.type() == CV_8UC3)
                cvtColor(_in, _out, COLOR_BGR2GRAY);
            else
                _in.copyTo(_out);
        }

        /**
         * @brief Assure order of candidate corners is clockwise direction
         */
        static void _reorderCandidatesCorners(std::vector<std::vector<Point2f>>& candidates) {

            for (unsigned int i = 0; i < candidates.size(); i++) {
                double dx1 = candidates[i][1].x - candidates[i][0].x;
                double dy1 = candidates[i][1].y - candidates[i][0].y;
                double dx2 = candidates[i][2].x - candidates[i][0].x;
                double dy2 = candidates[i][2].y - candidates[i][0].y;
                double crossProduct = (dx1 * dy2) - (dy1 * dx2);

                if (crossProduct < 0.0) { // not clockwise direction
                    swap(candidates[i][1], candidates[i][3]);
                }
            }
        }

        /**
         * @brief to make sure that the corner's order of both candidates (default/white) is the same
         */
        static std::vector< Point2f > alignContourOrder(Point2f corner, std::vector<Point2f> candidate) {
            uint8_t r = 0;
            double min = cv::norm(Vec2f(corner - candidate[0]), NORM_L2SQR);
            for (uint8_t pos = 1; pos < 4; pos++) {
                double nDiff = cv::norm(Vec2f(corner - candidate[pos]), NORM_L2SQR);
                if (nDiff < min) {
                    r = pos;
                    min = nDiff;
                }
            }
            std::rotate(candidate.begin(), candidate.begin() + r, candidate.end());
            return candidate;
        }

        /**
         * @brief Check candidates that are too close to each other, save the potential candidates
         *        (i.e. biggest/smallest contour) and remove the rest
         */
        static void _filterTooCloseCandidates(const std::vector<std::vector<Point2f>>& candidatesIn,
            std::vector<std::vector<std::vector<Point2f>>>& candidatesSetOut,
            const std::vector<std::vector<Point>>& contoursIn,
            std::vector<std::vector<std::vector<Point>>>& contoursSetOut,
            double minMarkerDistanceRate, bool detectInvertedMarker) {

            CV_Assert(minMarkerDistanceRate >= 0);
            std::vector<int> candGroup;
            candGroup.resize(candidatesIn.size(), -1);
            std::vector<std::vector<unsigned int>> groupedCandidates;
            for (unsigned int i = 0; i < candidatesIn.size(); i++) {
                bool isSingleContour = true;
                for (unsigned int j = i + 1; j < candidatesIn.size(); j++) {

                    int minimumPerimeter = min((int)contoursIn[i].size(), (int)contoursIn[j].size());

                    // fc is the first corner considered on one of the markers, 4 combinations are possible
                    for (int fc = 0; fc < 4; fc++) {
                        double distSq = 0;
                        for (int c = 0; c < 4; c++) {
                            // modC is the corner considering first corner is fc
                            int modC = (c + fc) % 4;
                            distSq += (candidatesIn[i][modC].x - candidatesIn[j][c].x) *
                                (candidatesIn[i][modC].x - candidatesIn[j][c].x) +
                                (candidatesIn[i][modC].y - candidatesIn[j][c].y) *
                                (candidatesIn[i][modC].y - candidatesIn[j][c].y);
                        }
                        distSq /= 4.;

                        // if mean square distance is too low, remove the smaller one of the two markers
                        double minMarkerDistancePixels = double(minimumPerimeter) * minMarkerDistanceRate;
                        if (distSq < minMarkerDistancePixels * minMarkerDistancePixels) {
                            isSingleContour = false;
                            // i and j are not related to a group
                            if (candGroup[i] < 0 && candGroup[j] < 0) {
                                // mark candidates with their corresponding group number
                                candGroup[i] = candGroup[j] = (int)groupedCandidates.size();

                                // create group
                                std::vector<unsigned int> grouped;
                                grouped.push_back(i);
                                grouped.push_back(j);
                                groupedCandidates.push_back(grouped);
                            }
                            // i is related to a group
                            else if (candGroup[i] > -1 && candGroup[j] == -1) {
                                int group = candGroup[i];
                                candGroup[j] = group;

                                // add to group
                                groupedCandidates[group].push_back(j);
                            }
                            // j is related to a group
                            else if (candGroup[j] > -1 && candGroup[i] == -1) {
                                int group = candGroup[j];
                                candGroup[i] = group;

                                // add to group
                                groupedCandidates[group].push_back(i);
                            }
                        }
                    }
                }
                if (isSingleContour && candGroup[i] < 0)
                {
                    candGroup[i] = (int)groupedCandidates.size();
                    std::vector<unsigned int> grouped;
                    grouped.push_back(i);
                    grouped.push_back(i); // step "save possible candidates" require minimum 2 elements
                    groupedCandidates.push_back(grouped);
                }
            }

            // save possible candidates
            candidatesSetOut.clear();
            contoursSetOut.clear();

            std::vector<std::vector<Point2f>> biggerCandidates;
            std::vector<std::vector<Point>> biggerContours;
            std::vector<std::vector<Point2f>> smallerCandidates;
            std::vector<std::vector<Point>> smallerContours;

            // save possible candidates
            for (unsigned int i = 0; i < groupedCandidates.size(); i++) {
                unsigned int smallerIdx = groupedCandidates[i][0];
                unsigned int biggerIdx = smallerIdx;
                double smallerArea = contourArea(candidatesIn[smallerIdx]);
                double biggerArea = smallerArea;

                // evaluate group elements
                for (unsigned int j = 1; j < groupedCandidates[i].size(); j++) {
                    unsigned int currIdx = groupedCandidates[i][j];
                    double currArea = contourArea(candidatesIn[currIdx]);

                    // check if current contour is bigger
                    if (currArea >= biggerArea) {
                        biggerIdx = currIdx;
                        biggerArea = currArea;
                    }

                    // check if current contour is smaller
                    if (currArea < smallerArea && detectInvertedMarker) {
                        smallerIdx = currIdx;
                        smallerArea = currArea;
                    }
                }

                // add contours and candidates
                biggerCandidates.push_back(candidatesIn[biggerIdx]);
                biggerContours.push_back(contoursIn[biggerIdx]);
                if (detectInvertedMarker) {
                    smallerCandidates.push_back(alignContourOrder(candidatesIn[biggerIdx][0], candidatesIn[smallerIdx]));
                    smallerContours.push_back(contoursIn[smallerIdx]);
                }
            }
            // to preserve the structure :: candidateSet< defaultCandidates, whiteCandidates >
            // default candidates
            candidatesSetOut.push_back(biggerCandidates);
            contoursSetOut.push_back(biggerContours);
            // white candidates
            candidatesSetOut.push_back(smallerCandidates);
            contoursSetOut.push_back(smallerContours);
        }

        /**
         * @brief Detect square candidates in the input image
         */
        static void _detectCandidates(InputArray _grayImage, std::vector<std::vector<std::vector<Point2f>>>& candidatesSetOut,
            std::vector<std::vector<std::vector<Point>>>& contoursSetOut, const Ptr<DetectorParameters>& _params) {
            UMat grey = _grayImage.getUMat();
            CV_DbgAssert(grey.total() != 0);
            CV_DbgAssert(grey.type() == CV_8UC1);

            /// 1. DETECT FIRST SET OF CANDIDATES
            std::vector<std::vector<Point2f>> candidates;
            std::vector<std::vector<Point>> contours;
            _detectInitialCandidates(grey, candidates, contours, _params);
            /// 2. SORT CORNERS
            _reorderCandidatesCorners(candidates);

            /// 3. FILTER OUT NEAR CANDIDATE PAIRS
            // save the outter/inner border (i.e. potential candidates)
            _filterTooCloseCandidates(candidates, candidatesSetOut, contours, contoursSetOut,
                _params->minMarkerDistanceRate, _params->detectInvertedMarker);
        }

        static size_t _findOptPyrImageForCanonicalImg(
            const std::vector<UMat>& img_pyr,
            const int scaled_width,
            const int cur_perimeter,
            const int min_perimeter) {
            CV_Assert(scaled_width > 0);
            size_t optLevel = 0;
            float dist = std::numeric_limits<float>::max();
            for (size_t i = 0; i < img_pyr.size(); ++i) {
                const float scale = img_pyr[i].cols / static_cast<float>(scaled_width);
                const float perimeter_scaled = cur_perimeter * scale;
                // instead of std::abs() favor the larger pyramid level by checking if the distance is postive
                // will slow down the algorithm but find more corners in the end
                const float new_dist = perimeter_scaled - min_perimeter;
                if (new_dist < dist && new_dist > 0.f) {
                    dist = new_dist;
                    optLevel = i;
                }
            }
            return optLevel;
        }

        /**
          * @brief Given an input image and candidate corners, extract the bits of the candidate, including
          * the border bits
          */
        static UMat _extractBits(InputArray _image, InputArray _corners, int markerSize,
            int markerBorderBits, int cellSize, double cellMarginRate,
            double minStdDevOtsu) {

            CV_Assert(_image.getUMat().channels() == 1);
            CV_Assert(_corners.total() == 4);
            CV_Assert(markerBorderBits > 0 && cellSize > 0 && cellMarginRate >= 0 && cellMarginRate <= 1);
            CV_Assert(minStdDevOtsu >= 0);

            // number of bits in the marker
            int markerSizeWithBorders = markerSize + 2 * markerBorderBits;
            int cellMarginPixels = int(cellMarginRate * cellSize);

            UMat resultImg; // marker image after removing perspective
            int resultImgSize = markerSizeWithBorders * cellSize;
            UMat resultImgCorners;
            Mat m_resultImgCorners(4, 1, CV_32FC2);
            m_resultImgCorners.ptr< Point2f >(0)[0] = Point2f(0, 0);
            m_resultImgCorners.ptr< Point2f >(0)[1] = Point2f((float)resultImgSize - 1, 0);
            m_resultImgCorners.ptr< Point2f >(0)[2] =
                Point2f((float)resultImgSize - 1, (float)resultImgSize - 1);
            m_resultImgCorners.ptr< Point2f >(0)[3] = Point2f(0, (float)resultImgSize - 1);
            m_resultImgCorners.copyTo(resultImgCorners);

            // remove perspective
            UMat transformation;
            getPerspectiveTransform(_corners, resultImgCorners).copyTo(transformation);
            warpPerspective(_image, resultImg, transformation, Size(resultImgSize, resultImgSize),
                INTER_NEAREST);

            // output image containing the bits
            UMat bits(markerSizeWithBorders, markerSizeWithBorders, CV_8UC1, Scalar::all(0));

            // check if standard deviation is enough to apply Otsu
            // if not enough, it probably means all bits are the same color (black or white)
            UMat mean, stddev;
            // Remove some border just to avoid border noise from perspective transformation
            UMat innerRegion = resultImg.colRange(cellSize / 2, resultImg.cols - cellSize / 2)
                .rowRange(cellSize / 2, resultImg.rows - cellSize / 2);
            meanStdDev(innerRegion, mean, stddev);
            Mat m_mean, m_stddev;
            mean.copyTo(m_mean);
            stddev.copyTo(m_stddev);
            if (m_stddev.ptr< double >(0)[0] < minStdDevOtsu) {
                // all black or all white, depending on mean value
                if (m_mean.ptr< double >(0)[0] > 127)
                    bits.setTo(1);
                else
                    bits.setTo(0);
                return bits;
            }

            // now extract code, first threshold using Otsu
            threshold(resultImg, resultImg, 125, 255, THRESH_BINARY | THRESH_OTSU);

            Mat m_bits;
            bits.copyTo(m_bits);

            // for each cell
            for (int y = 0; y < markerSizeWithBorders; y++) {
                for (int x = 0; x < markerSizeWithBorders; x++) {
                    int Xstart = x * (cellSize)+cellMarginPixels;
                    int Ystart = y * (cellSize)+cellMarginPixels;
                    UMat square = resultImg(Rect(Xstart, Ystart, cellSize - 2 * cellMarginPixels,
                        cellSize - 2 * cellMarginPixels));
                    // count white pixels on each cell to assign its value
                    size_t nZ = (size_t)countNonZero(square);
                    if (nZ > square.total() / 2) m_bits.at< unsigned char >(y, x) = 1;
                }
            }

            m_bits.copyTo(bits);

            return bits;
        }

        /**
          * @brief Return number of erroneous bits in border, i.e. number of white bits in border.
          */
        static int _getBorderErrors(const UMat& bits, int markerSize, int borderSize) {

            int sizeWithBorders = markerSize + 2 * borderSize;

            CV_Assert(markerSize > 0 && bits.cols == sizeWithBorders && bits.rows == sizeWithBorders);

            int totalErrors = 0;

            Mat m_bits;
            bits.copyTo(m_bits);

            for (int y = 0; y < sizeWithBorders; y++) {
                for (int k = 0; k < borderSize; k++) {
                    if (m_bits.ptr< unsigned char >(y)[k] != 0) totalErrors++;
                    if (m_bits.ptr< unsigned char >(y)[sizeWithBorders - 1 - k] != 0) totalErrors++;
                }
            }
            for (int x = borderSize; x < sizeWithBorders - borderSize; x++) {
                for (int k = 0; k < borderSize; k++) {
                    if (m_bits.ptr< unsigned char >(k)[x] != 0) totalErrors++;
                    if (m_bits.ptr< unsigned char >(sizeWithBorders - 1 - k)[x] != 0) totalErrors++;
                }
            }
            return totalErrors;
        }

        /**
         * @brief Tries to identify one candidate given the dictionary
         * @return candidate typ. zero if the candidate is not valid,
         *                           1 if the candidate is a black candidate (default candidate)
         *                           2 if the candidate is a white candidate
         */
        static uint8_t _identifyOneCandidate(const Ptr<Dictionary>& dictionary, InputArray _image,
            const std::vector<Point2f>& _corners, int& idx,
            const Ptr<DetectorParameters>& params, int& rotation,
            const float scale = 1.f)
        {
            CV_DbgAssert(_corners.size() == 4);
            CV_DbgAssert(_image.getUMat().total() != 0);
            CV_DbgAssert(params->markerBorderBits > 0);
            uint8_t typ = 1;
            // get bits
            // scale corners to the correct size to search on the corresponding image pyramid
            std::vector<Point2f> scaled_corners(4);
            for (int i = 0; i < 4; ++i) {
                scaled_corners[i].x = _corners[i].x * scale;
                scaled_corners[i].y = _corners[i].y * scale;
            }

            UMat candidateBits =
                _extractBits(_image, scaled_corners, dictionary->markerSize, params->markerBorderBits,
                    params->perspectiveRemovePixelPerCell,
                    params->perspectiveRemoveIgnoredMarginPerCell, params->minOtsuStdDev);

            // analyze border bits
            int maximumErrorsInBorder =
                int(dictionary->markerSize * dictionary->markerSize * params->maxErroneousBitsInBorderRate);
            int borderErrors =
                _getBorderErrors(candidateBits, dictionary->markerSize, params->markerBorderBits);

            // check if it is a white marker
            if (params->detectInvertedMarker) {
                // to get from 255 to 1
                UMat invertedImg;
                subtract(candidateBits.inv(), 254, invertedImg);
                int invBError = _getBorderErrors(invertedImg, dictionary->markerSize, params->markerBorderBits);
                // white marker
                if (invBError < borderErrors) {
                    borderErrors = invBError;
                    invertedImg.copyTo(candidateBits);
                    typ = 2;
                }
            }
            if (borderErrors > maximumErrorsInBorder) return 0; // border is wrong

            // take only inner bits
            UMat onlyBits =
                candidateBits.rowRange(params->markerBorderBits,
                    candidateBits.rows - params->markerBorderBits)
                .colRange(params->markerBorderBits, candidateBits.cols - params->markerBorderBits);

            // try to indentify the marker
            Mat m_onlyBits;
            onlyBits.copyTo(m_onlyBits);
            if (!dictionary->identify(m_onlyBits, idx, rotation, params->errorCorrectionRate))
                return 0;

            return typ;
        }

        /**
         * @brief rotate the initial corner to get to the right position
         */
        static void correctCornerPosition(std::vector<Point2f>& _candidate, int rotate) {
            std::rotate(_candidate.begin(), _candidate.begin() + 4 - rotate, _candidate.end());
        }

        /**
         * @brief Copy the contents of a corners vector to an OutputArray, settings its size.
         */
        static void _copyVector2Output(std::vector<std::vector< Point2f > >& vec, OutputArrayOfArrays out, const float scale = 1.f) {
            out.create((int)vec.size(), 1, CV_32FC2);

            if (out.isUMatVector()) {
                for (unsigned int i = 0; i < vec.size(); i++) {
                    out.create(4, 1, CV_32FC2, i);
                    UMat& m = out.getUMatRef(i);
                    Mat(Mat(vec[i]).t() * scale).copyTo(m);
                }
            }
            else if (out.isUMatVector()) {
                for (unsigned int i = 0; i < vec.size(); i++) {
                    out.create(4, 1, CV_32FC2, i);
                    UMat& m = out.getUMatRef(i);
                    Mat(Mat(vec[i]).t() * scale).copyTo(m);
                }
            }
            else if (out.kind() == _OutputArray::STD_VECTOR_VECTOR) {
                for (unsigned int i = 0; i < vec.size(); i++) {
                    out.create(4, 1, CV_32FC2, i);
                    UMat m = out.getUMat(i);
                    Mat(Mat(vec[i]).t() * scale).copyTo(m);
                }
            }
            else {
                CV_Error(cv::Error::StsNotImplemented,
                    "Only UMat vector, UUMat vector, and vector<vector> OutputArrays are currently supported.");
            }
        }

        /**
         * @brief Identify square candidates according to a marker dictionary
         */
        static void _identifyCandidates(InputArray grey,
            const std::vector<cv::UMat>& image_pyr,
            std::vector<std::vector<std::vector<Point2f>>>& _candidatesSet,
            std::vector<std::vector<std::vector<Point>>>& _contoursSet, const Ptr<Dictionary>& _dictionary,
            std::vector<std::vector<Point2f>>& _accepted, std::vector<std::vector<Point>>& _contours, std::vector<int>& ids,
            const Ptr<DetectorParameters>& params,
            OutputArrayOfArrays _rejected = noArray()) {
            CV_DbgAssert(grey.getUMat().total() != 0);
            CV_DbgAssert(grey.getUMat().type() == CV_8UC1);
            int ncandidates = (int)_candidatesSet[0].size();
            std::vector<std::vector<Point2f>> accepted;
            std::vector<std::vector<Point2f>> rejected;
            std::vector<std::vector<Point>> contours;

            std::vector<int> idsTmp(ncandidates, -1);
            std::vector<int> rotated(ncandidates, 0);
            std::vector<uint8_t> validCandidates(ncandidates, 0);

            //// Analyze each of the candidates
            parallel_for_(Range(0, ncandidates), [&](const Range& range) {
                const int begin = range.start;
                const int end = range.end;

                std::vector<std::vector<Point2f>>& candidates = params->detectInvertedMarker ? _candidatesSet[1] : _candidatesSet[0];
                std::vector<std::vector<Point>>& contourS = params->detectInvertedMarker ? _contoursSet[1] : _contoursSet[0];

                for (int i = begin; i < end; i++) {
                    int currId = -1;
                    // implements equation (4)
                    if (params->useAruco3Detection) {
                        const int perimeterOfContour = static_cast<int>(contourS[i].size());
                        const int min_perimeter = params->minSideLengthCanonicalImg * 4;
                        const size_t nearestImgId = _findOptPyrImageForCanonicalImg(image_pyr, grey.cols(), perimeterOfContour, min_perimeter);
                        const float scale = image_pyr[nearestImgId].cols / static_cast<float>(grey.cols());

                        validCandidates[i] = _identifyOneCandidate(_dictionary, image_pyr[nearestImgId], candidates[i], currId, params, rotated[i], scale);
                    }
                    else {
                        validCandidates[i] = _identifyOneCandidate(_dictionary, grey, candidates[i], currId, params, rotated[i]);
                    }

                    if (validCandidates[i] > 0)
                        idsTmp[i] = currId;
                }
            });

            for (int i = 0; i < ncandidates; i++) {
                if (validCandidates[i] > 0) {
                    // to choose the right set of candidates :: 0 for default, 1 for white markers
                    uint8_t set = validCandidates[i] - 1;

                    // shift corner positions to the correct rotation
                    correctCornerPosition(_candidatesSet[set][i], rotated[i]);

                    if (!params->detectInvertedMarker && validCandidates[i] == 2)
                        continue;

                    // add valid candidate
                    accepted.push_back(_candidatesSet[set][i]);
                    ids.push_back(idsTmp[i]);

                    contours.push_back(_contoursSet[set][i]);

                }
                else {
                    rejected.push_back(_candidatesSet[0][i]);
                }
            }

            // parse output
            _accepted = accepted;

            _contours = contours;

            if (_rejected.needed()) {
                _copyVector2Output(rejected, _rejected);
            }
        }

        static inline void findCornerInPyrImage(const float scale_init, const int closest_pyr_image_idx,
            const std::vector<cv::UMat>& grey_pyramid, UMat corners,
            const Ptr<DetectorParameters>& params) {
            // scale them to the closest pyramid level
            if (scale_init != 1.f)
                multiply(corners, scale_init, corners); // scale_init * scale_pyr
            for (int idx = closest_pyr_image_idx - 1; idx >= 0; --idx) {
                // scale them to new pyramid level
                multiply(corners, 2.f, corners); // *= scale_pyr;
                // use larger win size for larger images
                const int subpix_win_size = std::max(grey_pyramid[idx].cols, grey_pyramid[idx].rows) > 1080 ? 5 : 3;
                cornerSubPix(grey_pyramid[idx], corners,
                    Size(subpix_win_size, subpix_win_size),
                    Size(-1, -1),
                    TermCriteria(TermCriteria::MAX_ITER | TermCriteria::EPS,
                        params->cornerRefinementMaxIterations,
                        params->cornerRefinementMinAccuracy));
            }
        }

        /**
         * Line fitting  A * B = C :: Called from function refineCandidateLines
         * @param nContours, contour-container
         */
        static Point3f _interpolate2Dline(const std::vector<cv::Point2f>& nContours) {
            CV_Assert(nContours.size() >= 2);
            float minX, minY, maxX, maxY;
            minX = maxX = nContours[0].x;
            minY = maxY = nContours[0].y;

            for (unsigned int i = 0; i < nContours.size(); i++) {
                minX = nContours[i].x < minX ? nContours[i].x : minX;
                minY = nContours[i].y < minY ? nContours[i].y : minY;
                maxX = nContours[i].x > maxX ? nContours[i].x : maxX;
                maxY = nContours[i].y > maxY ? nContours[i].y : maxY;
            }

            Mat A = Mat::ones((int)nContours.size(), 2, CV_32F); // Coefficient UMatrix (N x 2)
            Mat B((int)nContours.size(), 1, CV_32F);				// Variables   UMatrix (N x 1)
            Mat C;											// Constant

            if (maxX - minX > maxY - minY) {
                for (unsigned int i = 0; i < nContours.size(); i++) {
                    A.at<float>(i, 0) = nContours[i].x;
                    B.at<float>(i, 0) = nContours[i].y;
                }
                solve(A, B, C, DECOMP_NORMAL);

                return Point3f(C.at<float>(0, 0), -1., C.at<float>(1, 0));
            }
            else {
                for (unsigned int i = 0; i < nContours.size(); i++) {
                    A.at<float>(i, 0) = nContours[i].y;
                    B.at<float>(i, 0) = nContours[i].x;
                }
                
                solve(A, B, C, DECOMP_NORMAL);

                return Point3f(-1., C.at<float>(0, 0), C.at<float>(1, 0));
            }

        }

        /**
         * Find the Point where the lines crosses :: Called from function refineCandidateLines
         * @param nLine1
         * @param nLine2
         * @return Crossed Point
         */
        static Point2f _getCrossPoint(Point3f nLine1, Point3f nLine2) {
            Matx22f A(nLine1.x, nLine1.y, nLine2.x, nLine2.y);
            Vec2f B(-nLine1.z, -nLine2.z);
            return Vec2f(A.solve(B).val);
        }

        /**
         * Refine Corners using the contour vector :: Called from function detectMarkers
         * @param nContours, contour-container
         * @param nCorners, candidate Corners
         * @param camUMatrix, cameraUMatrix input 3x3 floating-point camera matrix
         * @param distCoeff, distCoeffs vector of distortion coefficient
         */
        static void _refineCandidateLines(std::vector<Point>& nContours, std::vector<Point2f>& nCorners) {
            std::vector<Point2f> contour2f(nContours.begin(), nContours.end());
            /* 5 groups :: to group the edges
             * 4 - classified by its corner
             * extra group - (temporary) if contours do not begin with a corner
             */
            std::vector<Point2f> cntPts[5];
            int cornerIndex[4] = { -1 };
            int group = 4;

            for (unsigned int i = 0; i < nContours.size(); i++) {
                for (unsigned int j = 0; j < 4; j++) {
                    if (nCorners[j] == contour2f[i]) {
                        cornerIndex[j] = i;
                        group = j;
                    }
                }
                cntPts[group].push_back(contour2f[i]);
            }
            for (int i = 0; i < 4; i++)
            {
                CV_Assert(cornerIndex[i] != -1);
            }

            // saves extra group into corresponding
            if (!cntPts[4].empty()) {
                for (unsigned int i = 0; i < cntPts[4].size(); i++)
                    cntPts[group].push_back(cntPts[4].at(i));
                cntPts[4].clear();
            }

            //Evaluate contour direction :: using the position of the detected corners
            int inc = 1;

            inc = ((cornerIndex[0] > cornerIndex[1]) && (cornerIndex[3] > cornerIndex[0])) ? -1 : inc;
            inc = ((cornerIndex[2] > cornerIndex[3]) && (cornerIndex[1] > cornerIndex[2])) ? -1 : inc;

            // calculate the line :: who passes through the grouped points
            Point3f lines[4];
            for (int i = 0; i < 4; i++) {
                lines[i] = _interpolate2Dline(cntPts[i]);
            }

            /*
             * calculate the corner :: where the lines crosses to each other
             * clockwise direction		no clockwise direction
             *      0                           1
             *      .---. 1                     .---. 2
             *      |   |                       |   |
             *    3 .___.                     0 .___.
             *          2                           3
             */
            for (int i = 0; i < 4; i++) {
                if (inc < 0)
                    nCorners[i] = _getCrossPoint(lines[i], lines[(i + 1) % 4]);	// 01 12 23 30
                else
                    nCorners[i] = _getCrossPoint(lines[i], lines[(i + 3) % 4]);	// 30 01 12 23
            }
        }

        void detectMarkersCompute(InputArray _image, const Ptr<Dictionary>& _dictionary, OutputArrayOfArrays _corners,
            OutputArray _ids, const Ptr<DetectorParameters>& _params,
            OutputArrayOfArrays _rejectedImgPoints) {

            CV_Assert(!_image.empty());
            CV_Assert(_params->markerBorderBits > 0);
            // check that the parameters are set correctly if Aruco3 is used
            CV_Assert(!(_params->useAruco3Detection == true &&
                _params->minSideLengthCanonicalImg == 0 &&
                _params->minMarkerLengthRatioOriginalImg == 0.0));

            UMat grey;
            _convertToGrey(_image.getUMat(), grey);

            // Aruco3 functionality is the extension of Aruco.
            // The description can be found in:
            // [1] Speeded up detection of squared fiducial markers, 2018, FJ Romera-Ramirez et al.
            // if Aruco3 functionality if not wanted
            // change some parameters to be sure to turn it off
            if (!_params->useAruco3Detection) {
                _params->minMarkerLengthRatioOriginalImg = 0.0;
                _params->minSideLengthCanonicalImg = 0;
            }
            else {
                // always turn on corner refinement in case of Aruco3, due to upsampling
                _params->cornerRefinementMethod = CORNER_REFINE_SUBPIX;
            }

            /// Step 0: equation (2) from paper [1]
            const float fxfy = (!_params->useAruco3Detection ? 1.f : _params->minSideLengthCanonicalImg /
                (_params->minSideLengthCanonicalImg + std::max(grey.cols, grey.rows) * _params->minMarkerLengthRatioOriginalImg));

            /// Step 1: create image pyramid. Section 3.4. in [1]
            std::vector<cv::UMat> grey_pyramid;
            int closest_pyr_image_idx = 0, num_levels = 0;
            //// Step 1.1: resize image with equation (1) from paper [1]
            if (_params->useAruco3Detection) {
                const float scale_pyr = 2.f;
                const float img_area = static_cast<float>(grey.rows * grey.cols);
                const float min_area_marker = static_cast<float>(_params->minSideLengthCanonicalImg * _params->minSideLengthCanonicalImg);
                // find max level
                num_levels = static_cast<int>(log2(img_area / min_area_marker) / scale_pyr);
                // the closest pyramid image to the downsampled segmentation image
                // will later be used as start index for corner upsampling
                const float scale_img_area = img_area * fxfy * fxfy;
                closest_pyr_image_idx = cvRound(log2(img_area / scale_img_area) / scale_pyr);
            }
            cv::buildPyramid(grey, grey_pyramid, num_levels);
            
            // resize to segmentation image
            // in this reduces size the contours will be detected
            if (fxfy != 1.f)
                cv::resize(grey, grey, cv::Size(cvRound(fxfy * grey.cols), cvRound(fxfy * grey.rows)));

            /// STEP 2: Detect marker candidates
            std::vector<std::vector<Point2f>> candidates;
            std::vector<std::vector<Point>> contours;
            std::vector<int> ids;

            std::vector<std::vector<std::vector<Point2f>>> candidatesSet;
            std::vector<std::vector<std::vector<Point>>> contoursSet;

            /// STEP 2 Detect marker candidates :: traditional way
            _detectCandidates(grey, candidatesSet, contoursSet, _params);

            /// STEP 2: Check candidate codification (identify markers)
            _identifyCandidates(grey, grey_pyramid, candidatesSet, contoursSet, _dictionary,
                candidates, contours, ids, _params, _rejectedImgPoints);

            // copy to output arrays
            _copyVector2Output(candidates, _corners);
            Mat(ids).copyTo(_ids);

            /// STEP 3: Corner refinement :: use corner subpix
            if (_params->cornerRefinementMethod == CORNER_REFINE_SUBPIX) {
                CV_Assert(_params->cornerRefinementWinSize > 0 && _params->cornerRefinementMaxIterations > 0 &&
                    _params->cornerRefinementMinAccuracy > 0);
                // Do subpixel estimation. In Aruco3 start on the lowest pyramid level and upscale the corners
                parallel_for_(Range(0, _corners.cols()), [&](const Range& range) {
                    const int begin = range.start;
                    const int end = range.end;

                    for (int i = begin; i < end; i++) {
                        if (_params->useAruco3Detection) {
                            const float scale_init = (float)grey_pyramid[closest_pyr_image_idx].cols / grey.cols;
                            findCornerInPyrImage(scale_init, closest_pyr_image_idx, grey_pyramid, _corners.getUMat(i), _params);
                        }
                        else
                            cornerSubPix(grey, _corners.getUMat(i),
                                Size(_params->cornerRefinementWinSize, _params->cornerRefinementWinSize),
                                Size(-1, -1),
                                TermCriteria(TermCriteria::MAX_ITER | TermCriteria::EPS,
                                    _params->cornerRefinementMaxIterations,
                                    _params->cornerRefinementMinAccuracy));
                    }
                });
            }

            /// STEP 3, Optional : Corner refinement :: use contour container
            if (_params->cornerRefinementMethod == CORNER_REFINE_CONTOUR) {

                if (!_ids.empty()) {

                    // do corner refinement using the contours for each detected markers
                    parallel_for_(Range(0, _corners.cols()), [&](const Range& range) {
                        for (int i = range.start; i < range.end; i++) {
                            _refineCandidateLines(contours[i], candidates[i]);
                        }
                    });

                    // copy the corners to the output array
                    _copyVector2Output(candidates, _corners);
                }
            }
            if (_params->cornerRefinementMethod != CORNER_REFINE_APRILTAG &&
                _params->cornerRefinementMethod != CORNER_REFINE_SUBPIX) {
                // scale to orignal size, this however will lead to inaccurate detections!
                _copyVector2Output(candidates, _corners, 1.f / fxfy);
            }
        }

    }
    
    bool findChessboardCornersCUDA(cv::InputArray image_, cv::Size pattern_size, cv::OutputArray corners_, int flags)
    {
        if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
            
            int type = image_.type(), depth = CV_MAT_DEPTH(type), cn = CV_MAT_CN(type);
            CV_CheckType(type, depth == CV_8U && (cn == 1 || cn == 3),
                "Only 8-bit grayscale or color images are supported");
            if (pattern_size.width <= 2 || pattern_size.height <= 2)
            {
                CV_Error(Error::StsOutOfRange, "Both width and height of the pattern should have bigger than 2");
            }
            if (!corners_.needed())
                CV_Error(Error::StsNullPtr, "Null pointer to corners");

            Mat img;
            if (image_.channels() != 1)
                cvtColor(image_, img, COLOR_BGR2GRAY);
            else
                img = image_.getMat();

            int max_points = std::max(100, pattern_size.width * pattern_size.height * 2);

            // setup search based on flags
            if (flags & CALIB_CB_NORMALIZE_IMAGE)
            {
                Mat tmp;
                cv::equalizeHist(img, tmp);
                swap(img, tmp);
                flags ^= CALIB_CB_NORMALIZE_IMAGE;
            }
            if (flags & CALIB_CB_EXHAUSTIVE)
            {
                max_points = std::max(1000, pattern_size.width * pattern_size.height * 2);
                flags ^= CALIB_CB_EXHAUSTIVE;
            }
            if (flags)
                CV_Error(Error::StsOutOfRange, cv::format("Invalid remaining flags %d", (int)flags));

            cuda::GpuMat gpu_img;
            gpu_img.upload(img);
            
            Ptr<cuda::CornersDetector> detector = cuda::createGoodFeaturesToTrackDetector(img.type(), max_points, 0.01, 0.0, 3, true, 0.04);

            std::vector<cv::Mat> maps;
            cuda::GpuMat gpu_corners;

            detector->detect(gpu_img, gpu_corners);

            gpu_corners.download(corners_);

            if (!corners_.empty()) {
                return true;
            }
        }
        //CV_Error(Error::GpuApiCallError, "No CUDA enabled devices found");
        return false;
    }
    
    void doFindCUDA(Mat* image_, cv::Size pattern_size, std::vector<Point2f>* corners_, std::vector<bool>* success, int success_n, int flags) {
        success->at(success_n) = findChessboardCornersCUDA(*image_, pattern_size, *corners_, flags);
        if (success->at(success_n)) {
            Mat img;
            cvtColor(*image_, img, COLOR_BGR2GRAY);
            cornerSubPix(img, *corners_, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.001));
        }
    }

    void doFindChessboard(Mat *image_, cv::Size pattern_size, std::vector<Point2f> *corners_, std::vector<bool> *success, int success_n, int flags) {
        success->at(success_n) = findChessboardCornersSB(*image_, pattern_size, *corners_, flags);
        if (success->at(success_n)) {
            Mat img;
            cvtColor(*image_, img, COLOR_BGR2GRAY);
            cornerSubPix(img, *corners_, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.001));
        }
    }

    void doFindCharuco(Mat* image_, const cv::Ptr<cv::aruco::CharucoBoard>& board_, const cv::Ptr<cv::aruco::Dictionary>& dictionary_, std::vector<Point2f>* corners_, std::vector<int>* ids_,
        const cv::Ptr<cv::aruco::DetectorParameters>& parameters_, std::vector<bool>* success, int success_n, bool useGPU) {
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        if(useGPU)
            cv::aruco::detectMarkersCompute(*image_, dictionary_, corners, ids, parameters_);
        else
            cv::aruco::detectMarkers(*image_, dictionary_, corners, ids, parameters_);
        if (ids.size() > 0) {
            cv::aruco::interpolateCornersCharuco(corners, ids, *image_, board_, *corners_, *ids_);
            if(ids_->size() > 4)
                success->at(success_n) = true;
        }
        else
            success->at(success_n) = false;
    }

    void doFindMarkers(Mat* image_, const cv::Ptr<cv::aruco::Dictionary>& dictionary_, std::vector<std::vector<Point2f>>* corners_, std::vector<int>* ids_,
        const cv::Ptr<cv::aruco::DetectorParameters>& parameters_, std::vector<bool>* success, int success_n, bool useGPU) {
        if(useGPU)
            cv::aruco::detectMarkersCompute(*image_, dictionary_, *corners_, *ids_, parameters_);
        else
            cv::aruco::detectMarkers(*image_, dictionary_, *corners_, *ids_, parameters_);
        if (ids_->size() > 0)
            success->at(success_n) = true;
        else
            success->at(success_n) = false;
    }

    void processFramesMT(std::vector<Mat> &images, cv::Size pattern_size,
        std::vector<std::vector<Point2f>> &image_corners, std::vector<bool>& success, int flags, bool useGPU) {
        if (images.size() > 0) {
            image_corners.resize(images.size());
            success.resize(images.size());
            std::vector<std::thread> threads;
            int thread_count = (int)images.size();
            if (useGPU) {
                for (int i = 0; i < thread_count; i++) {
                    threads.push_back(std::thread(doFindCUDA, &(images[i]), pattern_size, &(image_corners[i]), &success, i, flags));
                }
            }
            else {
                for (int i = 0; i < thread_count; i++) {
                    threads.push_back(std::thread(doFindChessboard, &(images[i]), pattern_size, &(image_corners[i]), &success, i, flags));
                }
            }
            std::for_each(threads.begin(), threads.end(), std::mem_fn(&std::thread::join));
        }
    }

    void processFramesMT(std::vector<Mat>& images, cv::Size pattern_size,
        std::vector<std::vector<Point2f>>& image_corners, int flags, bool useGPU) {
        std::vector<bool> dummy;
        processFramesMT(images, pattern_size, image_corners, dummy, flags, useGPU);
    }

    void processFramesMT(std::vector<Mat>& images, const cv::Ptr<cv::aruco::CharucoBoard>& board, std::vector<std::vector<cv::Point2f>>& charucoCorners,
        std::vector<std::vector<int>>& charucoIds, std::vector<bool>& success, const cv::Ptr<cv::aruco::DetectorParameters>& parameters, bool useGPU) {
        if (images.size() > 0) {
            charucoCorners.resize(images.size());
            charucoIds.resize(images.size());
            success.resize(images.size());
            std::vector<std::thread> threads;
            int thread_count = (int)images.size();
            for (int i = 0; i < thread_count; i++) {
                threads.push_back(std::thread(doFindCharuco, &(images[i]), board, board->dictionary, &(charucoCorners[i]), &(charucoIds[i]), parameters, &success, i, useGPU));
            }
            std::for_each(threads.begin(), threads.end(), std::mem_fn(&std::thread::join));
        }
    }

    void processFramesMT(std::vector<Mat>& images, const cv::Ptr<cv::aruco::CharucoBoard>& board, std::vector<std::vector<cv::Point2f>>& charucoCorners,
        std::vector<std::vector<int>>& charucoIds, const cv::Ptr<cv::aruco::DetectorParameters>& parameters, bool useGPU) {
        std::vector<bool> dummy;
        processFramesMT(images, board, charucoCorners, charucoIds, dummy, parameters, useGPU);
    }

    void detectMarkersMT(std::vector<Mat>& images, const cv::Ptr<cv::aruco::Dictionary>& dictionary, std::vector<std::vector<std::vector<cv::Point2f>>>& markerCorners,
        std::vector<std::vector<int>>& markerIds, std::vector<bool>& success, const cv::Ptr<cv::aruco::DetectorParameters>& parameters, bool useGPU) {
        if (images.size() > 0) {
            markerCorners.resize(images.size());
            markerIds.resize(images.size());
            success.resize(images.size());
            std::vector<std::thread> threads;
            int thread_count = (int)images.size();
            for (int i = 0; i < thread_count; i++) {
                threads.push_back(std::thread(doFindMarkers, &(images[i]), dictionary, &(markerCorners[i]), &(markerIds[i]), parameters, &success, i, useGPU));
            }
            std::for_each(threads.begin(), threads.end(), std::mem_fn(&std::thread::join));
        }
    }

    void detectMarkersMT(std::vector<Mat>& images, const cv::Ptr<cv::aruco::Dictionary>& dictionary, std::vector<std::vector<std::vector<cv::Point2f>>>& markerCorners,
        std::vector<std::vector<int>>& markerIds, const cv::Ptr<cv::aruco::DetectorParameters>& parameters, bool useGPU) {
        std::vector<bool> dummy;
        detectMarkersMT(images, dictionary, markerCorners, markerIds, dummy, parameters, useGPU);
    }

}

namespace me {

    ARComputeTask::ARComputeTask(ComputeTaskParams& params) {
        this->params = params;
    }

    bool ARComputeTask::start(const uint32_t numthreads) {
        if (params.marker_corners != nullptr && params.marker_ids != nullptr && params.marker_parameters != nullptr && params.dictionary != nullptr && params.filepath.size() > 0 && !busy()) {
            if (video.isOpened())
                video.release();
            video.open(params.filepath, cv::CAP_FFMPEG, { cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY });
            if (video.isOpened()) {
                total_frames = video.get(cv::CAP_PROP_FRAME_COUNT);
                counter = 0;
                if (params.board_corners != nullptr)
                    params.board_corners->resize(total_frames);
                if (params.board_ids != nullptr)
                    params.board_ids->resize(total_frames);
                if (params.board_success != nullptr)
                    params.board_success->resize(total_frames);
                if (params.marker_corners != nullptr)
                    params.marker_corners->resize(total_frames);
                if (params.marker_ids != nullptr)
                    params.marker_ids->resize(total_frames);
                if (params.marker_success != nullptr)
                    params.marker_success->resize(total_frames);
                for (uint32_t i = 0; i < numthreads; i++) {
                    threads.push_back(std::thread([this, numthreads, i] {
                        std::vector<bool> dummy;
                        dummy.resize(total_frames);
                        while (true) {
                            int frame;
                            cv::Mat m_frame;
                            bool success = false;
                            {
                                std::unique_lock<std::mutex> lock(reader_mutex);
                                frame = video.get(cv::CAP_PROP_POS_FRAMES);
                                success = video.read(m_frame);
                            }
                            if(success && params.marker_success != nullptr)
                                cv::doFindMarkers(&m_frame, params.dictionary, &params.marker_corners->at(frame), &params.marker_ids->at(frame), params.marker_parameters, params.marker_success, frame, i == numthreads - 1);
                            else if(success){
                                cv::doFindMarkers(&m_frame, params.dictionary, &params.marker_corners->at(frame), &params.marker_ids->at(frame), params.marker_parameters, &dummy, frame, i == numthreads - 1);
                            }
                            if (success && params.board != nullptr && params.board_corners != nullptr && params.board_ids != nullptr && params.marker_corners->at(frame).size() > 0) {
                                cv::aruco::interpolateCornersCharuco(params.marker_corners->at(frame), params.marker_ids->at(frame), m_frame, params.board, params.board_corners->at(frame), params.board_ids->at(frame));
                                if (params.board_ids->at(frame).size() > 4 && params.board_success != nullptr)
                                    params.board_success->at(frame) = true;
                            }
                            counter++;
                            {
                                std::unique_lock<std::mutex> lock(count_mutex);
                                if (counter >= total_frames || should_terminate) {
                                    return;
                                }
                            }
                        }
                    }));
                }
                return true;
            }
        }
        return false;
    }

    void ARComputeTask::stop() {
        {
            std::unique_lock<std::mutex> lock(count_mutex);
            should_terminate = true;
        }
        for (std::thread& active_thread : threads) {
            active_thread.join();
        }
        threads.clear();
        counter = 0;
        total_frames = 0;
        if (video.isOpened())
            video.release();
    }

    bool ARComputeTask::busy() {
        bool poolbusy;
        {
            std::unique_lock<std::mutex> lock(count_mutex);
            poolbusy = !(counter >= total_frames);
        }
        return poolbusy;
    }

    void ARComputeTask::wait() {
        for (std::thread& active_thread : threads) {
            active_thread.join();
        }
        threads.clear();
        counter = 0;
        total_frames = 0;
        if (video.isOpened())
            video.release();
    }

    int ARComputeTask::get_total() {
        return total_frames;
    }

    int ARComputeTask::get_remaining() {
        return total_frames - counter;
    }

    int ARComputeTask::get_complete() {
        return counter;
    }

    ARComputeTask::~ARComputeTask() {
        stop();
    }

}
