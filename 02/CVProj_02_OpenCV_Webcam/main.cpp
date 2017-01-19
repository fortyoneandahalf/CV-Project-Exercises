#include <vector>
#include "opencv2/opencv.hpp"

/**
 * @brief Create histogram of matrix data
 * @param frame The matrix to create the histogram from
 * @param histSize The number of histogram bins
 * @return hist A cv::Mat object containing the histogram
 */
cv::Mat setupHistogram(cv::Mat frame, int histSize)
{
    // Settings for histograms
    float range[] = {0, histSize};
    const float* histRange = {range};
    bool binSizeUniform = true;
    bool accumulate = false;

    // Create histogram for BW image
    cv::Mat hist;
    cv::calcHist(&frame, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, binSizeUniform, accumulate);

    return hist;
}

/**
 * @brief Create visualisation of histogram
 * @param hist Histogram to visualise as a cv::Mat object
 * @param histImage cv::Mat object to add the visualisation too; may be empty
 * @param histSize The number of histogram bins
 * @return histImage A cv::Mat object containing the visualised histogram
 */
cv::Mat setupHistogramImage(cv::Mat hist, cv::Mat histImage, int histSize)
{
    // Settings for histogram display
    int histWidth = 512;
    int histHeight = 400;
    int binWidth = cvRound((double) histWidth/histSize);

    // Create matrix to display histogram and normalise histogram into the size of the display matrix
    if(histImage.rows == 0)
        histImage = cv::Mat(histHeight, histWidth, CV_8UC3, cv::Scalar(0,0,0));

    cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());

    for(int i = 0; i < histSize; i++)
    {
        line(
            histImage,
            cv::Point(binWidth*(i-1), histHeight - cvRound(hist.at<float>(i-1))),
            cv::Point(binWidth*(i), histHeight - cvRound(hist.at<float>(i)) ),
            cv::Scalar(255, 0, 0),
            2,
            8,
            0);
    }

    return histImage;
}


/**
 * @brief Exercise 2 Part 3 - Histograms
 * @return
 */
int part3(cv::Mat frame, cv::Mat bwFrame)
{
    // Create greyscale histogram
    int histSize = 256;
    cv::Mat bwHist = setupHistogram(bwFrame, histSize);
    cv::Mat bwHistImage = setupHistogramImage(bwHist, cv::Mat(), histSize);

    // Display greyscale histogram
    cv::namedWindow("Greyscale Histogram", CV_WINDOW_AUTOSIZE);
    cv::imshow("Greyscale Histogram", bwHistImage);

    // Split colour stream into three channels
    std::vector<cv::Mat> rgbChannels;
    split(frame, rgbChannels);

    // Create colour channel histograms
    cv::Mat rHist = setupHistogram(rgbChannels[0], histSize);
    cv::Mat gHist = setupHistogram(rgbChannels[1], histSize);
    cv::Mat bHist = setupHistogram(rgbChannels[2], histSize);
    // Put all three histograms into same iamge window
    cv::Mat rgbHistImage;
    rgbHistImage = setupHistogramImage(rHist, cv::Mat(), histSize);
    rgbHistImage = setupHistogramImage(bHist, rgbHistImage, histSize);
    rgbHistImage = setupHistogramImage(gHist, rgbHistImage, histSize);

    // Display colour histograms
    cv::namedWindow("RGB Histogram", CV_WINDOW_AUTOSIZE);
    cv::imshow("RGB Histogram", rgbHistImage);

    return 0;
}


/**
 * @brief Exercise 2 Part 4 - Thresholding
 * @return
 */
cv::Mat part4(cv::Mat bwFrame)
{
    cv::Mat thresholdImage0, thresholdImage1, thresholdImage2, thresholdImage3, thresholdImage4;

    cv::threshold(
        bwFrame,
        thresholdImage0,
        125,
        255,
        0);
    cv::threshold(
        bwFrame,
        thresholdImage1,
        125,
        255,
        1);
    cv::threshold(
        bwFrame,
        thresholdImage2,
        125,
        255,
        2);
    cv::threshold(
        bwFrame,
        thresholdImage3,
        125,
        255,
        3);
    cv::threshold(
        bwFrame,
        thresholdImage4,
        125,
        255,
        4);

//    cv::imshow("Binary Thresholding", thresholdImage0);
//    cv::imshow("Binary Inverted Thresholding", thresholdImage1);
//    cv::imshow("Truncate Thresholding", thresholdImage2);
//    cv::imshow("To Zero Thresholding", thresholdImage3);
//    cv::imshow("To Zero Inverted Thresholding", thresholdImage4);

    return thresholdImage2;
}


/**
 * @brief Exercise 2 Part 5 - Filtering
 * @return
 */
cv::Mat part5(cv::Mat frame, int type)
{
    cv::Mat blurredImage;

    switch(type)
    {
    case 0:
        cv::blur(frame, blurredImage, cv::Size(7, 7));
        break;
    case 1:
        cv::GaussianBlur(frame, blurredImage, cv::Size(7, 7), 5, 5);
        break;
    case 2:
        cv::medianBlur(frame, blurredImage, 9);
        break;
    default:
        return 1;
    }

    return blurredImage;
}


/**
 * @brief Exercise 2 Part 6 - Canny Edge Detector
 * @return
 */
cv::Mat part6(cv::Mat frame)
{
    cv::Mat edges;

    cv::Canny(frame, edges, 30, 80);

    return edges;
}


/**
 * @brief Exercise 2 Part 7 - Morphological Operations
 * @return
 */
cv::Mat part7(cv::Mat frame)
{
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));

    cv::dilate(frame, frame, element);

    return frame;
}


/**
 * @brief Exercise 2 Part 8 - Create Image Pyramid
 * @return
 */
std::vector<cv::Mat> createPyramid(cv::Mat frame, int levels)
{
    std::vector<cv::Mat> pyramid;
    cv::Mat temp = frame;

    pyramid.push_back(temp);

    // Already have the first pyramid element in the vector, so start i from 1
    for(int i = 1; i < levels; i++)
    {
        cv::pyrDown(temp, temp, cv::Size(temp.cols/2, temp.rows/2));
        pyramid.push_back(temp);
    }

    return pyramid;
}

/**
 * @brief Exercise 2 Part 8 - Display Image Pyramid
 * @return
 */
int displayPyramid(std::vector<cv::Mat> pyramid)
{
    for(int i = 0; i < pyramid.size(); i++)
    {
        cv::imshow("Pyramid", pyramid[i]);
        cv::waitKey(0);
    }
}


int main()
{
    cv::VideoCapture cap;

    if(!cap.open(0))
        return 0;

    for(;;)
    {
        // Capture input from webcam
        cv::Mat frame;
        cap >> frame;

        if(frame.empty()) break;

        // Create black and white version of the webcam input
        cv::Mat bwFrame;
        cvtColor(frame, bwFrame, CV_RGB2GRAY);

        // Display the two streams
        cv::imshow("Smile!", frame);
//        cv::imshow("Smile like it's 1950!", bwFrame);

        cv::Mat processedImage;

//        part3(frame, bwFrame);
//        processedImage = part4(bwFrame);
//        processedImage = part5(frame, 2);
//        processedImage = part6(bwFrame);
//        processedImage = part7(processedImage);

//        cv::imshow("Processed Image", processedImage);

        // Exit on ESC
        char c = (char)cv::waitKey(1);
        if(c == 27) break;
        else if(c == 's')
        {
            displayPyramid(createPyramid(frame, 3));
        }
    }

    return 0;
}


