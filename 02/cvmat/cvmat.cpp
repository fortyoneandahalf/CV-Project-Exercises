# include "opencv2/opencv.hpp"
# include <iostream>

int main_orig(void)
{
    // cv::Mat(int row, int col, int type) Constructor
    // allocate a 2*2 matrix of unsigned char type
    // with 1 channel
    cv::Mat m0 (2 ,2 , CV_8UC1);
    std::cout << " m0 = \n " << m0 << std::endl ;

    // a 2*2 matrix of unsigned char type
    // with 3 channels
    cv::Mat m1 (2 ,2 , CV_8UC3);
    std::cout << " m1 = \n " << m1 << std::endl ;

    // a 3*2 matrix of unsigned char type
    // with 3 channels
    // assigne values for each channel
    // using cv :: Scalar
    cv::Mat m2 (3 ,2 , CV_8UC3 , cv::Scalar (1 ,0 ,2));
    std::cout << " m2 = \n " << m2 << std::endl ;

    return 0;
}


int main_part1(void)
{
    // cv::Mat(int row, int col, int type) Constructor
    // allocate a 2*2 matrix of unsigned char type
    // with 1 channel
    cv::Mat m0 (2 , 2, CV_8UC3, cv::Scalar(126, 0, 255));
    std::cout << " m0 = \n " << m0 << std::endl ;

    // a 2*2 matrix of unsigned char type
    // with 3 channels
    cv::Mat m1 (3 , 2, CV_8UC3, cv::Scalar(1 ,0 ,2));
    std::cout << " m1 = \n " << m1 << std::endl ;

    // a 3*2 matrix of unsigned char type
    // with 3 channels
    // assigne values for each channel
    // using cv :: Scalar
    cv::Mat m2 (2 , 2, CV_64FC3, cv::Scalar(1.1, 0.1, 2.1));
    std::cout << " m2 = \n " << m2 << std::endl ;

    return 0;
}


int main_part2(void)
{
    // creat a mat object m0 and assign some value
    cv::Mat m0 (2 ,2 , CV_8UC1 , cv::Scalar (8));

    // creat a second mat object , and copy the
    // content from m0
    cv::Mat m_tmp(m0);

    // m0 and m_tmp display
    std::cout << " m0 = \n " << m0 << std::endl ;
    std::cout << " m_tmp = \n " << m_tmp << std::endl ;

    // do some change to m0
    m0.at<uchar>(0, 0) = 4;

    // check the resulted output
    std::cout << " m0 = \n " << m0 << std::endl ;
    std::cout << " m_tmp = \n " << m_tmp << std::endl ;

    return 0;
}


int main(int argc, char** argv)
{
    cv::Mat image;

    image = cv::imread(argv[1]);
    cv::imshow("Gray image", image);

    cv::waitKey(0);

    return 0;
}
