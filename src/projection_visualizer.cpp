#include <ros/ros.h>
#include <cv.h>
#include <highgui.h>
#include <cassert>
#include <fstream>

const double K = 0.255;

void setColor(IplImage* img, int x, int y, int channel, unsigned char color)
{
    (*(img->imageData + (y)*img->widthStep + (x)*img->nChannels + (channel))) = color;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "projection_visualizer_node");

    assert(argc == 2 && "bad input");

    std::ifstream fin(argv[1]);
    assert(fin.is_open() && "opening input file error");

    size_t W, H;
    fin >> W >> H;
    IplImage *img = cvCreateImage(cvSize(W, H), IPL_DEPTH_8U, 1);
    
    for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
    {
        static int color;
        fin >> color;
        if (color == -1) setColor(img, x, y, 0, 0);
        else             setColor(img, x, y, 0, color * K);
    }
    fin.close();

    cvShowImage("img", img);
    cvWaitKey();

    cvReleaseImage(&img);
}