#include"opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include<thread>
#include<mutex>
#include <math.h>

using namespace cv;
using namespace std;

#define eps 0.0000000001
#define PI acos(-1.0)

int fail = 0,total=0;
Point center(0, 0);//圆心

int findCircles(Mat edge,Mat out){
    //储存检测圆的容器
    std::vector<Vec3f> circles;
    //霍夫找圆
    HoughCircles(edge, circles, CV_HOUGH_GRADIENT, 2, 50, 200, 200, 100, 300);
    //找出最可能符合条件的圆盘
    //半径最大的且满足整个圆在图像内的那个
    int max = -1, pos = 0, count = 0;
    for (int i = 0; i < circles.size(); i++){
        Vec3f f = circles[i];
        if (f[2] > max && f[0] + f[2] < edge.rows && f[0] - f[2] >= 0 && f[1] + f[2] < edge.cols && f[1] - f[2]>0){
            max = f[2];
            pos = i;
            count++;
        }
    }
    if (circles.size() == 0 || max == -1) {
        return -1;
    } else {
        center = Point(circles[pos][0], circles[pos][1]);//找到的圆心
        int   radius = circles[pos][2];//找到的半径
        //画圆，第五个参数调高可让线更粗
        circle(out, center, radius, Scalar(0, 255, 0), 1);
        //画出圆心
        circle(out, center, 3, Scalar(0, 255, 0), 1);
    }
    return count;
}

vector<Vec4i> pre_lines;
void findLinesP(Mat edge,Mat out)
{
    vector<Vec4i> lines2;//线段检测
    HoughLinesP(edge, lines2, 1, CV_PI / 180, 80, 60, 10);
    bool find_lines = false;
    vector<Vec4i> temp;
    for (int i = 0; i < lines2.size(); i++)
    {
        Vec4i l = lines2[i];
        Point A(l[0], l[1]), B(l[2], l[3]);
        if (powf(abs(A.x - center.x), 2) + powf(abs(A.y - center.y), 2) >
            powf(abs(B.x - center.x), 2) + powf(abs(B.y - center.y), 2)) {
            Point tep = B;
            B = A;
            A = tep;
        }
        //根据圆心到指针的距离阈值滤掉其他线段
        if (powf(abs(A.x - center.x), 2) + powf(abs(A.y - center.y), 2) < 900) {
            temp.push_back(Vec4i(A.x, A.y, B.x, B.y));
            find_lines = true;
            line(out, A, B, Scalar(0, 0, 255), 4, CV_AA);
        }
    }
    if (!find_lines){
        fail++;
        for (auto l : pre_lines) {
            line(out, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 4, CV_AA);
        }
    } else {
        pre_lines = temp;
    }
    total++;
//    imshow("edge", out);
//    waitKey(1);
}

Mat dashboard(Mat image){
    Mat edge_circle, edge_lines,gray;
    Mat out = image.clone();
    Mat image_circle, image_lines;
    //预处理
    Mat element = getStructuringElement(0, Size(3, 3));
    erode(image, image, element);
    cvtColor(image, gray, CV_BGR2GRAY);
    //圆
    blur(gray, edge_circle, Size(3, 3));
    erode(edge_circle, edge_circle, element);
    double low = 100;
    vector<vector<Point>>contours;
    int count = 0;
    while (count > 3 || count < 1){
        contours.clear();
        Canny(edge_circle, image_circle, low, low * 4, 3);
        findContours(image_circle, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
        drawContours(image_circle, contours, -1, Scalar(255), 2);
        count = findCircles(image_circle,out);
        low -= 10;
        if (low <= 0){
            fail++;
            cout << "can not find circles" << endl;
        }
    }
    //直线
    medianBlur(gray, edge_lines, 3);
    Canny(edge_lines, image_lines, low, low * 3, 3);
    findLinesP(image_lines,out);
    return out;
}

int pre_total = total;
void playVideo(string path)
{
    VideoCapture capture(path);
    VideoWriter writer;
    Size size = Size(capture.get(CV_CAP_PROP_FRAME_WIDTH), capture.get(CV_CAP_PROP_FRAME_HEIGHT));
    writer.open("your output's path", CV_FOURCC('M','J','P','G') , 25, size, true);
    while (true) {
        Mat frame;
        Mat edge;
        capture >> frame;
        if (!frame.empty())
            writer << dashboard(frame);
        if (pre_total == total)
            break;
        else
            pre_total = total;
    }
}
int main(int argc, const char** argv)
{
    string path = "your video's path";
    playVideo(path);
    cout << "fail " << fail << endl;
    cout << "total " << total << endl;
    return 0;
}
