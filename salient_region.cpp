
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <map>

#define eps 0.0000000001
#define PI acos(-1.0)

using namespace std;
using namespace cv;

//deal with empty vector
vector<Vec4i> pre;
int fail = 0;
void findLinesP(Mat& edge,Mat& out, int min_length)
{
    vector<Vec4i> lines;
    HoughLinesP(edge, lines, 1, CV_PI / 180, 200, min_length, 1);
    if (lines.empty()) {
        for (auto i : pre) {
            line(out, Point(i[0], i[1]), Point(i[2], i[3]), Scalar(0, 0, 255), 4, CV_AA);
        }
        fail++;
    } else {
        for (auto i : lines) {
            line(out, Point(i[0], i[1]), Point(i[2], i[3]), Scalar(0, 0, 255), 2, CV_AA);
        }
    }
    pre = lines;
}

//TODO: this function may perform better on the whole image
void SalientRegionDetectionBasedOnLC(Mat src, Mat out, int min_length){
    int HistGram[256]={0};
    int row=src.rows, col=src.cols;
    int gray[row][col];
    int val;
    Mat sal = Mat::zeros(src.size(), CV_8UC1);
    Point3_<uchar>* p;
    //count the times of each grey level
    for (int i = 0; i < row; i++){
        for (int j = 0; j < col; j++){
            //get the RGB value of src at (i, j)
            p = src.ptr<Point3_<uchar>> (i,j);
            val = (p->x + p->y + p->z)/3;
            HistGram[val]++;
            gray[i][j]=val;
        }
    }
    int Dist[256];
    int X, Y;
    int max_gray=0;
    int min_gray=999999999;
    for (Y = 0; Y < 256; Y++){
        val = 0;
        //calculate grey level Y's color distance
        for (X = 0; X < 256; X++)
            val += abs(Y - X) * HistGram[X];
        Dist[Y] = val;
        max_gray = max(max_gray,val);
        min_gray = min(min_gray,val);
    }
    for (Y = 0; Y < row; Y++){
        for (X = 0; X < col; X++){
            if ((Dist[gray[Y][X]] - min_gray)*255/(max_gray - min_gray) > 30)
                //normalization
                sal.at<uchar>(Y,X) = (Dist[gray[Y][X]] - min_gray)*255/(max_gray - min_gray);
        }
    }
    //clear some noisy points
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(sal, sal, element);
    findLinesP(sal, out, min_length);
}

void splitPointer(Mat image, int height, int left, int top, int width) {
    GaussianBlur(image, image, Size(3, 3), 2, 2);
    Mat result = image(Rect(left, top, width, height));
    //restrict the length of lines to fliter wrong lines
    SalientRegionDetectionBasedOnLC(result.clone(), result, height>width?2*height/3:2*width/3);
}

int main() {
    //file's param: height, left, top, width
    string position_path = "your pointer's position";
    string video_path = "your video's path";
    ifstream ifs(position_path);
    vector<vector<int>> position;
    while (!ifs.eof()) {
        vector<int> pos;
        int number;
        char dots;
        for (int i = 0; i < 3; i++) {
            ifs >> number;
            pos.push_back(number);
            ifs >> dots;
        }
        ifs >> number;
        pos.push_back(number);
        if (pos[0] == 0) {
            pos = position[position.size()-1];
        }
        position.push_back(pos);
    }
    VideoCapture capture(video_path);
    int num = 0;
    Mat srcImage;
    Size size = Size(capture.get(CV_CAP_PROP_FRAME_WIDTH), capture.get(CV_CAP_PROP_FRAME_HEIGHT));
    VideoWriter writer;
    writer.open("your output's path", CV_FOURCC('M','J','P','G') , 25, size, true);
    while (true) {
        capture >> srcImage;
        splitPointer(srcImage, position[num][0]-3, position[num][1], position[num][2], position[num][3]-3);
        line(srcImage, Point(position[num][1], position[num][2]), Point(position[num][1]+position[num][3], position[num][2]), Scalar(255, 144, 30), 2);
        line(srcImage, Point(position[num][1], position[num][2]), Point(position[num][1], position[num][2]+position[num][0]), Scalar(255, 144, 30), 2);
        line(srcImage, Point(position[num][1]+position[num][3], position[num][2]), Point(position[num][1]+position[num][3], position[num][2]+position[num][0]), Scalar(255, 144, 30), 2);
        line(srcImage, Point(position[num][1], position[num][2]+position[num][0]), Point(position[num][1]+position[num][3], position[num][2]+position[num][0]), Scalar(255, 144, 30), 2);
        imshow("result", srcImage);
        waitKey(1);
        writer << srcImage;
        num++;
    }
    cout << fail << endl;
    return 0;
}
