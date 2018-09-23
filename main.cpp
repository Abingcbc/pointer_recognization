#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

//----------------------------几何
#define eps 0.0000000001
#define PI acos(-1.0)
#define MAXRANGE 170
int dcmp(double x){
    if(fabs(x)<eps)return 0;
    else return x<0 ? -1:1;
}
double Dot(Point A,Point B){return A.x*B.x+A.y*B.y;}//向量点积
double Length(Point A){return sqrt(Dot(A,A));}//向量模长
double Cross(Point A,Point B){return A.x*B.y-A.y*B.x;}//向量叉积
double Angle(Point A,Point B){return acos(Dot(A,B)/Length(A)/Length(B));}//求向量的夹角
double DistanceToLine(Point P,Point A,Point B)//点到直线的距离
{
    Point v1=B-A,v2=P-A;
    return fabs(Cross(v1,v2))/Length(v1);//如果不加绝对值是带有方向的距离
}
double DistancetoSegment(Point P,Point A,Point B){//点到线段的距离
    if(A==B)return Length(P-A);
    Point v1=B-A,v2=P-A,v3=P-B;
    if(dcmp(Dot(v1,v2))<0)return  Length(v2);
    else if(dcmp(Dot(v1,v3))>0)return Length(v3);
    else return fabs(Cross(v1,v2))/Length(v1);
}
//---------------------------------

Mat srcImage = imread("/Users/cbc/Desktop/speed-table/5.png"), temp, dst, bina;
int low_v = 0, high_v = 100;
int radius;
Point center;
//-----------------------------------------------------------------------------
class MyLine{
public:
    int id;//编号
    int k;//倾斜角[0-360)
    int l;//长度
    Point start;
    Point over;
public:
    MyLine(int ID=0,int K=0,int L=0, Point Start=Point(0, 0), Point Over=Point(0, 0)) {
        static_cast<void>(id=ID),static_cast<void>(k=K),l=L;
        start = Start;
        over = Over;
    }//构造函数
    bool operator<(const MyLine &A){
        return k<A.k;
    }//重定义小于号
    void print(){printf("id: %3d  k: %3d°  l: %3d\n",id,k,l);}//输出函数
};//自定义直线
//-----------------------------------------------------------------------------

// 仿照matlab，自适应求高低两个门限
void _AdaptiveFindThreshold(CvMat *dx, CvMat *dy, double *low, double *high)
{
    Size size;
    IplImage *imge=0;
    int i,j;
    CvHistogram *hist;
    int hist_size = 255;
    float range_0[]={0,256};
    float* ranges[] = { range_0 };
    double PercentOfPixelsNotEdges = 0.7;
    size = cvGetSize(dx);
    imge = cvCreateImage(size, IPL_DEPTH_32F, 1);
    // 计算边缘的强度, 并存于图像中
    float maxv = 0;
    for(i = 0; i < size.height; i++ )
    {
        const short* _dx = (short*)(dx->data.ptr + dx->step*i);
        const short* _dy = (short*)(dy->data.ptr + dy->step*i);
        float* _image = (float *)(imge->imageData + imge->widthStep*i);
        for(j = 0; j < size.width; j++)
        {
            _image[j] = (float)(abs(_dx[j]) + abs(_dy[j]));
            maxv = maxv < _image[j] ? _image[j]: maxv;
            
        }
    }
    if(maxv == 0){
        *high = 0;
        *low = 0;
        cvReleaseImage( &imge );
        return;
    }
    
    // 计算直方图
    range_0[1] = maxv;
    hist_size = (int)(hist_size > maxv ? maxv:hist_size);
    hist = cvCreateHist(1, &hist_size, CV_HIST_ARRAY, ranges, 1);
    cvCalcHist( &imge, hist, 0, NULL );
    int total = (int)(size.height * size.width * PercentOfPixelsNotEdges);
    float sum=0;
    int icount = hist->mat.dim[0].size;
    
    float *h = (float*)cvPtr1D( hist->bins, 0 );
    for(i = 0; i < icount; i++)
    {
        sum += h[i];
        if( sum > total )
            break;
    }
    // 计算高低门限
    *high = (i+1) * maxv / hist_size ;
    *low = *high * 0.4;
    cvReleaseImage( &imge );
    cvReleaseHist(&hist);
}

void AdaptiveFindThreshold(const Mat &src, double *low, double *high, int aperture_size=3)
{
    const int cn = src.channels();
    Mat dx(src.rows, src.cols, CV_16SC(cn));
    Mat dy(src.rows, src.cols, CV_16SC(cn));
    
    Sobel(src, dx, CV_16S, 1, 0, aperture_size, 1, 0, BORDER_REPLICATE);
    Sobel(src, dy, CV_16S, 0, 1, aperture_size, 1, 0, BORDER_REPLICATE);
    
    CvMat _dx = dx, _dy = dy;
    _AdaptiveFindThreshold(&_dx, &_dy, low, high);
    
}

list<MyLine> findMostDirection(list<MyLine> listLine) {
    map<int, int> times;
    for (auto i: listLine) {
        times[i.k / 10]++;
    }
    int tptr = times.begin()->first;
    list<MyLine> result;
    for (auto i: times) {
        if (i.second > times[tptr]) {
            tptr = i.first;
        }
    }
    for (auto i: listLine) {
        if (i.k / 10 == tptr) {
            result.push_back(i);
        }
    }
    return result;
}

int main(){
    resize(srcImage, srcImage, Size(0, 0), 0.5, 0.5);
    blur(srcImage, srcImage, Size(3, 3));
    vector<vector<Point>>contours;
    cvtColor(srcImage, bina, CV_RGB2GRAY);
    //imshow("origin", srcImage);
    double low = 0.0, high = 0.0;
    AdaptiveFindThreshold(srcImage, &low, &high);
    cout << low << endl << high << endl;
    Mat element = getStructuringElement(0, Size(7, 7), Point(-1, -1));
    erode(srcImage, srcImage, element);
    //imshow("erode", srcImage);
    //dilate(srcImage, srcImage, element);
    //imshow("dilate", srcImage);
    Canny(srcImage, temp, low * 2, high * 2, 3);//提取边缘(如果不边缘提取就会浪费巨大时间）
    findContours(temp, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    drawContours(temp, contours, -1, Scalar(255), 2);
    cvtColor(temp, dst, CV_GRAY2BGR);//将边缘提取的灰度图转换为BGR图便于画线
    //imshow("canny", temp);
    //储存检测圆的容器
    vector<Vec3f> circles;
    //调用Hough变换检测圆
    //参数为：待检测图像，检测结果，检测方法（这个参数唯一）,累加器的分辨率，两个圆间的距离，canny门限的上限（下限自动设为上限的一半），圆心所需要的最小的投票数，最大和最小半径
    HoughCircles(temp,circles,CV_HOUGH_GRADIENT,2,50,200,100,100,300);
    //找出圆盘（因为最大的不一定是的，所以加了几个限制条件）
    int pos=0;
    int max=-1;
    for(int i = 0; i < circles.size(); i++ )
    {
        Vec3f f=circles[i];
        if(f[2]>max && f[0]+f[2]<temp.rows && f[0]-f[2]>=0 && f[1]+f[2]<temp.cols && f[1]-f[2]>0)
        {
            max=f[2];
            pos=i;
        }
    }
    center = Point(circles[pos][0],circles[pos][1]);//找到的圆心
    radius= circles[pos][2];//找到的半径
    circle(dst, center, 3, Scalar(0, 180, 0));
    circle(dst,center,radius,Scalar(255),2, CV_AA);
    list<MyLine> list_MyLine;
    vector<Vec4i> lines2;//线段检测
    HoughLinesP(temp, lines2, 1, CV_PI/180, 50, 90, 10 );
    for( int i = 0; i < lines2.size(); i++ )
    {
        Vec4i l = lines2[i];
        Point A(l[0], l[1]),B(l[2], l[3]);
        if (powf(abs(A.x - center.x), 2) + powf(abs(A.y - center.y), 2) >
            powf(abs(B.x - center.x), 2) + powf(abs(B.y - center.y), 2)) {
            Point tep = B;
            B = A;
            A = tep;
        }
        if(DistancetoSegment(center,A,B)<30)//根据圆心到指针的距离阈值滤掉其他线段
        {
            if (B.y < center.y && B.x > center.x) {
                list_MyLine.push_back(MyLine(i, atan2(abs(B.x - A.x), abs(B.y - A.y)) * 180 / PI,
                                       Length(Point(B.x - A.x, B.y - A.y)), A, B));
            }
            else if (B.y > center.y && B.x > center.x) {
                list_MyLine.push_back(MyLine(i, 90 + atan2(abs(B.y - A.y), abs(B.x - A.x)) * 180 / PI, Length(Point(B.x - A.x, B.y - A.y)), A, B));
            }
            else if (B.y > center.y && B.x < center.x) {
                list_MyLine.push_back(MyLine(i, 180 + atan2(abs(B.x - A.x), abs(B.y - A.y)) * 180 / PI, Length(Point(B.x - A.x, B.y - A.y)), A, B));
            }
            else if (B.y < center.y && B.y < center.y){
                list_MyLine.push_back(MyLine(i, 270 + atan2(abs(B.y - A.y), abs(B.x - A.x)) * 180 / PI, Length(Point(B.x - A.x, B.y - A.y)), A, B));
            }
            else if (B.y < center.y && B.x == center.x) {
                list_MyLine.push_back(MyLine(i, 0, Length(Point(B.x - A.x, B.y - A.y)), A, B));
            }
            else if (B.y == center.y && B.x > center.x) {
                list_MyLine.push_back(MyLine(i, 90, Length(Point(B.x - A.x, B.y - A.y)), A, B));
            }
            else if (B.y > center.y && B.x == center.x) {
                list_MyLine.push_back(MyLine(i, 180, Length(Point(B.x - A.x, B.y - A.y)), A, B));
            }
            else {
                list_MyLine.push_back(MyLine(i, 270, Length(Point(B.x - A.x, B.y - A.y)), A, B));
            }
        }
    }
    list<MyLine> final_list = findMostDirection(list_MyLine);
    float aver_k = 0, aver_aX = 0, aver_aY = 0, aver_bX = 0, aver_bY = 0;
    for (auto i:final_list) {
        aver_k += i.k;
        aver_aX += i.start.x;
        aver_aY += i.start.y;
        aver_bX += i.over.x;
        aver_bY += i.over.y;
    }
    aver_k /= final_list.size();
    aver_aX /= final_list.size();
    aver_aY /= final_list.size();
    aver_bX /= final_list.size();
    aver_bY /= final_list.size();
    cout << aver_k << endl;
    cout << aver_k / 360 * MAXRANGE << endl;
    line(dst, Point(aver_aX, aver_aY), Point(aver_bX, aver_bY), Scalar(0, 160, 0), 2, CV_AA);
    imshow("final", dst);
    
    
    //--------------------------------------------------------------------
    
    
    waitKey(0);
    return 0;
}
