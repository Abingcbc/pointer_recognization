#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

//----------------------------几何
#define eps 0.0000000001
#define PI acos(-1.0)
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

//-----------------------------------------------------------------------------
class MyLine{
public:
    int id;//编号
    int k;//倾斜角[0-360)
    int l;//长度
    Point start;
    Point over;
    
public:
    MyLine(int ID=0,int K=0,int L=0, Point Start=Point(0, 0), Point Over=Point(0, 0)){
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


int main(){
    Mat srcImage = imread("/Users/cbc/c++/dashboard0.jpg"), temp, dst;
    //imshow("origin", srcImage);
    Canny(srcImage, temp, 90, 550, 3);//提取边缘(如果不边缘提取就会浪费巨大时间）
    //imshow("canny", temp);
    cvtColor(temp, dst, CV_GRAY2BGR);//将边缘提取的灰度图转换为BGR图便于画线
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
    Point center(circles[pos][0],circles[pos][1]);//找到的圆心
    int  radius= circles[pos][2];//找到的半径
    circle(dst,center,radius,Scalar(255),2);
    //imshow("draw_circle", dst);
  
    //--------------------------------------指针及刻度
    list<MyLine> list_MyLine;
    vector<Vec4i> lines2;//线段检测
    HoughLinesP(temp, lines2, 1, CV_PI/180, 50, 90, 10 );
    for( int i = 0; i < lines2.size(); i++ )
    {
        Vec4i l = lines2[i];
        Point A(l[0], l[1]),B(l[2], l[3]);
        if(DistancetoSegment(center,A,B)<30)//根据圆心到指针的距离阈值滤掉其他线段
        {
            bool down=(A.y+B.y-2*center.y>0);//判断长的在过圆心的水平线上部还是下部
            if(A.x==B.x){//斜率为无穷的情况
                list_MyLine.push_back(MyLine(i,90+(down?180:0),Length(Point(A.x-B.x,A.y-B.y)), A, B));
            }
            else if(A.y==B.y){//水平的情况
                list_MyLine.push_back(MyLine(i,A.x+B.x-2*center.x>0 ? 0:180,Length(Point(A.x-B.x,A.y-B.y)), A, B));
            }
            else{
                if(down){
                    if(A.y>center.y)
                        list_MyLine.push_back(MyLine(i,360-atan2(A.y-B.y,A.x-B.x)*180/PI,Length(Point(A.x-B.x,A.y-B.y)), A, B));
                    else
                        list_MyLine.push_back(MyLine(i,360-atan2(B.y-A.y,B.x-A.x)*180/PI,Length(Point(A.x-B.x,A.y-B.y)), A, B));
                }
                else{
                    if(A.y<center.y)
                        list_MyLine.push_back(MyLine(i,abs(atan2(A.y-B.y,A.x-B.x)*180/PI),Length(Point(A.x-B.x,A.y-B.y)), A, B));
                    else
                        list_MyLine.push_back(MyLine(i,abs(atan2(B.y-A.y,B.x-A.x)*180/PI),Length(Point(A.x-B.x,A.y-B.y)), A, B));
                }
            }
            line(dst, A, B, Scalar(0,160,0), 2, CV_AA);
        }
        
    }
    
    imshow("draw_line_all", dst);
    
    //--------------------------------------------------------------------
    
    
    waitKey(0);
    return 0;
}
