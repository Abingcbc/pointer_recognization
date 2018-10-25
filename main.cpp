#include"opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include<io.h>
#include<thread>
#include<mutex>
using namespace cv;
using namespace std;

//------------------------------------------------
//平面几何相关函数
//------------------------------------------------
//#define timeout
#define eps 0.0000000001
#define PI acos(-1.0)
int dcmp(double x) {
	if (fabs(x) < eps)return 0;
	else return x < 0 ? -1 : 1;
}
double Dot(Point A, Point B) { return A.x*B.x + A.y*B.y; }//向量点积
double Length(Point A) { return sqrt(Dot(A, A)); }//向量模长
double Cross(Point A, Point B) { return A.x*B.y - A.y*B.x; }//向量叉积
double Angle(Point A, Point B) { return acos(Dot(A, B) / Length(A) / Length(B)); }//求向量的夹角
double DistanceToLine(Point P, Point A, Point B)//点到直线的距离
{
	Point v1 = B - A, v2 = P - A;
	return fabs(Cross(v1, v2)) / Length(v1);//如果不加绝对值是带有方向的距离
}
double DistancetoSegment(Point P, Point A, Point B) {//点到线段的距离
	if (A == B)return Length(P - A);
	Point v1 = B - A, v2 = P - A, v3 = P - B;
	if (dcmp(Dot(v1, v2)) < 0)return  Length(v2);
	else if (dcmp(Dot(v1, v3)) > 0)return Length(v3);
	else return fabs(Cross(v1, v2)) / Length(v1);
}

//---------------------------------

//-----------------------------------------------------------------------------
class MyLine {
public:
	int id;//编号
	int k;//倾斜角[0-360)
	int l;//长度
	Point start;
	Point over;
public:
	MyLine(int ID = 0, int K = 0, int L = 0, Point Start = Point(0, 0), Point Over = Point(0, 0)) {
		static_cast<void>(id = ID), static_cast<void>(k = K), l = L;
		start = Start;
		over = Over;
	}//构造函数
};//自定义直线

list<MyLine> findMostDirection(list<MyLine> listLine) {
	map<int, int> times;
	for (auto i : listLine) {
		times[i.k / 2]++;
	}
	int tptr = times.begin()->first;
	list<MyLine> result;
	for (auto i : times) {
		if (i.second > times[tptr]) {
			tptr = i.first;
		}
	}

	for (auto i : listLine) {
		if (i.k / 2 == tptr) {
			result.push_back(i);
		}
	}
	return result;
}

int fail = 0,total=0;
Point center(0, 0);//圆心
int MAXRANGE = 0;

int findCircles(Mat& edge,Mat& out)
{
	//储存检测圆的容器  
	std::vector<Vec3f> circles;
	//霍夫找圆
	//参数为：待检测图像，检测结果，检测方法（这个参数唯一）,累加器的分辨率，两个圆间的距离，
	//canny门限的上限（下限自动设为上限的一半），圆心所需要的最小的投票数，最大和最小半径  
	HoughCircles(edge, circles, CV_HOUGH_GRADIENT, 2, 50, 200, 200, 100, 300);

	//找出最可能符合条件的圆盘
	//半径最大的且满足整个圆在图像内的那个
	int max = -1, pos = 0, count = 0;
	for (int i = 0; i < circles.size(); i++)
	{
		Vec3f f = circles[i];

		if (f[2] > max && f[0] + f[2] < edge.rows && f[0] - f[2] >= 0 && f[1] + f[2] < edge.cols && f[1] - f[2]>0)
		{
			max = f[2];
			pos = i;
			count++;
		}
	}
	if (circles.size() == 0 || max == -1) {
		//printf("未找到圆.....\n");
		return -1;
	}
	else {
		center = Point(circles[pos][0], circles[pos][1]);//找到的圆心
		int   radius = circles[pos][2];//找到的半径
	   //画圆，第五个参数调高可让线更粗
		circle(out, center, radius, Scalar(0, 255, 0), 1);
		//画出圆心
		circle(out, center, 3, Scalar(0, 255, 0), 1);
		//cout << "Center" << center << endl;
	}
	return count;
}

void findLinesP(Mat& edge,Mat& out)
{
	list<MyLine> list_MyLine;
	vector<Vec4i> lines2;//线段检测
	HoughLinesP(edge, lines2, 1, CV_PI / 180, 100, 50, 10);
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
		if (DistancetoSegment(center, A, B) < 30)//根据圆心到指针的距离阈值滤掉其他线段
		{
			//四个象限以及四个特殊位置
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
			else if (B.y < center.y && B.x < center.x) {
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
			//line(edge, A, B, Scalar(255, 0, i * 20 + 40), 2, CV_AA);
		}
		//imshow("edge", edge);
	}
	if (list_MyLine.empty())
	{
		cout << "can not find line" << endl;
		fail++;
		return;
	}
	//对提取出的线段进行筛选
	list<MyLine> final_list = findMostDirection(list_MyLine);

	//对筛选出的线段进行取均值
	double aver_k = 0, aver_aX = 0, aver_aY = 0, aver_bX = 0, aver_bY = 0;
	for (auto i : final_list) {
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

	double value = aver_k / 360 * MAXRANGE;
	cout << "指针角度：" << aver_k << "  表盘读数：" << value<< endl;

	//四个象限以及四个特殊位置
	Point A, B(aver_bX, aver_bY);
	if (B.y < center.y && B.x > center.x) {
		A = Point(B.x - 50 * tan(aver_k / 180 * PI), B.y + 50);
	}
	else if (B.y > center.y && B.x > center.x) {
		A = Point(B.x - 100, B.y - 100 * tan((aver_k - 90) / 180 * PI));
	}
	else if (B.y > center.y && B.x < center.x) {
		A = Point(B.x + 50 * tan((aver_k - 180) / 180 * PI), B.y - 50);
	}
	else if (B.y < center.y && B.x < center.x) {
		A = Point(B.x + 50, B.y + 50 * tan((aver_k - 270) / 180 * PI));
	}
	else if (B.y < center.y && B.x == center.x) {
		A = Point(B.x, B.y + 50);
	}
	else if (B.y == center.y && B.x > center.x) {
		A = Point(B.x - 50, B.y);
	}
	else if (B.y > center.y && B.x == center.x) {
		A = Point(B.x, B.y - 50);
	}
	else {
		A = Point(B.x + 50, B.y);
	}
	if (abs(aver_k - 360) < 1 || abs(aver_k - 0) < 1 || abs(aver_k - 180) < 1)
	{
		//我发现一个奇怪的bug，当直线斜率趋近于无穷时，即角度为360，180 时，画直线需要耗费大量时间，见异常文件夹中图片
	}
	else
		line(out, A, B, Scalar(180, 160, 0), 2, CV_AA);
}


// define a trackbar callback
void dashboard(Mat image)
{
	Mat edge_circle, edge_lines,gray;
	Mat out = image.clone();
	//磨平图像
	cvtColor(image, gray, CV_BGR2GRAY);
	blur(gray, edge_circle, Size(3, 3));
	medianBlur(gray, edge_lines, 3);
	Mat element = getStructuringElement(0, Size(5, 5), Point(-1, -1));
	erode(edge_circle, edge_circle, element);
	double low = 100;
	vector<vector<Point>>contours;
	int count = 0;
	Mat image_circle, image_lines;
	clock_t start, stop;
	start = clock();
	while (count > 3 || count < 1)
	{
		contours.clear();
		Canny(edge_circle, image_circle, low, low * 4, 3);
		findContours(image_circle, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
		drawContours(image_circle, contours, -1, Scalar(255), 2);
		count = findCircles(image_circle,out);
		low -= 10;
		if (low <= 0)
		{
			fail++;
			cout << "can not find circles" << endl;
			return;
		}
	}
	stop = clock();
#ifdef timeout
	cout << "寻找圆时间  " << (stop - start) << endl;
#endif // timeout
	//contours.clear();
	Canny(edge_lines, image_lines, low, low * 3, 3);
	//findContours(image_lines, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	//drawContours(image_lines, contours, -1, Scalar(255), 2);
	start = clock();
	findLinesP(image_lines,out);
	stop = clock();
#ifdef timeout
	cout << "寻找直线时间  " << (stop - start) << endl;
#endif // timeout
	namedWindow("final");
	imshow("final", out);
	waitKey(20);
}
void onTrackbar2(int, void*)
{
	/*Mat edge, cedge;
	cedge.create(image.size(), image.type());//用image生成一个final
	//磨平图像
	blur(gray, edge, Size(3, 3));
	//medianBlur(gray, edge, 5);
	//GaussianBlur(gray, edge, Size(3, 3), 2, 2);

	// Run the edge detector on grayscale
	Canny(edge, edge, edgeThresh, edgeThresh * 5, 3);

	//center = findCircles(edge,final);

	//findLines();
	//findLinesP(edge,final);

	//imshow("未操作", cedge);*/
}

void MyCapture()
{
	string file = "C:/Users/OY/Desktop/1.avi";
	VideoCapture cap(file);

	int frame_cnt = 0;
	int num = 0;
	Mat img;

	while (true) 
	{ 
		bool success = cap.read(img);		
		if (!success) {
			cout << "Process " << num << " frames from" << file << endl;
			break;
		}
		if (img.empty()){
			cout << "frame capture failed." << endl;
			break;
		}

		if (frame_cnt % 30 == 0) {
			++num;
			string name = "C:/Users/OY/Desktop/ship/" + to_string(num) + ".jpg";
			imwrite(name, img);
		}
		++frame_cnt; 
	}		
	cout << cap.get(CV_CAP_PROP_FRAME_COUNT) << endl;	
	cout << cap.get(CV_CAP_PROP_FPS) << endl;	
	cap.release();
}

const char* pathPhoto[] = {
	"./速度表/",
	"./高度表/",
	"./异常/",
};
const char* pathVideo[] = {
	"./Video/速度表1.avi",
	"./Video/速度表2.avi",
	"./Video/高度表1.avi",
};
bool iffinshed = false, ifdone = true;
Mat g_frame;
mutex g_mutex;

void Photo(Mat image)
{
	clock_t start, stop;
	start = clock();
	if (image.size().height > 500 && image.size().width > 500)
	{
		resize(image, image, Size(0, 0), 0.5, 0.5);
	}
	dashboard(image);
	stop = clock();
	cout << "运行总时间  " << (stop - start) << endl;
}

void Photo(string path)
{
	if (_access(path.c_str(), 0) == -1)
	{
		cout << "未找到指定文件" << endl;
		return;
	}
	Mat image = imread(path, 1);
	Photo(image);
}
void playVideo(string path)
{
	VideoCapture capture(path);
	while (true) {
		Mat frame;
		Mat edge;
		capture >> frame;

		if (frame.empty())
		{
			iffinshed = true;
			break;
		}
		imshow("Video", frame);
		waitKey(5);
		if (ifdone)
		{
			g_frame = frame.clone();
			g_mutex.lock();
			ifdone = !ifdone;
			g_mutex.unlock();
		}
	}

}
void Video(string path)
{
	thread t(playVideo, path);
	t.detach();

	while (!iffinshed)
	{
		if (!ifdone)
		{
			total++;
			Photo(g_frame);
			g_mutex.lock();
			ifdone = !ifdone;
			g_mutex.unlock();
		}
	}
	cout << "总共识别 " << total << " 张图片" << "识别失败 " << fail << " 失败率 " << (double)fail / total << endl;

}
void MainPage()
{
	string path;
	int num = 0;
	int select = 0;
	clock_t start, stop;
	start = clock();
	cout << "这是飞机仪表盘的识别项目";
	cout << "本程序可以识别飞机仪表的静态图片和动态视频，读出其示数" << endl;
	while (true)
	{
		cout << "输入数字进行操作，输入 0 退出程序" << endl;
		cout << "1. 识别速度表图片" << endl;
		cout << "2. 识别高度表图片" << endl;
		cout << "3. 识别速度表视频" << endl;
		cout << "4. 识别高度表视频" << endl;
		cout << "5. 识别自定义图片" << endl;
		cin >> select;
		stop = clock();
		num = (stop - start + rand()) % 9 + 1;
		switch (select)
		{
		default:
			cout << "未知操作" << endl;
			break;
		case 0:
			return;
		case 1:
			path = pathPhoto[0]+to_string(num)+".png";
			cout << "识别的是" << path <<endl;
			MAXRANGE = 360;
			Photo(path);
			break;
		case 2:
			path = pathPhoto[1] + to_string(num) + ".png";
			cout << "识别的是" << path << endl;
			MAXRANGE = 12;
			Photo(path);
			break;
		case 3:
			MAXRANGE = 360;
			Video(pathVideo[0]);
			break;
		case 4:
			MAXRANGE = 12;
			Video(pathVideo[2]);
			break;
		case 5:
			cout << "请输入图片的地址：";
			cin >> path;
			cout << "请输入仪表量程：";
			cin >> MAXRANGE;
			Photo(path);
			break;
		case 99: //这是异常图片接口，用来识别异常图片，不对外界开发
			path = pathPhoto[3] + to_string(1) + ".jpg";		
			Photo(path);
			break;
		}
	}
}
int main(int argc, const char** argv)
{
	MainPage();
	system("pause");
	return 0;
}

