#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;

// 回调函数，用于响应滑动条事件，这里什么都不做
void on_trackbar(int, void*) {}

int main() {
    // 读取图片
    Mat image = imread("ply.png");
    if (image.empty()) {
        std::cout << "Could not read the image" << std::endl;
        return 1;
    }

    // 创建窗口
    namedWindow("Original Image", WINDOW_AUTOSIZE);
    imshow("Original Image", image);

    // 创建滑动条窗口
    namedWindow("HSV Threshold", WINDOW_AUTOSIZE);

    int hue_min = 0, hue_max = 180;
    int sat_min = 0, sat_max = 255;
    int val_min = 0, val_max = 255;

    createTrackbar("Hue Min", "HSV Threshold", &hue_min, 180, on_trackbar);
    createTrackbar("Hue Max", "HSV Threshold", &hue_max, 180, on_trackbar);
    createTrackbar("Sat Min", "HSV Threshold", &sat_min, 255, on_trackbar);
    createTrackbar("Sat Max", "HSV Threshold", &sat_max, 255, on_trackbar);
    createTrackbar("Val Min", "HSV Threshold", &val_min, 255, on_trackbar);
    createTrackbar("Val Max", "HSV Threshold", &val_max, 255, on_trackbar);

    Mat imgHSV;
    cvtColor(image, imgHSV, COLOR_BGR2HSV);

    while (true) {
        // 根据滑动条当前值过滤图像
        Mat mask;
        Scalar lower(hue_min, sat_min, val_min);
        Scalar upper(hue_max, sat_max, val_max);
        inRange(imgHSV, lower, upper, mask);

        // 显示结果
        Mat result;
        bitwise_and(image, image, result, mask);
        imshow("Filtered Image", result);

        // 等待用户按键，如果按键是'ESC'，则退出循环
        char key = (char)waitKey(1);
        if (key == 27) {
            break;
        }
    }

    // 保存处理后的图片
    imwrite("output.jpg", imgHSV);

    return 0;
}
