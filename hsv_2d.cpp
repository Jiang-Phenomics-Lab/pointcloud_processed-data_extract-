#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;

// �ص�������������Ӧ�������¼�������ʲô������
void on_trackbar(int, void*) {}

int main() {
    // ��ȡͼƬ
    Mat image = imread("ply.png");
    if (image.empty()) {
        std::cout << "Could not read the image" << std::endl;
        return 1;
    }

    // ��������
    namedWindow("Original Image", WINDOW_AUTOSIZE);
    imshow("Original Image", image);

    // ��������������
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
        // ���ݻ�������ǰֵ����ͼ��
        Mat mask;
        Scalar lower(hue_min, sat_min, val_min);
        Scalar upper(hue_max, sat_max, val_max);
        inRange(imgHSV, lower, upper, mask);

        // ��ʾ���
        Mat result;
        bitwise_and(image, image, result, mask);
        imshow("Filtered Image", result);

        // �ȴ��û����������������'ESC'�����˳�ѭ��
        char key = (char)waitKey(1);
        if (key == 27) {
            break;
        }
    }

    // ���洦����ͼƬ
    imwrite("output.jpg", imgHSV);

    return 0;
}
