#include "myslam/dataset.h"
#include "myslam/frame.h"
#include "myslam/config.h"

#include <boost/format.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

namespace myslam {

#if 1
    void imshowAndWait(string title, Mat img) {
        imshow("imshowAndWait: " + title, img);
        if (Config::Get<int>("isw_block") != 0) {
            while (1) {
                if (waitKey(50) != -1) {
                    break;
                }
            }
        } else {
            waitKey(1);
        }
    }
#else
    void imshowAndWait(Mat img) {}
#endif

Dataset::Dataset(const std::string& dataset_path)
    : dataset_path_(dataset_path), video(1) {

        cameraMatrix = (
            Mat_<double>(3, 3) <<
            421.288596, 0.000000, 335.509012,
            0.000000, 421.079122, 261.189746,
            0.000000, 0.000000, 1.000000
        );

        distortionCoefficient = (
            Mat_<double>(5, 1) <<
            -0.347294, 0.090865, 0.001867, 0.001221, 0.000000
        );

    }

bool Dataset::Init() {

    initUndistortRectifyMap(
        cameraMatrix, distortionCoefficient, Mat(), Mat(),
        Size(640, 480), CV_16SC2, map1, map2
    );

    Mat33 K;
    K << cameraMatrix.at<double>(0, 0), cameraMatrix.at<double>(0, 1), cameraMatrix.at<double>(0, 2),
        cameraMatrix.at<double>(1, 0), cameraMatrix.at<double>(1, 1), cameraMatrix.at<double>(1, 2),
        cameraMatrix.at<double>(2, 0), cameraMatrix.at<double>(2, 1), cameraMatrix.at<double>(2, 2);
    {
        Vec3 t;
        t << 0.061, 0, 0;
        t = K.inverse() * t;
        K = K * 0.5;
        Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                          t.norm(), SE3(SO3(), t)));
        cameras_.push_back(new_camera);
        LOG(INFO) << "Camera " << 0 << " extrinsics: " << t.transpose();
    }
    {
        Vec3 t;
        t << -0.061, 0, 0;
        t = K.inverse() * t;
        K = K * 0.5;
        Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                          t.norm(), SE3(SO3(), t)));
        cameras_.push_back(new_camera);
        LOG(INFO) << "Camera " << 1 << " extrinsics: " << t.transpose();
    }

    video.set(CAP_PROP_FRAME_WIDTH, 1280);
    video.set(CAP_PROP_FRAME_HEIGHT, 480);

    current_image_index_ = 0;
    return true;
}

Frame::Ptr Dataset::NextFrame() {
    cv::Mat image, img_left, img_right;

    // read images
    if (!first_frame_) {
        video >> image;
    } else {
        for (int i = 0; i < 30; i++) {
            video >> image;
        }
        first_frame_ = false;
    }

    if (image.data == nullptr) {
        LOG(WARNING) << "cannot find images at index " << current_image_index_;
        return nullptr;
    }

#define OUT(what) {cout << (what) << endl;};

    {
        Mat tmp;
        cvtColor(image, tmp, COLOR_BGR2GRAY);
        image = tmp;
    }

    img_left = image(Rect(0, 0, image.cols / 2, image.rows)).clone();
    img_right = image(Rect(image.cols / 2, 0, image.cols / 2, image.rows)).clone();
    
    {
        Mat tmp(img_left.rows, img_left.cols, img_left.type());
        remap(img_left, tmp, map1, map2, INTER_LINEAR);
        img_left = tmp;
    }

    {
        Mat tmp(img_right.rows, img_right.cols, img_right.type());
        remap(img_right, tmp, map1, map2, INTER_LINEAR);
        img_right = tmp;
    }

    cout << "Frame: " << current_image_index_ << endl;

//     imwrite("out/orig.png", image);
//     imwrite("out/lf.png", img_left);
//     imwrite("out/rt.png", img_right);

    if (Config::Get<int>("frame_wait") != 0) {
        imshowAndWait("src", image);
    }

    auto new_frame = Frame::CreateFrame();
    new_frame->left_img_ = img_left;
    new_frame->right_img_ = img_right;
    current_image_index_++;
    return new_frame;
}

}  // namespace myslam
