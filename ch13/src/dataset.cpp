#include "myslam/dataset.h"
#include "myslam/frame.h"

#include <boost/format.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

namespace myslam {

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

        initUndistortRectifyMap(
            cameraMatrix, distortionCoefficient, Mat(), Mat(),
            Size(640, 480), CV_16SC2, map1, map2
        );

    }

bool Dataset::Init() {

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
    current_image_index_ = 0;
    return true;
}

Frame::Ptr Dataset::NextFrame() {
    cv::Mat image, image_leftD, image_rightD;
    cv::Mat image_left, image_right;

    // read images
    video >> image;

    if (image.data == nullptr) {
        LOG(WARNING) << "cannot find images at index " << current_image_index_;
        return nullptr;
    }

    cvtColor(image, image, COLOR_BGR2GRAY);

    image_leftD = image(Rect(0, 0, image.cols / 2, image.rows));
    image_rightD = image(Rect(image.cols / 2, 0, image.cols / 2, image.rows));

    remap(image_leftD, image_left, map1, map2, INTER_LINEAR);
    remap(image_rightD, image_right, map1, map2, INTER_LINEAR);

    imshow("image", image);
    if ((waitKey(500) & 0xFF) == 'q') {
		return nullptr;
	}

    cout << "Frame: " << current_image_index_ << endl;

    auto new_frame = Frame::CreateFrame();
    new_frame->left_img_ = image_left;
    new_frame->right_img_ = image_right;
    current_image_index_++;
    return new_frame;
}

}  // namespace myslam
