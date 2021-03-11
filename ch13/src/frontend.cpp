//
// Created by gaoxiang on 19-5-2.
//

#include "boost/format.hpp"

#include <opencv2/opencv.hpp>

#include "myslam/algorithm.h"
#include "myslam/backend.h"
#include "myslam/config.h"
#include "myslam/feature.h"
#include "myslam/frontend.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/viewer.h"
#include "myslam/util.h"

namespace myslam {

std::string to_string(FrontendStatus status) {
    switch (status) {
    case FrontendStatus::INITING:
        return std::string("INITING");
    case FrontendStatus::TRACKING_GOOD:
        return std::string("TRACKING_GOOD");
    case FrontendStatus::TRACKING_BAD:
        return std::string("TRACKING_BAD");
    case FrontendStatus::LOST:
        return std::string("LOST");
    default:
        return std::string("Unknown");
    }
}

Frontend::Frontend() {
    gftt_ =
        cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    num_features_ = Config::Get<int>("num_features");
    num_features_init_ = Config::Get<int>("num_features_init");
    num_features_tracking_ = Config::Get<int>("num_features_tracking");
    num_features_tracking_bad_ = Config::Get<int>("num_features_tracking_bad");
    num_features_needed_for_keyframe_ = Config::Get<int>("num_features_needed_for_keyframe");
//     feature_track_canvas.create(200, 200, CV_8UC3);
//     feature_track_canvas = cv::Scalar(0, 0, 255);
}

bool Frontend::AddFrame(myslam::Frame::Ptr frame) {
    current_frame_ = frame;
    
    features_cnts.emplace_back();

    switch (status_) {
        case FrontendStatus::INITING:
            StereoInit();
            viewer_->PostValue("init_cnts", 1);
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            Track();
            viewer_->PostValue("init_cnts", 0);
            break;
        case FrontendStatus::LOST:
            Reset();
            viewer_->PostValue("init_cnts", 0);
            break;
    }

    last_frame_ = current_frame_;
    return true;
}

bool Frontend::Track() {
    LOG(INFO) << "Enter Track";
    if (last_frame_) {
        current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
    }

    int num_track_last = TrackLastFrame();
    tracking_inliers_ = EstimateCurrentPose();

    if (tracking_inliers_ > num_features_tracking_) {
        // tracking good
        status_ = FrontendStatus::TRACKING_GOOD;
    } else if (tracking_inliers_ > num_features_tracking_bad_) {
        // tracking bad
        status_ = FrontendStatus::TRACKING_BAD;
    } else {
        // lost
        status_ = FrontendStatus::LOST;
    }

    InsertKeyframe();
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

    if (Config::Get<int>("features_log")) {
        std::stringstream ss;
        for (int i = 0; i < current_frame_->features_left_.size(); i++) {
            Feature::Ptr ftl = current_frame_->features_left_[i];
            ss << ftl->position_.pt << "," << !ftl->map_point_.expired() << "|";
        }
        LOG(INFO) << "Feature log: track " << ss.str();
    }

    if (viewer_) viewer_->AddCurrentFrame(current_frame_);
    return true;
}

bool Frontend::InsertKeyframe() {
    if (tracking_inliers_ >= num_features_needed_for_keyframe_) {
        // still have enough features, don't insert keyframe
        return false;
    }
    LOG(INFO) << "InsertKeyframe: tracking_inliers " << tracking_inliers_ << " < needed " << num_features_needed_for_keyframe_;
    // current frame is a new keyframe
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);

    LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
              << current_frame_->keyframe_id_;

    SetObservationsForKeyFrame();
    DetectFeatures();  // detect new features

    // track in right image
    FindFeaturesInRight();
    // triangulate map points
    TriangulateNewPoints();
    // update backend because we have a new keyframe
    backend_->UpdateMap();

    if (viewer_) viewer_->UpdateMap();

    return true;
}

void Frontend::SetObservationsForKeyFrame() {
    for (auto &feat : current_frame_->features_left_) {
        auto mp = feat->map_point_.lock();
        if (mp) mp->AddObservation(feat);
    }
}

int Frontend::TriangulateNewPoints() {
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    SE3 current_pose_Twc = current_frame_->Pose().inverse();
    int cnt_triangulated_pts = 0;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        if (current_frame_->features_left_[i]->map_point_.expired() &&
            current_frame_->features_right_[i] != nullptr) {
            // 左图的特征点未关联地图点且存在右图匹配点，尝试三角化
            std::vector<Vec3> points{
                camera_left_->pixel2camera(
                    Vec2(current_frame_->features_left_[i]->position_.pt.x,
                         current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_right_[i]->position_.pt.x,
                         current_frame_->features_right_[i]->position_.pt.y))};
            Vec3 pworld = Vec3::Zero();

            if (triangulation(poses, points, pworld) && pworld[2] > 0) {
                auto new_map_point = MapPoint::CreateNewMappoint();
                pworld = current_pose_Twc * pworld;
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(
                    current_frame_->features_left_[i]);
                new_map_point->AddObservation(
                    current_frame_->features_right_[i]);

                current_frame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_right_[i]->map_point_ = new_map_point;
                map_->InsertMapPoint(new_map_point);
                cnt_triangulated_pts++;
            }
        }
    }
    LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
    return cnt_triangulated_pts;
}

int Frontend::EstimateCurrentPose() {
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
        LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // vertex
    VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
    vertex_pose->setId(0);
    vertex_pose->setEstimate(current_frame_->Pose());
    optimizer.addVertex(vertex_pose);

    // K
    Mat33 K = camera_left_->K();

    // edges
    int index = 1;
    std::vector<EdgeProjectionPoseOnly *> edges;
    std::vector<Feature::Ptr> features;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        auto mp = current_frame_->features_left_[i]->map_point_.lock();
        if (mp) {
            features.push_back(current_frame_->features_left_[i]);
            EdgeProjectionPoseOnly *edge =
                new EdgeProjectionPoseOnly(mp->pos_, K);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(
                toVec2(current_frame_->features_left_[i]->position_.pt));
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }

    bool pose_estimation_log = Config::Get<int>("pose_estimation_log") != 0;
    // estimate the Pose the determine the outliers
    // The original is 5.991;
    const double chi2_th = Config::Get<double>("chi2_th");
    int cnt_outlier = 0;
    for (int iteration = 0; iteration < 4; ++iteration) {
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        cnt_outlier = 0;
        
        if (pose_estimation_log) {
            LOG(INFO) << "Iteration: " << iteration;
        }

        // count the outliers
        for (size_t i = 0; i < edges.size(); ++i) {
            auto e = edges[i];
            if (features[i]->is_outlier_) {
                e->computeError();
            }
            if (e->chi2() > chi2_th) {
                features[i]->is_outlier_ = true;
                e->setLevel(1);
                cnt_outlier++;
                if (pose_estimation_log) {
                    LOG(INFO) << "Removed: " << e->chi2();
                }
            } else {
                features[i]->is_outlier_ = false;
                e->setLevel(0);
            };

            if (iteration == 2) {
                e->setRobustKernel(nullptr);
            }
        }
    }

    LOG(INFO) << "EstimateCurrentPose: Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
              << features.size() - cnt_outlier;
    // Set pose and outlier
    current_frame_->SetPose(vertex_pose->estimate());

    LOG(INFO) << "Current Pose = \n" << current_frame_->Pose().matrix();

    for (auto &feat : features) {
        if (feat->is_outlier_) {
            feat->map_point_.reset();
            feat->is_outlier_ = false;  // maybe we can still use it in future
        }
    }
    return features.size() - cnt_outlier;
}

int Frontend::TrackLastFrame() {
    // use LK flow to estimate points in the right image
    LOG(INFO) << "Enter TrackLastFrame";
    std::vector<cv::Point2f> kps_last, kps_current;
    for (auto &kp : last_frame_->features_left_) {
        if (kp->map_point_.lock()) {
            // use project point
            auto mp = kp->map_point_.lock();
            auto px =
                camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(cv::Point2f(px[0], px[1]));
        } else {
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(kp->position_.pt);
        }
    }
    LOG(INFO) << "Built initial guess";

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        last_frame_->left_img_, current_frame_->left_img_, kps_last,
        kps_current, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    LOG(INFO) << "Did optical flow";
    int num_good_pts = 0;

    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_current[i], 7);
            Feature::Ptr feature(new Feature(current_frame_, kp));
            feature->map_point_ = last_frame_->features_left_[i]->map_point_;
            current_frame_->features_left_.push_back(feature);
            num_good_pts++;
        }
    }
    
    LOG(INFO) << "Associated features";

    LOG(INFO) << "TrackLastFrame: Find " << num_good_pts << " in the last image.";
    return num_good_pts;
}

bool Frontend::StereoInit() {
    if (last_frame_) {
        current_frame_->SetPose(last_frame_->Pose());
    }
    int num_features_left = DetectFeatures();
    int num_coor_features = FindFeaturesInRight();

    if (num_coor_features < num_features_init_) {
        return false;
    }

    bool build_map_success = BuildInitMap();
    if (build_map_success) {
        status_ = FrontendStatus::TRACKING_GOOD;
        if (viewer_) {
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
    }

    if (Config::Get<int>("features_log")) {
        std::stringstream ss;
        for (int i = 0; i < current_frame_->features_left_.size(); i++) {
            Feature::Ptr ftl = current_frame_->features_left_[i];
            Feature::Ptr ftr = current_frame_->features_right_[i];
            ss << ftl->position_.pt << "," << (bool(ftr) ? to_string(ftr->position_.pt) : "no") << "," << !ftl->map_point_.expired() << "|";
        }
        LOG(INFO) << "Feature log: init " << ss.str();
    }

    return build_map_success;
}

int Frontend::DetectFeatures() {
    cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
    for (auto &feat : current_frame_->features_left_) {
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                      feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(current_frame_->left_img_, keypoints, mask);
    int cnt_detected = 0;
    for (auto &kp : keypoints) {
        current_frame_->features_left_.push_back(
            Feature::Ptr(new Feature(current_frame_, kp)));
        cnt_detected++;
    }

//     cv::Mat canvas = current_frame_->left_img_.clone();
//     int radius = Config::Get<int>("gftt_radius");
//     for (auto &kp : keypoints) {
//         cv::circle(canvas, (cv::Point)kp.pt, radius, cv::Scalar(255, 0, 0), cv::FILLED);
//     }
//     cv::imshow("GFTT", canvas);
//     cv::waitKey(1);

    LOG(INFO) << "DetectFeatures: Detect " << cnt_detected << " new features";
    return cnt_detected;
}

int Frontend::FindFeaturesInRight() {
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_left, kps_right;
    for (auto &kp : current_frame_->features_left_) {
        kps_left.push_back(kp->position_.pt);
        auto mp = kp->map_point_.lock();
        if (mp) {
            // use projected points as initial guess
            auto px =
                camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
            kps_right.push_back(cv::Point2f(px[0], px[1]));
        } else {
            // use same pixel in left iamge
            kps_right.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        current_frame_->left_img_, current_frame_->right_img_, kps_left,
        kps_right, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_right[i], 7);
            Feature::Ptr feat(new Feature(current_frame_, kp));
            feat->is_on_left_image_ = false;
            current_frame_->features_right_.push_back(feat);
            num_good_pts++;
        } else {
            current_frame_->features_right_.push_back(nullptr);
        }
    }

    LOG(INFO) << "FindFeaturesInRight: Find " << num_good_pts << " in the right image.";
/*
    cv::Mat lf = current_frame_->left_img_;
    cv::Mat rt = current_frame_->right_img_;
    cv::Mat canvas(
        lf.rows,
        lf.cols + rt.cols,
        lf.type()
    );
    lf.copyTo(canvas(cv::Rect(0, 0, lf.cols, lf.rows)));
    rt.copyTo(canvas(cv::Rect(lf.cols, 0, rt.cols, rt.rows)));
    int radius = Config::Get<int>("gftt_radius");
    for (int i = 0; i < current_frame_->features_left_.size(); i++) {
        if (current_frame_->features_right_[i] == nullptr) {
            continue;
        }
        cv::Point ptl = current_frame_->features_left_[i]->position_.pt;
        cv::Point ptr = current_frame_->features_right_[i]->position_.pt;
        ptr.x += lf.cols;
        cv::circle(canvas, ptl, radius, cv::Scalar(255, 0, 0), cv::FILLED);
        cv::circle(canvas, ptr, radius, cv::Scalar(255, 0, 0), cv::FILLED);
//         LOG(INFO) << i << ":" << ptl.x << "," << ptl.y << "|" << ptr.x << "," << ptr.y;
    }
    cv::imshow("Optical Flow", canvas);
    cv::waitKey(1);
*/
    return num_good_pts;
}

bool Frontend::BuildInitMap() {
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    bool triangulation_log = Config::Get<int>("triangulation_log") != 0;
    if (triangulation_log) {
        LOG(INFO) << "triangulate:" << poses[0].matrix() << "|" << poses[1].matrix();
    }
    size_t cnt_init_landmarks = 0;
    int not_ok = 0, neg_dist = 0;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        if (current_frame_->features_right_[i] == nullptr) continue;
        // create map point from triangulation
        std::vector<Vec3> points{
            camera_left_->pixel2camera(
                Vec2(current_frame_->features_left_[i]->position_.pt.x,
                     current_frame_->features_left_[i]->position_.pt.y)),
            camera_right_->pixel2camera(
                Vec2(current_frame_->features_right_[i]->position_.pt.x,
                     current_frame_->features_right_[i]->position_.pt.y))};
        Vec3 pworld = Vec3::Zero();

        bool ok = triangulation(poses, points, pworld);
        if (!ok) {
            not_ok++;
        }
        if (pworld[2] <= 0) {
            neg_dist++;
        }
        if (ok && pworld[2] > 0) {
            auto new_map_point = MapPoint::CreateNewMappoint();
            new_map_point->SetPos(pworld);
            new_map_point->AddObservation(current_frame_->features_left_[i]);
            new_map_point->AddObservation(current_frame_->features_right_[i]);
            current_frame_->features_left_[i]->map_point_ = new_map_point;
            current_frame_->features_right_[i]->map_point_ = new_map_point;
            cnt_init_landmarks++;
            map_->InsertMapPoint(new_map_point);
        }
        if (triangulation_log) {
            LOG(INFO) << "triangulate:" << points[0].transpose() << "|" << points[1].transpose() << "|" << pworld.transpose() << "|" << ok;
        }
    }
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);
    backend_->UpdateMap();

    LOG(INFO) << "Initial map created with " << cnt_init_landmarks
              << " map points";
    LOG(INFO) << "notok: " << not_ok << "|neg_dist" << neg_dist;

    return true;
}

bool Frontend::Reset() {
    // LOG(INFO) << "Reset is not implemented. ";
    if (last_frame_) {
        current_frame_->SetPose(last_frame_->Pose());
    }
    std::stringstream ss;
    ss << "features_cnts: ";
    for (auto ls : features_cnts) {
        for (int val : ls) {
            ss << val << ",";
        }
        ss << '|';
    }
    features_cnts.clear();
    LOG(INFO) << ss.str();
    LOG(INFO) << "Doing Reset";
    if (Config::Get<int>("reset_wait")) {
        while ((cv::waitKey(50) & 0xFF) != ' ') {}
    }
    status_ = FrontendStatus::INITING;
    return true;
}

}  // namespace myslam
