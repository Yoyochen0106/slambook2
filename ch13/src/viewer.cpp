//
// Created by gaoxiang on 19-5-4.
//
#include "myslam/viewer.h"
#include "myslam/feature.h"
#include "myslam/frame.h"
#include "myslam/config.h"
#include "myslam/util.h"

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

namespace myslam {

Viewer::Viewer() {
    viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
}

void Viewer::Close() {
    viewer_running_ = false;
    viewer_thread_.join();
}

void Viewer::PostImshow(std::string title, cv::Mat image) {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    postImshows_.push_back(std::make_pair(title, image));
}

void Viewer::AddCurrentFrame(Frame::Ptr current_frame) {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    current_frame_ = current_frame;
}

void Viewer::UpdateMap() {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    assert(map_ != nullptr);
    active_keyframes_ = map_->GetActiveKeyFrames();
    active_landmarks_ = map_->GetActiveMapPoints();
    map_updated_ = true;
}

static void LoadDataPlotConfig(DataPlot &plot, const cv::FileNode &cfg) {
    if (cfg.type() == cv::FileNode::Type::NONE) {
        LOG(ERROR) << "Empty config FileNode";
    }
    if (!cfg["inherit"].isNone()) LoadDataPlotConfig(plot, Config::Get<cv::FileNode>(static_cast<std::string>(cfg["inherit"])));
    if (!cfg["size"].isNone()) plot.size_ = cv::Size(int(cfg["size"][0]), int(cfg["size"][1]));
    if (!cfg["maxmin"].isNone()) plot.value_max_ = double(cfg["maxmin"][0]);
    if (!cfg["maxmin"].isNone()) plot.value_min_ = double(cfg["maxmin"][1]);
    if (!cfg["item_width"].isNone()) plot.item_w_ = int(cfg["item_width"]);
}

void Viewer::ThreadLoop() {
    pangolin::CreateWindowAndBind("MySLAM", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState vis_camera(
        pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& vis_display =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(vis_camera));

    const float blue[3] = {0, 0, 1};
    const float green[3] = {0, 1, 0};

    int waitKey_time = Config::Get<int>("waitKey_time");
    int sleep_time = Config::Get<int>("sleep_time");
    
    DataPlot plot0;
    DataPlot plot1;
    LoadDataPlotConfig(plot0, Config::Get<cv::FileNode>("plot0"));
    LoadDataPlotConfig(plot1, Config::Get<cv::FileNode>("plot1"));
    plot0.Setup();
    plot1.Setup();

    LOG(INFO) << "waitKey_time=" << waitKey_time << "|sleep_time=" << sleep_time;
    double v = 0;

    while (!pangolin::ShouldQuit() && viewer_running_) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        vis_display.Activate(vis_camera);

        std::unique_lock<std::mutex> lock(viewer_data_mutex_);
        if (current_frame_) {
            DrawFrame(current_frame_, green);
            FollowCurrentFrame(vis_camera);

            cv::Mat img = PlotFrameImage();
            auto &ls = current_frame_->features_left_;
            int cnt;
            cnt = std::count_if(begin(ls), end(ls), [](Feature::Ptr ft) { return !ft->is_outlier_; });
            plot0.Push(cnt);
            plot0.Draw(img, cv::Point(0, 0));

            cv::imshow("image", img);

            std::vector<PostImshowItemType> imshows;
            imshows.swap(postImshows_);
            for (PostImshowItemType item : imshows) {
                cv::imshow(item.first, item.second);
            }

            cv::waitKey(waitKey_time);
        }

        if (map_) {
            DrawMapPoints();
        }

        pangolin::FinishFrame();
        usleep(1000 * sleep_time);
    }

    LOG(INFO) << "Stop viewer";
}

cv::Mat Viewer::PlotFrameImage() {
    cv::Mat img_out;
    cv::cvtColor(current_frame_->left_img_, img_out, CV_GRAY2BGR);
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        auto feat = current_frame_->features_left_[i];
        if (feat->map_point_.lock()) {
            cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(0, 250, 0), 2);
        } else {
            cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(0, 0, 192), 2);
        }
    }

    if (Config::Get<int>("show_text")) {

        double feature_track_font = Config::Get<double>("feature_track_font");
        int y = img_out.size[0];

        std::stringstream ss;

        ss.str(std::string());
        ss << current_frame_->features_left_.size();
        cv::putText(img_out, ss.str(), cv::Point(0, y), cv::FONT_HERSHEY_PLAIN, feature_track_font, cv::Scalar(255, 0, 0));
        y -= Config::Get<int>("text_gap");

        ss.str(std::string());
        ss << map_->GetAllMapPoints().size();
        cv::putText(img_out, ss.str(), cv::Point(0, y), cv::FONT_HERSHEY_PLAIN, feature_track_font, cv::Scalar(255, 0, 0));
        y -= Config::Get<int>("text_gap");

        ss.str(std::string());
        ss << map_->GetActiveMapPoints().size();
        cv::putText(img_out, ss.str(), cv::Point(0, y), cv::FONT_HERSHEY_PLAIN, feature_track_font, cv::Scalar(255, 0, 0));
        y -= Config::Get<int>("text_gap");

    }

    return img_out;
}

void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera) {
    SE3 Twc = current_frame_->Pose().inverse();
    pangolin::OpenGlMatrix m(Twc.matrix());
    vis_camera.Follow(m, true);
}

void Viewer::DrawFrame(Frame::Ptr frame, const float* color) {
    SE3 Twc = frame->Pose().inverse();
    const float sz = 1.0;
    const int line_width = 2.0;
    const float fx = 400;
    const float fy = 400;
    const float cx = 512;
    const float cy = 384;
    const float width = 1080;
    const float height = 768;

    glPushMatrix();

    Sophus::Matrix4f m = Twc.matrix().template cast<float>();
    glMultMatrixf((GLfloat*)m.data());

    if (color == nullptr) {
        glColor3f(1, 0, 0);
    } else
        glColor3f(color[0], color[1], color[2]);

    glLineWidth(line_width);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glEnd();
    glPopMatrix();
}

void Viewer::DrawMapPoints() {
    const float red[3] = {1.0, 0, 0};
    for (auto& kf : active_keyframes_) {
        DrawFrame(kf.second, red);
    }

    glPointSize(2);
    glBegin(GL_POINTS);
    for (auto& landmark : active_landmarks_) {
        auto pos = landmark.second->Pos();
        glColor3f(red[0], red[1], red[2]);
        glVertex3d(pos[0], pos[1], pos[2]);
    }
    glEnd();
}

}  // namespace myslam
