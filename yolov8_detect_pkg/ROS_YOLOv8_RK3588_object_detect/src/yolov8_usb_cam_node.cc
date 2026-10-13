#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <yolov8_ros_msgs/BoundingBoxes.h>
#include <yolov8_ros_msgs/BoundingBox.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <sys/types.h>

#include "yolov8.h"
#include "image_drawing.h"
#include "common.h"

// 与 package.xml 中 <name> 一致，供 ros::package::getPath 查找 share 路径
static constexpr char kCatkinPackageName[] = "ros_yolov8_rk3588_object_detect";

static int bgr8_to_rgb_image_buffer(const cv::Mat& bgr, image_buffer_t* out,
                                    std::vector<unsigned char>* storage) {
  if (bgr.empty() || bgr.channels() != 3 || bgr.type() != CV_8UC3)
    return -1;
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  int w = rgb.cols;
  int h = rgb.rows;
  int aligned_w = (w + 15) / 16 * 16;
  size_t nbytes = static_cast<size_t>(aligned_w) * static_cast<size_t>(h) * 3u;
  storage->resize(nbytes);
  unsigned char* dst = storage->data();
  memset(dst, 114, nbytes);
  for (int y = 0; y < h; y++) {
    memcpy(dst + static_cast<size_t>(y) * static_cast<size_t>(aligned_w) * 3u, rgb.ptr(y),
           static_cast<size_t>(w) * 3u);
  }
  memset(out, 0, sizeof(*out));
  out->format = IMAGE_FORMAT_RGB888;
  out->width = w;
  out->height = h;
  out->width_stride = aligned_w;
  out->height_stride = h;
  out->virt_addr = dst;
  out->size = static_cast<int>(nbytes);
  out->fd = 0;
  return 0;
}

static sensor_msgs::ImagePtr rgb_buffer_to_bgr_image_msg(const image_buffer_t* img,
                                                           const std_msgs::Header& header) {
  int w = img->width;
  int h = img->height;
  int ws = img->width_stride > 0 ? img->width_stride : w;
  cv::Mat continuous(h, w, CV_8UC3);
  for (int y = 0; y < h; y++) {
    memcpy(continuous.ptr(y),
           static_cast<const unsigned char*>(img->virt_addr) +
               static_cast<size_t>(y) * static_cast<size_t>(ws) * 3u,
           static_cast<size_t>(w) * 3u);
  }
  cv::Mat bgr;
  cv::cvtColor(continuous, bgr, cv::COLOR_RGB2BGR);
  cv_bridge::CvImage cv_img(header, sensor_msgs::image_encodings::BGR8, bgr);
  return cv_img.toImageMsg();
}

static bool mkdir_p(const std::string& path) {
  if (path.empty())
    return false;
  std::string p = path;
  while (!p.empty() && p.back() == '/')
    p.pop_back();
  if (p.empty())
    return false;
  std::string acc;
  acc.reserve(p.size() + 1);
  for (size_t i = 0; i < p.size(); ++i) {
    acc.push_back(p[i]);
    if (p[i] == '/' && acc.size() > 1) {
      if (mkdir(acc.c_str(), 0755) != 0 && errno != EEXIST)
        return false;
    }
  }
  if (mkdir(acc.c_str(), 0755) != 0 && errno != EEXIST)
    return false;
  return true;
}

class Yolov8UsbCamNode {
 public:
  Yolov8UsbCamNode()
      : nh_("~"),
        app_inited_(false),
        ema_fps_(0.0),
        fps_ema_alpha_(0.15),
        last_frame_wall_valid_(false),
        report_fps_(true),
        overlay_fps_(true),
        publish_fps_(true),
        save_detected_enable_(false),
        save_detected_interval_sec_(1.0),
        last_detect_save_valid_(false) {
    std::string pkg = ros::package::getPath(kCatkinPackageName);
    if (pkg.empty()) {
      ROS_FATAL("ros::package::getPath(%s) failed; check ROS_PACKAGE_PATH", kCatkinPackageName);
      throw std::runtime_error("getPath");
    }
    std::string def_model = pkg + "/model/yolov8relupt.rknn";
    nh_.param<std::string>("model_path", model_path_, def_model);
    nh_.param<std::string>("input_topic", input_topic_, std::string("/usb_cam/image_raw"));
    nh_.param<std::string>("output_topic", output_topic_, std::string("image_detect"));
    nh_.param<std::string>("target_class", target_class_, std::string(""));
    nh_.param("conf_threshold", conf_threshold_, 0.7f);
    nh_.param("fps_ema_alpha", fps_ema_alpha_, 0.15);
    nh_.param("report_fps", report_fps_, true);
    nh_.param("overlay_fps", overlay_fps_, true);
    nh_.param("publish_fps", publish_fps_, true);
    nh_.param("save_detected_enable", save_detected_enable_, false);
    nh_.param<std::string>("save_detected_path", save_detected_path_,
                           std::string("/home/orangepi/detected-results"));
    nh_.param("save_detected_interval_sec", save_detected_interval_sec_, 1.0);
    if (save_detected_interval_sec_ < 0.05)
      save_detected_interval_sec_ = 0.05;

    if (init_post_process() != 0) {
      ROS_FATAL("init_post_process failed (label file missing?)");
      throw std::runtime_error("init_post_process");
    }
    memset(&app_ctx_, 0, sizeof(app_ctx_));
    int ret = init_yolov8_model(model_path_.c_str(), &app_ctx_);
    if (ret != 0) {
      deinit_post_process();
      ROS_FATAL("init_yolov8_model failed ret=%d path=%s", ret, model_path_.c_str());
      throw std::runtime_error("init_yolov8_model");
    }
    app_inited_ = true;
    ROS_INFO("YOLOv8 RKNN: model=%s in=%s out=%s", model_path_.c_str(), input_topic_.c_str(),
             output_topic_.c_str());
    if (save_detected_enable_) {
      ROS_INFO("save_detected: path=%s interval=%.2fs", save_detected_path_.c_str(),
               save_detected_interval_sec_);
    }

    pub_ = nh_.advertise<sensor_msgs::Image>(output_topic_, 1);
    det_pub_ = nh_.advertise<std_msgs::String>("detections", 10);
    det_count_pub_ = nh_.advertise<std_msgs::UInt32>("detection_count", 10);
    if (publish_fps_) {
      fps_pub_ = nh_.advertise<std_msgs::Float32>("fps", 10);
    }
    bbox_pub_ = nh_.advertise<yolov8_ros_msgs::BoundingBoxes>("BoundingBoxes", 10);
    sub_ = nh_.subscribe(input_topic_, 1, &Yolov8UsbCamNode::onImage, this);
  }

  ~Yolov8UsbCamNode() {
    if (app_inited_) {
      release_yolov8_model(&app_ctx_);
    }
    deinit_post_process();
  }

 private:
  void onImage(const sensor_msgs::ImageConstPtr& msg) {
    const ros::WallTime t_frame_start = ros::WallTime::now();

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& e) {
      ROS_WARN_THROTTLE(2.0, "cv_bridge: %s", e.what());
      return;
    }

    image_buffer_t src_image;
    memset(&src_image, 0, sizeof(src_image));
    std::vector<unsigned char> rgb_storage;
    if (bgr8_to_rgb_image_buffer(cv_ptr->image, &src_image, &rgb_storage) != 0) {
      ROS_WARN_THROTTLE(2.0, "invalid BGR8 image");
      return;
    }

    object_detect_result_list od_results;
    memset(&od_results, 0, sizeof(od_results));
    const ros::WallTime t_infer0 = ros::WallTime::now();
    int ret = inference_yolov8_model(&app_ctx_, &src_image, &od_results);
    const double ms_infer_only = (ros::WallTime::now() - t_infer0).toSec() * 1000.0;
    if (ret != 0) {
      ROS_WARN_THROTTLE(1.0, "inference_yolov8_model ret=%d", ret);
      return;
    }

    {
      std_msgs::UInt32 dc;
      dc.data = static_cast<uint32_t>(od_results.count);
      det_count_pub_.publish(dc);
    }

    // 发布 BoundingBoxes（供 yolo_follow 等节点使用）
    {
      yolov8_ros_msgs::BoundingBoxes bbox_msg;
      bbox_msg.header = msg->header;
      bbox_msg.image_header = msg->header;
      for (int i = 0; i < od_results.count; i++) {
        object_detect_result* det = &od_results.results[i];
        std::string cls_name = coco_cls_to_name(det->cls_id);
        // 置信度过滤
        if (det->prop < conf_threshold_)
          continue;
        // target_class_ 为空时跟踪所有类别，否则只跟踪指定类别
        if (!target_class_.empty() && cls_name != target_class_)
          continue;
        yolov8_ros_msgs::BoundingBox box;
        box.probability = det->prop;
        box.xmin = det->box.left;
        box.ymin = det->box.top;
        box.xmax = det->box.right;
        box.ymax = det->box.bottom;
        box.Class = cls_name;
        bbox_msg.bounding_boxes.push_back(box);
      }
      bbox_msg.len = bbox_msg.bounding_boxes.size();
      bbox_pub_.publish(bbox_msg);
    }

    char text[256];
    std::string det_payload;
    for (int i = 0; i < od_results.count; i++) {
      object_detect_result* det = &od_results.results[i];
      int x1 = det->box.left;
      int y1 = det->box.top;
      int x2 = det->box.right;
      int y2 = det->box.bottom;
      draw_rectangle(&src_image, x1, y1, x2 - x1, y2 - y1, COLOR_BLUE, 3);
      snprintf(text, sizeof(text), "%s %.1f%%", coco_cls_to_name(det->cls_id), det->prop * 100.f);
      draw_text(&src_image, text, x1, y1 - 20, COLOR_RED, 10);
      if (!det_payload.empty())
        det_payload.push_back('|');
      det_payload += coco_cls_to_name(det->cls_id);
      det_payload.push_back(':');
      det_payload += std::to_string(static_cast<double>(det->prop));
    }
    if (od_results.count > 0) {
      std_msgs::String det_msg;
      det_msg.data = det_payload;
      det_pub_.publish(det_msg);
    }

    if (last_frame_wall_valid_) {
      const double dt = (t_frame_start - last_frame_wall_).toSec();
      if (dt > 1e-6) {
        const double inst_fps = 1.0 / dt;
        if (ema_fps_ <= 1e-6) {
          ema_fps_ = inst_fps;
        } else {
          ema_fps_ = fps_ema_alpha_ * inst_fps + (1.0 - fps_ema_alpha_) * ema_fps_;
        }
      }
    }
    last_frame_wall_ = t_frame_start;
    last_frame_wall_valid_ = true;

    if (overlay_fps_ && ema_fps_ > 0.5) {
      char fps_text[64];
      snprintf(fps_text, sizeof(fps_text), "FPS %.1f  infer %.0f ms", ema_fps_, ms_infer_only);
      draw_text(&src_image, fps_text, 8, 24, COLOR_YELLOW, 12);
    }

    if (report_fps_ && ema_fps_ > 0.5) {
      const double ms_total = (ros::WallTime::now() - t_frame_start).toSec() * 1000.0;
      ROS_INFO_THROTTLE(1.0,
                        "FPS (EMA) %.1f | infer %.0f ms | frame total %.0f ms",
                        ema_fps_, ms_infer_only, ms_total);
    }
    if (publish_fps_) {
      std_msgs::Float32 f;
      f.data = static_cast<float>(ema_fps_);
      fps_pub_.publish(f);
    }

    std_msgs::Header hdr = msg->header;
    sensor_msgs::ImagePtr out = rgb_buffer_to_bgr_image_msg(&src_image, hdr);
    pub_.publish(out);

    if (save_detected_enable_ && od_results.count > 0) {
      const ros::WallTime now_wall = ros::WallTime::now();
      if (!last_detect_save_valid_ ||
          (now_wall - last_detect_save_wall_).toSec() >= save_detected_interval_sec_) {
        last_detect_save_wall_ = now_wall;
        last_detect_save_valid_ = true;
        
        std::string timestamp_str = std::to_string(now_wall.toNSec());
        
        // 保存识别图（带框）到 detect 子目录
        std::string detect_path = save_detected_path_ + "/detect";
        if (mkdir_p(detect_path)) {
          std::string detect_fname = detect_path + "/detect_" + timestamp_str + ".jpg";
          try {
            cv_bridge::CvImageConstPtr cv_out =
                cv_bridge::toCvCopy(out, sensor_msgs::image_encodings::BGR8);
            if (!cv::imwrite(detect_fname, cv_out->image)) {
              ROS_WARN_THROTTLE(5.0, "save_detected: imwrite failed: %s", detect_fname.c_str());
            } else {
              ROS_INFO_THROTTLE(1.0, "save_detected: wrote detect/%s (objects=%d)", 
                                ("detect_" + timestamp_str + ".jpg").c_str(), od_results.count);
            }
          } catch (const cv_bridge::Exception& e) {
            ROS_WARN_THROTTLE(2.0, "save_detected cv_bridge: %s", e.what());
          }
        } else {
          ROS_WARN_THROTTLE(5.0, "save_detected: mkdir failed: %s", detect_path.c_str());
        }
        
        // 保存原图（不带框）到 origin 子目录
        std::string origin_path = save_detected_path_ + "/origin";
        if (mkdir_p(origin_path)) {
          std::string origin_fname = origin_path + "/origin_" + timestamp_str + ".jpg";
          try {
            if (!cv::imwrite(origin_fname, cv_ptr->image)) {
              ROS_WARN_THROTTLE(5.0, "save_origin: imwrite failed: %s", origin_fname.c_str());
            } else {
              ROS_INFO_THROTTLE(1.0, "save_origin: wrote origin/%s", 
                                ("origin_" + timestamp_str + ".jpg").c_str());
            }
          } catch (const cv_bridge::Exception& e) {
            ROS_WARN_THROTTLE(2.0, "save_origin cv_bridge: %s", e.what());
          }
        } else {
          ROS_WARN_THROTTLE(5.0, "save_origin: mkdir failed: %s", origin_path.c_str());
        }
      }
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher det_pub_;
  ros::Publisher det_count_pub_;
  ros::Publisher fps_pub_;
  ros::Publisher bbox_pub_;
  rknn_app_context_t app_ctx_;
  bool app_inited_;
  std::string model_path_;
  std::string input_topic_;
  std::string output_topic_;
  std::string target_class_;
  float conf_threshold_;
  double ema_fps_;
  double fps_ema_alpha_;
  ros::WallTime last_frame_wall_;
  bool last_frame_wall_valid_;
  bool report_fps_;
  bool overlay_fps_;
  bool publish_fps_;
  bool save_detected_enable_;
  std::string save_detected_path_;
  double save_detected_interval_sec_;
  ros::WallTime last_detect_save_wall_;
  bool last_detect_save_valid_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "yolov8_usb_cam_node");
  if (!ros::ok())
    return 1;
  try {
    Yolov8UsbCamNode node;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("%s", e.what());
    return 1;
  }
  return 0;
}
