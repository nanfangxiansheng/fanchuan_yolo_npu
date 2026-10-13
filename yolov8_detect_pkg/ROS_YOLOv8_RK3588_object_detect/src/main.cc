#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "yolov8.h"
#include "image_utils.h"
#include "file_utils.h"
#include "image_drawing.h"
#include <string> 
#include <chrono>  
#include <vector> 
#include <iostream>
#include <dirent.h> // For POSIX directory functions  
#include <sys/types.h>  
#include <unistd.h>  
#include <cstring> // For strcmp  
#include <opencv2/opencv.hpp>  


// stb_image 已经在 image_utils.c 中定义，这里直接声明外部函数
extern "C" {
unsigned char* stbi_load(char const *filename, int *x, int *y, int *channels_in_file, int desired_channels);
void stbi_image_free(void *retval_from_stbi_load);
const char* stbi_failure_reason(void);
}

int read_image_opencv(const char* path, image_buffer_t* image) 
{
    // 使用 stb_image 读取图像（比OpenCV更稳定）
    int width, height, channels;
    unsigned char* img_data = stbi_load(path, &width, &height, &channels, 3);  // 强制3通道RGB
    if (img_data == NULL) {
        printf("stbi_load failed: %s, reason: %s\n", path, stbi_failure_reason());
        return -1;
    }

    printf("Image loaded: %s, size=%dx%d, channels=%d\n", path, width, height, channels);

    // RGA要求宽度16字节对齐，检查并处理
    int aligned_width = (width + 15) / 16 * 16;  // 向上对齐到16的倍数
    int original_width = width;  // 保存原始宽度用于后续处理
    bool need_free = false;  // 标记是否需要用free()而非stbi_image_free()
    
    if (aligned_width != width) {
        printf("Width %d not 16-aligned, padding to %d for RGA\n", width, aligned_width);
        
        // 分配对齐后的内存
        size_t aligned_size = aligned_width * height * 3;
        unsigned char* aligned_data = (unsigned char*)malloc(aligned_size);
        if (aligned_data == NULL) {
            printf("malloc failed for aligned buffer\n");
            stbi_image_free(img_data);
            return -1;
        }
        
        // 填充灰色(114)
        memset(aligned_data, 114, aligned_size);
        
        // 复制原始数据到对齐缓冲区
        for (int y = 0; y < height; y++) {
            memcpy(aligned_data + y * aligned_width * 3, 
                   img_data + y * width * 3, 
                   width * 3);
        }
        
        // 释放原始数据，使用对齐后的数据
        stbi_image_free(img_data);
        img_data = aligned_data;
        width = aligned_width;
        need_free = true;
    }

    // 设置图像参数
    image->format = IMAGE_FORMAT_RGB888;
    image->width = original_width;  // 保持原始宽度用于检测框坐标计算
    image->height = height;
    image->width_stride = aligned_width;  // 对齐后的宽度stride
    image->height_stride = height;
    image->fd = 0;
    
    // 设置图像数据
    size_t size = width * height * 3;
    image->size = size;
    image->virt_addr = img_data;
    
    return 0;
}

int write_image(const char* path, const image_buffer_t* img) 
{
    int width = img->width;
    int height = img->height;
    int width_stride = img->width_stride > 0 ? img->width_stride : width;
    int channels = (img->format == IMAGE_FORMAT_RGB888) ? 3 : 
                   (img->format == IMAGE_FORMAT_GRAY8) ? 1 : 4;
    void* data = img->virt_addr;

    // 如果有padding（width_stride > width），需要先复制到连续内存
    cv::Mat bgr_img;
    if (width_stride > width) {
        // 创建连续的目标图像
        cv::Mat continuous_img(height, width, CV_8UC(channels));
        
        // 逐行复制，跳过padding
        for (int y = 0; y < height; y++) {
            memcpy(continuous_img.data + y * width * channels,
                   (unsigned char*)data + y * width_stride * channels,
                   width * channels);
        }
        
        // RGB转BGR
        if (channels == 3 && img->format == IMAGE_FORMAT_RGB888) {
            cv::cvtColor(continuous_img, bgr_img, cv::COLOR_RGB2BGR);
        } else {
            bgr_img = continuous_img;
        }
    } else {
        // 没有padding，直接使用
        cv::Mat cv_img(height, width, CV_8UC(channels), data);
        
        if (channels == 3 && img->format == IMAGE_FORMAT_RGB888) {
            cv::cvtColor(cv_img, bgr_img, cv::COLOR_RGB2BGR);
        } else {
            bgr_img = cv_img;
        }
    }

    bool success = cv::imwrite(path, bgr_img);
    return success ? 0 : -1;
}

//去除文件地址&后缀
std::string extractFileNameWithoutExtension(const std::string& path) 
{  
    auto pos = path.find_last_of("/\\");  
    std::string filename = (pos == std::string::npos) ? path : path.substr(pos + 1);  
      
    // 查找并去除文件后缀  
    pos = filename.find_last_of(".");  
    if (pos != std::string::npos) {  
        filename = filename.substr(0, pos);  
    }  
      
    return filename;  
}

// 处理一个文件夹中的所有图像文件  
void processImagesInFolder(const std::string& folderPath, rknn_app_context_t* rknn_app_ctx, const std::string& outputFolderPath) 
{  
    DIR *dir = opendir(folderPath.c_str());  
    if (dir == nullptr) {  
        perror("opendir");  
        return;  
    }  
  
    struct dirent *entry;  
    while ((entry = readdir(dir)) != nullptr) 
    {  
        std::string fileName = entry->d_name;  
        std::string fullPath = folderPath + "/" + fileName;  
         // 检查文件扩展名（支持大小写）
        std::string lowerFileName = fileName;
        for (size_t i = 0; i < lowerFileName.size(); i++) {
            lowerFileName[i] = tolower(lowerFileName[i]);
        }
        if ((lowerFileName.size() >= 4 && lowerFileName.substr(lowerFileName.size() - 4) == ".jpg") ||  
            (lowerFileName.size() >= 5 && lowerFileName.substr(lowerFileName.size() - 5) == ".jpeg") ||  
            (lowerFileName.size() >= 4 && lowerFileName.substr(lowerFileName.size() - 4) == ".png")) {  
  
            std::string outputFileName = outputFolderPath + "/" + extractFileNameWithoutExtension(fullPath) + "_out.png";  
  
            int ret;  
            image_buffer_t src_image;  
            memset(&src_image, 0, sizeof(image_buffer_t));  
 
            ret = read_image_opencv(fullPath.c_str(), &src_image); 

  
            if (ret != 0) {  
                printf("read image fail! ret=%d image_path=%s\n", ret, fullPath.c_str());  
                continue;  
            }  
  
            object_detect_result_list od_results;  
            
            ret = inference_yolov8_model(rknn_app_ctx, &src_image, &od_results);  
            if (ret != 0) {  
                printf("inference_yolov8_model fail! ret=%d\n", ret);  
                if (src_image.virt_addr != NULL) {  
                    stbi_image_free(src_image.virt_addr);  
                }  
                continue;  
            } 

            // 画框和概率  
            char text[256];  
            for (int i = 0; i < od_results.count; i++) 
            {  
                object_detect_result *det_result = &(od_results.results[i]);  
                printf("%s @ (%d %d %d %d) %.3f\n", coco_cls_to_name(det_result->cls_id),  
                       det_result->box.left, det_result->box.top,  
                       det_result->box.right, det_result->box.bottom,  
                       det_result->prop);  
                int x1 = det_result->box.left;  
                int y1 = det_result->box.top;  
                int x2 = det_result->box.right;  
                int y2 = det_result->box.bottom;  

                draw_rectangle(&src_image, x1, y1, x2 - x1, y2 - y1, COLOR_BLUE, 3);  
                sprintf(text, "%s %.1f%%", coco_cls_to_name(det_result->cls_id), det_result->prop * 100);  
                draw_text(&src_image, text, x1, y1 - 20, COLOR_RED, 10);  
            }  

            write_image(outputFileName.c_str(), &src_image);  

            if (src_image.virt_addr != NULL) 
            {  
                stbi_image_free(src_image.virt_addr);  
            }  
        }  
    }  
  
    closedir(dir);  
}   
  
int main(int argc, char **argv)  
{   
    // ***** 以下三个参数：rknn模型路径、输入图片所在文件夹路径、输出结果图片所在文件夹路径（均为绝对路径）
    const std::string modelPath = "/home/orangepi/rknn-210/YOLOv8_RK3588_object_detect/model/yolov8relupt.rknn";  
    const std::string imageFolder = "/home/orangepi/rknn-210/YOLOv8_RK3588_object_detect/inputimage";  
    const std::string outputFolder = "/home/orangepi/rknn-210/YOLOv8_RK3588_object_detect/outputimage"; 
    
    int ret;  
    rknn_app_context_t rknn_app_ctx;  
    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t)); 

    init_post_process(); 

    ret = init_yolov8_model(modelPath.c_str(), &rknn_app_ctx);  
    if (ret != 0) 
    {  
        printf("init_yolov8_model fail! ret=%d model_path=%s\n", ret, modelPath.c_str());  
        return -1;  
    }      

    processImagesInFolder(imageFolder, &rknn_app_ctx, outputFolder); 

    ret = release_yolov8_model(&rknn_app_ctx); 

    if (ret != 0) 
    {  
        printf("release_yolov8_model fail! ret=%d\n", ret);  
    }  

    deinit_post_process();  

    return 0;  
}

