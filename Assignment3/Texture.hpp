//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
     
        u = std::clamp(u, 0.0f, 1.0f);
        v = std::clamp(v, 0.0f, 1.0f);
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        int x = static_cast<int>(u_img);
        int y = static_cast<int>(v_img);

        x = std::clamp(x, 0, width - 1);
        y = std::clamp(y, 0, height - 1);
        auto color = image_data.at<cv::Vec3b>(y, x);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    Eigen::Vector3f getColorBilinear(float u, float v);

};
#endif //RASTERIZER_TEXTURE_H
