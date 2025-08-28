//
// Created by LEI XU on 4/27/19.
//
#include "Texture.hpp"

Eigen::Vector3f Texture::getColorBilinear(float u, float v)
{
	u = std::clamp(u, 0.0f, 1.0f);
	v = std::clamp(v, 0.0f, 1.0f);
	float u_img = u * width;
	float v_img = (1 - v) * height;
	int u0 = static_cast<int>(floor(u_img));
	int u1 = static_cast<int>(ceil(u_img));
	int v0 = static_cast<int>(floor(v_img));
	int v1 = static_cast<int>(ceil(v_img));

	u0 = std::clamp(u0, 0, width - 1);
	u1 = std::clamp(u1, 0, width - 1);
	v0 = std::clamp(v0, 0, height - 1);
	v1 = std::clamp(v1, 0, height - 1);

	auto color00 = image_data.at<cv::Vec3b>(v0, u0);
	auto color01 = image_data.at<cv::Vec3b>(v1, u0);
	auto color10 = image_data.at<cv::Vec3b>(v0, u1);
	auto color11 = image_data.at<cv::Vec3b>(v1, u1);

	float u_ratio = u_img - u0;
	float v_ratio = v_img - v0;

	auto u_color1 = color00 * (1 - u_ratio) + color10 * u_ratio;
	auto u_color2 = color01 * (1 - u_ratio) + color11 * u_ratio;
	
	auto color = u_color1 * (1 - v_ratio) + u_color2 * v_ratio;
	return Eigen::Vector3f(color[0], color[1], color[2]);
	
}