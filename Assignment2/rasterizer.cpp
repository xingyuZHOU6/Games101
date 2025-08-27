// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
     // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f A, B, C, P;
    A = _v[0];
    B = _v[1];
    C = _v[2];
    P = { x, y, 0 };    // 待测点，z值不影响二维叉积计算

    Vector3f AB, BC, CA, AP, BP, CP;
    AB = B - A; BC = C - B; CA = A - C;
    AP = P - A; BP = P - B; CP = P - C;

    // 计算二维叉积（只计算z分量）
    float cross1 = AB.x() * AP.y() - AB.y() * AP.x();
    float cross2 = BC.x() * BP.y() - BC.y() * BP.x();
    float cross3 = CA.x() * CP.y() - CA.y() * CP.x();

    //注：修改为float型变量后，由于浮点精度误差理论上不应该与0进行比较
    //同时未考虑等于0，即点位于边上的情况
    bool sign = (cross1 > 0 && cross2 > 0 && cross3 > 0) ||
                (cross1 < 0 && cross2 < 0 && cross3 < 0);

    return sign;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
        
    }
    downsampling();
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    //find out the bounding box 计算包围盒
    float min_x = FLT_MAX;
    float max_x = FLT_MIN;
    float min_y = FLT_MAX;
    float max_y = FLT_MIN;

    for (const auto& point : v) {
        min_x = std::min(min_x, point.x());
        max_x = std::max(max_x, point.x());
        min_y = std::min(min_y, point.y());
        max_y = std::max(max_y, point.y());
    }
    /*
   //iterate through the pixel 遍历包围盒内像素
    for (int y = floor(min_y); y <= ceil(max_y); ++y) {
        for (int x = floor(min_x); x <= ceil(max_x); ++x) {
            // find if the current pixel is inside the triangle 判断像素是否位于三角形内部
            float pixel_x = x + 0.5f;
            float pixel_y = y + 0.5f;
            if (insideTriangle(pixel_x, pixel_y, t.v)) {
                //如果下一句中元组报错，请选择更新的C++标准（C++17及以上）
                auto [alpha, beta, gamma] = computeBarycentric2D(pixel_x, pixel_y, t.v);
                float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated = z_interpolated*w_reciprocal;
                int index = get_index(x, y);
                if (z_interpolated < depth_buf[index]) {
                    Vector3f point = Vector3f(x, y, z_interpolated);
                    set_pixel(point, t.getColor());
                    depth_buf[index] = z_interpolated;
                }
            }
        }
    }*/
    //ssaa
    float min_x_ssaa = min_x*ssaa_factor;
    float max_x_ssaa = max_x*ssaa_factor;
    float min_y_ssaa = min_y*ssaa_factor;
    float max_y_ssaa = max_y*ssaa_factor;
    //clamp
    int ss_xmin = std::max(0,(int)floor(min_x_ssaa));
    int ss_xmax = std::min(int(width*ssaa_factor),(int)ceil(max_x_ssaa));
    int ss_ymin = std::max(0,(int)floor(min_y_ssaa));
    int ss_ymax = std::min(int(height*ssaa_factor),(int)ceil(max_y_ssaa));

    for(int y = ss_ymin;y<ss_ymax;y++)
    {
        for(int x = ss_xmin;x<ss_xmax;x++)
        {
            float pixel_x = (x+0.5f)/ssaa_factor;
            float pixel_y = (y+0.5f)/ssaa_factor;

            //if insert triangle
            if(insideTriangle(pixel_x,pixel_y,t.v))
            {
            //barycentic
            auto [alpha,beta,gamma] = computeBarycentric2D(pixel_x,pixel_y,t.v);
            float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            z_interpolated = z_interpolated*w_reciprocal;
            int ss_index = get_index_ssaa(x, y);
            if (z_interpolated < depth_buf_ssaa[ss_index]) {
                    frame_buf_ssaa[ss_index] = t.getColor();
                    depth_buf_ssaa[ss_index] = z_interpolated;
                }
            }
             
            

        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        //clear frame_buf
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(frame_buf_ssaa.begin(),frame_buf_ssaa.end(),Eigen::Vector3f{0,0,0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_buf_ssaa.begin(),depth_buf_ssaa.end(),std::numeric_limits<float>::max());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    //init frame_buf
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    frame_buf_ssaa.resize(w*ssaa_factor*h*ssaa_factor);
    depth_buf_ssaa.resize(w*ssaa_factor*h*ssaa_factor);
     // 初始化超采样缓冲区
    std::fill(frame_buf_ssaa.begin(), frame_buf_ssaa.end(), Eigen::Vector3f{0, 0, 0});
    std::fill(depth_buf_ssaa.begin(), depth_buf_ssaa.end(), std::numeric_limits<float>::max());
}

int rst::rasterizer::get_index(int x, int y)
{
    //inverse y
    return (height-1-y)*width + x;
}
//ssaa index
int rst::rasterizer::get_index_ssaa(int x,int y)
{
    return (height*ssaa_factor-1-y)*width*ssaa_factor+x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}
//sampling
void rst::rasterizer::downsampling()
{
    if(ssaa_factor==1)
    {
        std::copy(frame_buf_ssaa.begin(),frame_buf_ssaa.end(),frame_buf.begin());
        return;
    }

    for(int y = 0;y<height;y++)
    {
        for(int x =0;x<width;x++)
        {
            Eigen::Vector3f color_avg(0,0,0);

            for(int dy=0;dy<ssaa_factor;dy++)
            {
                for(int dx=0;dx<ssaa_factor;dx++)
                {
                    int ss_x =x*ssaa_factor+dx;
                    int ss_y = y*ssaa_factor+dy;
                    int ss_index = get_index_ssaa(ss_x,ss_y);
                    color_avg+=frame_buf_ssaa[ss_index];
                }
                

            }
                color_avg = color_avg/(ssaa_factor*ssaa_factor);

                int index = get_index(x,y);
                frame_buf[index] = color_avg;
            }
        }
}

// clang-format on
