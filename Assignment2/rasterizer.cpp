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
    
    float crossP1 = (_v[1].x() - _v[0].x()) * (y - _v[0].y()) - (x - _v[0].x()) * (_v[1].y() - _v[0].y()); 
    float crossP2 = (_v[2].x() - _v[1].x()) * (y - _v[1].y()) - (x - _v[1].x()) * (_v[2].y() - _v[1].y()); 
    float crossP3 = (_v[0].x() - _v[2].x()) * (y - _v[2].y()) - (x - _v[2].x()) * (_v[0].y() - _v[2].y()); 
    
    if ((crossP1 * crossP2) > 0 && (crossP1 * crossP3) > 0) { // 如果三个叉乘结果同号，说明点在三角形内部
        return true;
    } else {
        return false;
    }
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
}

// // 插值计算
// static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
// {
//     return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
// }

// static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f& vert1, const Eigen::Vector2f& vert2, const Eigen::Vector2f& vert3, float weight)
// {
//     auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
//     auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

//     u /= weight;
//     v /= weight;

//     return Eigen::Vector2f(u, v);
// }

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    // 这里参考了https://zhuanlan.zhihu.com/p/415328934
    auto v = t.toVector4(); // 这里的理解是将三角形的三个顶点坐标齐次化表示
    int sampling_frequency, sampling_times;
    sampling_frequency = 4;
    sampling_times = sampling_frequency * sampling_frequency;
    float sampling_period = 1.0f / sampling_frequency;   
    int min_x = width, min_y = height, max_x = 0, max_y = 0;

    for (auto vert : v) {
        if (vert.x() < min_x) min_x = vert.x();
        if (vert.y() < min_y) min_y = vert.y();
        if (vert.x() > max_x) max_x = vert.x();
        if (vert.y() > max_y) max_y = vert.y();
    }
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    // 先找边界
    // float min_x = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    // float min_y = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
    // float max_x = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
    // float max_y = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));




    // 不采用 SSAA or MSAA
    // for (int x = min_x; x <= max_x; ++x) {
    //     for (int y = min_y; y <= max_y; ++y) {
    //         if (insideTriangle(x, y, t.v)) {
    //             // 最小深度 无穷远
    //             float min_depth = FLT_MAX;
                
    //             auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //             float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //             float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //             z_interpolated *= w_reciprocal;

    //             min_depth = std::min(min_depth, z_interpolated);

    //             if (depth_buf[get_index(x, y)] >  min_depth) {
    //                 // 获得最上层应该渲染的颜色
    //                 Vector3f color = t.getColor();
    //                 Vector3f point;
    //                 point << x, y, min_depth;
    //                 // 更新深度
    //                 depth_buf[get_index(x, y)] = min_depth;
    //                 // 更新点的颜色
    //                 set_pixel(point, color);
    //             }
    //         }
    //     }
    // }

    // 方法1 MSAA 2 x 2 z插值求最小值
    // 遍历bounding box 中所有测试点
    // for (int x = min_x; x <= max_x; ++x) {
    //     for (int y = min_y; y <= max_y; ++y) {
    //         std::vector<std::pair<float, float>> pixels;
    //         pixels.push_back(std::make_pair(x + 0.25, y + 0.25));
    //         pixels.push_back(std::make_pair(x + 0.25, y + 0.75));
    //         pixels.push_back(std::make_pair(x + 0.75, y + 0.25));
    //         pixels.push_back(std::make_pair(x + 0.75, y + 0.75));

    //         float min_depth = FLT_MAX;  // 最小深度 无穷远
    //         int count = 0;
    //         for (auto pixel : pixels) {
    //             if (insideTriangle(pixel.first, pixel.second, t.v)) {
    //                 auto[alpha, beta, gamma] = computeBarycentric2D(pixel.first, pixel.second, t.v);
    //                 float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //                 float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //                 z_interpolated *= w_reciprocal;

    //                 // std::cout << z_interpolated << std::endl;
    //                 min_depth = std::min(min_depth, z_interpolated); // 求 2 * 2 采样的 z 插值最小值

    //                 if (depth_buf[get_index(pixel.first, pixel.second)] >  min_depth) {
    //                     count++;
    //                 }
    //             }
    //         } // for (auto pixel : pixels) 

    //         if (count != 0) {
    //             if (depth_buf[get_index(x, y) > min_depth]) {
    //                 // 获得最上层应该渲染的颜色
    //                 Vector3f color = t.getColor() * count / 4.f + (4.f - count) / 4.f * frame_buf[get_index(x, y)];
    //                 Vector3f point;
    //                 point << x, y, min_depth;
    //                 // 更新深度
    //                 depth_buf[get_index(x, y)] = min_depth;
    //                 // 更新点的颜色
    //                 set_pixel(point, color);
    //             }
    //         }
    //     }
    // }


    // 方法2 MSAA 2 x 2 z插值求平均值
   

    // Vector3f color = t.getColor();
    // for (int i = min_x; i < max_x; i++) {
    //     for (int j = min_y; j < max_y; j++) {
    //         float z_depth = 0, blend_rate;
    //         int count = 0;

    //         for (int r = 0; r < sampling_frequency; r++) {
    //             for (int l = 0; l < sampling_frequency; l++) {
    //                 float x = i + (r + 0.5f) * sampling_period;
    //                 float y = j + (l + 0.5f) * sampling_period;

    //                 if (insideTriangle(x, y, t.v)) {
    //                     auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //                     float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //                     float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //                     z_interpolated *= w_reciprocal;

    //                     z_depth += z_interpolated; // 把 2 * 2 采样 中在三角形的内部的像素 z 插值累加
    //                     count++;
    //                 }
    //             }
    //         }

    //         if (count > 0) {
    //             z_depth = z_depth / count;
    //             blend_rate = 1.0f * count / sampling_times;
    //             Vector3f point;
    //             point << i*1.0f, j*1.0f, z_depth;

    //             if (z_depth < depth_buf[get_index(i, j)]) {
    //                 depth_buf[get_index(i, j)] = z_depth;
    //                 Vector3f color = t.getColor() * blend_rate;
    //                 set_pixel(point, color);
    //             }
    //         }
    //     }
    // }

    // 方法3 SSAA
    for (int x = min_x; x < max_x; x++) {
        for (int y = min_y; y < max_y; y++) {
            for (float i = start_point; i < 1.f; i += pixel_size_sm) {
                for (float j = start_point; j < 1.f; j += pixel_size_sm) {
                    if (insideTriangle(x + i, y + j, t.v)) {
                        auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        int index = get_index_ssaa(x, y, i, j);
                        if (z_interpolated < depth_buff_ssaa[index]) {
                            frame_buf_ssaa[index] = t.getColor();
                            depth_buff_ssaa[index] = z_interpolated;
                        }
                    }
                }
            }
        }
    }

    // 降采样
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            Eigen::Vector3f color = {0.f, 0.f, 0.f};
            float infinity = std::numeric_limits<float>::infinity();

            for (float i = start_point; i < 1.f; i += pixel_size_sm) {
                for (float j = start_point; j < 1.f; j += pixel_size_sm) {
                    color += frame_buf_ssaa[get_index_ssaa(x, y, i, j)];
                }
            }

            Eigen::Vector3f point;
            point << x, y, 0.f;
            set_pixel(point, color / (ssaa_h * ssaa_w));
        }
    }

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
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
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(frame_buf_ssaa.begin(), frame_buf_ssaa.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_buff_ssaa.begin(), depth_buff_ssaa.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    frame_buf_ssaa.resize(w * ssaa_w * h * ssaa_h);
    depth_buff_ssaa.resize(w * ssaa_w * h *ssaa_h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_index_ssaa(int x, int y, float i, float j) 
{
    int ssaa_height = height * ssaa_h;
    int ssaa_width = width * ssaa_w;

    i = int ((i - start_point) / pixel_size_sm);
    j = int ((j - start_point) / pixel_size_sm);

    return (ssaa_height - 1 - y * ssaa_h + j) * ssaa_width + x * ssaa_w + i;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on