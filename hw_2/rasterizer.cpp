// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions){
    auto id = get_next_id();
    pos_buf.emplace(id, positions);
    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices){
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols){
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f){
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

// 判断像素中心是否在三角形内，利用叉乘
static bool insideTriangle(float x, float y, const Vector3f* _v){   
    float x_mid = x;
    float y_mid = y;

    Eigen::Vector2f AB(_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y());
    Eigen::Vector2f BC(_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y());
    Eigen::Vector2f CA(_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y());

    Eigen::Vector2f AP(x_mid - _v[0].x(), y_mid - _v[0].y());
    Eigen::Vector2f BP(x_mid - _v[1].x(), y_mid - _v[1].y());
    Eigen::Vector2f CP(x_mid - _v[2].x(), y_mid - _v[2].y());

    // 判断三个顶点和p点构成的两个向量的叉乘值是否同向
    if (((AB[0]*AP[1] - AB[1]*AP[0])>=0) == ((BC[0]*BP[1] - BC[1]*BP[0])>=0) && 
        ((AB[0]*AP[1] - AB[1]*AP[0])>=0) == ((CA[0]*CP[1] - CA[1]*CP[0])>=0))
        return true;
    else
        return false;
}

// 计算像素点对应的重心坐标的权值
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v){
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type){
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind){
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
        for (auto & vert : v){
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i){
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

// 屏幕空间光栅化
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();     // (x, y, z, 1)

    // 找到三角形的 bounding box
    int l, r, b, o;
    l = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    r = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
    b = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
    o = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));

    std::vector<Vector2f> dirc
    {
        {0.25,0.25},
        {0.75,0.25},
        {0.25,0.75},
        {0.75,0.75}
    };

    // 遍历 bounding box 里的像素
    for (int i = l; i <= r; i++){
        for (int j = b; j <= o; j++){
            // 不用 SSAA
            // //判断像素中心点是否在连续三角形内，若在三角形内，就尝试对该像素进行着色，若深度测试通过，便着色
            // if (insideTriangle(i + 0.5, j + 0.5, t.v)){
            //     // 利用重心插值获得每一个像素点的插值深度
            //     float alpha, beta, gamma;
            //     std::tie(alpha, beta, gamma) = computeBarycentric2D(i + 0.5, j + 0.5, t.v);
            //     // 在三维实体中，需要对插值参数进行矫正
            //     // v.w()->点的？，v.z()->点的？，这里都是代表什么
            //     float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            //     float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            //     z_interpolated *= w_reciprocal;

            //     // std::cout << z_interpolated << std::endl;
            //     // 判断深度的大小来确定颜色，理论上是越小越靠近摄像机，但是这里z_interpolated是负值
            //     z_interpolated *= -1;
            //     if (z_interpolated <= depth_buf[get_index(i, j)]){
            //         depth_buf[get_index(i, j)] = z_interpolated;
            //         Eigen::Vector3f color = t.getColor();
            //         set_pixel(Eigen::Vector3f(i, j, z_interpolated), color);
            //     }
            // }

            // 使用 SSAA
            float minZ = FLT_MAX;   // 记录最小深度作为该点深度值
            int count = 0;          // 在三角形中的点的个数
            // 对四个点坐标进行判断
            for (int k = 0; k < 4; k++) {
                if (insideTriangle(i + dirc[k][0], j + dirc[k][1], t.v)) {
                    float alpha, beta, gamma;
                    std::tie(alpha, beta, gamma) = computeBarycentric2D(i + 0.5, j + 0.5, t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    z_interpolated *= -1;
                    minZ = std::min(minZ, z_interpolated);
                    count++;
                }
                if (count != 0) {
                    if (minZ <= depth_buf[get_index(i, j)]) {
                        depth_buf[get_index(i, j)] = minZ;
                        // Vector3f color = t.getColor() * count / 4;
                        // 求出4个采样点的平均颜色
                        Vector3f color = t.getColor() * count / 4 + frame_buf[get_index(i, j)]*(4 - count) / 4.0f;
                        set_pixel(Eigen::Vector3f(i, j, minZ), color);
                    }
                }
            }
        }
    }

    
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m){
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v){
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p){
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff){
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color){
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth){
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h){
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y){
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on