#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos){
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
                 0, 1, 0, -eye_pos[1],
                 0, 0, 1, -eye_pos[2],
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

// 逐个元素地构建模型变换矩阵并返回该矩阵，传入参数-角度
Eigen::Matrix4f get_model_matrix(float rotation_angle){
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    // 绕z轴旋转的矩阵
    model << cos(rotation_angle/180 * MY_PI), -sin(rotation_angle/180 * MY_PI), 0, 0,
              sin(rotation_angle/180 * MY_PI), cos(rotation_angle/180 * MY_PI), 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

    return model;
}

// 使用给定的参数逐个元素地构建透视投影矩阵并返回该矩阵
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar){
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // 透视投影：棱锥体转换成长方体->正交投影
    Eigen::Matrix4f pres, orth;
    float n = -zNear, f = -zFar, A = n + f, B = -n * f;
    pres << n, 0, 0, 0,
             0, n, 0, 0,
             0, 0, A, B,
             0, 0, 1, 0;

    // 计算 l, r, b, t
    float t = tan((eye_fov * MY_PI / 180) / 2) * fabs(zNear);
    float r = aspect_ratio * t;
    float l = -r;
    float b = -t;

    // 正交变换：先平移到原点，再进行缩放
    Eigen::Matrix4f trans, scale;
    trans << 1, 0, 0, -(r + l) / (r - l),
             0, 1, 0, -(t + b) / (t - b),
             0, 0, 1, -(n + f) / (n - f),
             0, 0, 0, 1;
    
    scale << 2 / (r - l), 0, 0, 0,
             0, 2 / (t - b), 0, 0,
             0, 0, 2 / (n - f), 0,
             0, 0, 0, 1;

    orth = scale * trans;
    
    // float radians = eye_fov/180 * MY_PI/2;
    // orth << -1/(tan(radians) * n * aspect_ratio), 0, 0, 0,
    //         0, -1/(tan(radians) * n), 0, 0, 
    //         0, 0, 2/(n - f), -(n + f)/(n - f),
    //         0, 0, 0, 1;

    projection = orth * pres * projection;
    return projection;
}

// 绕任意一个过原点的轴旋转
Eigen::Matrix4f get_model_matrix_rotateanyaxis(Vector3f axis, float angle){
    // 首先将旋转轴平移到 X/Y/Z 轴上，然后旋转，最后再平移还原回去
    // 这里直接使用 罗德里德公式： M = I + sinθ * Rk + (1 - cosθ) * Rk^2

    float rotate_angle = angle / 180 * MY_PI;
    Eigen::Matrix3f I3 = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f Rk, model3;
    Rk << 0, -axis(2), axis(1),
          axis(2), 0, -axis(0),
          -axis(1), axis(0), 0;
    
    model3 = I3 + sin(rotate_angle) * Rk + (1 - cos(rotate_angle)) * Rk * Rk;

    // 将三维的旋转矩阵转换成四维的
    Eigen::Matrix4f model4 = Eigen::Matrix4f::Identity();
    model4.block(0,0,3,3) << model3;
    return model4;
}


int main(int argc, const char** argv){
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // 绕 Z 轴旋转
        // r.set_model(get_model_matrix(angle));
        // 绕任意轴旋转
        r.set_model(get_model_matrix_rotateanyaxis(Vector3f(0,0,1), angle));

        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // 绕 Z 轴旋转
        // r.set_model(get_model_matrix(angle));
        // 绕任意轴旋转
        r.set_model(get_model_matrix_rotateanyaxis(Vector3f(1,1,1), angle));

        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
