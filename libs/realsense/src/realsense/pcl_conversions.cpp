#include<realsense/pcl_conversions.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

typedef pcl::PointXYZRGB P_pcl;
typedef pcl::PointCloud<P_pcl> point_cloud;
typedef point_cloud::Ptr ptr_cloud;

using namespace cv;

// Get RGB values based on normals - texcoords, normals value [u v]
std::tuple<uint8_t, uint8_t, uint8_t> getTexcolor(rs2::video_frame& _texture, rs2::texture_coordinate _texcoords){
    const int w = _texture.get_width(), h = _texture.get_height();
    
    // convert normals [u v] to basic coords [x y]
    int x = std::min(std::max(int(_texcoords.u*w + .5f), 0), w - 1);
    int y = std::min(std::max(int(_texcoords.v*h + .5f), 0), h - 1);

    int idx = x * _texture.get_bytes_per_pixel() + y * _texture.get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t*>(_texture.get_data());
    return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx+1], texture_data[idx+2]);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointsToPcl(rs2::points& _points, rs2::video_frame& _colored_frame, const Eigen::Matrix4f& _transform){

    // OpenCV Mat for showing the rgb color image, just as part of processing
    Mat colorr(Size(640, 480), CV_8UC3, (void*)_colored_frame.get_data(), Mat::AUTO_STEP);
        
    auto sp = _points.get_profile().as<rs2::video_stream_profile>();
    ptr_cloud cloud(new point_cloud);
    
    // Config of PCL Cloud object
    cloud->width = static_cast<uint32_t>(sp.width());
    cloud->height = static_cast<uint32_t>(sp.height());
    cloud->is_dense = false;
    cloud->points.resize(_points.size());

    auto tex_coords = _points.get_texture_coordinates();
    auto vertices = _points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (unsigned i = 0; i < _points.size(); ++i) {
        Eigen::Vector4f rp = {vertices[i].x, vertices[i].y, vertices[i].z, 1};
        rp = _transform*rp;

        cloud->points[i].x = rp[0];
        cloud->points[i].y = rp[1];
        cloud->points[i].z = rp[2];

        std::tuple<uint8_t, uint8_t, uint8_t> current_color;
        current_color = getTexcolor(_colored_frame, tex_coords[i]);
        cloud->points[i].r = std::get<0>(current_color);
        cloud->points[i].g = std::get<1>(current_color);
        cloud->points[i].b = std::get<2>(current_color);

    }
    return cloud;
}