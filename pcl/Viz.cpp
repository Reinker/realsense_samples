#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <unistd.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_options.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>


class Viz {
  private :
    rs2::config cfg;
    rs2::pipeline pipe;
    rs2::pipeline_profile selection;
    rs2::device dev;
    

    int get_device(){
      this->selection = pipe.start(cfg);
      this->dev = selection.get_device();
      return 0;
    }

    int set_advanced_mode(std::string json_path){
      if (dev.is<rs400::advanced_mode>()){
        auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
        std::ifstream t(json_path);
        std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
        advanced_mode_dev.load_json(str);
        return 0;
      }else{
        return -1; 
      }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_data(rs2::points& points, rs2::video_frame& color,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
      const rs2::vertex* vertex = points.get_vertices();
      const rs2::texture_coordinate*  texure = points.get_texture_coordinates();
      for (auto & v : cloud->points){
        v.x = vertex->x;
        v.y = vertex->y;
        v.z = vertex->z;
        int x = std::min(std::max(int(texure->u * color.get_width() + .5f), 0), color.get_width() - 1);
        int y = std::min(std::max(int(texure->v * color.get_height() + .5f), 0), color.get_height() - 1);
        int bytes = x * color.get_bytes_per_pixel();   
        int strides = y * color.get_stride_in_bytes(); 
        int index =  (bytes + strides);
        const auto new_tex = reinterpret_cast<const uint8_t*>(color.get_data());
        v.r = new_tex[index];
        v.g = new_tex[index + 1];
        v.b = new_tex[index + 2];
        vertex++;
        texure++;
      }
      return cloud;
    }
  public :
    void run(){
      rs2::pointcloud pc;
      rs2::config cfg;
      int width = 1280;
      int height = 720;
      cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);
      cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);
      if(get_device()!=0) return;
      if(set_advanced_mode("./HighResHighDensityPreset.json")!=0) return;

      rs2::frameset frames;
      rs2::sensor depth_sensor = dev.first<rs2::depth_sensor>();
      pcl::visualization::CloudViewer viewer("Realsense Vewer");
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_points;
      cloud->width = width;
      cloud->height = height;
      cloud->is_dense = false;
      cloud->points.resize (width * height);

      while (!viewer.wasStopped()){
        frames = pipe.wait_for_frames();
        rs2::video_frame depth_frame = frames.get_depth_frame();
        rs2::video_frame color_frame = frames.get_color_frame();
        pc.map_to(color_frame);
        rs2::points points = pc.calculate(depth_frame);
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pcl_points = pcl_data(points, color_frame, cloud);
        pass.setInputCloud(pcl_points);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 1.0);
        pass.filter(*cloud_filtered);
        viewer.showCloud(cloud_filtered);
        //viewer.showCloud(pcl_points);
      }
      return;
    }
};

int main(){
  Viz viz;
  viz.run();

  return 0;
}

