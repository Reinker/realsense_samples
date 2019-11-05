#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_options.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/opencv.hpp>

class Viz {
  private :
    cv::Mat depth;
    cv::Mat color;
    rs2::pipeline pipe;
    rs2::frameset frames;
    rs2::colorizer color_map;
    rs2::config cfg;
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

    void get_depth_frame(){
      rs2::video_frame depth_frame = frames.get_depth_frame().apply_filter(color_map);
      depth = cv::Mat(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_8UC3, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
    }

    void get_color_frame(){
      rs2::video_frame color_frame = frames.get_color_frame();
      color = cv::Mat(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    }
  public : 
    void run(){
      cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
      cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
      if(get_device()!=0) return;
      if(set_advanced_mode("./ShortRangePreset.json")!=0) return;

      while (cv::waitKey(1)!='q'){
        frames = pipe.wait_for_frames();
        get_depth_frame();
        get_color_frame();
        cv::imshow("Depth", depth);
        cv::imshow("Color", color);
      }

      return;
    }
};

int main(){
  Viz viz;
  viz.run();

  return 0;
}

