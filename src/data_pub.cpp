/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_resource.hpp>
#include <unistd.h>
#include <limits.h>
#include <vector>
#include <dirent.h>
#include <iostream>

#define PLAYRATE 20
#define IMAGENUM 20
#define MAXSIZE 50
std::string getConfigPath()
{
  std::string content;
  std::string prefix_path;
  ament_index_cpp::get_resource("packages", "realsense_image_raw", content, &prefix_path);
  return prefix_path;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("realsense_image_raw_pub");
  auto pub_im = node->create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw", 10);
  
  std::string strFilePath;
  char imageFile[MAXSIZE];
  std::vector<sensor_msgs::msg::Image::SharedPtr> vec_msg_im;

  for(int i=0; i<IMAGENUM; i++)
  {
    strFilePath = getConfigPath();
    std::sprintf(imageFile, "/datainputImage%d.jpg", i);
    strFilePath += imageFile;
    std::cout << strFilePath << std::endl;
    if(strFilePath.empty())
    {
      std::cout << "FILE: ["<<strFilePath<<"] NOT EXIST!" << std::endl;
      return 1;
    }
    cv::Mat image = cv::imread(strFilePath, 1);
    if(image.empty()) {
      std::cout << "Empty Image!" << std::endl;
      return 2;
    }
    sensor_msgs::msg::Image::SharedPtr msg_im = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    msg_im->header.frame_id = std::string("camera_depth_optical_frame");
    vec_msg_im.push_back(msg_im);
  }
  
  rclcpp::WallRate loop_rate(PLAYRATE);
  std::cout<<"[INFO] []: Start to punlish Rgb! -- "<<"PLAY_RATE: "<<PLAYRATE<<", IMAGE_NUM: "<<IMAGENUM<<std::endl;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while(rclcpp::ok()) // loop play
  {
      for(int i = 0;i < IMAGENUM;i++)
      {
        vec_msg_im[i]->header.stamp = rclcpp::Clock().now();
        pub_im->publish(vec_msg_im[i]);
	cv::waitKey(1);
      }
      for(int i = IMAGENUM - 2;i > 0;i--)
      {
        vec_msg_im[i]->header.stamp = rclcpp::Clock().now();
        pub_im->publish(vec_msg_im[i]);
	cv::waitKey(1);
      }
      loop_rate.sleep();
      executor.spin_once(std::chrono::seconds(0));
  }
  vec_msg_im.clear();
  return 0;
}
