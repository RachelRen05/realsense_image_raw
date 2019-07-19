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
#include <iostream>  
#include <rclcpp/rclcpp.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <ament_index_cpp/get_resource.hpp>
      
int image_num = 0;

std::string getConfigPath()
{
  std::string content;
  std::string prefix_path;
  ament_index_cpp::get_resource("packages", "realsense_image_raw", content, &prefix_path);
  return prefix_path + "/data";
}

void imCallBack(sensor_msgs::msg::Image::SharedPtr msg)
{
  if(!rclcpp::ok())
    return;

  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);/*sensor_msgs::image_encodings::BGR8*/
  {  
    std::stringstream stream;  
    stream << getConfigPath() + "inputImage"<< image_num<< ".jpg";
    std::string filename = stream.str();
    std::cout << "save image file: " << filename << std::endl;

    cv::imwrite(filename, cv_ptr->image);
    image_num++;
  }       
}

int main (int argc, char** argv)
{ 
  
  rclcpp::init(argc, argv);  
  auto node = rclcpp::Node::make_shared("realsene_image_raw_sub");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_im = node->create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw", qos, imCallBack);
  rclcpp::WallRate loop_rate(1);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while(rclcpp::ok())
  {
    loop_rate.sleep();
    executor.spin_once(std::chrono::seconds(0));
  }
    
  return 0;
}  

