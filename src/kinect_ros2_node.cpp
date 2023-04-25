#include "rclcpp/rclcpp.hpp"
#include "kinect_ros2/kinect_ros2_component.hpp"
#include "class_loader/class_loader.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "rclcpp_components/node_factory.hpp"
#include <boost/format.hpp>

static std::vector<class_loader::ClassLoader *> kClassLoaders;

std::string GenerateNodeTypeName(const std::string& class_name)
{
  return boost::str(boost::format("rclcpp_components::NodeFactoryTemplate<%s>") % class_name);
}

std::vector<rclcpp_components::NodeInstanceWrapper> LoadImageProcNodes()
{
  const std::vector<std::string> classes = {
    GenerateNodeTypeName("depth_image_proc::PointCloudXyzrgbNode")
  };

  auto path_prefix = ament_index_cpp::get_package_prefix("depth_image_proc");
  std::unique_ptr<class_loader::ClassLoader> loader = std::make_unique<class_loader::ClassLoader>(path_prefix + "/lib/libdepth_image_proc.so");

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  options.arguments({"kinect_ros2:__node:=rgb"});
  std::vector<rclcpp_components::NodeInstanceWrapper> nodes;

  for(const auto& class_name : classes)
  {
    std::shared_ptr<rclcpp_components::NodeFactory> node_factory = loader->createInstance<rclcpp_components::NodeFactory>(class_name);
    rclcpp_components::NodeInstanceWrapper node_wrapper = node_factory->create_node_instance(options);
    if(node_wrapper.get_node_base_interface() == nullptr)
    {
      throw std::runtime_error(boost::str(boost::format("Failed to load class {}") % class_name));
    }
    kClassLoaders.push_back(loader.get());
    nodes.push_back(node_wrapper);
  }
  return nodes;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto kinect_component = std::make_shared<kinect_ros2::KinectRosComponent>(options);
  std::vector<rclcpp_components::NodeInstanceWrapper> nodes = LoadImageProcNodes();

  exec.add_node(kinect_component);
  std::for_each(nodes.begin(), nodes.end(), [&exec](rclcpp_components::NodeInstanceWrapper& node){
    exec.add_node(node.get_node_base_interface());
  });

  exec.spin();
  std::for_each(nodes.begin(), nodes.end(), [&exec](auto node){
    exec.remove_node(node.get_node_base_interface());
  });
  nodes.clear();

  rclcpp::shutdown();

  return 0;
}
