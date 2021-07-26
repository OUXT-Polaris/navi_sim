#include <yaml-cpp/yaml.h>

#include <iostream>

int main()
{
  const auto yaml = YAML::LoadFile("/tmp/context.yaml");
  std::cout << "========== Context ==========" << std::endl;
  std::cout << yaml << std::endl;
  std::cout << "========== Success or Failure ==========" << std::endl;
  if (yaml["actions"]["failure"]["state"].as<std::string>() == "finished") {
    std::cout << "failure!" << std::endl;
    std::exit(-1);
  }
  if (yaml["actions"]["success"]["state"].as<std::string>() == "finished") {
    std::cout << "success!" << std::endl;
    std::exit(0);
  }
  std::cout << "failure!" << std::endl;
  std::exit(-1);
}
