#include <igl/opengl/glfw/Viewer.h>
#include <igl/png/writePNG.h>
#include <Eigen/Core>
#include <cxxopts.hpp>
#include <filesystem>
#include <iostream>
#include <string>

#include "Utils.h"
#include "MainUI.h"

using namespace gripper;

int main(int argc, const char* argv[]) {
  cxxopts::Options options("passive-gripper",
                           "A program that generates a gripper for an object");

  // clang-format off
  options.add_options("")
    ("batch", "Just process")
    ("f,file", "Object mesh file name", cxxopts::value<std::string>())
    ("o,output", "Output file prefix", cxxopts::value<std::string>());
  // clang-format on

  std::cout << "Num Threads: " << Eigen::nbThreads() << std::endl;
  std::cout << "Current Directory: " << std::filesystem::current_path()
            << std::endl;

  igl::opengl::glfw::Viewer viewer;
  MainUI voxelizeUI;
  viewer.plugins.push_back(&voxelizeUI);

  try {
    auto option = options.parse(argc, argv);
    if (option["batch"].as<bool>()) {
      std::string filename = option["file"].as<std::string>();
      std::string prefix = option["output"].as<std::string>();

      std::cout << filename << "," << prefix << std::endl;
      viewer.launch_init();
      viewer.load_mesh_from_file(filename);
      voxelizeUI.voxelPipeline.reset(
          new VoxelPipeline(&voxelizeUI,
                            voxelizeUI.GetMeshVertices(),
                            voxelizeUI.GetMeshFaces()));
      VoxelPipeline* pipeline = voxelizeUI.voxelPipeline.get();
      pipeline->UpdateSettings(VoxelPipelineSettings(), true);
      pipeline->WriteDXF(prefix + "_gripper.dxf");
      pipeline->WriteResult(prefix + "_result.txt");
      pipeline->WriteGripper(prefix + "_gripper.obj");
      pipeline->WriteRAPID(prefix + "_rapid.txt");

      viewer.core().camera_eye = Eigen::Vector3f(-5, 2, 5) * 0.5;
      

      // Eigen::MatrixXd all(4, 3);
      // all.row(0) = voxelizeUI.GetMeshVertices().colwise().minCoeff();
      // all.row(1) = voxelizeUI.GetMeshVertices().colwise().maxCoeff();
      // all.row(2) = pipeline->GetGripper().V().colwise().minCoeff();
      // all.row(3) = pipeline->GetGripper().V().colwise().maxCoeff();
      // viewer.core().align_camera_center(all);
      utils::CaptureScreen(
          viewer.core(), viewer.data_list, 1280, 800, prefix + "_preview.png");

      viewer.launch_shut();
    } else {
      viewer.launch();
    }
  } catch (cxxopts::OptionException& e) {
    std::cout << options.help() << std::endl;
  }

  return 0;
}
