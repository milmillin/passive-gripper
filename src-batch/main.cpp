#include <omp.h>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <boost/process.hpp>

#include <passive-gripper/Constants.h>
#include <passive-gripper/models/ContactPointMetric.h>
#include <passive-gripper/models/SettingsOverrider.h>
#include <utils.h>

#include "PipelineStages.h"
#include "Result.h"
#include "Testcase.h"

namespace fs = std::filesystem;
namespace bp = boost::process;

void Usage(char* argv0) {
  Error() << "Usage: " << argv0
          << " stl output_dir [-s stgo] [-h hook] [-x] [-m maxiters]"
          << std::endl;
}

int main(int argc, char** argv) {
  Log() << "Num threads: " << omp_get_max_threads() << std::endl;
  if (argc < 3) {
    Usage(argv[0]);
    return 1;
  }

  // stl_fn
  std::string stl_fn = argv[1];

  // output_dir
  std::string out_dir = argv[2];

  // -x
  bool restart_set = false;

  // -h
  bool hook_set = false;
  std::string hook_str;

  // -s
  bool stgo_set = false;
  std::string stgo_fn;

  // -m
  int maxiters = 15;

  // -n
  int ckpt_need = 1;

  for (int i = 3; i < argc; i++) {
    if (strncmp(argv[i], "-x", 4) == 0) {
      restart_set = true;
    } else if (strncmp(argv[i], "-h", 4) == 0) {
      hook_set = true;
      hook_str = argv[i + 1];
      i++;
    } else if (strncmp(argv[i], "-s", 4) == 0) {
      stgo_set = true;
      stgo_fn = argv[i + 1];
      i++;
    } else if (strncmp(argv[i], "-m", 4) == 0) {
      maxiters = std::stoi(argv[i + 1]);
      i++;
    } else if (strncmp(argv[i], "-n", 4) == 0) {
      ckpt_need = std::stoi(argv[i + 1]);
      i++;
    }
  }

  if (!fs::is_directory(out_dir) || !fs::exists(out_dir)) {
    fs::create_directory(out_dir);
    Log() << out_dir << " directory created " << std::endl;
  }

  // infer file names
  size_t lastdot = stl_fn.rfind('.');
  std::string raw_fn = stl_fn.substr(0, lastdot);

  std::string psg_fn = raw_fn + ".psg";
  std::string ckpt_fn = raw_fn + ".ckpt";
  std::string cpx_fn = raw_fn + ".cpx";

  size_t lastslash = raw_fn.rfind('/');
  if (lastslash == std::string::npos) lastslash = raw_fn.rfind('\\');
  std::string wopath_fn = raw_fn.substr(lastslash + 1, std::string::npos);

  std::string output_fn = out_dir + "/" + wopath_fn + "-results.txt";

  // set callback
  TestcaseCallback cb = [hook_set, &ckpt_fn, &hook_str, &out_dir, &output_fn](
                            size_t i, size_t need, const Result& r) {
    {
      std::ofstream output_f(output_fn, std::ios::out | std::ios::app);
      if (!output_f.is_open()) {
        Error() << "Warning: cannot open output file: " << output_fn
                << std::endl;
      }
      output_f << r << std::endl;
    }
    if (hook_set) {
      bp::ipstream out;
      bp::child c(hook_str,
                  bp::std_out > out,
                  r.out_fn,
                  psg::kBoolStr[!r.failed],
                  r.name,
                  std::to_string(r.cp_idx),
                  psg::kBoolStr[r.force_closure],
                  psg::kBoolStr[r.partial_force_closure],
                  ToString(r.min_wrench),
                  ToString(r.partial_min_wrench),
                  ToString(r.cost),
                  ToString(r.min_dist),
                  psg::kBoolStr[r.intersecting],
                  ToString(r.volume),
                  ToString(r.pi_volume),
                  std::to_string(r.duration),
                  out_dir);
      char a[1024];
      auto& out_s = Log() << "[child proc]" << std::endl;
      while (out.read(a, 1024)) {
        int read = out.gcount();
        for (int i = 0; i < read; i++) {
          out_s << a[i];
        }
      }
      out_s << std::endl;
      c.wait();
    }
    {
      std::ofstream ckpt_file(ckpt_fn);
      if (!ckpt_file.is_open()) {
        Error() << "Warning: cannot open checkpoint file: " << ckpt_fn
                << std::endl;
      } else {
        ckpt_file << i << ' ' << need << std::endl;
        Log() << "Checkpoint updated: " << i << ' ' << need << std::endl;
      }
    }
  };

  // START PROCESS
  try {
    // generates psg if not exist
    Log() << "Checking PSG file: " << psg_fn << std::endl;
    if (!fs::exists(psg_fn)) {
      Log() << "> Generating PSG file..." << std::endl;
      psg_batch::GeneratePSG(stl_fn, psg_fn);
    }

    psg::PassiveGripper psg;
    psg.DeserializeFn(psg_fn);
    Log() << "PSG file loaded" << std::endl;

    // generate cpx if not exist
    Log() << "Checking CPX file: " << cpx_fn << std::endl;
    if (!fs::exists(cpx_fn)) {
      Log() << "> Generating CPX file..." << std::endl;
      psg_batch::GenerateCPX(psg, cpx_fn);
    }

    // load checkpoint
    int ckpt_i = 0;
    if (!restart_set) {
      std::ifstream ckpt_file(ckpt_fn);
      if (!ckpt_file.is_open()) {
        Error() << "Warning: No checkpoint file; start from the beginning "
                << ckpt_fn << std::endl;
      } else {
        ckpt_file >> ckpt_i >> ckpt_need;
        Log() << "Checkpoint loaded: " << ckpt_i << ' ' << ckpt_need
              << std::endl;
        ckpt_i++;
      }
    }

    psg::SettingsOverrider stgo;
    if (stgo_set) stgo.Load(stgo_fn);
    ProcessFrom(raw_fn, out_dir, ckpt_i, ckpt_need, maxiters, stgo, cb);
  } catch (const std::exception& e) {
    Error() << e.what() << std::endl;
  }

  return 0;
}
