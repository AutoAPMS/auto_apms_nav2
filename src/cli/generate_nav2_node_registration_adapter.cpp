// Copyright 2026 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Configure-time CLI tool that:
//   1. Reads library names from nav2_behavior_tree/plugins_list.hpp
//   2. Loads each library individually to discover which BT nodes it registers
//   3. Generates a C++ source file with one NodeRegistrationFromLibraryAdapter
//      subclass per library
//   4. Generates a manifest YAML mapping every BT node name to the correct
//      adapter subclass
//   5. Prints fully-qualified class names to stdout (one per line) for CMake
//
// Usage: generate_nav2_node_registration_adapter <namespace> <output_cpp> <output_manifest>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "nav2_behavior_tree/plugins_list.hpp"

/// Convert snake_case library name to CamelCase class name.
std::string toCamelCase(const std::string & snake)
{
  std::string result;
  bool capitalize_next = true;
  for (char c : snake) {
    if (c == '_') {
      capitalize_next = true;
    } else {
      result += capitalize_next ? static_cast<char>(std::toupper(c)) : c;
      capitalize_next = false;
    }
  }
  return result;
}

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::cerr << "Usage: generate_nav2_node_registration_adapter <namespace> <output_cpp> <output_manifest>\n";
    return EXIT_FAILURE;
  }

  const std::string ns = argv[1];
  const std::string output_cpp = argv[2];
  const std::string output_manifest = argv[3];

  try {
    // Parse library names from BT_BUILTIN_PLUGINS.
    std::vector<std::string> lib_names;
    {
      std::istringstream ss(nav2::details::BT_BUILTIN_PLUGINS);
      std::string name;
      while (std::getline(ss, name, ';')) {
        if (!name.empty()) {
          lib_names.push_back(name);
        }
      }
    }
    if (lib_names.empty()) {
      throw std::runtime_error("No Nav2 BT plugin library names found in BT_BUILTIN_PLUGINS.");
    }

    // For each library, load it in isolation and record which nodes it adds.
    struct LibInfo
    {
      std::string lib_name;
      std::string class_name;
      std::set<std::string> node_names;
    };
    std::vector<LibInfo> libs;
    int total_nodes = 0;

    for (const auto & lib_name : lib_names) {
      // Baseline: record all node names present in a fresh factory (builtins).
      BT::BehaviorTreeFactory factory;
      std::set<std::string> baseline;
      for (const auto & [n, _] : factory.manifests()) {
        baseline.insert(n);
      }

      const auto os_name = BT::SharedLibrary::getOSName(lib_name);
      try {
        factory.registerFromPlugin(os_name);
      } catch (const std::exception & e) {
        std::cerr << "[generate_nav2_node_registration_adapter] Warning: skipping '" << lib_name << "': " << e.what() << "\n";
        continue;
      }

      // Nodes added by this library = current manifests minus baseline.
      LibInfo info;
      info.lib_name = lib_name;
      info.class_name = toCamelCase(lib_name);
      for (const auto & [n, _] : factory.manifests()) {
        if (baseline.count(n) == 0) {
          info.node_names.insert(n);
        }
      }
      if (!info.node_names.empty()) {
        total_nodes += static_cast<int>(info.node_names.size());
        libs.push_back(std::move(info));
      }
    }

    // ----------------------------------------------------------------
    // Write the generated C++ source file.
    // ----------------------------------------------------------------
    {
      std::ofstream out(output_cpp);
      if (!out.is_open()) {
        throw std::runtime_error("Cannot open output file: " + output_cpp);
      }

      out << "// Auto-generated Nav2 BT node registration adapters.\n"
          << "// Generated from " << libs.size() << " Nav2 BT plugin libraries (" << total_nodes << " nodes).\n"
          << "// Do not edit — this file is regenerated at configure time.\n\n"
          << "#include \"auto_apms_behavior_tree_core/node/node_registration_from_library_adapter.hpp\"\n"
          << "#include \"behaviortree_cpp/utils/shared_library.h\"\n"
          << "#include \"pluginlib/class_list_macros.hpp\"\n\n"
          << "namespace " << ns << "\n{\n\n";

      for (const auto & lib : libs) {
        out << "class " << lib.class_name
            << "\n  : public auto_apms_behavior_tree::core::NodeRegistrationFromLibraryAdapter\n{\n"
            << "  std::string getPluginLibraryPath() const override { return BT::SharedLibrary::getOSName(\""
            << lib.lib_name << "\"); }\n"
            << "};\n\n";
      }

      out << "}  // namespace " << ns << "\n\n";

      for (const auto & lib : libs) {
        out << "PLUGINLIB_EXPORT_CLASS(\n"
            << "  " << ns << "::" << lib.class_name << ",\n"
            << "  auto_apms_behavior_tree::core::NodeRegistrationInterface)\n";
      }

      out.close();
    }

    // ----------------------------------------------------------------
    // Write the manifest YAML.
    // ----------------------------------------------------------------
    {
      std::ofstream out(output_manifest);
      if (!out.is_open()) {
        throw std::runtime_error("Cannot open output file: " + output_manifest);
      }

      out << "# Auto-generated Nav2 BT node manifest.\n"
          << "# Generated from " << libs.size() << " Nav2 BT plugin libraries (" << total_nodes << " nodes).\n"
          << "# Do not edit — this file is regenerated at configure time.\n\n";

      for (const auto & lib : libs) {
        for (const auto & node : lib.node_names) {
          out << node << ":\n"
              << "  class_name: " << ns << "::" << lib.class_name << "\n\n";
        }
      }

      out.close();
    }

    // ----------------------------------------------------------------
    // Print class names to stdout for CMake to capture.
    // ----------------------------------------------------------------
    for (const auto & lib : libs) {
      std::cout << ns << "::" << lib.class_name << "\n";
    }

    std::cerr << "[generate_nav2_node_registration_adapter] Generated " << total_nodes << " node entries from " << libs.size()
              << " libraries.\n";
  } catch (const std::exception & e) {
    std::cerr << "ERROR (generate_nav2_node_registration_adapter): " << e.what() << "\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
