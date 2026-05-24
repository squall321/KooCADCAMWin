// @lat: [[process/test-strategy#golden]]
// CLI tool: generates the golden cylinder STEP at build time.
// Usage: make_sample_step <output-path>

#include "engine/PlaceholderCylinder.hpp"
#include "io/StepIO.hpp"

#include <filesystem>
#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: make_sample_step <output-path>\n";
        return 2;
    }
    namespace fs = std::filesystem;
    const fs::path out = argv[1];
    fs::create_directories(out.parent_path());

    auto shape = koocadcam::engine::PlaceholderCylinder::make();
    std::string err;
    if (!koocadcam::io::StepIO::write(shape, out, err)) {
        std::cerr << "make_sample_step: write failed: " << err << "\n";
        return 1;
    }
    std::cout << "make_sample_step: wrote " << out.string() << "\n";
    return 0;
}
