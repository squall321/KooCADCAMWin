// @lat: [[engine/reverse-route#analyze-cli]]
//
// koo_analyze — load any STEP file, run the reverse-engineering recognizers,
// and print every machining feature recovered (skill, confidence, measured
// dimensions).  Honest tool for testing recognition on real foreign CAD:
// it reports what the recognizers DO recover and, by the recovered/total face
// ratio, how much of the part is left unrecognized.
//
//   Usage: koo_analyze <file.step> [min_confidence=0.5]

#include "io/StepIO.hpp"
#include "re/Recognizer.hpp"
#include "skills/Workpiece.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <map>
#include <string>

using namespace koocadcam;

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: koo_analyze <file.step> [min_confidence=0.5]\n";
        return 2;
    }
    const std::string path = argv[1];
    const double minConf = (argc >= 3) ? std::atof(argv[2]) : 0.5;

    std::string err;
    auto shape = io::StepIO::read(path, err);
    if (!shape) {
        std::cerr << "koo_analyze: failed to read STEP: " << err << "\n";
        return 1;
    }

    skill::Workpiece wp(*shape);

    double xMin, yMin, zMin, xMax, yMax, zMax;
    wp.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    double vol = 0.0;
    try { GProp_GProps gp; BRepGProp::VolumeProperties(*shape, gp); vol = gp.Mass(); }
    catch (...) {}

    std::cout << "=== koo_analyze: " << path << " ===\n";
    std::cout << "faces=" << wp.faceCount()
              << "  edges=" << wp.edgeCount()
              << "  volume_mm3=" << vol << "\n";
    std::cout << "bbox_mm = [" << xMin << ", " << yMin << ", " << zMin << "] .. ["
              << xMax << ", " << yMax << ", " << zMax << "]"
              << "  (" << (xMax - xMin) << " x " << (yMax - yMin)
              << " x " << (zMax - zMin) << ")\n\n";

    // Reverse-engineer: every recognizer, de-duplicated, above the threshold.
    auto cands = re::analyze(wp);
    auto deduped = re::dedupe(cands);
    std::sort(deduped.begin(), deduped.end(),
              [](const auto& a, const auto& b){ return a.confidence > b.confidence; });

    std::map<std::string, int> bySkill;
    int shown = 0;
    std::cout << "Recovered features (confidence >= " << minConf << "):\n";
    for (const auto& c : deduped) {
        if (c.confidence < minConf) continue;
        ++shown;
        ++bySkill[c.skill_id];
        std::cout << "  [" << c.confidence << "] " << c.skill_id
                  << "  " << c.recovered_params.dump() << "\n";
    }
    if (shown == 0)
        std::cout << "  (none — no known machining feature recognized at this threshold)\n";

    std::cout << "\nSummary:\n";
    std::cout << "  total recognizer candidates : " << cands.size() << "\n";
    std::cout << "  after dedupe                : " << deduped.size() << "\n";
    std::cout << "  recovered (>= " << minConf << ")        : " << shown << "\n";
    for (const auto& kv : bySkill)
        std::cout << "    - " << kv.first << " x" << kv.second << "\n";
    return 0;
}
