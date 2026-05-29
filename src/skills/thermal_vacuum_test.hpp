#pragma once
// @lat: [[engine/skills#thermal_vacuum_test]]
//
// thermal_vacuum_test — TVAC qualification cycle for a flight article.
// Captures the chamber vacuum pressure (torr), the hot/cold temperature
// excursion span (°C peak-to-peak), and the number of thermal cycles.
// The hardware B-rep is unchanged — the test is non-destructive, so
// this skill stamps the qualification metadata.
//
// Signature pattern:
//   - pattern.kind                  = "thermal_vacuum_test"
//   - pattern.chamber_pressure_torr = vacuum level (torr)
//   - pattern.temp_range_c          = hot-cold excursion (°C peak-to-peak)
//   - pattern.cycles                = number of thermal cycles
//   - pattern.geometry_changed      = false
//
// DFM checks:
//   chamber_pressure_torr  ∈ (0, 1e-3]   (DFM-SPACE-VAC error if too high)
//   temp_range_c           ∈ (0, 400]    (DFM-SPACE-TEMP error if outside)
//   cycles                 ∈ [1, 50]     (DFM-SPACE-CYC error if outside)
//
// Synthesis: metadata-only.
// Recognize: metadata replay.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace thermal_vacuum_test {

constexpr const char* kSkillId = "thermal_vacuum_test";

struct Input
{
    double chamber_pressure_torr = 1.0e-5;
    double temp_range_c          = 150.0;
    int    cycles                = 4;
};

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace thermal_vacuum_test
}  // namespace koocadcam::skill
