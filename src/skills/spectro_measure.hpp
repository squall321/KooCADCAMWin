#pragma once
// @lat: [[engine/skills#spectro_measure]]
//
// spectro_measure — record an instrumental spectroscopic measurement.
// Four modalities are supported:
//
//   "UV-Vis"  — absorbance vs wavelength (typically 190–800 nm)
//   "IR"      — vibrational absorption (≈ 2500–25000 nm; cm⁻¹ space)
//   "Raman"   — inelastic scatter (excitation line + Stokes shift)
//   "MS"      — mass spectrometry (m/z domain — wavelength irrelevant but
//               recorded as 0 by convention)
//
// The workpiece (cuvette / sample holder) is geometrically unchanged; the
// signature records instrument type, primary wavelength setting and the
// requested scan count for signal averaging.
//
// Signature pattern:
//   pattern.kind              = "spectro_measure"
//   pattern.instrument_type   = "UV-Vis" | "IR" | "Raman" | "MS"
//   pattern.wavelength_nm     = double
//   pattern.scan_count        = int
//   pattern.geometry_changed  = false
//
// DFM:
//   DFM-INPUT         instrument_type unknown (error)
//   DFM-SPECTRO-WL    wavelength_nm <= 0 for optical modalities (error)
//   DFM-SPECTRO-WL    wavelength_nm out of expected band for modality (info)
//   DFM-SPECTRO-SCAN  scan_count < 1 (error)
//
// Recognize: metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace spectro_measure {

constexpr const char* kSkillId = "spectro_measure";

struct Input
{
    std::string  instrument_type  = "UV-Vis";    // "UV-Vis" | "IR" | "Raman" | "MS"
    double       wavelength_nm    = 254.0;
    int          scan_count       = 8;
};

SkillOutput                    apply   (const Workpiece& wp, const Input& in);
DFMReport                      validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace spectro_measure
}  // namespace koocadcam::skill
