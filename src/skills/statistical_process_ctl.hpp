#pragma once
// @lat: [[engine/skills#statistical_process_ctl]]
//
// statistical_process_ctl — SPC capability skill.  Computes the classic
// Cp and Cpk indices from a process sample (mean μ, sigma σ) and the
// upper/lower specification limits.
//
//          USL − LSL                 ⎛ USL − μ     μ − LSL ⎞
//   Cp  = ───────────       Cpk = min⎜ ───────  ,  ─────── ⎟
//             6 σ                    ⎝   3 σ          3 σ   ⎠
//
//   Cp  measures POTENTIAL capability (spread vs. spec width).
//   Cpk measures REAL capability (also accounts for off-centring).
//
//   Typical interpretation (industry):
//      Cpk < 1.00   → process incapable
//      Cpk 1.00–1.33→ marginal
//      Cpk 1.33–1.67→ capable (target for most automotive)
//      Cpk > 1.67   → highly capable (Six-Sigma class)
//
// Geometrically a no-op.  Signature records μ, σ, USL, LSL, computed Cp,
// computed Cpk, and the verdict bucket.
//
// Input:
//   characteristic   — std::string, name of measured characteristic
//   mean             — double, sample mean (μ)
//   sigma            — double, sample std-dev (σ), must be > 0
//   USL              — double, upper specification limit
//   LSL              — double, lower specification limit, USL > LSL
//   sample_size      — int, optional N (>= 1) for record-keeping
//
// DFM checks (error-level):
//   DFM-INPUT             sigma <= 0
//   DFM-INPUT             USL <= LSL
//   DFM-INPUT             sample_size < 1
//
// DFM checks (info-level):
//   DFM-SPC-INCAPABLE     computed Cpk < 1.0 (record + warn)
//   DFM-SPC-OFFCENTRE     |Cpk − Cp| > 0.1·Cp  (process drifts from centre)
//
// Recognize:
//   Metadata-only.

#include "Datum.hpp"
#include "Skill.hpp"

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

namespace statistical_process_ctl {

constexpr const char* kSkillId = "statistical_process_ctl";

struct Input
{
    std::string  characteristic = "";
    double       mean           = 0.0;
    double       sigma          = 1.0;
    double       USL            = 0.0;
    double       LSL            = 0.0;
    int          sample_size    = 30;
};

// Pure helper — exposed for unit tests.  Returns {Cp, Cpk}.
// Pre-condition: sigma > 0 and USL > LSL.
std::pair<double, double> computeCpCpk(double mean, double sigma,
                                       double USL, double LSL);

SkillOutput apply(const Workpiece& wp, const Input& in);
DFMReport   validate(const Workpiece& wp, const Input& in);
std::vector<RecognizedFeature> recognize(const Workpiece& wp);

}  // namespace statistical_process_ctl
}  // namespace koocadcam::skill
