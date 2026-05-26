#pragma once
// @lat: [[engine/skills#Skill interface]]
//
// Universal contract for packaged, analyzable machining skills (가공 스킬).
//
// Every skill in `src/skills/` must provide the same four-method shape so
// that process plans can serialize / analyze / re-execute generically:
//
//   1. apply()      — synthesis (forward)
//                     Input × Workpiece → Workpiece' + FeatureSignature
//
//   2. recognize()  — analysis (reverse)
//                     Workpiece → vector<RecognizedFeature>
//                     For each pattern match, returns the recovered
//                     Input + a confidence score.
//
//   3. validate()   — DFM gate
//                     Input × Workpiece → DFMReport
//                     Skill-specific manufacturability check.
//
//   4. signature()  — the geometric pattern this skill leaves
//                     (declared statically — used by recognize())
//
// The signature concept is the key to analyzability: each skill's result
// has a predictable topological pattern (e.g., drill_hole leaves
// 1 cylindrical face + 2 circular edges + 1 flat bottom face), and the
// original parameters can be recovered from that pattern.

#include "DFM.hpp"

#include <TopoDS_Shape.hxx>

#include <nlohmann/json.hpp>

#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece;

// ── Tooling metadata ──────────────────────────────────────────────────────
//
// CAM execution info: what tool, what path, estimated machining cost.
// Skills populate this so CAPP (process planning) can downstream choose
// tool changes, fixturing, and cycle-time estimation.
struct ToolingMeta
{
    std::string  tool_type;      // "drill" | "end_mill" | "ball_mill" | "tap" | "thread_mill"
    double       tool_dia_mm = 0.0;
    double       tool_length_mm = 0.0;
    std::string  tool_material = "carbide";   // "hss" | "carbide" | "diamond" | …
    double       cutting_speed_sfm = 0.0;     // surface feet per minute
    double       feed_per_tooth_mm = 0.0;
    int          flute_count = 0;
    double       stock_removed_mm3 = 0.0;
    double       est_cycle_time_s = 0.0;
    // Free-form extra (skill-specific extensions)
    nlohmann::json extra = nlohmann::json::object();
};

// ── Feature signature ─────────────────────────────────────────────────────
//
// The "fingerprint" a skill leaves on the workpiece.  Stored in two forms:
//
//   - `params`: the original Input JSON (verbatim, for verification &
//               serialization in process plans)
//   - `pattern`: the geometric topology pattern that the skill leaves,
//                described in a machine-readable form that `recognize()`
//                can match against an unknown workpiece.
//
// `pattern.kind` examples:
//   "drill_hole"           1 cyl_face + 2 circ_edges + (optional) bottom planar face
//   "counterbore"          drill_hole + 1 wider cyl_face + 1 step circular edge
//   "mill_rect_pocket"     4 planar walls + 4 corner cyl_faces + 1 bottom face
//   "fillet_edge"          1 cylindrical/toroidal blend face along an edge
//
// Each skill defines its own pattern schema as a JSON object so the
// recognizer can verify the topology in an extensible way.
struct FeatureSignature
{
    std::string       skill_id;    // "drill_hole"
    nlohmann::json    params;      // original Input as JSON
    nlohmann::json    pattern;     // skill-specific topology fingerprint
    ToolingMeta       tooling;
};

// ── Output of a skill apply ───────────────────────────────────────────────
struct SkillOutput
{
    std::shared_ptr<Workpiece>   workpiece;
    FeatureSignature             signature;
};

// ── Output of a skill recognize ───────────────────────────────────────────
//
// recognize() returns zero or more candidates.  Each candidate is a
// guess at what input parameters could have produced an observed
// feature on the workpiece.  Confidence in [0, 1]; LLM / process-plan
// inference can use confidence to pick among competing skills.
struct RecognizedFeature
{
    std::string       skill_id;
    nlohmann::json    recovered_params;
    double            confidence = 0.0;

    // Diagnostic (which face / edge groups in the workpiece this match used)
    nlohmann::json    matched_geometry;
};

// ── Common error ──────────────────────────────────────────────────────────
class SkillError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

}  // namespace koocadcam::skill
