// @lat: [[engine/cam-3axis-verify]]
//
// ToolLibrary — a small machine tool magazine (T-number catalog) + a nearest-tool
// selector.  Slice-1 generators stamp each toolpath with a skill-authored
// ToolingMeta (type / diameter / material / flutes) but no MACHINE tool number, so
// the post could only emit a descriptive (TOOLCHANGE) comment — never an executable
// T/M6 that actually swaps the tool.  ToolLibrary closes that gap: it maps a
// ToolingMeta onto the closest real magazine pocket and returns a stable T-number,
// so toGCodeProgram can emit `T<n> M6` (an executable tool change) with the
// descriptive label kept as a trailing comment.  When nothing in the magazine is a
// reasonable match (wrong family, or diameter far from any pocket) selection FAILS
// and the caller falls back to the comment-only path — we never silently machine
// with the wrong tool.

#ifndef KOOCADCAM_CAM_TOOLLIBRARY_HPP
#define KOOCADCAM_CAM_TOOLLIBRARY_HPP

#include "skills/Skill.hpp"

#include <optional>
#include <string>
#include <vector>

namespace koocadcam::cam {

// One magazine pocket: a physical tool with a machine T-number.
struct ToolEntry
{
    int         t_number = 0;         // magazine pocket (T1, T2, …)
    std::string family;               // "drill" | "end_mill" | "ball_mill" | "tap" | …
    double      dia_mm = 0.0;
    std::string material = "carbide"; // "hss" | "carbide" | …
    int         flute_count = 0;
    std::string label;                // human label for the (TOOLCHANGE) comment
};

// The default magazine — a compact, realistic spread of the tools slice-1 skills
// ask for (drills, end mills, ball mills, spot/chamfer, tap, thread mill, reamer).
// Deterministic and static so a program's T-numbers are stable across runs.
const std::vector<ToolEntry>& defaultToolLibrary();

// Family aliases → the canonical magazine family, so a skill's tool_type
// ("spot_face_cutter", "counterbore_tool", "ballnose", …) still finds a pocket.
// Returns the canonical family, or "" if the type is not machinable here.
std::string canonicalToolFamily(const std::string& tool_type);

// Pick the closest magazine pocket for a skill's tooling requirement.  Matching:
//   1. the family must agree (a drill is never a valid end mill),
//   2. among that family, the pocket with the smallest diameter error wins,
//   3. a diameter error over kToolDiaTolerance_mm (or > 40 % of the request) FAILS.
// Returns std::nullopt when no pocket qualifies — the caller keeps the descriptive
// (TOOLCHANGE) comment and does NOT emit a T/M6 (better no swap than a wrong tool).
std::optional<ToolEntry> selectTool(const skill::ToolingMeta& tm,
                                    const std::vector<ToolEntry>& library = defaultToolLibrary());

// Absolute diameter tolerance for a magazine match (mm).  A request within this of
// a pocket, OR within 40 % of the requested diameter, is accepted (whichever is
// looser) — real magazines are coarse and a slightly-off endmill still cuts.
inline constexpr double kToolDiaTolerance_mm = 1.0;

}  // namespace koocadcam::cam

#endif  // KOOCADCAM_CAM_TOOLLIBRARY_HPP
