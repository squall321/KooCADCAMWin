// @lat: [[engine/cam-3axis-verify]]

#include "ToolLibrary.hpp"

#include <algorithm>
#include <cmath>

namespace koocadcam::cam {

// A compact, deterministic magazine.  T-numbers are stable pocket assignments so a
// program's tool changes are reproducible.  Diameters are common metric stock; the
// selector snaps a skill's requested diameter onto the nearest pocket of the right
// family.  This is intentionally small — it is the DEFAULT magazine, not an
// exhaustive catalog; a real machine's library would be injected by the caller.
const std::vector<ToolEntry>& defaultToolLibrary()
{
    static const std::vector<ToolEntry> lib = {
        // drills
        { 1,  "drill",       3.0,  "carbide", 2, "drill 3mm carbide" },
        { 2,  "drill",       5.0,  "carbide", 2, "drill 5mm carbide" },
        { 3,  "drill",       8.0,  "carbide", 2, "drill 8mm carbide" },
        // end mills (flat)
        { 4,  "end_mill",    3.0,  "carbide", 4, "end mill 3mm 4fl" },
        { 5,  "end_mill",    6.0,  "carbide", 4, "end mill 6mm 4fl" },
        { 6,  "end_mill",   10.0,  "carbide", 4, "end mill 10mm 4fl" },
        { 7,  "end_mill",   12.0,  "carbide", 4, "end mill 12mm 4fl" },
        // ball mills (finishing)
        { 8,  "ball_mill",   3.0,  "carbide", 2, "ball mill 3mm 2fl" },
        { 9,  "ball_mill",   6.0,  "carbide", 2, "ball mill 6mm 2fl" },
        // spot / chamfer (spot_face, countersink, spot-drill)
        { 10, "spot_drill",  8.0,  "carbide", 2, "spot/chamfer 8mm" },
        { 11, "spot_drill", 12.0,  "carbide", 2, "spot/chamfer 12mm" },
        // tap + thread mill
        { 12, "tap",         5.0,  "hss",     3, "tap M5" },
        { 13, "thread_mill", 6.0,  "carbide", 3, "thread mill 6mm" },
        // reamer (ream skill)
        { 14, "reamer",      6.0,  "carbide", 6, "reamer 6mm H7" },
    };
    return lib;
}

std::string canonicalToolFamily(const std::string& tool_type)
{
    // Many skills stamp a COMPOUND, ';'-joined tool_type ("end_mill;drill",
    // "drill;tap;counterbore", …) listing every op of a multi-step feature in the
    // order they run — the FIRST token is the primary/leading operation.  A naive
    // substring scan over the whole string lets an incidental later token (a stray
    // "drill"/"tap"/"thread" inside a milled-primary compound) steal the family and
    // pick a physically wrong tool.  So canonicalise ONLY the primary token: split
    // on ';' and match the first segment alone.
    std::string primary = tool_type;
    const auto semi = primary.find(';');
    if (semi != std::string::npos) primary = primary.substr(0, semi);

    auto has = [&](const char* sub) { return primary.find(sub) != std::string::npos; };

    if (has("ball"))                       return "ball_mill";   // "ball_mill", "ballnose"
    if (has("thread_mill") || has("threadmill")) return "thread_mill";
    if (has("tap"))                        return "tap";
    if (has("ream"))                       return "reamer";
    if (has("spot") || has("countersink") || has("chamfer") || has("center_drill"))
        return "spot_drill";               // spot_face_cutter, countersink, chamfer_mill
    if (has("drill"))                      return "drill";       // drill/gun_drill/micro_drill
    if (has("end_mill") || has("endmill") || has("flat") || has("mill") || has("cutter"))
        return "end_mill";                 // generic milling cutter (box/slot/pocket)
    return "";
}

std::optional<ToolEntry> selectTool(const skill::ToolingMeta& tm,
                                    const std::vector<ToolEntry>& library)
{
    const std::string family = canonicalToolFamily(tm.tool_type);
    if (family.empty()) return std::nullopt;
    if (tm.tool_dia_mm <= 0.0) return std::nullopt;

    const ToolEntry* best = nullptr;
    double bestErr = 0.0;
    for (const auto& e : library) {
        if (e.family != family) continue;
        const double err = std::abs(e.dia_mm - tm.tool_dia_mm);
        if (best == nullptr || err < bestErr) { best = &e; bestErr = err; }
    }
    if (best == nullptr) return std::nullopt;

    // Accept only when the pocket is genuinely the right size.  The tolerance is a
    // RELATIVE band (a pocket within 25 % of the request cuts a close-enough feature)
    // plus a SMALL absolute grace for the coarse magazine — but the grace is capped
    // at 20 % of the request so it can NEVER admit a gross oversize on a small tool:
    // a 2 mm request can no longer resolve to the 3 mm pocket (err 1.0 > tol 0.5).
    // A pocket outside the band FAILS → the caller keeps the descriptive comment and
    // does not machine with a silently-wrong tool.
    const double relTol   = 0.25 * tm.tool_dia_mm;
    const double absGrace = std::min(kToolDiaTolerance_mm, 0.20 * tm.tool_dia_mm);
    if (bestErr > std::max(relTol, absGrace)) return std::nullopt;
    return *best;
}

}  // namespace koocadcam::cam
