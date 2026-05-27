// @lat: [[engine/reverse-route#Recognizer]]

#include "Recognizer.hpp"

#include "process/StepInvocation.hpp"

#include "skills/bore_cylindrical.hpp"
#include "skills/bore_with_shelf.hpp"
#include "skills/chamfer_edge.hpp"
#include "skills/counterbore.hpp"
#include "skills/countersink.hpp"
#include "skills/drill_hole.hpp"
#include "skills/fillet_edge.hpp"
#include "skills/hollow_cavity.hpp"
#include "skills/mill_circular_pocket.hpp"
#include "skills/mill_rect_pocket.hpp"
#include "skills/mill_slot.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <string>
#include <string_view>
#include <unordered_set>

namespace koocadcam::re {

using nlohmann::json;

// ── Recognizer registry ──────────────────────────────────────────────────
//
// Each entry wraps one skill's recognize() into the uniform RecognizeFn
// signature.  Slice-1 lists the eleven recognizers whose round-trip
// behaviour is documented.  Adding a new skill is a one-line entry once
// its recognize() has acceptable test coverage.

namespace {

std::vector<RecognizeFn> buildRegistry()
{
    std::vector<RecognizeFn> reg;
    // Stock removal first (informational only — registry order is independent
    // of plan order; plan order is set by inferProcessPlan() below).
    reg.emplace_back(&skill::hollow_cavity::recognize);
    reg.emplace_back(&skill::mill_circular_pocket::recognize);
    reg.emplace_back(&skill::mill_rect_pocket::recognize);
    reg.emplace_back(&skill::mill_slot::recognize);
    // Holes & drills.
    reg.emplace_back(&skill::drill_hole::recognize);
    reg.emplace_back(&skill::counterbore::recognize);
    reg.emplace_back(&skill::countersink::recognize);
    reg.emplace_back(&skill::bore_cylindrical::recognize);
    reg.emplace_back(&skill::bore_with_shelf::recognize);
    // Edge operations.
    reg.emplace_back(&skill::fillet_edge::recognize);
    reg.emplace_back(&skill::chamfer_edge::recognize);
    return reg;
}

// ── Skill-id grouping ────────────────────────────────────────────────────
//
// Used by inferProcessPlan() to bucket candidates by machining stage.
// Substring/prefix matching keeps it stable as new skill IDs are added
// under existing families.

enum class Group { A_Stock = 0, B_Hole = 1, C_Edge = 2, Unknown = 3 };

Group classify(std::string_view skillId)
{
    static const std::string_view kStock[] = {
        "hollow_cavity",
        "mill_open_pocket",
        "profile_milling",
        "mill_rect_pocket",
        "mill_circular_pocket",
        "mill_slot",
        "mill_keyway",
        "dovetail_slot",
        "T_slot",
    };
    for (auto s : kStock) {
        if (skillId == s) return Group::A_Stock;
    }

    if (skillId.find("drill") == 0) return Group::B_Hole;
    if (skillId.find("bore")  == 0) return Group::B_Hole;
    static const std::string_view kHole[] = {
        "counterbore",
        "countersink",
        "spot_drill",
        "ream",
        "pocket_with_corner_relief",
    };
    for (auto s : kHole) {
        if (skillId == s) return Group::B_Hole;
    }

    static const std::string_view kEdge[] = {
        "chamfer_edge",
        "fillet_edge",
        "face_milling",
    };
    for (auto s : kEdge) {
        if (skillId == s) return Group::C_Edge;
    }

    return Group::Unknown;
}

}  // namespace

const std::vector<RecognizeFn>& recognizerRegistry()
{
    static const std::vector<RecognizeFn> kRegistry = buildRegistry();
    return kRegistry;
}

// ── analyze() ────────────────────────────────────────────────────────────

std::vector<skill::RecognizedFeature> analyze(const skill::Workpiece& wp)
{
    std::vector<skill::RecognizedFeature> all;
    for (const auto& fn : recognizerRegistry()) {
        try {
            auto cands = fn(wp);
            all.insert(all.end(), cands.begin(), cands.end());
        } catch (const std::exception& e) {
            spdlog::warn("re::analyze: a recognizer threw: {}", e.what());
        } catch (...) {
            spdlog::warn("re::analyze: a recognizer threw an unknown exception");
        }
    }
    std::stable_sort(all.begin(), all.end(),
        [](const skill::RecognizedFeature& a, const skill::RecognizedFeature& b) {
            return a.confidence > b.confidence;
        });
    return all;
}

std::vector<skill::RecognizedFeature>
analyzeFiltered(const skill::Workpiece& wp, double min_confidence)
{
    auto all = analyze(wp);
    all.erase(std::remove_if(all.begin(), all.end(),
        [min_confidence](const skill::RecognizedFeature& c) {
            return c.confidence < min_confidence;
        }), all.end());
    return all;
}

// ── dedupe() ─────────────────────────────────────────────────────────────
//
// Fingerprint extraction: walk matched_geometry and collect every integer
// face ID it references.  Two candidates "overlap" iff their face-ID sets
// intersect.  We greedily process candidates in confidence-descending
// order, keeping the first and dropping any subsequent candidate whose
// face-ID set intersects an already-kept set.
//
// Two skills may legitimately claim the same cylindrical face (e.g.
// drill_hole and bore_cylindrical, or drill_hole and mill_circular_pocket
// for borderline aspect ratios).  Slice-1 picks the higher-raw-confidence
// candidate; resolving "drill vs. pocket" with a smarter heuristic
// (depth/diameter ratio, surrounding context) is a future improvement.

namespace {

// Recursively extract integer face IDs from matched_geometry JSON.  Keys
// we care about: any field whose name contains "face_id" (singular or
// plural).  Values can be either a single integer or an array of integers.
void collectFaceIds(const json& mg, std::unordered_set<int>& out)
{
    if (mg.is_object()) {
        for (auto it = mg.begin(); it != mg.end(); ++it) {
            const std::string key = it.key();
            // Heuristic: any key mentioning "face_id" or "face_ids"
            // contributes integer IDs.
            const bool isFaceKey =
                key.find("face_id")  != std::string::npos ||
                key.find("face_ids") != std::string::npos ||
                key.find("cylinder_ids") != std::string::npos ||
                key.find("cyl_face")  != std::string::npos;
            if (isFaceKey) {
                if (it.value().is_number_integer()) {
                    out.insert(it.value().get<int>());
                } else if (it.value().is_array()) {
                    for (const auto& v : it.value()) {
                        if (v.is_number_integer()) out.insert(v.get<int>());
                    }
                }
            } else if (it.value().is_object() || it.value().is_array()) {
                collectFaceIds(it.value(), out);
            }
        }
    } else if (mg.is_array()) {
        for (const auto& v : mg) collectFaceIds(v, out);
    }
}

bool intersects(const std::unordered_set<int>& a, const std::unordered_set<int>& b)
{
    // Iterate over the smaller set for efficiency.
    const auto& small = (a.size() < b.size()) ? a : b;
    const auto& large = (a.size() < b.size()) ? b : a;
    for (int v : small) {
        if (large.count(v)) return true;
    }
    return false;
}

}  // namespace

std::vector<skill::RecognizedFeature>
dedupe(const std::vector<skill::RecognizedFeature>& candidates)
{
    // Work on a confidence-sorted copy (descending).
    std::vector<skill::RecognizedFeature> sorted = candidates;
    std::stable_sort(sorted.begin(), sorted.end(),
        [](const skill::RecognizedFeature& a, const skill::RecognizedFeature& b) {
            return a.confidence > b.confidence;
        });

    std::vector<skill::RecognizedFeature> kept;
    std::vector<std::unordered_set<int>> keptFaceSets;

    for (const auto& c : sorted) {
        std::unordered_set<int> fids;
        collectFaceIds(c.matched_geometry, fids);

        if (fids.empty()) {
            // No identifiable geometry fingerprint → conservatively keep,
            // because we cannot prove overlap.
            kept.push_back(c);
            keptFaceSets.push_back(std::move(fids));
            continue;
        }
        bool overlap = false;
        for (const auto& kf : keptFaceSets) {
            if (intersects(fids, kf)) { overlap = true; break; }
        }
        if (overlap) continue;
        kept.push_back(c);
        keptFaceSets.push_back(std::move(fids));
    }
    return kept;
}

// ── inferProcessPlan() ───────────────────────────────────────────────────
//
// Buckets the de-duplicated candidates by Group A/B/C using the classify()
// helper above and appends each bucket to the ProcessPlan in stock-removal →
// hole → edge order.

process::ProcessPlan
inferProcessPlan(const skill::Workpiece& wp, double min_confidence)
{
    auto candidates = analyzeFiltered(wp, min_confidence);
    candidates      = dedupe(candidates);

    // Bucket by group.
    std::vector<skill::RecognizedFeature> buckets[4];   // A, B, C, Unknown
    for (auto& c : candidates) {
        const Group g = classify(c.skill_id);
        buckets[static_cast<int>(g)].push_back(std::move(c));
    }

    // Within each bucket, confidence is already descending (we sorted in
    // analyze()/dedupe()).  Make it explicit for safety.
    for (auto& b : buckets) {
        std::stable_sort(b.begin(), b.end(),
            [](const skill::RecognizedFeature& a, const skill::RecognizedFeature& b2) {
                return a.confidence > b2.confidence;
            });
    }

    process::ProcessPlan plan;

    auto appendBucket = [&plan](const std::vector<skill::RecognizedFeature>& bucket,
                                const char* groupName) {
        for (const auto& c : bucket) {
            process::StepInvocation step;
            step.skill_id   = c.skill_id;
            step.params     = c.recovered_params;
            step.depends_on = {};   // slice-1: linear; topological deps TODO.
            step.note       = std::string("re::inferProcessPlan group=") +
                              groupName + " conf=" + std::to_string(c.confidence);
            plan.append(step);
        }
    };

    appendBucket(buckets[static_cast<int>(Group::A_Stock)], "A_stock_removal");
    appendBucket(buckets[static_cast<int>(Group::B_Hole)],  "B_hole");
    appendBucket(buckets[static_cast<int>(Group::C_Edge)],  "C_edge");
    // Unknown skill_ids are appended LAST (between edge ops and end) so
    // their inclusion isn't silently dropped; in practice the registry is
    // controlled so this bucket should stay empty.
    appendBucket(buckets[static_cast<int>(Group::Unknown)], "Unknown");

    return plan;
}

}  // namespace koocadcam::re
