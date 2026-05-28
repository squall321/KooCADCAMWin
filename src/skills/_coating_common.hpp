#pragma once
// @lat: [[engine/skills#coating / surface treatment — shared tables]]
//
// **Shared internal helpers, not a public primitive.**
//
// Used ONLY by the five COATING / SURFACE TREATMENT skills:
//   - anodize        (aluminum oxide layer, 5 – 75 μm)
//   - electroplate   (electrochemical metal deposition, 1 – 200 μm)
//   - paint_op       (liquid paint, 10 – 500 μm)
//   - powder_coat    (electrostatic powder, 50 – 300 μm)
//   - pvd_coat       (physical vapor deposition, 0.5 – 10 μm)
//
// All five skills:
//   - Add a thin coating to a target face WITHOUT changing OCCT geometry.
//   - Record (face_id, finish, thickness, color, …) into the FeatureSignature.
//   - Recognize() ONLY by replaying signatures from `workpiece.features()` —
//     a coating leaves no macroscopic B-rep trace.
//
// Anyone tempted to reach for these tables from a different skill family
// should instead consider promoting the relevant entry into a public table.
// These helpers are NOT stabilised contracts.
//
// Provided here:
//   - Anodize color list with RGB hints
//   - Plating metal compatibility table (info-only DFM hints)
//   - Powder-coat texture options
//   - PVD compound hardness ranges (Vickers HV)
//   - `buildCoatingNoOp()` — common no-op workpiece clone + sig stamp helper
//   - `recognizeFromMetadata()` — replay signatures filtered by skill_id

#include "DFM.hpp"
#include "Skill.hpp"
#include "Workpiece.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <memory>
#include <string>
#include <vector>

namespace koocadcam::skill::coating_common {

// ── RGB hint ─────────────────────────────────────────────────────────────
struct RgbHint
{
    int r;  // 0 – 255
    int g;
    int b;
};

// ── Anodize colors ───────────────────────────────────────────────────────
//
// "Natural" is the bare aluminum oxide (silvery / clear).  Other colors
// are achieved by absorbing organic / inorganic dyes into the porous oxide
// before sealing.  Colors below are typical Type II anodize references —
// real shade varies with bath chemistry / dye concentration / Al alloy.
struct AnodizeColorEntry
{
    const char* name;
    RgbHint     rgb;
};

inline const std::vector<AnodizeColorEntry>& anodizeColors()
{
    static const std::vector<AnodizeColorEntry> table = {
        { "natural",   { 200, 200, 200 } },   // clear / silvery aluminum oxide
        { "clear",     { 210, 210, 210 } },   // sealed clear (slight tone)
        { "black",     {  20,  20,  20 } },
        { "gold",      { 200, 160,  60 } },
        { "bronze",    { 130,  90,  50 } },
        { "red",       { 170,  30,  30 } },
        { "blue",      {  30,  60, 170 } },
        { "green",     {  40, 130,  60 } },
        { "purple",    { 110,  60, 150 } },
        { "orange",    { 220, 120,  40 } },
    };
    return table;
}

inline bool isKnownAnodizeColor(const std::string& color)
{
    if (color.empty()) return false;
    const auto& t = anodizeColors();
    return std::any_of(t.begin(), t.end(),
        [&](const AnodizeColorEntry& e) { return e.name == color; });
}

inline RgbHint anodizeRgbHint(const std::string& color)
{
    const auto& t = anodizeColors();
    for (const auto& e : t)
        if (e.name == color) return e.rgb;
    return { 200, 200, 200 };  // fallback = natural
}

// ── Anodize seal types ───────────────────────────────────────────────────
//
// Sealing closes the porous oxide and locks in the dye.  The three classic
// industrial choices:
//   "hot_water"        — boil in DI water (~95 °C), low-cost, lower abrasion R
//   "nickel_acetate"   — mid-temp Ni acetate bath, best corrosion resistance
//   "none"             — unsealed (used as substrate for further coating)
inline bool isKnownAnodizeSeal(const std::string& seal)
{
    return seal == "hot_water"
        || seal == "nickel_acetate"
        || seal == "none";
}

// ── Aluminum-family check (info-only DFM) ────────────────────────────────
//
// Anodize works on aluminum and (poorly) on titanium / magnesium.  This
// is an info-only check — Workpiece::material() is a free-form string and
// we cannot block a process plan on it.
inline bool isAluminumFamily(const std::string& material)
{
    // Cheap substring check — covers "aluminum_6061", "aluminum-7075",
    // "Al6061", "aluminium" (British spelling), etc.
    auto lc = material;
    std::transform(lc.begin(), lc.end(), lc.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return lc.find("alumin") != std::string::npos
        || lc.find("al6") != std::string::npos
        || lc.find("al7") != std::string::npos
        || lc.find("al-") != std::string::npos;
}

// ── Plating-base compatibility table ─────────────────────────────────────
//
// Each entry: plating metal × base material families it bonds well to.
// Used for info-only DFM hints — none of these block the process.
struct PlatingCompatibility
{
    const char*              plating_metal;
    std::vector<const char*> good_bases;       // substring matches
    std::vector<const char*> requires_strike;  // intermediate plate needed
    double                   typical_thickness_um;
    double                   hardness_hv;      // approx HV of deposit
};

inline const std::vector<PlatingCompatibility>& platingTable()
{
    static const std::vector<PlatingCompatibility> table = {
        { "nickel",
          { "steel", "copper", "brass", "aluminum" },
          { "aluminum" },             // zincate strike on Al
          15.0, 500.0 },
        { "chrome",
          { "steel", "nickel" },
          { "aluminum", "zinc", "brass" },  // nickel underplate typical
          5.0,  900.0 },
        { "zinc",
          { "steel", "iron" },
          {},
          10.0, 50.0 },
        { "copper",
          { "steel", "aluminum", "brass" },
          { "aluminum" },
          20.0, 60.0 },
        { "gold",
          { "nickel", "copper", "brass" },
          { "steel", "aluminum" },   // nickel strike typical
          2.5,  140.0 },
    };
    return table;
}

inline const PlatingCompatibility* findPlating(const std::string& metal)
{
    const auto& t = platingTable();
    for (const auto& e : t)
        if (e.plating_metal == metal) return &e;
    return nullptr;
}

inline bool isKnownPlatingMetal(const std::string& metal)
{
    return findPlating(metal) != nullptr;
}

// "good" if the base material's lowercased string contains a substring from
// the entry's good_bases list.  "needs_strike" if instead it matches a
// requires_strike entry.  Otherwise unknown.
enum class PlatingFit { Good, NeedsStrike, Unknown };

inline PlatingFit classifyPlatingFit(const PlatingCompatibility& pc,
                                     const std::string& base)
{
    std::string lc = base;
    std::transform(lc.begin(), lc.end(), lc.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    for (const char* g : pc.good_bases)
        if (lc.find(g) != std::string::npos) return PlatingFit::Good;
    for (const char* s : pc.requires_strike)
        if (lc.find(s) != std::string::npos) return PlatingFit::NeedsStrike;
    return PlatingFit::Unknown;
}

// ── Powder-coat texture options ──────────────────────────────────────────
//
// "smooth"   — glossy (gloss-meter ~ 80+ GU), pre-bake levelled
// "matte"    — low gloss (10 – 30 GU), matting agents added
// "wrinkle"  — controlled-shrink (visible wrinkle texture)
// "metallic" — flake-bearing for metallic sheen (silver / champagne / etc.)
inline bool isKnownPowderTexture(const std::string& tex)
{
    return tex == "smooth"
        || tex == "matte"
        || tex == "wrinkle"
        || tex == "metallic";
}

// ── PVD compound hardness ranges (Vickers HV) ────────────────────────────
//
// All values approximate industry refs — the deposit hardness depends on
// bath chemistry, substrate, deposition pressure, etc.  Ranges are used
// for info-only DFM hints (if the caller's reported hardness_hv is far
// outside the expected window, emit a warning).
struct PvdCompoundEntry
{
    const char* compound;          // "TiN" | "TiAlN" | "CrN" | "DLC"
    double      hardness_min_hv;
    double      hardness_max_hv;
    RgbHint     color_hint;        // typical visual tone
};

inline const std::vector<PvdCompoundEntry>& pvdCompounds()
{
    static const std::vector<PvdCompoundEntry> table = {
        { "TiN",   2000.0, 2500.0, { 200, 160,  40 } },   // characteristic gold
        { "TiAlN", 2800.0, 3500.0, { 100,  90,  80 } },   // dark violet-grey
        { "CrN",   1700.0, 2200.0, { 180, 180, 180 } },   // silver-grey
        { "DLC",   1500.0, 4000.0, {  30,  30,  30 } },   // near-black (diamond-like carbon)
    };
    return table;
}

inline const PvdCompoundEntry* findPvdCompound(const std::string& compound)
{
    const auto& t = pvdCompounds();
    for (const auto& e : t)
        if (e.compound == compound) return &e;
    return nullptr;
}

inline bool isKnownPvdCompound(const std::string& compound)
{
    return findPvdCompound(compound) != nullptr;
}

// ── No-op coating apply helper ──────────────────────────────────────────
//
// All five coating skills share the same "no-op + signature stamp" shape:
//   1. resolve the entry face datum (after DFM passed)
//   2. capture the face area (for tooling.stock_removed_mm3 = 0 and an
//      auxiliary metadata anchor)
//   3. clone the workpiece carrying the original shape and material
//   4. preserve prior features
//   5. stamp the new signature
//
// Per-skill apply() fills the `params` / `pattern` / `tooling` JSON before
// calling this helper.
struct StampResult
{
    std::shared_ptr<Workpiece> workpiece;
    FeatureSignature           signature;
    double                     entry_face_area_mm2;
};

inline StampResult stampNoOpCoating(const Workpiece& wp,
                                    int entry_face_id,
                                    FeatureSignature sig)
{
    GProp_GProps props;
    BRepGProp::SurfaceProperties(wp.face(entry_face_id), props);
    const double faceArea = props.Mass();

    // Add the area into the pattern so a downstream consumer can re-locate
    // the coated face by area + face_id after STEP round-trip.
    if (sig.pattern.is_object()) {
        if (!sig.pattern.contains("target_face_area_mm2"))
            sig.pattern["target_face_area_mm2"] = faceArea;
        if (!sig.pattern.contains("target_face_id"))
            sig.pattern["target_face_id"] = entry_face_id;
        if (!sig.pattern.contains("geometry_changed"))
            sig.pattern["geometry_changed"] = false;
    }

    auto wpNew = std::make_shared<Workpiece>(wp.shape(), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);

    return StampResult{ wpNew, sig, faceArea };
}

// ── Metadata-only recognize helper ───────────────────────────────────────
inline std::vector<RecognizedFeature> recognizeFromMetadata(
    const Workpiece& wp, const std::string& target_skill_id)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != target_skill_id) continue;
        RecognizedFeature rf;
        rf.skill_id         = target_skill_id;
        rf.recovered_params = f.params;
        rf.confidence       = 1.0;
        rf.matched_geometry = { { "source", "metadata" } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::coating_common
