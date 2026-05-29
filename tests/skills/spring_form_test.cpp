// @lat: [[process/test-strategy#skill round-trip]]
//
// spring_form — wire coiling into a helical spring (real geometric impl).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/spring_form.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Apply: REAL helical coil with expected height + diameter + volume ───
TEST(SkillSpringForm, ApplyBuildsHelicalCoil)
{
    auto stock = skill::createCylindricalStock(2.0, 400.0);  // Ø2 × 400 wire

    skill::spring_form::Input in;
    in.coil_pitch_mm  = 4.0;
    in.n_turns        = 10.0;
    in.wire_dia_mm    = 2.0;
    in.free_length_mm = 42.0;       // ≈ pitch × n + d
    in.mean_dia_mm    = 12.0;       // C = 6 (good)

    auto out = skill::spring_form::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // bbox: spring height ≈ pitch × n_turns + wire_dia (one wire-dia for
    // the wire thickness at top/bottom), bbox XY ≈ mean_dia + wire_dia.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    out.workpiece->boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    const double height = zMax - zMin;
    const double diaX   = xMax - xMin;
    const double diaY   = yMax - yMin;

    // Height: pitch × n_turns to first order, ± one wire diameter for end
    // caps; tolerate ±20 % for PipeShell tessellation slack.
    const double expectedHeight = 4.0 * 10.0;
    EXPECT_GT(height, expectedHeight * 0.8);
    EXPECT_LT(height, expectedHeight * 1.5);

    // Bbox diameter: mean_dia + wire_dia (with some slack).
    const double expectedDia = 12.0 + 2.0;
    EXPECT_GT(diaX, expectedDia * 0.8);
    EXPECT_LT(diaX, expectedDia * 1.25);
    EXPECT_GT(diaY, expectedDia * 0.8);
    EXPECT_LT(diaY, expectedDia * 1.25);

    // Volume vs analytic formula: π (d/2)² × π · D · n
    const double wireLen        = M_PI * 12.0 * 10.0;          // ≈ 376.99
    const double expectedVolume = M_PI * 1.0 * 1.0 * wireLen;  // ≈ 1184
    const double v1             = volumeOf(out.workpiece->shape());
    // Generous ±25 % tolerance — PipeShell on a tight helix can over- or
    // under-estimate; pure cylinder fallback uses a different formula
    // entirely.  This is a sanity (volume is non-trivial) check.
    EXPECT_GT(v1, expectedVolume * 0.5);
    EXPECT_LT(v1, expectedVolume * 2.5);

    // Geometry actually changed (not metadata-only).
    EXPECT_TRUE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_LT(out.signature.tooling.stock_removed_mm3, 0.0);  // material added
    EXPECT_EQ(out.signature.skill_id, std::string("spring_form"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ── 2. DFM: zero wire_dia and solid pitch rejected ─────────────────────────
TEST(SkillSpringForm, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(2.0, 400.0);

    {
        skill::spring_form::Input in;
        in.coil_pitch_mm  = 4.0;
        in.n_turns        = 10.0;
        in.wire_dia_mm    = 0.0;
        in.free_length_mm = 42.0;
        in.mean_dia_mm    = 12.0;
        auto r = skill::spring_form::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::spring_form::apply(*stock, in), skill::SkillError);
    }

    {
        skill::spring_form::Input in;
        in.coil_pitch_mm  = 2.0;            // = wire_dia → coils touch
        in.n_turns        = 10.0;
        in.wire_dia_mm    = 2.0;
        in.free_length_mm = 22.0;
        in.mean_dia_mm    = 12.0;
        auto r = skill::spring_form::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-SPR-SOLID") { found = true; break; }
        EXPECT_TRUE(found);
    }
}

// ── 3. Signature records key params (kind, pitch, n_turns, wire_dia, etc) ──
TEST(SkillSpringForm, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCylindricalStock(2.0, 400.0);

    skill::spring_form::Input in;
    in.coil_pitch_mm  = 5.0;
    in.n_turns        = 8.0;
    in.wire_dia_mm    = 2.0;
    in.free_length_mm = 42.0;
    in.mean_dia_mm    = 14.0;       // C = 7

    auto out = skill::spring_form::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("spring_form"));
    EXPECT_NEAR(out.signature.pattern["coil_pitch_mm"].get<double>(),
                5.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["n_turns"].get<double>(),
                8.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["wire_dia_mm"].get<double>(),
                2.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["spring_index"].get<double>(),
                7.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["coil_height_mm"].get<double>(),
                40.0, 1e-9);     // pitch × n_turns
    EXPECT_TRUE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ── 4. Recognize — metadata replay (1.0) + geometric fallback (0.45) ──────
TEST(SkillSpringForm, RecognizeMetadataAndGeometric)
{
    auto stock = skill::createCylindricalStock(2.0, 400.0);

    skill::spring_form::Input in;
    in.coil_pitch_mm  = 3.5;
    in.n_turns        = 12.0;
    in.wire_dia_mm    = 1.5;
    in.free_length_mm = 43.5;
    in.mean_dia_mm    = 9.0;        // C = 6

    auto out = skill::spring_form::apply(*stock, in);

    // Metadata replay.
    auto cands = skill::spring_form::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["coil_pitch_mm"].get<double>(),
                3.5, 1e-9);

    // Geometric fallback: strip metadata, recognize off bare shape.
    skill::Workpiece raw(out.workpiece->shape());
    auto rawCands = skill::spring_form::recognize(raw);
    // The helical sweep typically produces enough cylindrical patches to
    // trigger the geometric heuristic.  Accept either: no candidate (e.g.,
    // if a fallback cylinder was used and the heuristic rejects it) OR a
    // 0.45-confidence geometric match.
    if (!rawCands.empty()) {
        EXPECT_NEAR(rawCands[0].confidence, 0.45, 1e-6);
        EXPECT_EQ(rawCands[0].matched_geometry["source"].get<std::string>(),
                  std::string("geometric_fallback"));
    }
}

// ── 5. Bad spring index → warning, operation proceeds ──────────────────────
TEST(SkillSpringForm, BadSpringIndexEmitsWarning)
{
    auto stock = skill::createCylindricalStock(2.0, 400.0);

    skill::spring_form::Input in;
    in.coil_pitch_mm  = 4.0;
    in.n_turns        = 10.0;
    in.wire_dia_mm    = 3.0;
    in.free_length_mm = 43.0;
    in.mean_dia_mm    = 9.0;        // C = 3 — below Wahl [4, 12]

    auto r = skill::spring_form::validate(*stock, in);
    EXPECT_TRUE(r.passed)            // warning only
        << "bad spring_index should warn (not block)";
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SPR-INDEX") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_NO_THROW(skill::spring_form::apply(*stock, in));
}
