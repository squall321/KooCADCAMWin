// @lat: [[process/test-strategy#skill round-trip]]
//
// spring_form — wire coiling into a helical spring (metadata-only).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/spring_form.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. ApplyHandlesInput — wire stock + valid coil parameters ──────────────
TEST(SkillSpringForm, ApplyHandlesInput)
{
    // "Stock" is a stand-in wire segment; geometry passes through.
    auto stock = skill::createCylindricalStock(2.0, 200.0);  // Ø2 × 200 wire
    const double v0 = volumeOf(stock->shape());

    skill::spring_form::Input in;
    in.coil_pitch_mm  = 4.0;
    in.n_turns        = 10.0;
    in.wire_dia_mm    = 2.0;
    in.free_length_mm = 42.0;       // ≈ pitch × n + d
    in.mean_dia_mm    = 12.0;       // C = 6 (good)

    auto out = skill::spring_form::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), v0, 1e-9);  // metadata-only
    EXPECT_EQ(out.signature.skill_id, std::string("spring_form"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ── 2. ValidateRejectsBadInput — zero wire_dia & solid-coil pitch ──────────
TEST(SkillSpringForm, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(2.0, 200.0);

    // Case A: wire_dia_mm = 0 (DFM-INPUT).
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

    // Case B: pitch ≤ wire_dia (DFM-SPR-SOLID).
    {
        skill::spring_form::Input in;
        in.coil_pitch_mm  = 2.0;
        in.n_turns        = 10.0;
        in.wire_dia_mm    = 2.0;       // touch
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

// ── 3. SignatureRecordsKind + pitch + n_turns + wire_dia + free_length ─────
TEST(SkillSpringForm, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCylindricalStock(2.0, 200.0);

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
    EXPECT_NEAR(out.signature.pattern["free_length_mm"].get<double>(),
                42.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["spring_index"].get<double>(),
                7.0, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ── 4. RecognizeMetadataReplay — exact parameter recovery ──────────────────
TEST(SkillSpringForm, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(2.0, 200.0);

    skill::spring_form::Input in;
    in.coil_pitch_mm  = 3.5;
    in.n_turns        = 12.0;
    in.wire_dia_mm    = 1.5;
    in.free_length_mm = 43.5;
    in.mean_dia_mm    = 9.0;        // C = 6

    auto out = skill::spring_form::apply(*stock, in);
    auto cands = skill::spring_form::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["coil_pitch_mm"].get<double>(),
                3.5, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["n_turns"].get<double>(),
                12.0, 1e-9);

    // Workpiece stripped of metadata — recognize is empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::spring_form::recognize(raw).empty());
}

// ── 5. SPECIFIC: bad spring index (C < 4) → warning, operation proceeds ───
TEST(SkillSpringForm, BadSpringIndexEmitsWarning)
{
    auto stock = skill::createCylindricalStock(2.0, 200.0);

    skill::spring_form::Input in;
    in.coil_pitch_mm  = 4.0;
    in.n_turns        = 10.0;
    in.wire_dia_mm    = 3.0;
    in.free_length_mm = 43.0;
    in.mean_dia_mm    = 9.0;        // C = 3 — too tight

    auto r = skill::spring_form::validate(*stock, in);
    EXPECT_TRUE(r.passed)            // warning only
        << "bad spring_index should warn (not block)";
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SPR-INDEX") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_NO_THROW(skill::spring_form::apply(*stock, in));
}
