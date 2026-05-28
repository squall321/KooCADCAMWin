// @lat: [[process/test-strategy#skill round-trip]]
//
// peen_form — shot-peen forming (metadata-only).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/peen_form.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. ApplyHandlesInput — geometry unchanged, signature stamped ───────────
TEST(SkillPeenForm, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 2.0, "aluminum_7075");
    const double v0 = volumeOf(stock->shape());
    const int f0 = stock->faceCount();

    skill::peen_form::Input in;
    in.peen_face               = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.peening_intensity       = 0.20;
    in.target_curvature_mm_inv = 0.001;        // R = 1000 mm
    in.coverage_pct            = 200.0;

    auto out = skill::peen_form::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), v0, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), f0);
    EXPECT_EQ(out.signature.skill_id, std::string("peen_form"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ── 2. ValidateRejectsBadInput — non-positive intensity & curvature ────────
TEST(SkillPeenForm, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 2.0);

    // Case A: zero intensity.
    {
        skill::peen_form::Input in;
        in.peen_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.peening_intensity       = 0.0;
        in.target_curvature_mm_inv = 0.001;
        auto r = skill::peen_form::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::peen_form::apply(*stock, in), skill::SkillError);
    }

    // Case B: non-positive curvature.
    {
        skill::peen_form::Input in;
        in.peen_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.peening_intensity       = 0.20;
        in.target_curvature_mm_inv = -0.001;
        auto r = skill::peen_form::validate(*stock, in);
        EXPECT_FALSE(r.passed);
    }
}

// ── 3. SignatureRecordsKind + intensity + curvature ────────────────────────
TEST(SkillPeenForm, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 2.0);

    skill::peen_form::Input in;
    in.peen_face               = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.peening_intensity       = 0.25;
    in.target_curvature_mm_inv = 0.0005;        // R = 2000 mm
    in.coverage_pct            = 250.0;

    auto out = skill::peen_form::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("peen_form"));
    EXPECT_NEAR(out.signature.pattern["peening_intensity"].get<double>(),
                0.25, 1e-9);
    EXPECT_NEAR(out.signature.pattern["target_curvature_mm_inv"].get<double>(),
                0.0005, 1e-9);
    EXPECT_NEAR(out.signature.pattern["target_radius_mm"].get<double>(),
                2000.0, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("peen_nozzle"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 4. RecognizeMetadataReplay — exact parameter recovery ──────────────────
TEST(SkillPeenForm, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 2.0);

    skill::peen_form::Input in;
    in.peen_face               = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.peening_intensity       = 0.15;
    in.target_curvature_mm_inv = 0.002;
    in.coverage_pct            = 180.0;

    auto out = skill::peen_form::apply(*stock, in);
    auto cands = skill::peen_form::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["peening_intensity"].get<double>(),
                0.15, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["target_curvature_mm_inv"].get<double>(),
                0.002, 1e-9);

    // Workpiece stripped of metadata — recognize is empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::peen_form::recognize(raw).empty());
}

// ── 5. SPECIFIC: tight curvature → DFM warning but operation proceeds ──────
TEST(SkillPeenForm, TightCurvatureEmitsWarning)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 2.0);

    skill::peen_form::Input in;
    in.peen_face               = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.peening_intensity       = 0.20;
    in.target_curvature_mm_inv = 0.05;        // R = 20 mm — very tight
    in.coverage_pct            = 200.0;

    auto r = skill::peen_form::validate(*stock, in);
    EXPECT_TRUE(r.passed)        // warning, not error
        << "tight curvature should warn (not block)";
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PEEN-CURV") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_NO_THROW(skill::peen_form::apply(*stock, in));
}
