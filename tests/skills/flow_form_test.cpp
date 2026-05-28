// @lat: [[process/test-strategy#skill round-trip]]
//
// flow_form — tube wall reduction with axial elongation.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/flow_form.hpp"
#include "skills/bore_cylindrical.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

// Build a tubular preform for flow_form (Ø_outer = 40, Ø_inner = 30, len = 50).
std::shared_ptr<skill::Workpiece> makeTubePreform()
{
    auto stock = skill::createCylindricalStock(40.0, 50.0);
    skill::bore_cylindrical::Input bin;
    bin.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bin.position_x_mm  = 0.0;
    bin.position_y_mm  = 0.0;
    bin.axis_dir       = gp_Dir(0, 0, -1);
    bin.diameter_mm    = 30.0;
    bin.depth_mm       = 50.0;
    bin.tolerance_class = "H8";
    auto out = skill::bore_cylindrical::apply(*stock, bin);
    return out.workpiece;
}
}  // namespace

// ── 1. ApplyHandlesInput — tube is reformed to thinner wall + longer ───────
TEST(SkillFlowForm, ApplyHandlesInput)
{
    auto tube = makeTubePreform();
    ASSERT_FALSE(tube->shape().IsNull());

    skill::flow_form::Input in;
    in.outer_dia_mm          = 40.0;
    in.original_thickness_mm = 5.0;     // (40-30)/2
    in.final_thickness_mm    = 2.5;     // 50% reduction
    in.original_length_mm    = 50.0;
    in.pass_count            = 2;

    auto out = skill::flow_form::apply(*tube, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GT(volumeOf(out.workpiece->shape()), 0.0);
    EXPECT_EQ(out.signature.skill_id, std::string("flow_form"));
}

// ── 2. ValidateRejectsBadInput — final ≥ original thickness rejected ───────
TEST(SkillFlowForm, ValidateRejectsBadInput)
{
    auto tube = makeTubePreform();

    skill::flow_form::Input in;
    in.outer_dia_mm          = 40.0;
    in.original_thickness_mm = 5.0;
    in.final_thickness_mm    = 6.0;     // INCREASES — invalid
    in.original_length_mm    = 50.0;
    in.pass_count            = 1;

    auto r = skill::flow_form::validate(*tube, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::flow_form::apply(*tube, in), skill::SkillError);

    // Also: max 80% reduction in single sequence.
    skill::flow_form::Input in2;
    in2.outer_dia_mm          = 40.0;
    in2.original_thickness_mm = 5.0;
    in2.final_thickness_mm    = 0.5;    // 90% reduction — invalid
    in2.original_length_mm    = 50.0;
    in2.pass_count            = 1;

    auto r2 = skill::flow_form::validate(*tube, in2);
    bool foundMin = false;
    for (const auto& f : r2.findings)
        if (f.code == "DFM-FF-MIN") { foundMin = true; break; }
    EXPECT_TRUE(foundMin);
}

// ── 3. SignatureRecordsKind + original/final thickness + elongation ────────
TEST(SkillFlowForm, SignatureRecordsKindAndKeyParams)
{
    auto tube = makeTubePreform();

    skill::flow_form::Input in;
    in.outer_dia_mm          = 40.0;
    in.original_thickness_mm = 5.0;
    in.final_thickness_mm    = 2.5;
    in.original_length_mm    = 50.0;
    in.pass_count            = 2;

    auto out = skill::flow_form::apply(*tube, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("flow_form"));
    EXPECT_NEAR(out.signature.pattern["original_thickness_mm"].get<double>(),
                5.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["final_thickness_mm"].get<double>(),
                2.5, 1e-9);
    // Volume preservation: t halved → length doubled.
    EXPECT_NEAR(out.signature.pattern["final_length_mm"].get<double>(),
                100.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["elongation_pct"].get<double>(),
                100.0, 1e-3);
}

// ── 4. RecognizeMetadataReplay — exact parameter recovery ──────────────────
TEST(SkillFlowForm, RecognizeMetadataReplay)
{
    auto tube = makeTubePreform();

    skill::flow_form::Input in;
    in.outer_dia_mm          = 40.0;
    in.original_thickness_mm = 5.0;
    in.final_thickness_mm    = 3.0;
    in.original_length_mm    = 50.0;
    in.pass_count            = 1;

    auto out = skill::flow_form::apply(*tube, in);
    auto cands = skill::flow_form::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["original_thickness_mm"].get<double>(),
                5.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["final_thickness_mm"].get<double>(),
                3.0, 1e-9);
}

// ── 5. SPECIFIC: solid bar (no inner Ø) rejected by DFM-FF-TUBE ────────────
TEST(SkillFlowForm, SolidBarRejected)
{
    // No bore → solid stock → DFM-FF-TUBE.
    auto solid = skill::createCylindricalStock(40.0, 50.0);

    skill::flow_form::Input in;
    in.outer_dia_mm          = 40.0;
    in.original_thickness_mm = 5.0;
    in.final_thickness_mm    = 2.5;
    in.original_length_mm    = 50.0;
    in.pass_count            = 1;

    auto r = skill::flow_form::validate(*solid, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-FF-TUBE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::flow_form::apply(*solid, in), skill::SkillError);
}
