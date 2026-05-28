// @lat: [[process/test-strategy#skill round-trip]]
//
// label_apply skill — thin slab fused onto a planar face tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/label_apply.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Apply fuses a slab (volume INCREASES) ─────────────────────────────
TEST(SkillLabelApply, ApplyAddsSlabVolume)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::label_apply::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm      = 30.0;
    in.position_y_mm      = 20.0;
    in.label_width_mm     = 20.0;
    in.label_height_mm    = 10.0;
    in.label_thickness_mm = 0.1;
    in.label_text         = "MX-001";

    auto out = skill::label_apply::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double expected = 20.0 * 10.0 * 0.1;  // 20 mm³
    const double added = volumeOf(out.workpiece->shape()) - volBefore;
    EXPECT_GT(added, expected * 0.5);
    EXPECT_LT(added, expected * 1.5);

    EXPECT_EQ(out.signature.skill_id, std::string("label_apply"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_TRUE(out.signature.pattern["has_text"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["additive"].get<bool>());
}

// ── 2. DFM-LABEL-THICK rejects too-thick labels ──────────────────────────
TEST(SkillLabelApply, ValidateRejectsThickLabel)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);

    skill::label_apply::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 30.0;
    in.position_y_mm      = 20.0;
    in.label_width_mm     = 20.0;
    in.label_height_mm    = 10.0;
    in.label_thickness_mm = 1.0;   // > 0.5

    auto r = skill::label_apply::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LABEL-THICK") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::label_apply::apply(*stock, in), skill::SkillError);
}

// ── 3. DFM-LABEL-FIT rejects oversized label ─────────────────────────────
TEST(SkillLabelApply, ValidateRejectsOversizedLabel)
{
    // Stock face area = 60 × 40 = 2400 mm². Label larger → reject.
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);

    skill::label_apply::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 30.0;
    in.position_y_mm      = 20.0;
    in.label_width_mm     = 100.0;   // bigger than 60 mm face
    in.label_height_mm    = 50.0;
    in.label_thickness_mm = 0.1;

    auto r = skill::label_apply::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LABEL-FIT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 4. Negative / zero dimensions rejected ───────────────────────────────
TEST(SkillLabelApply, ValidateRejectsNonPositiveDims)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);

    skill::label_apply::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 30.0;
    in.position_y_mm      = 20.0;
    in.label_width_mm     = 0.0;    // bad
    in.label_height_mm    = 10.0;
    in.label_thickness_mm = 0.1;

    auto r = skill::label_apply::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ── 5. Recognize via metadata ────────────────────────────────────────────
TEST(SkillLabelApply, RecognizeReplaysMetadata)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);

    skill::label_apply::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 30.0;
    in.position_y_mm      = 20.0;
    in.label_width_mm     = 15.0;
    in.label_height_mm    = 8.0;
    in.label_thickness_mm = 0.15;
    in.label_text         = "SERIAL";

    auto out = skill::label_apply::apply(*stock, in);
    auto cands = skill::label_apply::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].skill_id, std::string("label_apply"));
    EXPECT_NEAR(cands[0].recovered_params["label_width_mm"].get<double>(),
                15.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["label_height_mm"].get<double>(),
                8.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["label_thickness_mm"].get<double>(),
                0.15, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["label_text"], "SERIAL");
    EXPECT_GT(cands[0].confidence, 0.9);
}

// ── 6. Label without text is accepted (text optional) ────────────────────
TEST(SkillLabelApply, LabelWithoutTextOK)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);

    skill::label_apply::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 30.0;
    in.position_y_mm      = 20.0;
    in.label_width_mm     = 10.0;
    in.label_height_mm    = 5.0;
    in.label_thickness_mm = 0.1;
    in.label_text         = "";

    auto out = skill::label_apply::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_FALSE(out.signature.pattern["has_text"].get<bool>());
}
