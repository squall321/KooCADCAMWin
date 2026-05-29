// @lat: [[process/test-strategy#skill round-trip]]
//
// finish_apparel skill — 5-case sweep:
//   1. Apply clones geometry unchanged
//   2. Validate rejects unknown technique
//   3. Signature records kind + technique + bath_temp_c
//   4. Recognize metadata replay
//   5. SPECIFIC: washing keeps temp; pressing/labeling zero it; cycle differs

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/finish_apparel.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

skill::finish_apparel::Input washInput()
{
    skill::finish_apparel::Input in;
    in.technique   = "washing";
    in.bath_temp_c = 60.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillFinishApparel, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::finish_apparel::apply(*stock, washInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("finish_apparel"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillFinishApparel, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::finish_apparel::Input in;
    in.technique   = "bleaching";    // unknown
    in.bath_temp_c = 60.0;

    auto r = skill::finish_apparel::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::finish_apparel::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillFinishApparel, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = washInput();
    in.bath_temp_c = 40.0;

    auto out = skill::finish_apparel::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("finish_apparel"));
    EXPECT_EQ(out.signature.pattern["technique"].get<std::string>(),
              std::string("washing"));
    EXPECT_NEAR(out.signature.pattern["bath_temp_c"].get<double>(),
                40.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillFinishApparel, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::finish_apparel::apply(*stock, washInput());

    auto cands = skill::finish_apparel::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["technique"].get<std::string>(),
              std::string("washing"));
    EXPECT_NEAR(cands[0].recovered_params["bath_temp_c"].get<double>(),
                60.0, 1e-9);

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::finish_apparel::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "finish_apparel cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: technique selects tooling + zeroes temp for non-wash ────
TEST(SkillFinishApparel, TechniqueSelectsToolingAndTemp)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::finish_apparel::Input wash;
    wash.technique   = "washing";
    wash.bath_temp_c = 80.0;
    auto washOut = skill::finish_apparel::apply(*stock, wash);
    EXPECT_EQ(washOut.signature.tooling.tool_type,
              std::string("industrial_washer"));
    EXPECT_NEAR(washOut.signature.pattern["bath_temp_c"].get<double>(),
                80.0, 1e-9);

    skill::finish_apparel::Input press;
    press.technique   = "pressing";
    press.bath_temp_c = 0.0;
    auto pressOut = skill::finish_apparel::apply(*stock, press);
    EXPECT_EQ(pressOut.signature.tooling.tool_type,
              std::string("steam_press"));
    EXPECT_NEAR(pressOut.signature.pattern["bath_temp_c"].get<double>(),
                0.0, 1e-9);

    skill::finish_apparel::Input label;
    label.technique   = "labeling";
    label.bath_temp_c = 0.0;
    auto labelOut = skill::finish_apparel::apply(*stock, label);
    EXPECT_EQ(labelOut.signature.tooling.tool_type,
              std::string("label_applicator"));

    // Wash takes longer than press takes longer than label.
    EXPECT_GT(washOut.signature.tooling.est_cycle_time_s,
              pressOut.signature.tooling.est_cycle_time_s);
    EXPECT_GT(pressOut.signature.tooling.est_cycle_time_s,
              labelOut.signature.tooling.est_cycle_time_s);

    // bath_temp_c on non-wash technique triggers info finding (still passes).
    skill::finish_apparel::Input pressHot;
    pressHot.technique   = "pressing";
    pressHot.bath_temp_c = 40.0;
    auto rHot = skill::finish_apparel::validate(*stock, pressHot);
    EXPECT_TRUE(rHot.passed);
    EXPECT_TRUE(hasFinding(rHot, "DFM-FIN-TEMP"));
}
