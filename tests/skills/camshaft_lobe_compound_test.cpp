// @lat: [[process/test-strategy#compound camshaft_lobe_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/camshaft_lobe_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

int findCylFaceId(const skill::Workpiece& wp) {
    for (int i = 0; i < wp.faceCount(); ++i)
        if (wp.isFaceCylinder(i)) return i;
    return -1;
}

skill::camshaft_lobe_compound::Input goodInput(const skill::Workpiece& wp)
{
    skill::camshaft_lobe_compound::Input in;
    in.face_id               = findCylFaceId(wp);
    in.lobe_position_z_mm    = 30.0;
    in.base_circle_radius_mm = 15.0;
    in.max_lift_mm           = 8.0;
    in.ramp_angle_deg        = 20.0;
    in.duration_deg          = 240.0;
    in.lobe_width_mm         = 12.0;
    in.profile_sample_count  = 24;     // keep small for fast test
    return in;
}

}  // namespace

// ─── 1. apply removes cam-profile volume ─────────────────────────────────
TEST(SkillCamshaftLobe, ApplyCutsCamProfile)
{
    // 50 mm OD, 60 mm long shaft (R = 25mm > baseR(15) + lift(8) = 23).
    auto stock = skill::createCylindricalStock(50.0, 60.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    const double v0 = volumeOf(stock->shape());
    auto out = skill::camshaft_lobe_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
}

// ─── 2. validate rejects max_lift < 0.5 mm ───────────────────────────────
TEST(SkillCamshaftLobe, ValidateRejectsLiftTooSmall)
{
    auto stock = skill::createCylindricalStock(50.0, 60.0);
    auto in    = goodInput(*stock);
    in.max_lift_mm = 0.2;

    auto r = skill::camshaft_lobe_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CAM-LIFT") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::camshaft_lobe_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. signature records engine_feature_type + sample count ─────────────
TEST(SkillCamshaftLobe, SignatureRecordsEngineFeature)
{
    auto stock = skill::createCylindricalStock(50.0, 60.0);
    auto in    = goodInput(*stock);
    auto out   = skill::camshaft_lobe_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("camshaft_lobe_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("engine_feature_type")
              .get<std::string>(),
              std::string("camshaft_lobe"));
    EXPECT_EQ(out.signature.pattern.at("profile_sample_count").get<int>(),
              in.profile_sample_count);
    EXPECT_NEAR(out.signature.pattern.at("max_lift_mm").get<double>(),
                8.0, 1e-6);
}

// ─── 4. recognize via metadata replay ────────────────────────────────────
TEST(SkillCamshaftLobe, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(50.0, 60.0);
    auto in    = goodInput(*stock);
    auto out   = skill::camshaft_lobe_compound::apply(*stock, in);

    auto cands = skill::camshaft_lobe_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}

// ─── 5. SPECIFIC: at least one wedge subfeature was applied ──────────────
TEST(SkillCamshaftLobe, AtLeastOneWedgeApplied)
{
    auto stock = skill::createCylindricalStock(50.0, 60.0);
    auto in    = goodInput(*stock);
    auto out   = skill::camshaft_lobe_compound::apply(*stock, in);

    EXPECT_GT(out.signature.pattern.at("subfeature_count").get<int>(), 0);
}
