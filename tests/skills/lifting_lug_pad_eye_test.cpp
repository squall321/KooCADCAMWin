// @lat: [[process/test-strategy#compound macro]]
//
// lifting_lug_pad_eye — COMPOUND skill: padeye plate + pin hole + face
// chamfer + base weld-prep bevel.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/lifting_lug_pad_eye.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

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

skill::lifting_lug_pad_eye::Input goodInput()
{
    skill::lifting_lug_pad_eye::Input in;
    in.plate_t_mm      = 25.0;
    in.pin_hole_dia_mm = 40.0;
    in.lug_height_mm   = 250.0;
    in.lug_width_mm    = 180.0;
    in.chamfer_mm      = 3.0;
    in.base_bevel_mm   = 8.0;
    return in;
}

}  // namespace

// ─── 1. Apply: shape valid; volume reflects pin hole removed ──────────────
TEST(SkillLiftingLugPadEye, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);

    auto in = goodInput();
    auto out = skill::lifting_lug_pad_eye::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Approx body volume MINUS pin hole.
    const double bodyVol = in.lug_width_mm * in.lug_height_mm * in.plate_t_mm;
    const double pinVol  = M_PI * (in.pin_hole_dia_mm / 2.0) *
                           (in.pin_hole_dia_mm / 2.0) * in.plate_t_mm;
    const double upper   = bodyVol - pinVol;           // before chamfer + bevel
    const double vol = volumeOf(out.workpiece->shape());
    EXPECT_GT(vol, 0.0);
    EXPECT_LT(vol, upper);   // bevel/chamfer remove additional material

    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_GE(out.signature.pattern["subfeature_count"].get<int>(), 2);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillLiftingLugPadEye, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);

    // Bad: plate too thin for pin bearing pressure.
    auto in1 = goodInput();
    in1.plate_t_mm = 5.0;
    auto r1 = skill::lifting_lug_pad_eye::validate(*stock, in1);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-LUG-T-PIN"));
    EXPECT_THROW(skill::lifting_lug_pad_eye::apply(*stock, in1),
                 skill::SkillError);

    // Bad: pin-to-top edge distance < 1.5 × pin_dia.
    auto in2 = goodInput();
    in2.pin_hole_dia_mm = 80.0;   // top edge = 250×0.25 = 62.5 < 1.5×80 = 120
    auto r2 = skill::lifting_lug_pad_eye::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-LUG-PIN-EDGE"));
}

// ─── 3. Signature records compound kind ───────────────────────────────────
TEST(SkillLiftingLugPadEye, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);
    auto out = skill::lifting_lug_pad_eye::apply(*stock, goodInput());

    EXPECT_EQ(out.signature.skill_id, std::string("lifting_lug_pad_eye"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("lifting_lug_pad_eye"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_GE(out.signature.pattern["subfeature_count"].get<int>(), 2);

    EXPECT_NEAR(out.signature.params["plate_t_mm"].get<double>(),       25.0, 1e-9);
    EXPECT_NEAR(out.signature.params["pin_hole_dia_mm"].get<double>(),  40.0, 1e-9);
    EXPECT_NEAR(out.signature.params["lug_height_mm"].get<double>(),   250.0, 1e-9);
}

// ─── 4. Recognize: metadata replay returns confidence 1.0 ────────────────
TEST(SkillLiftingLugPadEye, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);
    auto out = skill::lifting_lug_pad_eye::apply(*stock, goodInput());

    auto cands = skill::lifting_lug_pad_eye::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["pin_hole_dia_mm"].get<double>(),
                40.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::lifting_lug_pad_eye::recognize(raw).empty());
}

// ─── 5. SPECIFIC: pin centered at top-3rd; top edge > 1.5 × pin_dia ───────
TEST(SkillLiftingLugPadEye, PinPositionAtTopThirdAndEdgeWithinSpec)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);
    auto in = goodInput();
    auto out = skill::lifting_lug_pad_eye::apply(*stock, in);

    const double pinZ  = out.signature.pattern["pin_center"]["z"].get<double>();
    EXPECT_NEAR(pinZ, in.lug_height_mm * 0.75, 1e-9);

    const double topEdge = out.signature.pattern["pin_top_edge_mm"].get<double>();
    EXPECT_GE(topEdge, 1.5 * in.pin_hole_dia_mm);
}
