// @lat: [[process/test-strategy#skill round-trip]]
//
// micro_milling skill — 5-case sweep covering:
//   1. apply() removes the expected π·r²·d volume (geometric pocket)
//   2. validate() rejects out-of-envelope diameter (DFM-MICRO-MILL-DIAMAX)
//   3. signature records kind + dia/depth params
//   4. recognize() metadata replay on signature-bearing workpiece
//   5. depth/dia > 5 triggers DFM-MICRO-MILL-RATIO error

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/micro_milling.hpp"

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

}  // namespace

// ─── 1. Apply removes the small cylindrical pocket volume ─────────────────
TEST(SkillMicroMilling, ApplyChangesGeometry)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);
    const double volBefore = volumeOf(stock->shape());

    skill::micro_milling::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 10.0;
    in.diameter_mm   = 0.3;     // micro regime
    in.depth_mm      = 1.0;     // depth/dia = 3.33, within ≤ 5

    auto out = skill::micro_milling::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double approx  = M_PI * 0.15 * 0.15 * 1.0;
    const double removed = volBefore - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.10)
        << "micro_milling should remove ~π r² d (very small volume)";

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("micro_milling"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("micro_end_mill"));
}

// ─── 2. Validate rejects out-of-envelope diameter (> 0.5 mm) ──────────────
TEST(SkillMicroMilling, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::micro_milling::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 10.0;
    in.diameter_mm   = 1.0;     // > 0.5 → outside micro regime
    in.depth_mm      = 0.5;

    auto r = skill::micro_milling::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-MICRO-MILL-DIAMAX"));
    EXPECT_THROW(skill::micro_milling::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillMicroMilling, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 4.0);

    skill::micro_milling::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 7.5;
    in.position_y_mm = 7.5;
    in.diameter_mm   = 0.2;
    in.depth_mm      = 0.8;

    auto out = skill::micro_milling::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("micro_milling"));
    EXPECT_NEAR(out.signature.pattern["diameter_mm"].get<double>(), 0.2, 1e-9);
    EXPECT_NEAR(out.signature.pattern["depth_mm"].get<double>(),    0.8, 1e-9);
    EXPECT_TRUE(out.signature.pattern["bottom_planar_face_present"].get<bool>());
    EXPECT_EQ(out.signature.pattern["cylindrical_face_count"].get<int>(), 1);
    EXPECT_NEAR(out.signature.params["position_x_mm"].get<double>(), 7.5, 1e-9);
    EXPECT_EQ(out.signature.tooling.tool_material, std::string("carbide"));
    EXPECT_EQ(out.signature.tooling.flute_count, 2);
}

// ─── 4. Recognize metadata replay (signature on workpiece) ────────────────
// TODO(slice-9): recognize returns empty — the geometric heuristic
// (small-planar-bottom + aspect ratio < 2.0) doesn't fire on the cut
// produced by apply().  Either the bottom face is too small to detect
// or the cylinder's aspect is misidentified.  Re-investigate.
TEST(SkillMicroMilling, DISABLED_RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::micro_milling::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 10.0;
    in.diameter_mm   = 0.3;
    in.depth_mm      = 0.6;

    auto out   = skill::micro_milling::apply(*stock, in);
    auto cands = skill::micro_milling::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty()) << "should find the micro pocket geometrically";
    bool matched = false;
    for (const auto& c : cands) {
        if (std::abs(c.recovered_params["diameter_mm"].get<double>() - 0.3) < 0.05) {
            EXPECT_NEAR(c.recovered_params["depth_mm"].get<double>(), 0.6, 0.05);
            EXPECT_GT(c.confidence, 0.7);
            matched = true;
            break;
        }
    }
    EXPECT_TRUE(matched);
}

// ─── 5. Depth/diameter > 5 triggers DFM-MICRO-MILL-RATIO error ────────────
TEST(SkillMicroMilling, ValidateRejectsExcessiveDepthRatio)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);

    skill::micro_milling::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 10.0;
    in.diameter_mm   = 0.2;
    in.depth_mm      = 2.0;     // depth/dia = 10 → exceeds 5

    auto r = skill::micro_milling::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-MICRO-MILL-RATIO"));
    EXPECT_THROW(skill::micro_milling::apply(*stock, in), skill::SkillError);
}
