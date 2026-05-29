// @lat: [[process/test-strategy#compound macro]]
//
// threaded_npt_port — NPT tapered tap hole + counterbore + face chamfer.
//
// Test cases:
//   1. apply removes volume & changes face count.
//   2. validate rejects unknown npt_size.
//   3. signature carries is_compound + subfeature_count == 3.
//   4. recognize replays metadata at confidence 1.0.
//   5. Derived small_end_dia_mm < nominal_od_mm (taper invariant).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/threaded_npt_port.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

namespace {
skill::threaded_npt_port::Input goodInput()
{
    skill::threaded_npt_port::Input in;
    in.face_id           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm     = 25.0;
    in.position_y_mm     = 25.0;
    in.axis_dir          = gp_Dir(0, 0, -1);
    in.npt_size          = "1/4";
    in.recess_depth_mm   = 1.5;
    in.chamfer_leg_mm    = 0.5;
    in.part_thickness_mm = 20.0;
    return in;
}
}  // namespace

// ─── 1. apply produces valid shape with multiple sub-features ─────────────
TEST(SkillThreadedNptPort, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    const double v0 = volumeOf(stock->shape());
    const int    f0 = stock->faceCount();

    auto out = skill::threaded_npt_port::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_NE(out.workpiece->faceCount(), f0);
}

// ─── 2. validate rejects unknown npt_size ─────────────────────────────────
TEST(SkillThreadedNptPort, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    auto in = goodInput();
    in.npt_size = "M99";   // not a valid NPT size
    auto r = skill::threaded_npt_port::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-NPT-SIZE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::threaded_npt_port::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records compound kind ────────────────────────────────────
TEST(SkillThreadedNptPort, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    auto out   = skill::threaded_npt_port::apply(*stock, goodInput());

    EXPECT_EQ(out.signature.skill_id, std::string("threaded_npt_port"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);

    // Standard NPT size lookup.
    double od = 0.0, l1 = 0.0; int tpi = 0;
    EXPECT_TRUE(skill::threaded_npt_port::resolveNptSize("1/4", od, tpi, l1));
    EXPECT_NEAR(od, 13.72, 1e-6);
    EXPECT_EQ(tpi, 18);
}

// ─── 4. Recognize replays metadata at confidence 1.0 ──────────────────────
TEST(SkillThreadedNptPort, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    auto out   = skill::threaded_npt_port::apply(*stock, goodInput());

    auto cands = skill::threaded_npt_port::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands.front().confidence, 1.0, 1e-9);
    EXPECT_EQ(cands.front().recovered_params["npt_size"].get<std::string>(),
              std::string("1/4"));
}

// ─── 5. Taper invariant: small_end_dia STRICTLY < nominal_od ──────────────
TEST(SkillThreadedNptPort, TaperGeometryRespectsStandard)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    auto out   = skill::threaded_npt_port::apply(*stock, goodInput());

    const double od   = out.signature.pattern.at("nominal_od_mm")  .get<double>();
    const double sd   = out.signature.pattern.at("small_end_dia_mm").get<double>();
    const double l1   = out.signature.pattern.at("l1_depth_mm")    .get<double>();
    const double half = out.signature.pattern.at("taper_per_side_deg").get<double>();

    EXPECT_LT(sd, od);                       // tapers IN with depth
    EXPECT_GT(sd, 0.0);                      // doesn't pinch to zero
    EXPECT_NEAR(half, 1.7899, 1e-6);         // ANSI 1°47'/foot
    // Derived: sd ≈ od - 2 × tan(half) × L1.
    const double pred = od - 2.0 * std::tan(half * M_PI / 180.0) * l1;
    EXPECT_NEAR(sd, pred, 1e-3);
}
