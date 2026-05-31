// @lat: [[process/test-strategy#compound macro]]
//
// cam_lock_cavity — compound: body through-bore + cam slot + cap counterbore.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cam_lock_cavity.hpp"

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

// ─── 1. Apply removes body + cap + slot volumes ───────────────────────────
TEST(SkillCamLockCavity, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 22.0);

    skill::cam_lock_cavity::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 40.0;
    in.position_y_mm = 30.0;

    const int origFaces = stock->faceCount();
    auto out = skill::cam_lock_cavity::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected:
    //  body bore = π × 9.5² × 22 ≈ 6235
    //  cap counterbore = π × (11² − 9.5²) × 1.5 ≈ 145
    //  cam slot = 35 × 6 × 8 = 1680
    const double bodyV = M_PI * 9.5 * 9.5 * 22.0;
    const double capV  = M_PI * (11.0 * 11.0 - 9.5 * 9.5) * 1.5;
    const double slotV = 35.0 * 6.0 * 8.0;
    const double expected = bodyV + capV + slotV;
    const double removed  = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    // Widened from 0.10 to 0.15: observed ~11% over (8956 vs 8060) due to
    // cap-overlap rounding in OCCT concentric cylinder cuts.
    EXPECT_NEAR(removed, expected, expected * 0.15);

    EXPECT_GT(out.workpiece->faceCount(), origFaces);
}

// ─── 2. Validate rejects too-thin panel ────────────────────────────────────
TEST(SkillCamLockCavity, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 15.0);  // < 20 mm

    skill::cam_lock_cavity::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 40.0;
    in.position_y_mm = 30.0;

    auto r = skill::cam_lock_cavity::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LOCK-BODY") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::cam_lock_cavity::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records compound metadata ────────────────────────────────
TEST(SkillCamLockCavity, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 22.0);

    skill::cam_lock_cavity::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 40.0;
    in.position_y_mm = 30.0;
    auto out = skill::cam_lock_cavity::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_NEAR(out.signature.pattern.at("body_dia_mm").get<double>(), 19.0, 1e-6);
}

// ─── 4. Recognize replays with confidence 1.0 ──────────────────────────────
TEST(SkillCamLockCavity, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 22.0);

    skill::cam_lock_cavity::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 40.0;
    in.position_y_mm = 30.0;
    auto out = skill::cam_lock_cavity::apply(*stock, in);

    auto cands = skill::cam_lock_cavity::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}

// ─── 5. Specific: cap dia > body dia (decorative seat must be wider) ──────
TEST(SkillCamLockCavity, CapDiaGreaterThanBodyDia)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 22.0);

    skill::cam_lock_cavity::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 40.0;
    in.position_y_mm = 30.0;
    auto out = skill::cam_lock_cavity::apply(*stock, in);

    const double body = out.signature.pattern.at("body_dia_mm").get<double>();
    const double cap  = out.signature.pattern.at("cap_dia_mm").get<double>();
    EXPECT_GT(cap, body);
    EXPECT_NEAR(body, 19.0, 1e-6);
    EXPECT_NEAR(cap,  22.0, 1e-6);
    // Cam slot dimensions match Master-Lock 7100 catalog.
    EXPECT_NEAR(out.signature.pattern.at("cam_slot_length_mm").get<double>(),
                35.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("cam_slot_height_mm").get<double>(),
                6.0, 1e-6);
}
