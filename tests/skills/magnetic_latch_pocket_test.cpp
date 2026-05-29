// @lat: [[process/test-strategy#compound macro]]
//
// magnetic_latch_pocket — compound: magnet pocket + chamfer + 2 mount pilots.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/magnetic_latch_pocket.hpp"

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

// ─── 1. Apply removes ~ pocket + 2 mount pilot volumes ─────────────────────
TEST(SkillMagneticLatchPocket, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 12.0);

    skill::magnetic_latch_pocket::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 40.0;
    in.position_y_mm = 20.0;

    const int origFaces = stock->faceCount();
    auto out = skill::magnetic_latch_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected:
    //  pocket  = π × (8.05/2)² × 5 ≈ 254.6
    //  chamfer cone-frustum extra ≈ small (~5)
    //  mount pilots = 2 × π × 1.25² × 6 ≈ 58.9
    const double pocketV = M_PI * (8.05 / 2.0) * (8.05 / 2.0) * 5.0;
    const double mountV  = 2.0 * M_PI * 1.25 * 1.25 * 6.0;
    const double expected = pocketV + mountV;
    const double removed  = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    // Allow 15% tolerance for chamfer extras.
    EXPECT_NEAR(removed, expected, expected * 0.15);

    EXPECT_GT(out.workpiece->faceCount(), origFaces);
}

// ─── 2. Validate rejects too-thin panel ────────────────────────────────────
TEST(SkillMagneticLatchPocket, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 5.0);  // < 7 mm

    skill::magnetic_latch_pocket::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 40.0;
    in.position_y_mm = 20.0;

    auto r = skill::magnetic_latch_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-MAG-DEPTH") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::magnetic_latch_pocket::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records compound metadata ────────────────────────────────
TEST(SkillMagneticLatchPocket, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 12.0);

    skill::magnetic_latch_pocket::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 40.0;
    in.position_y_mm = 20.0;
    auto out = skill::magnetic_latch_pocket::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("mount_positions").size(), 2u);
}

// ─── 4. Recognize replays with confidence 1.0 ──────────────────────────────
TEST(SkillMagneticLatchPocket, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 12.0);

    skill::magnetic_latch_pocket::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 40.0;
    in.position_y_mm = 20.0;
    auto out = skill::magnetic_latch_pocket::apply(*stock, in);

    auto cands = skill::magnetic_latch_pocket::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}

// ─── 5. Specific: H10 slip-fit clearance 0.02–0.10 mm ──────────────────────
TEST(SkillMagneticLatchPocket, MagnetSlipFitInH10Range)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 12.0);

    skill::magnetic_latch_pocket::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 40.0;
    in.position_y_mm = 20.0;
    auto out = skill::magnetic_latch_pocket::apply(*stock, in);

    const double nominal = out.signature.pattern.at("magnet_dia_mm").get<double>();
    const double slipfit = out.signature.pattern.at("magnet_slipfit_dia_mm").get<double>();
    const double clearance = slipfit - nominal;
    EXPECT_NEAR(nominal, 8.0, 1e-6);
    EXPECT_GE(clearance, 0.02);
    EXPECT_LE(clearance, 0.10);
    // Mount pitch must match the 30 mm catalog standard.
    EXPECT_NEAR(out.signature.pattern.at("mount_pitch_mm").get<double>(),
                30.0, 1e-6);
}
