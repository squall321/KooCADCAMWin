// @lat: [[process/test-strategy#medical lcp combi hole]]
//
// locking_compression_plate_hole — AO LCP combi-hole: oblong DCP slot
// + threaded locking bore at one end.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/locking_compression_plate_hole.hpp"

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

// ─── 1. apply produces valid compound shape ──────────────────────────────
TEST(SkillLcpHole, ApplyProducesCompoundShape)
{
    // 60 × 12 × 4 mm bone plate stock
    auto stock = skill::createCuboidStock(60.0, 12.0, 4.0);
    const double stockVol = volumeOf(stock->shape());
    const int    stockFC  = stock->faceCount();

    skill::locking_compression_plate_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 30.0;
    in.position_y_mm = 6.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.slot_dir      = gp_Dir(1, 0, 0);
    in.dynamic_compression_slot_length_mm = 8.0;
    in.locking_thread_dia_mm   = 3.5;
    in.locking_thread_pitch_mm = 1.25;
    in.plate_thickness_mm      = 4.0;

    auto out = skill::locking_compression_plate_hole::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double removed = stockVol - volumeOf(out.workpiece->shape());
    // Stadium area ≈ (8 - 3.5) × 3.5 + π × 1.75² ≈ 15.75 + 9.62 ≈ 25.37
    // Times plate thickness 4 → ≈ 101 mm³.
    EXPECT_GT(removed, 40.0);
    EXPECT_LT(removed, 250.0);

    EXPECT_GT(out.workpiece->faceCount(), stockFC);

    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("medical_feature_type").get<std::string>(),
              std::string("lcp_combi_hole"));
    EXPECT_NEAR(out.signature.pattern.at("slot_length_mm").get<double>(), 8.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("locking_thread_dia_mm").get<double>(),
                3.5, 1e-6);
}

// ─── 2. validate rejects too-short slot ──────────────────────────────────
TEST(SkillLcpHole, ValidateRejectsShortSlot)
{
    auto stock = skill::createCuboidStock(60.0, 12.0, 4.0);

    skill::locking_compression_plate_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0; in.position_y_mm = 6.0;
    in.slot_dir      = gp_Dir(1, 0, 0);
    in.dynamic_compression_slot_length_mm = 4.0;   // < 1.5 × 3.5 = 5.25
    in.locking_thread_dia_mm   = 3.5;
    in.locking_thread_pitch_mm = 1.25;
    in.plate_thickness_mm      = 4.0;

    auto r = skill::locking_compression_plate_hole::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LCP-SLOT") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::locking_compression_plate_hole::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. validate rejects out-of-range pitch ──────────────────────────────
TEST(SkillLcpHole, ValidateRejectsBadPitch)
{
    auto stock = skill::createCuboidStock(60.0, 12.0, 4.0);

    skill::locking_compression_plate_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0; in.position_y_mm = 6.0;
    in.slot_dir      = gp_Dir(1, 0, 0);
    in.dynamic_compression_slot_length_mm = 8.0;
    in.locking_thread_dia_mm   = 3.5;
    in.locking_thread_pitch_mm = 3.0;     // > 2.0 envelope → reject
    in.plate_thickness_mm      = 4.0;

    auto r = skill::locking_compression_plate_hole::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LCP-PITCH") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. validate rejects thin plate ──────────────────────────────────────
TEST(SkillLcpHole, ValidateRejectsThinPlate)
{
    auto stock = skill::createCuboidStock(60.0, 12.0, 4.0);

    skill::locking_compression_plate_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0; in.position_y_mm = 6.0;
    in.slot_dir      = gp_Dir(1, 0, 0);
    in.dynamic_compression_slot_length_mm = 8.0;
    in.locking_thread_dia_mm   = 3.5;
    in.locking_thread_pitch_mm = 1.25;
    in.plate_thickness_mm      = 1.0;     // < 0.5 × 3.5 = 1.75 → reject

    auto r = skill::locking_compression_plate_hole::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LCP-THICK") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 5. recognize metadata replay ────────────────────────────────────────
TEST(SkillLcpHole, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 12.0, 4.0);

    skill::locking_compression_plate_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0; in.position_y_mm = 6.0;
    in.slot_dir      = gp_Dir(1, 0, 0);
    in.dynamic_compression_slot_length_mm = 8.0;
    in.locking_thread_dia_mm   = 3.5;
    in.locking_thread_pitch_mm = 1.25;
    in.plate_thickness_mm      = 4.0;

    auto out  = skill::locking_compression_plate_hole::apply(*stock, in);
    auto cands = skill::locking_compression_plate_hole::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_NEAR(cands[0].recovered_params.at("locking_thread_dia_mm").get<double>(),
                3.5, 1e-6);
}
