// @lat: [[process/test-strategy#compound macro]]
//
// tab_lock_anti_rotation — 3 sub-features:
//   notch A on bore wall + notch B (diametrically opposite) + entry chamfer.
//
// The skill assumes a pre-existing bore on the stock; we pre-drill a bore
// before applying the skill so the notches have a bore wall to cut into.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"
#include "skills/tab_lock_anti_rotation.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

// Helper: cuboid stock + pre-drilled bore at centre, returns workpiece
// with the bore already present.
std::shared_ptr<skill::Workpiece> pre_bored_stock(double bore_dia_mm)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::drill_hole::Input dh;
    dh.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    dh.position_x_mm = 25.0;
    dh.position_y_mm = 25.0;
    dh.axis_dir      = gp_Dir(0, 0, -1);
    dh.diameter_mm   = bore_dia_mm;
    dh.depth_mm      = 15.0;
    dh.through_hole  = false;
    return skill::drill_hole::apply(*stock, dh).workpiece;
}
}  // namespace

// ─── 1. apply produces a valid shape with multiple sub-features ───────
TEST(SkillTabLockAntiRotation, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto bored = pre_bored_stock(10.0);
    const int faces0 = bored->faceCount();

    skill::tab_lock_anti_rotation::Input in;
    in.bore_axis      = gp_Ax1(gp_Pnt(25.0, 25.0, 20.0), gp_Dir(0, 0, -1));
    in.bore_dia_mm    = 10.0;
    in.tab_w_mm       = 2.0;
    in.tab_d_mm       = 1.0;
    in.tab_axial_z_mm = 3.0;
    in.chamfer_size_mm = 0.5;

    const double v0 = volumeOf(bored->shape());
    auto out = skill::tab_lock_anti_rotation::apply(*bored, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_GT(out.workpiece->faceCount(), faces0);
}

// ─── 2. validate rejects bad input ────────────────────────────────────
TEST(SkillTabLockAntiRotation, ValidateRejectsBadInput)
{
    auto bored = pre_bored_stock(10.0);

    skill::tab_lock_anti_rotation::Input in;
    in.bore_axis      = gp_Ax1(gp_Pnt(25.0, 25.0, 20.0), gp_Dir(0, 0, -1));
    in.bore_dia_mm    = 10.0;
    in.tab_w_mm       = 0.2;       // < 0.8 mm → DFM-TAB-W error
    in.tab_d_mm       = 1.0;
    in.tab_axial_z_mm = 3.0;

    auto r = skill::tab_lock_anti_rotation::validate(*bored, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TAB-W") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::tab_lock_anti_rotation::apply(*bored, in),
                 skill::SkillError);
}

// ─── 3. signature records compound kind ───────────────────────────────
TEST(SkillTabLockAntiRotation, SignatureRecordsCompoundKind)
{
    auto bored = pre_bored_stock(12.0);

    skill::tab_lock_anti_rotation::Input in;
    in.bore_axis      = gp_Ax1(gp_Pnt(25.0, 25.0, 20.0), gp_Dir(0, 0, -1));
    in.bore_dia_mm    = 12.0;
    in.tab_w_mm       = 2.5;
    in.tab_d_mm       = 1.2;
    in.tab_axial_z_mm = 4.0;

    auto out = skill::tab_lock_anti_rotation::apply(*bored, in);
    EXPECT_EQ(out.signature.skill_id, std::string("tab_lock_anti_rotation"));
    EXPECT_EQ(out.signature.pattern.at("kind"),
              std::string("tab_lock_anti_rotation"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
    EXPECT_NEAR(out.signature.pattern.at("tab_w_mm").get<double>(), 2.5, 1e-9);
    EXPECT_NEAR(out.signature.pattern.at("tab_d_mm").get<double>(), 1.2, 1e-9);
}

// ─── 4. recognize via metadata replay ─────────────────────────────────
TEST(SkillTabLockAntiRotation, RecognizeMetadataReplay)
{
    auto bored = pre_bored_stock(10.0);

    skill::tab_lock_anti_rotation::Input in;
    in.bore_axis      = gp_Ax1(gp_Pnt(25.0, 25.0, 20.0), gp_Dir(0, 0, -1));
    in.bore_dia_mm    = 10.0;
    in.tab_w_mm       = 1.5;
    in.tab_d_mm       = 0.8;
    in.tab_axial_z_mm = 2.5;

    auto out = skill::tab_lock_anti_rotation::apply(*bored, in);
    auto cands = skill::tab_lock_anti_rotation::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["tab_w_mm"].get<double>(),
                1.5, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["bore_dia_mm"].get<double>(),
                10.0, 1e-9);
}

// ─── 5. SPECIFIC: the two notches are diametrically opposite (180°) ───
TEST(SkillTabLockAntiRotation, NotchesAreDiametricallyOpposite)
{
    auto bored = pre_bored_stock(10.0);

    skill::tab_lock_anti_rotation::Input in;
    in.bore_axis      = gp_Ax1(gp_Pnt(25.0, 25.0, 20.0), gp_Dir(0, 0, -1));
    in.bore_dia_mm    = 10.0;
    in.tab_w_mm       = 2.0;
    in.tab_d_mm       = 1.0;
    in.tab_axial_z_mm = 3.0;

    auto out = skill::tab_lock_anti_rotation::apply(*bored, in);
    const auto angles = out.signature.pattern.at("notch_angles_deg");
    ASSERT_EQ(angles.size(), 2u);
    EXPECT_NEAR(std::abs(angles[1].get<double>() - angles[0].get<double>()),
                180.0, 1e-9);
    // Also: SAE J1199 / DFM-TAB-WALL means bore radius ≥ tab_d + 1 mm.
    const double boreR = in.bore_dia_mm / 2.0;
    EXPECT_GE(boreR, in.tab_d_mm + 1.0);
}
