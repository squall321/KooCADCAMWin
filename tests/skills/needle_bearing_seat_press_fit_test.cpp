// @lat: [[process/test-strategy#compound bearing seat]]
//
// needle_bearing_seat_press_fit — compound: H7 bore + retention shoulder
// + 0.5 × 30° chamfer.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/needle_bearing_seat_press_fit.hpp"

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

// ─── 1. Apply produces valid shape with multiple sub-features ─────────────
TEST(SkillNeedleBearingSeatPressFit, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::needle_bearing_seat_press_fit::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.axis_dir         = gp_Dir(0, 0, -1);
    in.outer_dia_mm     = 16.0;
    in.depth_mm         = 12.0;
    in.retention_dia_mm = 10.0;

    const double v0 = volumeOf(stock->shape());
    const int    f0 = stock->faceCount();

    auto out = skill::needle_bearing_seat_press_fit::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    const int    f1 = out.workpiece->faceCount();

    // Outer bore + retention column.
    const double outerVol      = M_PI * 8.0 * 8.0 * 12.0;
    const double retentionVol  = M_PI * 5.0 * 5.0 * (12.0 + 16.0 / 8.0);
    const double removed       = v0 - v1;
    EXPECT_GT(removed, (outerVol + retentionVol) * 0.85);
    EXPECT_LT(removed, (outerVol + retentionVol) * 1.5);

    EXPECT_GT(f1, f0);
}

// ─── 2. Validate rejects retention_dia ≥ outer_dia ────────────────────────
TEST(SkillNeedleBearingSeatPressFit, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::needle_bearing_seat_press_fit::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.outer_dia_mm     = 16.0;
    in.depth_mm         = 12.0;
    in.retention_dia_mm = 16.0;   // equal → invalid (no shoulder step)

    auto r = skill::needle_bearing_seat_press_fit::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SEAT-GEOM") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::needle_bearing_seat_press_fit::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature records compound kind ──────────────────────────────────
TEST(SkillNeedleBearingSeatPressFit, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::needle_bearing_seat_press_fit::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.outer_dia_mm     = 16.0;
    in.depth_mm         = 12.0;
    in.retention_dia_mm = 10.0;
    auto out = skill::needle_bearing_seat_press_fit::apply(*stock, in);

    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    // H7 fit class for press-fit bore.
    EXPECT_EQ(out.signature.pattern.at("fit_class").get<std::string>(),
              std::string("H7"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillNeedleBearingSeatPressFit, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::needle_bearing_seat_press_fit::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.outer_dia_mm     = 16.0;
    in.depth_mm         = 12.0;
    in.retention_dia_mm = 10.0;
    auto out = skill::needle_bearing_seat_press_fit::apply(*stock, in);

    auto cands = skill::needle_bearing_seat_press_fit::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["outer_dia_mm"].get<double>(),
                16.0, 1e-6);
}

// ─── 5. Chamfer is fixed 0.5 × 30° per SKF press-fit standard ────────────
TEST(SkillNeedleBearingSeatPressFit, ChamferMatchesSKFStandard)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    skill::needle_bearing_seat_press_fit::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.outer_dia_mm     = 16.0;
    in.depth_mm         = 12.0;
    in.retention_dia_mm = 10.0;
    auto out = skill::needle_bearing_seat_press_fit::apply(*stock, in);

    EXPECT_NEAR(out.signature.pattern.at("chamfer_axial_mm").get<double>(),
                0.5, 1e-9);
    EXPECT_NEAR(out.signature.pattern.at("chamfer_angle_deg").get<double>(),
                30.0, 1e-9);
    // retention_length_mm = outer_dia / 8.
    EXPECT_NEAR(out.signature.pattern.at("retention_length_mm").get<double>(),
                16.0 / 8.0, 1e-9);
}
