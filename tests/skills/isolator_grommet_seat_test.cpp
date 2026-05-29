// @lat: [[process/test-strategy#compound isolator_grommet_seat]]
//
// Verifies REAL multi-cut geometry: counterbore + ID groove + rotation notch.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/isolator_grommet_seat.hpp"

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

// ─── T1. apply: volume delta matches ISO-M spec (±20%) ──────────────────
TEST(SkillIsolatorSeat, ApplyRealGeometryVolumeAndFaces)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 8.0);
    const int    stockFaces = stock->faceCount();
    const double stockVol   = volumeOf(stock->shape());

    skill::isolator_grommet_seat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.isolator_size = "ISO-M";

    auto out = skill::isolator_grommet_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // ISO-M: cb 12.0 × 5.0 → π·36·5 = 565.5
    // Groove annulus (6.7²−6²) × π × 0.8 ≈ 22.3
    // Notch 1.5 × 0.8 × 0.7 ≈ 0.84
    // Total ≈ 588.6
    const double rCB = 6.0;
    const double rGroove = 6.7;
    const double approx =
        M_PI * rCB * rCB * 5.0 +
        M_PI * (rGroove * rGroove - rCB * rCB) * 0.8 +
        1.5 * 0.8 * 0.7;
    const double removed = stockVol - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.20);

    EXPECT_GT(out.workpiece->faceCount(), stockFaces);
}

// ─── T2. validate rejects unknown isolator size + apply throws ──────────
TEST(SkillIsolatorSeat, ValidateRejectsUnknownSpec)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 8.0);

    skill::isolator_grommet_seat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.isolator_size = "ISO-MEGA";   // unknown

    auto r = skill::isolator_grommet_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool foundSpec = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CONN-SIZE") { foundSpec = true; break; }
    EXPECT_TRUE(foundSpec);

    EXPECT_THROW(skill::isolator_grommet_seat::apply(*stock, in),
                 skill::SkillError);
}

// ─── T3. signature carries spec key + is_compound + 3 subs ──────────────
TEST(SkillIsolatorSeat, SignatureCarriesSpecAndCompoundFlag)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 12.0);

    skill::isolator_grommet_seat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.isolator_size = "ISO-L";

    auto out = skill::isolator_grommet_seat::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("isolator_grommet_seat"));
    EXPECT_EQ(out.signature.pattern.at("isolator_size").get<std::string>(),
              std::string("ISO-L"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
}

// ─── T4. recognize returns metadata replay candidate ────────────────────
TEST(SkillIsolatorSeat, RecognizeReturnsCandidate)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 12.0);

    skill::isolator_grommet_seat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.isolator_size = "ISO-XL";

    auto out = skill::isolator_grommet_seat::apply(*stock, in);
    auto cands = skill::isolator_grommet_seat::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
    EXPECT_EQ(cands.front().skill_id,
              std::string("isolator_grommet_seat"));
    EXPECT_EQ(
        cands.front().recovered_params.at("isolator_size").get<std::string>(),
        std::string("ISO-XL"));
}

// ─── T5. SPEC TABLE: ISO-S has smaller cb than ISO-XL (size ordering) ──
TEST(SkillIsolatorSeat, SpecTableCounterboreScalesWithSize)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 12.0);

    skill::isolator_grommet_seat::Input s_in;
    s_in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    s_in.axis_dir      = gp_Dir(0, 0, -1);
    s_in.position_x_mm = 20.0;
    s_in.position_y_mm = 20.0;
    s_in.isolator_size = "ISO-S";
    auto s_out = skill::isolator_grommet_seat::apply(*stock, s_in);

    skill::isolator_grommet_seat::Input xl_in;
    xl_in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    xl_in.axis_dir      = gp_Dir(0, 0, -1);
    xl_in.position_x_mm = 20.0;
    xl_in.position_y_mm = 20.0;
    xl_in.isolator_size = "ISO-XL";
    auto xl_out = skill::isolator_grommet_seat::apply(*stock, xl_in);

    EXPECT_NEAR(s_out.signature.pattern.at("cb_dia_mm").get<double>(),
                8.0, 1e-3);
    EXPECT_NEAR(xl_out.signature.pattern.at("cb_dia_mm").get<double>(),
                22.0, 1e-3);
    EXPECT_GT(xl_out.signature.pattern.at("cb_dia_mm").get<double>(),
              s_out.signature.pattern.at("cb_dia_mm").get<double>());
}
