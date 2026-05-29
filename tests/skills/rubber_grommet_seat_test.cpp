// @lat: [[process/test-strategy#compound feature]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/rubber_grommet_seat.hpp"

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

// ─── 1. ApplyProducesValidShapeWithMultipleSubFeatures ──────────────────
TEST(SkillRubberGrommetSeat, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);

    skill::rubber_grommet_seat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.hole_dia_mm   = 8.0;
    in.plate_t_mm    = 5.0;

    auto out = skill::rubber_grommet_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume removed ≥ through-hole volume.
    const double pilotApprox = M_PI * 4.0 * 4.0 * 5.0;  // ~251 mm³
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, pilotApprox * 0.9);

    // Face count grew (chamfer + groove + through cylinder add faces).
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount() + 2);

    EXPECT_EQ(out.signature.skill_id, std::string("rubber_grommet_seat"));
}

// ─── 2. ValidateRejectsBadInput ─────────────────────────────────────────
TEST(SkillRubberGrommetSeat, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);

    skill::rubber_grommet_seat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.hole_dia_mm   = 2.0;   // < 3 mm → DFM-GROMMET
    in.plate_t_mm    = 5.0;

    auto r = skill::rubber_grommet_seat::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GROMMET") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::rubber_grommet_seat::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. SignatureRecordsCompoundKind ────────────────────────────────────
TEST(SkillRubberGrommetSeat, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);

    skill::rubber_grommet_seat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.hole_dia_mm   = 8.0;
    in.plate_t_mm    = 5.0;

    auto out = skill::rubber_grommet_seat::apply(*stock, in);
    const auto& pat = out.signature.pattern;

    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("rubber_grommet_seat"));
    EXPECT_TRUE(pat.at("is_compound").get<bool>());
    EXPECT_GE(pat.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(pat.at("subfeature_count").get<int>(), 3);
}

// ─── 4. RecognizeMetadataReplay ─────────────────────────────────────────
TEST(SkillRubberGrommetSeat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);
    skill::rubber_grommet_seat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.hole_dia_mm   = 10.0;
    in.plate_t_mm    = 5.0;
    auto out = skill::rubber_grommet_seat::apply(*stock, in);

    auto cands = skill::rubber_grommet_seat::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params.at("hole_dia_mm").get<double>(),
                10.0, 1e-6);
}

// ─── 5. specific: groove sits at mid-depth & groove_od > hole_dia ────────
TEST(SkillRubberGrommetSeat, GrooveGeometryAtMidDepth)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 6.0);
    skill::rubber_grommet_seat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.hole_dia_mm   = 10.0;
    in.plate_t_mm    = 6.0;
    auto out = skill::rubber_grommet_seat::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    // groove_z = zMax - plate_t/2 = 6 - 3 = 3 (stock top at zMax=6).
    EXPECT_NEAR(pat.at("groove_z_mm").get<double>(), 3.0, 1e-6);
    // groove_od > hole_dia (annular groove always wider than bore).
    EXPECT_GT(pat.at("groove_od_mm").get<double>(),
              pat.at("hole_dia_mm").get<double>());
    EXPECT_NEAR(pat.at("groove_od_mm").get<double>(),
                in.hole_dia_mm + 2.0 * 0.4, 1e-6);
}
