// @lat: [[process/test-strategy#hydraulic ports — orfs_face_seal_port]]
//
// orfs_face_seal_port — ORFS face-seal port: UN/UNF thread bore + flow bore
// + face O-ring groove + chamfered start.
//
// Tests:
//   1. apply removes volume within ±20 % of derived sum-of-parts.
//   2. validate rejects unknown orfs_dash (DFM-ORFS-CODE).
//   3. validate rejects thin face (DFM-ORFS-DEPTH).
//   4. signature carries is_compound + port_standard + size_code + groove.
//   5. recognize replays metadata at confidence 1.0.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/orfs_face_seal_port.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::orfs_face_seal_port::Input goodInput()
{
    skill::orfs_face_seal_port::Input in;
    in.face_id       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 40.0;
    in.position_y_mm = 40.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.orfs_dash     = "-08";
    in.depth_mm      = 14.0;
    return in;
}
}  // namespace

// ─── 1. apply produces volume change ──────────────────────────────────────
TEST(SkillOrfsFaceSealPort, ApplyProducesVolumeChangeAndExpectedRemoval)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 22.0);
    const double v0 = volumeOf(stock->shape());
    const int    f0 = stock->faceCount();

    auto out = skill::orfs_face_seal_port::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    const double dv = v0 - v1;
    EXPECT_GT(dv, 0.0);
    EXPECT_NE(out.workpiece->faceCount(), f0);

    const double derived = out.signature.pattern.at("derived_volume_removed")
                            .get<double>();
    EXPECT_GT(derived, 0.0);
    EXPECT_NEAR(dv, derived, derived * 0.20);
}

// ─── 2. validate rejects unknown orfs_dash ────────────────────────────────
TEST(SkillOrfsFaceSealPort, ValidateRejectsUnknownOrfsDash)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 22.0);
    auto in = goodInput();
    in.orfs_dash = "-99";
    auto r = skill::orfs_face_seal_port::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ORFS-CODE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::orfs_face_seal_port::apply(*stock, in), skill::SkillError);
}

// ─── 3. validate rejects thin face ────────────────────────────────────────
TEST(SkillOrfsFaceSealPort, ValidateRejectsTooThinFace)
{
    auto thin = skill::createCuboidStock(80.0, 80.0, 10.0);
    auto in = goodInput();
    auto r = skill::orfs_face_seal_port::validate(*thin, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ORFS-DEPTH") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature compound + standard + size + groove ID < OD ─────────────
TEST(SkillOrfsFaceSealPort, SignatureRecordsCompoundKindAndGrooveOrder)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 22.0);
    auto out   = skill::orfs_face_seal_port::apply(*stock, goodInput());

    EXPECT_EQ(out.signature.skill_id, std::string("orfs_face_seal_port"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("port_standard").get<std::string>(),
              std::string("ORFS_SAE_J1453"));
    EXPECT_EQ(out.signature.pattern.at("size_code").get<std::string>(),
              std::string("-08"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 4);

    // Groove invariant: ID < OD, depth > 0.
    const double gID = out.signature.pattern.at("oring_groove_id_mm").get<double>();
    const double gOD = out.signature.pattern.at("oring_groove_od_mm").get<double>();
    const double gD  = out.signature.pattern.at("oring_groove_depth_mm").get<double>();
    EXPECT_LT(gID, gOD);
    EXPECT_GT(gD, 0.0);
}

// ─── 5. recognize replays metadata at confidence 1.0 ──────────────────────
TEST(SkillOrfsFaceSealPort, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 22.0);
    auto out   = skill::orfs_face_seal_port::apply(*stock, goodInput());

    auto cands = skill::orfs_face_seal_port::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands.front().confidence, 1.0, 1e-9);
    EXPECT_EQ(cands.front().recovered_params["orfs_dash"].get<std::string>(),
              std::string("-08"));
}
