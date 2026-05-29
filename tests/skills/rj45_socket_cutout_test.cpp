// @lat: [[process/test-strategy#compound feature]]
//
// rj45_socket_cutout — 3-cut compound per port: main pocket + tab relief +
// front chamfer.
//
// Test cases:
//   1. ApplyProducesValidShapeWithMultipleSubFeatures
//   2. ValidateRejectsBadInput — port pitch too tight
//   3. SignatureRecordsCompoundKind
//   4. RecognizeMetadataReplay
//   5. PocketSizeMatchesTIA568 — 13.0 × 14.5 mm

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/rj45_socket_cutout.hpp"

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

// ─── 1. Apply produces valid shape ────────────────────────────────────────
TEST(SkillRj45SocketCutout, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 3.0);  // 3 mm panel ≤ 5

    skill::rj45_socket_cutout::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.port_count    = 2;
    in.spacing_mm    = 16.0;
    in.direction_deg = 0.0;

    const int facesBefore = stock->faceCount();
    auto out = skill::rj45_socket_cutout::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Per port: main pocket 13×14.5×3 + tab 3×1×2 + chamfer ≈ 13.6 mm³
    const double perPort = 13.0 * 14.5 * 3.0 + 3.0 * 1.0 * 2.0 + 13.6;
    const double approx  = perPort * 2;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.2);

    EXPECT_GT(out.workpiece->faceCount(), facesBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("rj45_socket_cutout"));
}

// ─── 2. Validate rejects too-tight pitch ──────────────────────────────────
TEST(SkillRj45SocketCutout, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 3.0);
    skill::rj45_socket_cutout::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.port_count    = 3;
    in.spacing_mm    = 10.0;     // < 14 mm TIA min

    auto r = skill::rj45_socket_cutout::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-RJ45-PITCH") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::rj45_socket_cutout::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records compound kind ───────────────────────────────────
TEST(SkillRj45SocketCutout, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 3.0);
    skill::rj45_socket_cutout::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.port_count    = 1;
    auto out = skill::rj45_socket_cutout::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    EXPECT_TRUE(pat.at("is_compound").get<bool>());
    EXPECT_GE(pat.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("rj45_socket_cutout"));
}

// ─── 4. Recognize: metadata replay → confidence 1.0 ───────────────────────
TEST(SkillRj45SocketCutout, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 3.0);
    skill::rj45_socket_cutout::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.port_count    = 4;
    auto out = skill::rj45_socket_cutout::apply(*stock, in);

    auto cands = skill::rj45_socket_cutout::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["port_count"].get<int>(), 4);
}

// ─── 5. Pocket dimensions match TIA/EIA-568 8P8C ──────────────────────────
TEST(SkillRj45SocketCutout, PocketSizeMatchesTIA568)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 3.0);
    skill::rj45_socket_cutout::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.port_count    = 1;
    auto out = skill::rj45_socket_cutout::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    EXPECT_NEAR(pat.at("pocket_width_mm").get<double>(),  13.0, 0.05);
    EXPECT_NEAR(pat.at("pocket_height_mm").get<double>(), 14.5, 0.05);
    EXPECT_NEAR(pat.at("tab_relief_w_mm").get<double>(),   3.0, 0.05);
    EXPECT_GE(pat.at("chamfer_mm").get<double>(),          0.3);
}
