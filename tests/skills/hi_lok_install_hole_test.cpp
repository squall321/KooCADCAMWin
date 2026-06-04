// @lat: [[process/test-strategy#aerospace hi_lok_install_hole]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/hi_lok_install_hole.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

int findTopFaceId(const skill::Workpiece& wp) {
    int best = -1;
    double bestZ = -1e9;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        const auto n = wp.faceNormal(i);
        if (n.Z() < 0.9) continue;
        const auto c = wp.faceCenter(i);
        if (c.Z() > bestZ) { bestZ = c.Z(); best = i; }
    }
    return best;
}

skill::hi_lok_install_hole::Input goodInput(const skill::Workpiece& wp) {
    skill::hi_lok_install_hole::Input in;
    in.face_id           = findTopFaceId(wp);
    in.center_x_mm       = 25.0;
    in.center_y_mm       = 25.0;
    in.fastener_dia_mm   = 4.8;
    in.countersink_style = "100";
    return in;
}
}  // namespace

// ─── 1. apply removes material with 100° countersink ────────────────────
TEST(SkillHiLok, ApplyWith100CskRemovesMaterial)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 8.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    const double v0 = volumeOf(stock->shape());
    auto out = skill::hi_lok_install_hole::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
}

// ─── 2. apply with "none" style: still cuts a single drill ──────────────
TEST(SkillHiLok, ApplyWithNoneStyle)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 8.0);
    auto in    = goodInput(*stock);
    in.countersink_style = "none";

    const double v0 = volumeOf(stock->shape());
    auto out = skill::hi_lok_install_hole::apply(*stock, in);
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 1);
    EXPECT_FALSE(out.signature.pattern.at("is_compound").get<bool>());
}

// ─── 3. validate rejects non-NAS1769 dia ────────────────────────────────
TEST(SkillHiLok, ValidateRejectsBadDia)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 8.0);
    auto in    = goodInput(*stock);
    in.fastener_dia_mm = 5.0;  // not in set

    auto r = skill::hi_lok_install_hole::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-FAST-DIA") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. validate rejects unknown countersink style ──────────────────────
TEST(SkillHiLok, ValidateRejectsBadCskStyle)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 8.0);
    auto in    = goodInput(*stock);
    in.countersink_style = "82";  // valid for ASME but not for Hi-Lok

    auto r = skill::hi_lok_install_hole::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CSK-STYLE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::hi_lok_install_hole::apply(*stock, in),
                 skill::SkillError);
}

// ─── 5. Signature + recognize ───────────────────────────────────────────
TEST(SkillHiLok, SignatureAndRecognize)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 8.0);
    auto in    = goodInput(*stock);
    in.countersink_style = "130";

    auto out = skill::hi_lok_install_hole::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("hi_lok_install_hole"));
    EXPECT_EQ(out.signature.pattern.at("aerospace_feature_type").get<std::string>(),
              std::string("hi_lok_install"));
    EXPECT_EQ(out.signature.pattern.at("countersink_style").get<std::string>(),
              std::string("130"));
    EXPECT_NEAR(out.signature.pattern.at("fastener_dia").get<double>(), 4.8, 1e-6);

    auto cands = skill::hi_lok_install_hole::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
