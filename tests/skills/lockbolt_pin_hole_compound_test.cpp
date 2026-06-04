// @lat: [[process/test-strategy#aerospace lockbolt_pin_hole_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/lockbolt_pin_hole_compound.hpp"

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

skill::lockbolt_pin_hole_compound::Input goodInput(const skill::Workpiece& wp) {
    skill::lockbolt_pin_hole_compound::Input in;
    in.face_id              = findTopFaceId(wp);
    in.center_x_mm          = 25.0;
    in.center_y_mm          = 25.0;
    in.pin_dia_mm           = 6.35;
    in.grip_length_mm       = 8.0;
    in.collar_clearance_mm  = 0.2;
    return in;
}
}  // namespace

// ─── 1. apply removes material (v0 - v1 > 0) ────────────────────────────
TEST(SkillLockboltPin, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    const double v0 = volumeOf(stock->shape());
    auto out = skill::lockbolt_pin_hole_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. validate rejects non-NAS1396 pin dia ────────────────────────────
TEST(SkillLockboltPin, ValidateRejectsBadDia)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in    = goodInput(*stock);
    in.pin_dia_mm = 5.0;  // not in NAS1396

    auto r = skill::lockbolt_pin_hole_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PIN-DIA") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 3. validate rejects insufficient grip ──────────────────────────────
TEST(SkillLockboltPin, ValidateRejectsThinGrip)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in    = goodInput(*stock);
    in.grip_length_mm = 2.0;  // < 6.35 × 0.6 = 3.81

    auto r = skill::lockbolt_pin_hole_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GRIP-LENGTH") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. validate rejects oversized collar clearance ─────────────────────
TEST(SkillLockboltPin, ValidateRejectsSloppyCollar)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in    = goodInput(*stock);
    in.collar_clearance_mm = 2.0;  // > pin_dia × 0.25 = 1.59

    auto r = skill::lockbolt_pin_hole_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-COLLAR-CLEAR") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 5. Signature + metadata replay ─────────────────────────────────────
TEST(SkillLockboltPin, SignatureAndRecognize)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in    = goodInput(*stock);
    auto out   = skill::lockbolt_pin_hole_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("lockbolt_pin_hole_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("aerospace_feature_type").get<std::string>(),
              std::string("lockbolt_pin_hole"));
    EXPECT_NEAR(out.signature.pattern.at("fastener_dia").get<double>(), 6.35, 1e-6);
    EXPECT_GT(out.signature.pattern.at("derived_volume_removed_mm3").get<double>(), 0.0);

    auto cands = skill::lockbolt_pin_hole_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
