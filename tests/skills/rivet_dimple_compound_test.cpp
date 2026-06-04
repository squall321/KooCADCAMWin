// @lat: [[process/test-strategy#aerospace rivet_dimple_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/rivet_dimple_compound.hpp"

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

skill::rivet_dimple_compound::Input goodInput(const skill::Workpiece& wp) {
    skill::rivet_dimple_compound::Input in;
    in.face_id            = findTopFaceId(wp);
    in.center_x_mm        = 25.0;
    in.center_y_mm        = 25.0;
    in.rivet_dia_mm       = 4.0;
    in.sheet_thickness_mm = 2.0;
    in.dimple_angle_deg   = 100.0;
    return in;
}
}  // namespace

// ─── 1. apply: v0 - v1 > 0 (material removed) ───────────────────────────
TEST(SkillRivetDimple, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 3.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    const double v0 = volumeOf(stock->shape());
    auto out = skill::rivet_dimple_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);                        // material removed
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. validate rejects non-standard rivet dia ─────────────────────────
TEST(SkillRivetDimple, ValidateRejectsBadDia)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 3.0);
    auto in    = goodInput(*stock);
    in.rivet_dia_mm = 5.0;  // not in MS20426 {3.2, 4.0, 4.8}

    auto r = skill::rivet_dimple_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-RIVET-DIA") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::rivet_dimple_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. validate rejects non-standard dimple angle ──────────────────────
TEST(SkillRivetDimple, ValidateRejectsBadAngle)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 3.0);
    auto in    = goodInput(*stock);
    in.dimple_angle_deg = 90.0;  // not in MS33558 {82, 100, 120}

    auto r = skill::rivet_dimple_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DIMPLE-ANGLE") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. validate rejects thin sheet ─────────────────────────────────────
TEST(SkillRivetDimple, ValidateRejectsThinSheet)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 0.6);
    auto in    = goodInput(*stock);
    in.sheet_thickness_mm = 0.5;  // < 4.0 × 0.3 = 1.2

    auto r = skill::rivet_dimple_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SHEET-THICKNESS") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 5. Signature records compound + aerospace feature ──────────────────
TEST(SkillRivetDimple, SignatureRecordsAerospaceCompound)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 3.0);
    auto in    = goodInput(*stock);
    auto out   = skill::rivet_dimple_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("rivet_dimple_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("aerospace_feature_type").get<std::string>(),
              std::string("flush_rivet_dimple"));
    EXPECT_NEAR(out.signature.pattern.at("fastener_dia").get<double>(), 4.0, 1e-6);
    EXPECT_GT(out.signature.pattern.at("derived_volume_removed_mm3").get<double>(), 0.0);

    // Recognize via metadata replay
    auto cands = skill::rivet_dimple_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
