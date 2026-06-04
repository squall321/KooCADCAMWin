// @lat: [[process/test-strategy#aerospace nas_bolt_pattern_circular]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/nas_bolt_pattern_circular.hpp"

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

skill::nas_bolt_pattern_circular::Input goodInput(const skill::Workpiece& wp) {
    skill::nas_bolt_pattern_circular::Input in;
    in.face_id                   = findTopFaceId(wp);
    in.center_x_mm               = 50.0;
    in.center_y_mm               = 50.0;
    in.pcd_mm                    = 60.0;
    in.bolt_count                = 6;
    in.bolt_dia_mm               = 6.35;
    in.bolt_hole_clearance_grade = "medium";
    return in;
}
}  // namespace

// ─── 1. apply: v0 - v1 > 0, N holes added ───────────────────────────────
TEST(SkillNasBoltPattern, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 8.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    const double v0 = volumeOf(stock->shape());
    auto out = skill::nas_bolt_pattern_circular::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    // Each hole adds 1 cyl face + 2 circular edges ⇒ face count grows.
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 6);
}

// ─── 2. validate rejects non-{4,6,8} bolt_count ─────────────────────────
TEST(SkillNasBoltPattern, ValidateRejectsBadCount)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 8.0);
    auto in    = goodInput(*stock);
    in.bolt_count = 5;

    auto r = skill::nas_bolt_pattern_circular::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BOLT-COUNT") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::nas_bolt_pattern_circular::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. validate rejects unknown clearance grade ────────────────────────
TEST(SkillNasBoltPattern, ValidateRejectsBadGrade)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 8.0);
    auto in    = goodInput(*stock);
    in.bolt_hole_clearance_grade = "tight";

    auto r = skill::nas_bolt_pattern_circular::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CLEARANCE-GRADE") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. clearance grades produce different hole diameters ───────────────
TEST(SkillNasBoltPattern, ClearanceGradeAffectsDia)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 8.0);
    auto in    = goodInput(*stock);

    in.bolt_hole_clearance_grade = "close";
    auto outC = skill::nas_bolt_pattern_circular::apply(*stock, in);
    in.bolt_hole_clearance_grade = "free";
    auto outF = skill::nas_bolt_pattern_circular::apply(*stock, in);

    const double diaC = outC.signature.pattern.at("clearance_dia_mm").get<double>();
    const double diaF = outF.signature.pattern.at("clearance_dia_mm").get<double>();
    EXPECT_LT(diaC, diaF);
    EXPECT_NEAR(diaC, in.bolt_dia_mm + 0.0, 1e-6);
    EXPECT_NEAR(diaF, in.bolt_dia_mm + 0.2, 1e-6);
}

// ─── 5. Signature pattern + recognize ───────────────────────────────────
TEST(SkillNasBoltPattern, SignatureAndRecognize)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 8.0);
    auto in    = goodInput(*stock);
    auto out   = skill::nas_bolt_pattern_circular::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("nas_bolt_pattern_circular"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("aerospace_feature_type").get<std::string>(),
              std::string("nas_bolt_circle"));
    EXPECT_EQ(out.signature.pattern.at("bolt_count").get<int>(), 6);
    EXPECT_NEAR(out.signature.pattern.at("pcd_mm").get<double>(), 60.0, 1e-6);

    auto cands = skill::nas_bolt_pattern_circular::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
