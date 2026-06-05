// @lat: [[process/test-strategy#compound busbar_terminal_lug_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/busbar_terminal_lug_compound.hpp"

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

skill::busbar_terminal_lug_compound::Input goodInput(const skill::Workpiece& wp) {
    skill::busbar_terminal_lug_compound::Input in;
    in.face_id             = findTopFaceId(wp);
    in.center_x_mm         = 50.0;
    in.center_y_mm         = 50.0;
    in.busbar_width_mm     = 40.0;
    in.busbar_thickness_mm = 5.0;
    in.lug_terminal_count  = 2;
    in.terminal_dia_mm     = 10.0;
    in.terminal_pcd_mm     = 16.0;
    return in;
}
}  // namespace

// ─── 1. Apply: removes busbar slot + terminal holes ──────────────────────
TEST(SkillBusbarTerminalLug, ApplyCutsSlotAndTerminals)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 15.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    auto out = skill::busbar_terminal_lug_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v0 = volumeOf(stock->shape());
    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_LT(v1, v0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects non-standard terminal_dia ───────────────────────
TEST(SkillBusbarTerminalLug, ValidateRejectsNonStandardTermDia)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 15.0);
    auto in    = goodInput(*stock);
    in.terminal_dia_mm = 7.0;       // not in {8, 10, 12}

    auto r = skill::busbar_terminal_lug_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LUG-TERMINAL-DIA") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 3. Validate rejects thin busbar ─────────────────────────────────────
TEST(SkillBusbarTerminalLug, ValidateRejectsThinBusbar)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 15.0);
    auto in    = goodInput(*stock);
    in.busbar_thickness_mm = 2.0;     // < 3 mm

    auto r = skill::busbar_terminal_lug_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-IEEE605") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records compound + terminal count ──────────────────────
TEST(SkillBusbarTerminalLug, SignatureRecordsTerminalCount)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 15.0);
    auto in    = goodInput(*stock);
    auto out   = skill::busbar_terminal_lug_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("busbar_terminal_lug_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("solar_feature_type").get<std::string>(),
              std::string("busbar_terminal_lug"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(),
              1 + in.lug_terminal_count);
    EXPECT_EQ(out.signature.pattern.at("terminal_count").get<int>(),
              in.lug_terminal_count);
}

// ─── 5. Recognize via metadata replay ────────────────────────────────────
TEST(SkillBusbarTerminalLug, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 15.0);
    auto in    = goodInput(*stock);
    auto out   = skill::busbar_terminal_lug_compound::apply(*stock, in);

    auto cands = skill::busbar_terminal_lug_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("terminal_dia_mm").get<double>(),
              in.terminal_dia_mm);
}
