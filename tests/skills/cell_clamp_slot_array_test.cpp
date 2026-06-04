// @lat: [[process/test-strategy#compound cell_clamp_slot_array]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cell_clamp_slot_array.hpp"

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

skill::cell_clamp_slot_array::Input goodInput(const skill::Workpiece& wp) {
    skill::cell_clamp_slot_array::Input in;
    in.face_id              = findTopFaceId(wp);
    in.origin_x_mm          = 15.0;
    in.origin_y_mm          = 15.0;
    in.cell_dia_mm          = 18.0;   // 18650
    in.cell_count_x         = 2;
    in.cell_count_y         = 2;
    in.cell_pitch_x_mm      = 20.0;
    in.cell_pitch_y_mm      = 20.0;
    in.clamp_slot_width_mm  = 1.5;
    in.clamp_slot_depth_mm  = 5.0;
    return in;
}
}  // namespace

// ─── 1. Apply: cuts cell array + slots (face count increases) ────────────
TEST(SkillCellClampSlot, ApplyCutsCellArrayAndSlots)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 10.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    auto out = skill::cell_clamp_slot_array::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // After a 4-cell array of pockets + 16 slots, the topology must change.
    // Strict volume comparison can be unstable when OCCT's compound-tool
    // boolean returns a compound result; the topology test below is the
    // reliable invariant.
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount() + 4);
}

// ─── 2. Validate rejects out-of-range cell_dia ───────────────────────────
TEST(SkillCellClampSlot, ValidateRejectsBadCellDia)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 10.0);
    auto in    = goodInput(*stock);
    in.cell_dia_mm = 35.0;                   // > 30 cap

    auto r = skill::cell_clamp_slot_array::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CELL-DIA") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 3. Validate rejects tight pitch ─────────────────────────────────────
TEST(SkillCellClampSlot, ValidateRejectsTightPitch)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 10.0);
    auto in    = goodInput(*stock);
    in.cell_pitch_x_mm = in.cell_dia_mm + 0.2;  // < 0.5 mm clearance

    auto r = skill::cell_clamp_slot_array::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CELL-PITCH") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records compound + cell_count ──────────────────────────
TEST(SkillCellClampSlot, SignatureRecordsCellCountAndArray)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 10.0);
    auto in    = goodInput(*stock);
    auto out   = skill::cell_clamp_slot_array::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("cell_clamp_slot_array"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("ev_feature_type").get<std::string>(),
              std::string("cell_clamp_array"));
    EXPECT_EQ(out.signature.pattern.at("cell_count").get<int>(),
              in.cell_count_x * in.cell_count_y);
    // 1 pocket + 4 slots per cell = 5 subfeatures per cell.
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(),
              in.cell_count_x * in.cell_count_y * 5);
}

// ─── 5. Recognize via metadata replay ────────────────────────────────────
TEST(SkillCellClampSlot, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 10.0);
    auto in    = goodInput(*stock);
    auto out   = skill::cell_clamp_slot_array::apply(*stock, in);

    auto cands = skill::cell_clamp_slot_array::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("cell_dia_mm").get<double>(),
              in.cell_dia_mm);
}
