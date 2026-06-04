// @lat: [[process/test-strategy#compound valve_seat_insert_compound]]
//
// valve_seat_insert_compound — engine head 3-angle valve seat (4 sub-features:
// P7 press-fit pocket + top cone + seat cone + throat cone).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/valve_seat_insert_compound.hpp"

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

skill::valve_seat_insert_compound::Input goodInput(const skill::Workpiece& wp)
{
    skill::valve_seat_insert_compound::Input in;
    in.face_id              = findTopFaceId(wp);
    in.center_x_mm          = 25.0;
    in.center_y_mm          = 25.0;
    in.valve_head_dia_mm    = 30.0;
    in.seat_press_fit_dia_mm = 34.0;
    in.seat_depth_mm        = 6.0;
    in.top_angle_deg        = 30.0;
    in.seat_angle_deg       = 45.0;
    in.throat_angle_deg     = 60.0;
    return in;
}

}  // namespace

// ─── 1. apply removes material and adds faces ────────────────────────────
TEST(SkillValveSeatInsert, ApplyCutsThreeAnglePocket)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    const double v0 = volumeOf(stock->shape());
    auto out = skill::valve_seat_insert_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // material removed
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. validate rejects invalid valve head dia ──────────────────────────
TEST(SkillValveSeatInsert, ValidateRejectsValveHeadOutOfRange)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    auto in    = goodInput(*stock);
    in.valve_head_dia_mm = 10.0;  // < 20

    auto r = skill::valve_seat_insert_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-VALVE-HEAD-DIA") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::valve_seat_insert_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. signature records engine_feature_type + 4 subfeatures ────────────
TEST(SkillValveSeatInsert, SignatureRecordsEngineFeature)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    auto in    = goodInput(*stock);
    auto out   = skill::valve_seat_insert_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("valve_seat_insert_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("engine_feature_type")
              .get<std::string>(),
              std::string("valve_seat_insert"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 4);
    EXPECT_NEAR(out.signature.pattern.at("valve_head_dia_mm").get<double>(),
                30.0, 1e-6);
    // P7 bounds should be < nominal (P7 is interference).
    EXPECT_LT(out.signature.pattern.at("seat_press_fit_p7_max_mm").get<double>(),
              in.seat_press_fit_dia_mm);
}

// ─── 4. recognize via metadata replay ────────────────────────────────────
TEST(SkillValveSeatInsert, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    auto in    = goodInput(*stock);
    auto out   = skill::valve_seat_insert_compound::apply(*stock, in);

    auto cands = skill::valve_seat_insert_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("valve_head_dia_mm").get<double>(),
              30.0);
}

// ─── 5. SPECIFIC: seat press-fit dia must exceed valve head dia ──────────
TEST(SkillValveSeatInsert, ValidateRejectsSeatSmallerThanValve)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    auto in    = goodInput(*stock);
    in.seat_press_fit_dia_mm = in.valve_head_dia_mm - 1.0;

    auto r = skill::valve_seat_insert_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SEAT-OD") { found = true; break; }
    EXPECT_TRUE(found);
}
