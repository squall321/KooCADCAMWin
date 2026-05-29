// @lat: [[process/test-strategy#compound macro]]
//
// concealed_hinge_cup — compound: 35 mm cup + 2 fixing pilots @ 32 mm pitch.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/concealed_hinge_cup.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. Apply removes ~ cup + 2 pilot volumes ──────────────────────────────
TEST(SkillConcealedHingeCup, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 18.0);

    skill::concealed_hinge_cup::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 50.0;
    in.position_y_mm = 30.0;
    in.edge_offset_mm = 6.0;

    const int origFaces = stock->faceCount();
    auto out = skill::concealed_hinge_cup::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected volume removed:
    //  cup  = π × 17.5² × 11.5 ≈ 11066
    //  pilots = 2 × π × 1.75² × 10 ≈ 192
    const double cupV   = M_PI * 17.5 * 17.5 * 11.5;
    const double pilotV = 2.0 * M_PI * 1.75 * 1.75 * 10.0;
    const double expected = cupV + pilotV;
    const double removed  = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.05);

    EXPECT_GT(out.workpiece->faceCount(), origFaces);
}

// ─── 2. Validate rejects too-thin panel ────────────────────────────────────
TEST(SkillConcealedHingeCup, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 10.0);  // < 14 mm

    skill::concealed_hinge_cup::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 50.0;
    in.position_y_mm = 30.0;

    auto r = skill::concealed_hinge_cup::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-HINGE-CUP") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::concealed_hinge_cup::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records compound metadata ────────────────────────────────
TEST(SkillConcealedHingeCup, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 18.0);

    skill::concealed_hinge_cup::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 50.0;
    in.position_y_mm = 30.0;
    auto out = skill::concealed_hinge_cup::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("concealed_hinge_cup"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("fixing_positions").size(), 2u);
}

// ─── 4. Recognize replays with confidence 1.0 ──────────────────────────────
TEST(SkillConcealedHingeCup, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 18.0);

    skill::concealed_hinge_cup::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 50.0;
    in.position_y_mm = 30.0;
    auto out = skill::concealed_hinge_cup::apply(*stock, in);

    auto cands = skill::concealed_hinge_cup::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}

// ─── 5. Specific: 32 mm pitch + 35 mm cup match Euro system ───────────────
TEST(SkillConcealedHingeCup, EuroSystem32mmPitchAnd35mmCup)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 18.0);

    skill::concealed_hinge_cup::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 50.0;
    in.position_y_mm = 30.0;
    auto out = skill::concealed_hinge_cup::apply(*stock, in);

    const double pitch = out.signature.pattern.at("fixing_pitch_mm").get<double>();
    const double cup   = out.signature.pattern.at("cup_dia_mm").get<double>();
    EXPECT_NEAR(pitch, 32.0, 1e-6);
    EXPECT_NEAR(cup,   35.0, 1e-6);
}
