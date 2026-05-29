// @lat: [[process/test-strategy#compound fastener-seat]]
//
// countersunk_bolt_seat — COMPOUND macro: through drill + countersink + optional relief.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/countersunk_bolt_seat.hpp"

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

// ─── 1. apply produces valid shape with N sub-features removed ────────────
TEST(SkillCountersunkBoltSeat, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double stockVol = volumeOf(stock->shape());

    skill::countersunk_bolt_seat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.fastener_size = "M4";
    in.csk_angle_deg = 90.0;
    in.with_relief   = false;

    auto out = skill::countersunk_bolt_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected: through pilot (4.5 dia × 10 deep) + 90° cone frustum addition.
    // Frustum from headDia=7.5 → pilot=4.5 has cone_depth = (7.5-4.5)/2 / tan(45) = 1.5 mm
    // Frustum volume - the pilot core within cone_depth.
    const double pilotDia = 4.5, headDia = 7.5;
    const double coneDep  = (headDia - pilotDia) / 2.0 / std::tan(45.0 * M_PI / 180.0);
    const double pilotR = pilotDia / 2.0, headR = headDia / 2.0;
    const double pilotVol  = M_PI * pilotR * pilotR * 10.0;
    const double frustVol  = (M_PI * coneDep / 3.0) *
                             (headR * headR + headR * pilotR + pilotR * pilotR);
    const double coneExtra = frustVol - M_PI * pilotR * pilotR * coneDep;
    const double approx    = pilotVol + std::max(0.0, coneExtra);

    const double removed = stockVol - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.05);

    // Face count must increase (more topology after the compound cut).
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());

    // Signature must be marked compound, subfeature_count >= 2.
    EXPECT_EQ(out.signature.skill_id, std::string("countersunk_bolt_seat"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
}

// ─── 2. validate rejects bad input ───────────────────────────────────────
TEST(SkillCountersunkBoltSeat, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // (a) Unknown size
    {
        skill::countersunk_bolt_seat::Input in;
        in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 25.0;
        in.position_y_mm = 25.0;
        in.fastener_size = "M99";
        auto r = skill::countersunk_bolt_seat::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::countersunk_bolt_seat::apply(*stock, in), skill::SkillError);
    }
    // (b) Non-standard cone angle
    {
        skill::countersunk_bolt_seat::Input in;
        in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 25.0;
        in.position_y_mm = 25.0;
        in.fastener_size = "M3";
        in.csk_angle_deg = 110.0;   // not in {82, 90, 100}
        auto r = skill::countersunk_bolt_seat::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-CSK-ANGLE") { found = true; break; }
        EXPECT_TRUE(found);
    }
    // (c) Stock too thin for head
    {
        auto thin = skill::createCuboidStock(50.0, 50.0, 1.0);
        skill::countersunk_bolt_seat::Input in;
        in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 25.0;
        in.position_y_mm = 25.0;
        in.fastener_size = "M8";    // head 14.5 mm; thin stock 1 mm
        auto r = skill::countersunk_bolt_seat::validate(*thin, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-CSK-THICKNESS") { found = true; break; }
        EXPECT_TRUE(found);
    }
}

// ─── 3. Signature records compound kind ──────────────────────────────────
TEST(SkillCountersunkBoltSeat, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::countersunk_bolt_seat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.fastener_size = "M5";
    in.csk_angle_deg = 82.0;       // UNC
    in.with_relief   = true;       // adds a 3rd sub-feature

    auto out = skill::countersunk_bolt_seat::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("countersunk_bolt_seat"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
    EXPECT_EQ(out.signature.pattern.at("fastener_size").get<std::string>(),
              std::string("M5"));
}

// ─── 4. recognize replays metadata at confidence 1.0 ─────────────────────
TEST(SkillCountersunkBoltSeat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::countersunk_bolt_seat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.fastener_size = "M3";

    auto out = skill::countersunk_bolt_seat::apply(*stock, in);
    auto cands = skill::countersunk_bolt_seat::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("fastener_size").get<std::string>(),
              std::string("M3"));
}

// ─── 5. Specific: head_dia matches M-spec table ──────────────────────────
TEST(SkillCountersunkBoltSeat, HeadDiameterMatchesIsoTable)
{
    // Validate the lookup helpers map to the published DIN 965 / ISO 7046 OD.
    EXPECT_NEAR(skill::countersunk_bolt_seat::headDiameterFor("M3"), 5.6,  1e-3);
    EXPECT_NEAR(skill::countersunk_bolt_seat::headDiameterFor("M4"), 7.5,  1e-3);
    EXPECT_NEAR(skill::countersunk_bolt_seat::headDiameterFor("M6"), 11.0, 1e-3);

    EXPECT_NEAR(skill::countersunk_bolt_seat::clearanceDiameterFor("M3"), 3.4, 1e-3);
    EXPECT_NEAR(skill::countersunk_bolt_seat::clearanceDiameterFor("M4"), 4.5, 1e-3);

    // Apply with M4 → pattern.pilot_dia_mm equals table clearance.
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::countersunk_bolt_seat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0; in.position_y_mm = 25.0;
    in.fastener_size = "M4";
    auto out = skill::countersunk_bolt_seat::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern.at("pilot_dia_mm").get<double>(), 4.5, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("head_dia_mm").get<double>(),  7.5, 1e-6);
}
