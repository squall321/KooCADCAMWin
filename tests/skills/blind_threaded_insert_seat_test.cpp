// @lat: [[process/test-strategy#compound fastener-seat]]
//
// blind_threaded_insert_seat — COMPOUND macro: drill_pilot + counterbore
// head seat + 45° chamfer + bottom binder relief well.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/blind_threaded_insert_seat.hpp"

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

// ─── 1. apply produces valid shape with N sub-features ───────────────────
TEST(SkillBlindThreadedInsertSeat, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);
    const int stockFaceCount = stock->faceCount();
    const double stockVol = volumeOf(stock->shape());

    skill::blind_threaded_insert_seat::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.insert_size      = "M4";
    in.insert_material  = "metal";

    auto out = skill::blind_threaded_insert_seat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // M4: pilot OD 5.5 mm, head 7.0, engagement 7.0, binder 0.8 mm,
    // headSeatDepth 0.5, binder dia = 5.5 + 0.4 = 5.9.
    const double pilotDia    = 5.5;
    const double headDia     = 7.0;
    const double engagement  = 7.0;
    const double headSeatDep = 0.5;
    const double binderDep   = 0.8;
    const double binderDia   = 5.9;

    const double pilotR  = pilotDia  / 2.0;
    const double headR   = headDia   / 2.0;
    const double binderR = binderDia / 2.0;
    const double pilotDepth = headSeatDep + engagement;
    const double approx =
        M_PI * pilotR * pilotR * pilotDepth +
        M_PI * (headR * headR - pilotR * pilotR) * headSeatDep +
        M_PI * (binderR * binderR - pilotR * pilotR) * binderDep;

    const double removed = stockVol - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.12);  // ±12% allowance for chamfer + numerics

    EXPECT_GT(out.workpiece->faceCount(), stockFaceCount);
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 4);
}

// ─── 2. validate rejects bad input ───────────────────────────────────────
TEST(SkillBlindThreadedInsertSeat, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    // (a) Unknown insert size
    {
        skill::blind_threaded_insert_seat::Input in;
        in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.insert_size      = "M99";
        auto r = skill::blind_threaded_insert_seat::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::blind_threaded_insert_seat::apply(*stock, in), skill::SkillError);
    }
    // (b) Unknown material
    {
        skill::blind_threaded_insert_seat::Input in;
        in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 25.0; in.position_y_mm = 25.0;
        in.insert_size      = "M4";
        in.insert_material  = "ceramic_unknown";
        auto r = skill::blind_threaded_insert_seat::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-INSERT-MATERIAL") { found = true; break; }
        EXPECT_TRUE(found);
    }
    // (c) Too thin stock
    {
        auto thin = skill::createCuboidStock(50.0, 50.0, 2.0);
        skill::blind_threaded_insert_seat::Input in;
        in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 25.0; in.position_y_mm = 25.0;
        in.insert_size      = "M6";
        in.insert_material  = "metal";
        auto r = skill::blind_threaded_insert_seat::validate(*thin, in);
        EXPECT_FALSE(r.passed);
    }
}

// ─── 3. Signature records compound kind ──────────────────────────────────
TEST(SkillBlindThreadedInsertSeat, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::blind_threaded_insert_seat::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.insert_size      = "M3";
    in.insert_material  = "thermoplastic";

    auto out = skill::blind_threaded_insert_seat::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("blind_threaded_insert_seat"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
}

// ─── 4. recognize replays metadata at confidence 1.0 ─────────────────────
TEST(SkillBlindThreadedInsertSeat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::blind_threaded_insert_seat::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 25.0;
    in.position_y_mm    = 25.0;
    in.insert_size      = "M5";
    in.insert_material  = "composite_fbg";

    auto out = skill::blind_threaded_insert_seat::apply(*stock, in);
    auto cands = skill::blind_threaded_insert_seat::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("insert_size").get<std::string>(),
              std::string("M5"));
    EXPECT_EQ(cands[0].recovered_params.at("insert_material").get<std::string>(),
              std::string("composite_fbg"));
}

// ─── 5. Specific: composite_fbg material widens binder relief 1.5× ───────
TEST(SkillBlindThreadedInsertSeat, MaterialModifiesBinderReservoir)
{
    // Material-slip table values.
    EXPECT_DOUBLE_EQ(skill::blind_threaded_insert_seat::materialSlipFor("metal"),         0.0);
    EXPECT_DOUBLE_EQ(skill::blind_threaded_insert_seat::materialSlipFor("thermoplastic"), 0.1);
    EXPECT_DOUBLE_EQ(skill::blind_threaded_insert_seat::materialSlipFor("thermoset"),     0.2);
    EXPECT_DOUBLE_EQ(skill::blind_threaded_insert_seat::materialSlipFor("composite_fbg"), 0.3);
    EXPECT_LT       (skill::blind_threaded_insert_seat::materialSlipFor("unknown_xyz"),   0.0);

    // M4 nominal binder reservoir = 0.8 mm.
    EXPECT_NEAR(skill::blind_threaded_insert_seat::binderReservoirFor("M4"), 0.8, 1e-3);
    EXPECT_NEAR(skill::blind_threaded_insert_seat::insertOuterDiameterFor("M4"), 5.5, 1e-3);
    EXPECT_NEAR(skill::blind_threaded_insert_seat::insertHeadDiameterFor ("M4"), 7.0, 1e-3);
    EXPECT_NEAR(skill::blind_threaded_insert_seat::insertEngagementFor   ("M4"), 7.0, 1e-3);

    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    // Apply with metal — binder depth = 0.8 mm.
    skill::blind_threaded_insert_seat::Input metalIn;
    metalIn.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    metalIn.position_x_mm    = 15.0;
    metalIn.position_y_mm    = 25.0;
    metalIn.insert_size      = "M4";
    metalIn.insert_material  = "metal";
    auto outMetal = skill::blind_threaded_insert_seat::apply(*stock, metalIn);
    EXPECT_NEAR(outMetal.signature.pattern.at("binder_depth_mm").get<double>(),
                0.8, 1e-6);

    // Apply with composite_fbg — binder depth = 0.8 × 1.5 = 1.2 mm.
    skill::blind_threaded_insert_seat::Input compIn;
    compIn.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    compIn.position_x_mm    = 35.0;
    compIn.position_y_mm    = 25.0;
    compIn.insert_size      = "M4";
    compIn.insert_material  = "composite_fbg";
    auto outComp = skill::blind_threaded_insert_seat::apply(*stock, compIn);
    EXPECT_NEAR(outComp.signature.pattern.at("binder_depth_mm").get<double>(),
                1.2, 1e-6);

    // Verify head_dia > pilot_dia for both.
    EXPECT_GT(outMetal.signature.pattern.at("head_dia_mm").get<double>(),
              outMetal.signature.pattern.at("pilot_dia_mm").get<double>());
    EXPECT_GT(outComp.signature.pattern.at("head_dia_mm").get<double>(),
              outComp.signature.pattern.at("pilot_dia_mm").get<double>());
}
