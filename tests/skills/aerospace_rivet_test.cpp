// @lat: [[process/test-strategy#skill round-trip]]
//
// aerospace_rivet skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of out-of-range diameter, metadata-only recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/aerospace_rivet.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

skill::aerospace_rivet::Input goodInput()
{
    skill::aerospace_rivet::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 10.0;
    in.rivet_dia_mm  = 4.0;
    in.length_mm     = 12.0;
    in.alloy         = "AA2117";
    in.style         = "solid";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillAerospaceRivet, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::aerospace_rivet::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("aerospace_rivet"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (rivet_dia 1.5 mm < 2.4 minimum) ───────
TEST(SkillAerospaceRivet, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.rivet_dia_mm = 1.5;     // below 2.4 mm aerospace minimum
    in.length_mm    = 6.0;

    auto r = skill::aerospace_rivet::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-RIVET-DIA"));
    EXPECT_THROW(skill::aerospace_rivet::apply(*stock, in), skill::SkillError);

    // length check failure independently
    auto in2 = goodInput();
    in2.rivet_dia_mm = 4.0;
    in2.length_mm    = 3.0;    // < 1.5 × 4 = 6 mm minimum
    auto r2 = skill::aerospace_rivet::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-RIVET-LEN"));
}

// ─── 3. Signature records kind + dia/length/alloy/style ──────────────────
TEST(SkillAerospaceRivet, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.rivet_dia_mm = 4.8;
    in.length_mm    = 14.0;
    in.alloy        = "AA5056";
    in.style        = "blind";

    auto out = skill::aerospace_rivet::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("aerospace_rivet"));
    EXPECT_NEAR(out.signature.pattern["rivet_dia_mm"].get<double>(), 4.8, 1e-9);
    EXPECT_NEAR(out.signature.pattern["length_mm"].get<double>(),   14.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["alloy"].get<std::string>(),
              std::string("AA5056"));
    EXPECT_EQ(out.signature.pattern["style"].get<std::string>(),
              std::string("blind"));
    EXPECT_EQ(out.signature.pattern["head_style"].get<std::string>(),
              std::string("flush_100deg"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillAerospaceRivet, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::aerospace_rivet::apply(*stock, goodInput());

    auto cands = skill::aerospace_rivet::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["rivet_dia_mm"].get<double>(),
                4.0, 1e-9);

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::aerospace_rivet::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "aerospace_rivet cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: blind style ⇒ blind_rivet_gun tooling type ──────────────
TEST(SkillAerospaceRivet, BlindStyleSelectsBlindRivetGunTool)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto solidIn = goodInput();
    solidIn.style = "solid";
    auto solidOut = skill::aerospace_rivet::apply(*stock, solidIn);
    EXPECT_EQ(solidOut.signature.tooling.tool_type,
              std::string("pneumatic_bucking_bar"));

    auto blindIn = goodInput();
    blindIn.style = "blind";
    auto blindOut = skill::aerospace_rivet::apply(*stock, blindIn);
    EXPECT_EQ(blindOut.signature.tooling.tool_type,
              std::string("blind_rivet_gun"));

    // Blind rivet should be faster than solid bucked.
    EXPECT_LT(blindOut.signature.tooling.est_cycle_time_s,
              solidOut.signature.tooling.est_cycle_time_s);
}
