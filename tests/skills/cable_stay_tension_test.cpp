// @lat: [[process/test-strategy#skill round-trip]]
//
// cable_stay_tension skill — 5-case sweep covering identity-geometry
// synthesis, DFM rejection of >5% tension deviation, metadata-only
// recognition, and the specific assertion that the signature pattern
// records tension_deviation_pct = (jack - design)/design × 100.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cable_stay_tension.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

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

skill::cable_stay_tension::Input goodInput()
{
    skill::cable_stay_tension::Input in;
    in.cable_id          = "S12N";
    in.design_tension_kn = 5000.0;
    in.jack_force_kn     = 5000.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillCableStayTension, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::cable_stay_tension::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("cable_stay_tension"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillCableStayTension, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.cable_id = "";
    auto r = skill::cable_stay_tension::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::cable_stay_tension::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.jack_force_kn = 5500.0;     // +10% over design — > 5% threshold
    auto r2 = skill::cable_stay_tension::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-CABLE-TENSION"));
    EXPECT_THROW(skill::cable_stay_tension::apply(*stock, in2), skill::SkillError);

    auto in3 = goodInput();
    in3.design_tension_kn = 0.0;
    auto r3 = skill::cable_stay_tension::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillCableStayTension, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.cable_id          = "S8S";
    in.design_tension_kn = 4000.0;
    in.jack_force_kn     = 4000.0;
    auto out = skill::cable_stay_tension::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("cable_stay_tension"));
    EXPECT_EQ(out.signature.pattern["cable_id"].get<std::string>(),
              std::string("S8S"));
    EXPECT_NEAR(out.signature.pattern["design_tension_kn"].get<double>(),
                4000.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["jack_force_kn"].get<double>(),
                4000.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillCableStayTension, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::cable_stay_tension::apply(*stock, goodInput());

    auto cands = skill::cable_stay_tension::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["cable_id"].get<std::string>(),
              std::string("S12N"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::cable_stay_tension::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "cable_stay_tension cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: tension_deviation_pct computed (jack - design)/design ───
TEST(SkillCableStayTension, TensionDeviationPctRecorded)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.design_tension_kn = 5000.0;
    in.jack_force_kn     = 5150.0;     // +3% — within 5% (info-only)

    auto r = skill::cable_stay_tension::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "3% deviation should NOT block (info-only)";
    EXPECT_TRUE(hasFinding(r, "DFM-CABLE-TENSION"));

    auto out = skill::cable_stay_tension::apply(*stock, in);
    const double dev_pct =
        out.signature.pattern["tension_deviation_pct"].get<double>();
    EXPECT_NEAR(dev_pct, 3.0, 1e-6);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("hydraulic_stressing_jack"));
}
