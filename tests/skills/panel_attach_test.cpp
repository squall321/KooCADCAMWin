// @lat: [[process/test-strategy#skill round-trip]]
//
// panel_attach skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of empty panel_type / low fastener count, metadata-only
// recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/panel_attach.hpp"

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

skill::panel_attach::Input goodInput()
{
    skill::panel_attach::Input in;
    in.panel_type     = "metal_siding";
    in.fastener_count = 24;
    in.sealant_type   = "silicone";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillPanelAttach, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::panel_attach::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("panel_attach"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (empty panel_type / low count) ─────────
TEST(SkillPanelAttach, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.panel_type = "";
    auto r = skill::panel_attach::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::panel_attach::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.fastener_count = 2;       // < 4 minimum
    auto r2 = skill::panel_attach::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-PA-COUNT"));
}

// ─── 3. Signature records kind + panel_type/fastener_count/sealant ────────
TEST(SkillPanelAttach, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.panel_type     = "sandwich";
    in.fastener_count = 40;
    in.sealant_type   = "butyl";

    auto out = skill::panel_attach::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("panel_attach"));
    EXPECT_EQ(out.signature.pattern["panel_type"].get<std::string>(),
              std::string("sandwich"));
    EXPECT_EQ(out.signature.pattern["fastener_count"].get<int>(), 40);
    EXPECT_EQ(out.signature.pattern["sealant_type"].get<std::string>(),
              std::string("butyl"));
    EXPECT_TRUE(out.signature.pattern["weather_sealed"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillPanelAttach, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::panel_attach::apply(*stock, goodInput());

    auto cands = skill::panel_attach::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["panel_type"].get<std::string>(),
              std::string("metal_siding"));
    EXPECT_EQ(cands[0].recovered_params["fastener_count"].get<int>(), 24);

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::panel_attach::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "panel_attach cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: glass panel selects suction_lifter, others impact_driver
TEST(SkillPanelAttach, GlassPanelSelectsSuctionLifterTool)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto metalIn = goodInput();
    metalIn.panel_type = "metal_siding";
    auto metalOut = skill::panel_attach::apply(*stock, metalIn);
    EXPECT_EQ(metalOut.signature.tooling.tool_type,
              std::string("impact_driver"));

    auto glassIn = goodInput();
    glassIn.panel_type = "glass";
    auto glassOut = skill::panel_attach::apply(*stock, glassIn);
    EXPECT_EQ(glassOut.signature.tooling.tool_type,
              std::string("suction_lifter"));

    // More fasteners ⇒ longer install time.
    auto manyIn = goodInput();
    manyIn.fastener_count = 100;
    auto manyOut = skill::panel_attach::apply(*stock, manyIn);
    EXPECT_GT(manyOut.signature.tooling.est_cycle_time_s,
              metalOut.signature.tooling.est_cycle_time_s);
}
