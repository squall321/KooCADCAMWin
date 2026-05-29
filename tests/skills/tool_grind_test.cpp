// @lat: [[process/test-strategy#skill round-trip]]
//
// tool_grind skill — 5-case sweep covering identity-geometry synthesis,
// DFM error on empty tool_id / non-positive edge radius, signature
// metadata round-trip, recognition by metadata replay, and the specific
// rake/relief carry assertion.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/tool_grind.hpp"

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

skill::tool_grind::Input goodInput()
{
    skill::tool_grind::Input in;
    in.tool_id               = "EM-6MM-4F-001";
    in.rake_angle_deg        = 10.0;
    in.relief_angle_deg      = 8.0;
    in.target_edge_radius_um = 10.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ─────────────────────────────────
TEST(SkillToolGrind, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::tool_grind::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("tool_grind"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (empty tool_id, zero edge radius) ─────
TEST(SkillToolGrind, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");

    auto in = goodInput();
    in.tool_id = "";                                  // empty
    auto r = skill::tool_grind::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::tool_grind::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.target_edge_radius_um = 0.0;                  // not > 0
    auto r2 = skill::tool_grind::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + tool_id + edge geometry ─────────────────
TEST(SkillToolGrind, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");

    auto in = goodInput();
    in.tool_id               = "DR-3MM-2F-CARBIDE";
    in.rake_angle_deg        = 15.0;
    in.relief_angle_deg      = 12.0;
    in.target_edge_radius_um = 8.0;

    auto out = skill::tool_grind::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("tool_grind"));
    EXPECT_EQ(out.signature.pattern["tool_id"].get<std::string>(),
              std::string("DR-3MM-2F-CARBIDE"));
    EXPECT_NEAR(out.signature.pattern["rake_angle_deg"].get<double>(),
                15.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["relief_angle_deg"].get<double>(),
                12.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["target_edge_radius_um"].get<double>(),
                8.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillToolGrind, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");
    auto out = skill::tool_grind::apply(*stock, goodInput());

    auto cands = skill::tool_grind::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["tool_id"].get<std::string>(),
              std::string("EM-6MM-4F-001"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::tool_grind::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "tool_grind cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: out-of-range rake angle emits warning but proceeds ─────
TEST(SkillToolGrind, ExtremeRakeAngleEmitsWarningButPasses)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");

    auto in = goodInput();
    in.rake_angle_deg = 45.0;                         // way above [-30, +30]
    auto r = skill::tool_grind::validate(*stock, in);
    EXPECT_TRUE(r.passed) << "warning must not block apply";
    EXPECT_TRUE(hasFinding(r, "DFM-TG-RAKE"));
    EXPECT_NO_THROW(skill::tool_grind::apply(*stock, in));

    auto in2 = goodInput();
    in2.relief_angle_deg = 30.0;                      // above [0, 25]
    auto r2 = skill::tool_grind::validate(*stock, in2);
    EXPECT_TRUE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-TG-RELIEF"));
}
