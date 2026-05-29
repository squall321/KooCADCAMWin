// @lat: [[process/test-strategy#skill round-trip]]
//
// edge_prep_hone skill — 5-case sweep covering identity-geometry synthesis,
// DFM error on empty tool_id / non-positive target radius, signature
// metadata round-trip, recognition by metadata replay, and the specific
// edge-profile classification (waterfall / trumpet / symmetric).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/edge_prep_hone.hpp"

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

skill::edge_prep_hone::Input goodInput()
{
    skill::edge_prep_hone::Input in;
    in.tool_id          = "IN-CNMG120408-001";
    in.target_radius_um = 15.0;
    in.asymmetry_factor = 1.5;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ─────────────────────────────────
TEST(SkillEdgePrepHone, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::edge_prep_hone::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("edge_prep_hone"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (empty tool_id, zero radius) ──────────
TEST(SkillEdgePrepHone, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");

    auto in = goodInput();
    in.tool_id = "";                                  // empty
    auto r = skill::edge_prep_hone::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::edge_prep_hone::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.target_radius_um = 0.0;                       // not > 0
    auto r2 = skill::edge_prep_hone::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + radius + asymmetry ──────────────────────
TEST(SkillEdgePrepHone, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");

    auto in = goodInput();
    in.tool_id          = "EM-12MM-4F-FINISHING";
    in.target_radius_um = 25.0;
    in.asymmetry_factor = 2.0;

    auto out = skill::edge_prep_hone::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("edge_prep_hone"));
    EXPECT_EQ(out.signature.pattern["tool_id"].get<std::string>(),
              std::string("EM-12MM-4F-FINISHING"));
    EXPECT_NEAR(out.signature.pattern["target_radius_um"].get<double>(),
                25.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["asymmetry_factor"].get<double>(),
                2.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillEdgePrepHone, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");
    auto out = skill::edge_prep_hone::apply(*stock, goodInput());

    auto cands = skill::edge_prep_hone::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["target_radius_um"].get<double>(),
                15.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::edge_prep_hone::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "edge_prep_hone cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: edge_profile classification (waterfall vs symmetric) ───
TEST(SkillEdgePrepHone, EdgeProfileClassifiedFromAsymmetry)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");

    auto inWf = goodInput();
    inWf.asymmetry_factor = 1.8;
    auto outWf = skill::edge_prep_hone::apply(*stock, inWf);
    EXPECT_EQ(outWf.signature.pattern["edge_profile"].get<std::string>(),
              std::string("waterfall"));

    auto inSym = goodInput();
    inSym.asymmetry_factor = 1.0;
    auto outSym = skill::edge_prep_hone::apply(*stock, inSym);
    EXPECT_EQ(outSym.signature.pattern["edge_profile"].get<std::string>(),
              std::string("symmetric"));

    auto inTrumpet = goodInput();
    inTrumpet.asymmetry_factor = 0.7;
    auto outT = skill::edge_prep_hone::apply(*stock, inTrumpet);
    EXPECT_EQ(outT.signature.pattern["edge_profile"].get<std::string>(),
              std::string("trumpet"));
}
