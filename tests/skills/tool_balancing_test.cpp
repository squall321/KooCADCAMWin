// @lat: [[process/test-strategy#skill round-trip]]
//
// tool_balancing skill — 5-case sweep covering identity-geometry synthesis,
// DFM error on empty tool_id / non-positive grade / non-positive rpm,
// signature metadata round-trip, recognition by metadata replay, and the
// specific grade-class classification (precision / high / standard / coarse).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/tool_balancing.hpp"

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

skill::tool_balancing::Input goodInput()
{
    skill::tool_balancing::Input in;
    in.tool_id       = "EM-12MM-HSK63-001";
    in.grade_g_mm_kg = 1.0;
    in.target_rpm    = 24000.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ─────────────────────────────────
TEST(SkillToolBalancing, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::tool_balancing::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("tool_balancing"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (empty tool_id, zero rpm) ─────────────
TEST(SkillToolBalancing, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");

    auto in = goodInput();
    in.tool_id = "";                                  // empty
    auto r = skill::tool_balancing::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::tool_balancing::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.target_rpm = 0.0;                             // not > 0
    auto r2 = skill::tool_balancing::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    auto in3 = goodInput();
    in3.grade_g_mm_kg = -1.0;                         // not > 0
    auto r3 = skill::tool_balancing::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + grade + rpm ─────────────────────────────
TEST(SkillToolBalancing, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");

    auto in = goodInput();
    in.tool_id       = "EM-20MM-HSK100-FINISHER";
    in.grade_g_mm_kg = 2.5;
    in.target_rpm    = 18000.0;

    auto out = skill::tool_balancing::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("tool_balancing"));
    EXPECT_EQ(out.signature.pattern["tool_id"].get<std::string>(),
              std::string("EM-20MM-HSK100-FINISHER"));
    EXPECT_NEAR(out.signature.pattern["grade_g_mm_kg"].get<double>(),
                2.5, 1e-9);
    EXPECT_NEAR(out.signature.pattern["target_rpm"].get<double>(),
                18000.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillToolBalancing, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");
    auto out = skill::tool_balancing::apply(*stock, goodInput());

    auto cands = skill::tool_balancing::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["target_rpm"].get<double>(),
                24000.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::tool_balancing::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "tool_balancing cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: grade_class classification by grade_g_mm_kg ────────────
TEST(SkillToolBalancing, GradeClassClassifiedFromGrade)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "carbide");

    auto inPrecision = goodInput();
    inPrecision.grade_g_mm_kg = 0.4;
    auto outP = skill::tool_balancing::apply(*stock, inPrecision);
    EXPECT_EQ(outP.signature.pattern["grade_class"].get<std::string>(),
              std::string("precision"));

    auto inHigh = goodInput();
    inHigh.grade_g_mm_kg = 2.5;
    auto outH = skill::tool_balancing::apply(*stock, inHigh);
    EXPECT_EQ(outH.signature.pattern["grade_class"].get<std::string>(),
              std::string("high"));

    auto inStd = goodInput();
    inStd.grade_g_mm_kg = 6.3;
    auto outS = skill::tool_balancing::apply(*stock, inStd);
    EXPECT_EQ(outS.signature.pattern["grade_class"].get<std::string>(),
              std::string("standard"));

    auto inCoarse = goodInput();
    inCoarse.grade_g_mm_kg = 16.0;
    auto outC = skill::tool_balancing::apply(*stock, inCoarse);
    EXPECT_EQ(outC.signature.pattern["grade_class"].get<std::string>(),
              std::string("coarse"));
}
