// @lat: [[process/test-strategy#skill round-trip]]
//
// turbine_balance skill — 5-case sweep: identity geometry, DFM rejection,
// signature payload, metadata replay, skill-specific (ISO grade info).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/turbine_balance.hpp"

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

skill::turbine_balance::Input goodInput()
{
    skill::turbine_balance::Input in;
    in.rotor_id            = "R-42";
    in.mass_imbalance_g_mm = 4.2;
    in.balance_grade       = "G2.5";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillTurbineBalance, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::turbine_balance::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("turbine_balance"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input: empty rotor_id AND negative imbalance ─
TEST(SkillTurbineBalance, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.rotor_id            = "";
    in.mass_imbalance_g_mm = -1.0;

    auto r = skill::turbine_balance::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::turbine_balance::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + rotor + grade + imbalance ───────────────
TEST(SkillTurbineBalance, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.rotor_id            = "TURBO-7";
    in.mass_imbalance_g_mm = 0.8;
    in.balance_grade       = "G1.0";

    auto out = skill::turbine_balance::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("turbine_balance"));
    EXPECT_EQ(out.signature.pattern["rotor_id"].get<std::string>(),
              std::string("TURBO-7"));
    EXPECT_NEAR(out.signature.pattern["mass_imbalance_g_mm"].get<double>(),
                0.8, 1e-9);
    EXPECT_EQ(out.signature.pattern["balance_grade"].get<std::string>(),
              std::string("G1.0"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillTurbineBalance, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::turbine_balance::apply(*stock, goodInput());

    auto cands = skill::turbine_balance::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["rotor_id"].get<std::string>(),
              std::string("R-42"));

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::turbine_balance::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "turbine_balance cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: unknown ISO grade emits info but does not block ────────
TEST(SkillTurbineBalance, UnknownGradeEmitsInfoAndProceeds)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.balance_grade = "G999";    // not in ISO set

    auto r = skill::turbine_balance::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "unknown grade should be info-level, not blocking";
    EXPECT_TRUE(hasFinding(r, "DFM-BAL-GRADE"));
    EXPECT_NO_THROW(skill::turbine_balance::apply(*stock, in));
}
