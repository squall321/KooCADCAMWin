// @lat: [[process/test-strategy#skill round-trip]]
//
// die_attach skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of out-of-range die size, metadata-only recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/die_attach.hpp"

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

skill::die_attach::Input goodInput()
{
    skill::die_attach::Input in;
    in.adhesive_type = "epoxy";
    in.die_size_mm   = 5.0;
    in.bond_force_n  = 1.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillDieAttach, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::die_attach::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("die_attach"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillDieAttach, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.die_size_mm = 0.1;          // < 0.3 mm
    auto r = skill::die_attach::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-DIE-SIZE"));
    EXPECT_THROW(skill::die_attach::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.bond_force_n = 50.0;       // > 30 N
    auto r2 = skill::die_attach::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-BOND-FORCE"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillDieAttach, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.adhesive_type = "AgPaste";
    in.die_size_mm   = 7.5;
    in.bond_force_n  = 2.5;

    auto out = skill::die_attach::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("die_attach"));
    EXPECT_EQ(out.signature.pattern["adhesive_type"].get<std::string>(),
              std::string("AgPaste"));
    EXPECT_NEAR(out.signature.pattern["die_size_mm"].get<double>(),  7.5, 1e-9);
    EXPECT_NEAR(out.signature.pattern["bond_force_n"].get<double>(), 2.5, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillDieAttach, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::die_attach::apply(*stock, goodInput());

    auto cands = skill::die_attach::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["die_size_mm"].get<double>(),
                5.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::die_attach::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "die_attach cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: solder adhesive ⇒ eutectic_die_bonder tooling ───────────
TEST(SkillDieAttach, SolderAdhesiveSelectsEutecticBonder)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto epoxyIn = goodInput();
    epoxyIn.adhesive_type = "epoxy";
    auto epoxyOut = skill::die_attach::apply(*stock, epoxyIn);
    EXPECT_EQ(epoxyOut.signature.tooling.tool_type,
              std::string("epoxy_die_bonder"));

    auto solderIn = goodInput();
    solderIn.adhesive_type = "solder";
    auto solderOut = skill::die_attach::apply(*stock, solderIn);
    EXPECT_EQ(solderOut.signature.tooling.tool_type,
              std::string("eutectic_die_bonder"));

    // Solder eutectic bonding takes longer than epoxy paste dispense.
    EXPECT_GT(solderOut.signature.tooling.est_cycle_time_s,
              epoxyOut.signature.tooling.est_cycle_time_s);
    // Solder gives thinner BLT than epoxy.
    EXPECT_LT(solderOut.signature.pattern["blt_um"].get<double>(),
              epoxyOut.signature.pattern["blt_um"].get<double>());
}
