// @lat: [[process/test-strategy#skill round-trip]]
//
// powder_atomization — produces a powder lot (no geometric change).
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndKeyParams
//   4. RecognizeMetadataReplay
//   5. Tooling type depends on gas (water → water_jet_nozzle)

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/powder_atomization.hpp"

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

}  // namespace

// ─── 1. Apply clones geometry unchanged ──────────────────────────────────
TEST(SkillPowderAtomization, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);
    const double volBefore   = volumeOf(stock->shape());
    const int    facesBefore = stock->faceCount();
    const int    edgesBefore = stock->edgeCount();

    skill::powder_atomization::Input in;
    in.material                = "stainless_316l";
    in.median_particle_size_um = 25.0;
    in.gas                     = "argon";
    in.yield_pct               = 75.0;

    auto out = skill::powder_atomization::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_EQ(out.workpiece->faceCount(), facesBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgesBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillPowderAtomization, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::powder_atomization::Input zeroD;
    zeroD.material                = "stainless_316l";
    zeroD.median_particle_size_um = 0.0;       // invalid
    zeroD.gas                     = "argon";
    zeroD.yield_pct               = 75.0;
    auto r1 = skill::powder_atomization::validate(*stock, zeroD);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::powder_atomization::apply(*stock, zeroD),
                 skill::SkillError);

    skill::powder_atomization::Input badYield;
    badYield.material                = "stainless_316l";
    badYield.median_particle_size_um = 25.0;
    badYield.gas                     = "argon";
    badYield.yield_pct               = 150.0;  // > 100
    auto r2 = skill::powder_atomization::validate(*stock, badYield);
    EXPECT_FALSE(r2.passed);

    skill::powder_atomization::Input emptyGas;
    emptyGas.material                = "stainless_316l";
    emptyGas.median_particle_size_um = 25.0;
    emptyGas.gas                     = "";     // invalid
    emptyGas.yield_pct               = 75.0;
    auto r3 = skill::powder_atomization::validate(*stock, emptyGas);
    EXPECT_FALSE(r3.passed);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillPowderAtomization, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::powder_atomization::Input in;
    in.material                = "titanium_grade5";
    in.median_particle_size_um = 45.0;
    in.gas                     = "argon";
    in.yield_pct               = 65.0;

    auto out = skill::powder_atomization::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("powder_atomization"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("powder_atomization"));
    EXPECT_EQ(out.signature.pattern["process_family"].get<std::string>(),
              std::string("powder_metallurgy"));
    EXPECT_EQ(out.signature.pattern["material"].get<std::string>(),
              std::string("titanium_grade5"));
    EXPECT_NEAR(out.signature.pattern["median_particle_size_um"].get<double>(),
                45.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["gas"].get<std::string>(),
              std::string("argon"));
    EXPECT_NEAR(out.signature.pattern["yield_pct"].get<double>(), 65.0, 1e-9);
    EXPECT_TRUE(out.signature.pattern["produces_powder_lot"].get<bool>());
}

// ─── 4. Recognize metadata replay ────────────────────────────────────────
TEST(SkillPowderAtomization, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::powder_atomization::Input in;
    in.material                = "inconel_718";
    in.median_particle_size_um = 30.0;
    in.gas                     = "nitrogen";
    in.yield_pct               = 80.0;

    auto out   = skill::powder_atomization::apply(*stock, in);
    auto cands = skill::powder_atomization::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("powder_atomization"));
    EXPECT_GT(cands[0].confidence, 0.99);
    EXPECT_EQ(cands[0].recovered_params["gas"].get<std::string>(),
              std::string("nitrogen"));
    EXPECT_NEAR(cands[0].recovered_params["median_particle_size_um"].get<double>(),
                30.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::powder_atomization::recognize(raw).empty());
}

// ─── 5. Gas drives tool_material selection ───────────────────────────────
TEST(SkillPowderAtomization, ToolMaterialDependsOnGas)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::powder_atomization::Input gas;
    gas.material                = "stainless_316l";
    gas.median_particle_size_um = 25.0;
    gas.gas                     = "argon";
    gas.yield_pct               = 75.0;
    auto outGas = skill::powder_atomization::apply(*stock, gas);
    EXPECT_EQ(outGas.signature.tooling.tool_material,
              std::string("gas_atomization_nozzle"));
    EXPECT_EQ(outGas.signature.tooling.tool_type, std::string("atomizer"));

    skill::powder_atomization::Input water;
    water.material                = "iron_pure";
    water.median_particle_size_um = 150.0;
    water.gas                     = "water";
    water.yield_pct               = 90.0;
    auto outWater = skill::powder_atomization::apply(*stock, water);
    EXPECT_EQ(outWater.signature.tooling.tool_material,
              std::string("water_jet_nozzle"));
}
