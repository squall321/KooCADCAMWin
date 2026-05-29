// @lat: [[process/test-strategy#skill round-trip]]
//
// propellant_load skill — small-arms powder charge no-op sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/propellant_load.hpp"

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

// ─── 1. Apply: geometry is COMPLETELY unchanged ──────────────────────────
TEST(SkillPropellantLoad, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::propellant_load::Input in;
    in.propellant_type = "ball";
    in.charge_grain    = 6.0;
    in.lot_id          = "LOT-0001";

    auto out = skill::propellant_load::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. ValidateRejectsBadInput: empty lot_id → error ────────────────────
TEST(SkillPropellantLoad, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");

    skill::propellant_load::Input in;
    in.propellant_type = "ball";
    in.charge_grain    = 6.0;
    in.lot_id          = "";          // traceability required

    auto r = skill::propellant_load::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::propellant_load::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillPropellantLoad, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");

    skill::propellant_load::Input in;
    in.propellant_type = "stick";
    in.charge_grain    = 24.5;
    in.lot_id          = "LOT-7777";

    auto out = skill::propellant_load::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("propellant_load"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("propellant_load"));
    EXPECT_EQ(out.signature.pattern["propellant_type"].get<std::string>(),
              std::string("stick"));
    EXPECT_NEAR(out.signature.pattern["charge_grain"].get<double>(),
                24.5, 1e-6);
    EXPECT_EQ(out.signature.pattern["lot_id"].get<std::string>(),
              std::string("LOT-7777"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillPropellantLoad, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");

    skill::propellant_load::Input in;
    in.propellant_type = "ball";
    in.charge_grain    = 6.0;
    in.lot_id          = "LOT-0042";

    auto out  = skill::propellant_load::apply(*stock, in);
    auto cands = skill::propellant_load::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["lot_id"].get<std::string>(),
              std::string("LOT-0042"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::propellant_load::recognize(raw).empty());
}

// ─── 5. Skill-specific: charge_grain over envelope → DFM error ───────────
TEST(SkillPropellantLoad, OverChargeIsError)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");

    skill::propellant_load::Input in;
    in.propellant_type = "ball";
    in.charge_grain    = 250.0;       // > 200 gr envelope
    in.lot_id          = "LOT-9999";

    auto r = skill::propellant_load::validate(*stock, in);
    EXPECT_FALSE(r.passed);

    bool foundChargeCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-AMMO-CHARGE") foundChargeCode = true;
    EXPECT_TRUE(foundChargeCode);
}
