// @lat: [[process/test-strategy#skill round-trip]]
//
// rocket_propellant_load skill — cryogenic / storable propellant transfer sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/rocket_propellant_load.hpp"

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

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillRocketPropellantLoad, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(3000.0, 3000.0, 12000.0,
                                          "aluminum_2219");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::rocket_propellant_load::Input in;
    in.propellant_type = "LOX";
    in.mass_kg         = 50000.0;
    in.load_temp_c     = -190.0;

    auto out = skill::rocket_propellant_load::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id,
              std::string("rocket_propellant_load"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. ValidateRejectsBadInput: unknown propellant + bad temp → errors ──
TEST(SkillRocketPropellantLoad, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(3000.0, 3000.0, 12000.0);

    skill::rocket_propellant_load::Input in;
    in.propellant_type = "MYSTERY";   // not in allow-list
    in.mass_kg         = 1000.0;
    in.load_temp_c     = 20.0;

    auto r = skill::rocket_propellant_load::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-PROP-TYPE"));
    EXPECT_THROW(skill::rocket_propellant_load::apply(*stock, in),
                 skill::SkillError);

    // LH2 loaded at room temperature is far outside boiling-point window.
    skill::rocket_propellant_load::Input in2;
    in2.propellant_type = "LH2";
    in2.mass_kg         = 1000.0;
    in2.load_temp_c     = 25.0;       // way outside [-260, -250]
    auto r2 = skill::rocket_propellant_load::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-PROP-TEMP"));
}

// ─── 3. Signature records kind + type + mass + temp ──────────────────────
TEST(SkillRocketPropellantLoad, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(3000.0, 3000.0, 12000.0);

    skill::rocket_propellant_load::Input in;
    in.propellant_type = "RP-1";
    in.mass_kg         = 120000.0;
    in.load_temp_c     = 15.0;

    auto out = skill::rocket_propellant_load::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("rocket_propellant_load"));
    EXPECT_EQ(out.signature.pattern["propellant_type"].get<std::string>(),
              std::string("RP-1"));
    EXPECT_NEAR(out.signature.pattern["mass_kg"].get<double>(),
                120000.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["load_temp_c"].get<double>(),
                15.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["propellant_family"].get<std::string>(),
              std::string("storable"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillRocketPropellantLoad, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(3000.0, 3000.0, 12000.0);

    skill::rocket_propellant_load::Input in;
    in.propellant_type = "N2O4";
    in.mass_kg         = 800.0;
    in.load_temp_c     = 15.0;

    auto out  = skill::rocket_propellant_load::apply(*stock, in);
    auto cands = skill::rocket_propellant_load::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["propellant_type"].get<std::string>(),
              std::string("N2O4"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::rocket_propellant_load::recognize(raw).empty())
        << "rocket_propellant_load cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: cryogenic family selects cryo_transfer_line tooling ────
TEST(SkillRocketPropellantLoad, CryogenicSelectsCryoTransferLine)
{
    auto stock = skill::createCuboidStock(3000.0, 3000.0, 12000.0);

    skill::rocket_propellant_load::Input cryo;
    cryo.propellant_type = "LOX";
    cryo.mass_kg         = 10000.0;
    cryo.load_temp_c     = -190.0;

    skill::rocket_propellant_load::Input storable;
    storable.propellant_type = "RP-1";
    storable.mass_kg         = 10000.0;
    storable.load_temp_c     = 15.0;

    auto outCryo     = skill::rocket_propellant_load::apply(*stock, cryo);
    auto outStorable = skill::rocket_propellant_load::apply(*stock, storable);

    EXPECT_EQ(outCryo.signature.tooling.tool_type,
              std::string("cryo_transfer_line"));
    EXPECT_EQ(outStorable.signature.tooling.tool_type,
              std::string("storable_transfer_pump"));
    EXPECT_EQ(outCryo.signature.pattern["propellant_family"].get<std::string>(),
              std::string("cryogenic"));

    // Cryogenic transfer slower than storable for same mass.
    EXPECT_GT(outCryo.signature.tooling.est_cycle_time_s,
              outStorable.signature.tooling.est_cycle_time_s);
}
