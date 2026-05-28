// @lat: [[process/test-strategy#skill round-trip]]
//
// ion_implant — dopant implantation metadata-only skill.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ion_implant.hpp"

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

// ── 1. Apply: geometry unchanged (clone, volume preserved) ───────────────
TEST(SkillIonImplant, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCylindricalStock(150.0, 0.775, "silicon_wafer");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::ion_implant::Input in;
    in.dopant         = "B";
    in.energy_kev     = 50.0;
    in.dose_atoms_cm2 = 1.0e15;
    in.tilt_deg       = 7.0;

    auto out = skill::ion_implant::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ── 2. ValidateRejectsBadInput: empty dopant, zero energy, zero dose ─────
TEST(SkillIonImplant, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(150.0, 0.775, "silicon_wafer");

    skill::ion_implant::Input emptyDop;
    emptyDop.dopant         = "";
    emptyDop.energy_kev     = 50.0;
    emptyDop.dose_atoms_cm2 = 1.0e15;
    auto r1 = skill::ion_implant::validate(*stock, emptyDop);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::ion_implant::apply(*stock, emptyDop),
                 skill::SkillError);

    skill::ion_implant::Input zeroE;
    zeroE.dopant         = "B";
    zeroE.energy_kev     = 0.0;
    zeroE.dose_atoms_cm2 = 1.0e15;
    auto r2 = skill::ion_implant::validate(*stock, zeroE);
    EXPECT_FALSE(r2.passed);

    skill::ion_implant::Input zeroDose;
    zeroDose.dopant         = "P";
    zeroDose.energy_kev     = 80.0;
    zeroDose.dose_atoms_cm2 = 0.0;
    auto r3 = skill::ion_implant::validate(*stock, zeroDose);
    EXPECT_FALSE(r3.passed);

    skill::ion_implant::Input negDose = zeroDose;
    negDose.dose_atoms_cm2 = -1.0;
    auto r3b = skill::ion_implant::validate(*stock, negDose);
    EXPECT_FALSE(r3b.passed);
}

// ── 3. SignatureRecordsKind + key params ─────────────────────────────────
TEST(SkillIonImplant, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCylindricalStock(200.0, 0.775, "silicon_wafer");

    skill::ion_implant::Input in;
    in.dopant         = "As";
    in.energy_kev     = 80.0;
    in.dose_atoms_cm2 = 5.0e14;
    in.tilt_deg       = 7.0;

    auto out = skill::ion_implant::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("ion_implant"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("ion_implant"));
    EXPECT_EQ(out.signature.pattern["dopant"].get<std::string>(),
              std::string("As"));
    EXPECT_NEAR(out.signature.pattern["energy_kev"].get<double>(),
                80.0, 1e-12);
    EXPECT_NEAR(out.signature.pattern["dose_atoms_cm2"].get<double>(),
                5.0e14, 1e6);          // float tolerance for large value
    EXPECT_NEAR(out.signature.pattern["tilt_deg"].get<double>(),
                7.0, 1e-12);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("ion_implanter"));
}

// ── 4. RecognizeMetadataReplay ───────────────────────────────────────────
TEST(SkillIonImplant, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(150.0, 0.775, "silicon_wafer");

    skill::ion_implant::Input in;
    in.dopant         = "P";
    in.energy_kev     = 30.0;
    in.dose_atoms_cm2 = 2.0e15;
    in.tilt_deg       = 0.0;
    auto out = skill::ion_implant::apply(*stock, in);

    auto cands = skill::ion_implant::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["dopant"].get<std::string>(),
              std::string("P"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::ion_implant::recognize(raw).empty());
}

// ── 5. SPECIFIC: extreme tilt emits info (shadowing risk) ────────────────
TEST(SkillIonImplant, ExtremeTiltEmitsInfo)
{
    auto stock = skill::createCylindricalStock(150.0, 0.775, "silicon_wafer");

    skill::ion_implant::Input in;
    in.dopant         = "B";
    in.energy_kev     = 50.0;
    in.dose_atoms_cm2 = 1.0e15;
    in.tilt_deg       = 75.0;          // > 60 → info

    auto r = skill::ion_implant::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "extreme tilt is advisory, not blocking";
    EXPECT_TRUE(hasFinding(r, "DFM-IMPLANT-TILT"));
    EXPECT_NO_THROW(skill::ion_implant::apply(*stock, in));
}
