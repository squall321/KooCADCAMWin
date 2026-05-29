// @lat: [[process/test-strategy#skill round-trip]]
//
// base_station_install — 5-case sweep: identity-geometry synthesis, DFM
// gates on site_id / power / sectors, metadata replay, atypical-sectors
// specific case.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/base_station_install.hpp"

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

// ── 1. Apply: geometry is COMPLETELY unchanged ───────────────────────────
TEST(SkillBaseStationInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::base_station_install::Input in;
    in.site_id       = "BTS-NW-042";
    in.equipment_kw  = 8.0;
    in.sectors_count = 3;

    auto out = skill::base_station_install::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("base_station_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: empty site_id, zero power, zero sectors rejected ─────────────
TEST(SkillBaseStationInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::base_station_install::Input bad;
    bad.site_id       = "";                  // invalid
    bad.equipment_kw  = 5.0;
    bad.sectors_count = 3;
    EXPECT_FALSE(skill::base_station_install::validate(*stock, bad).passed);
    EXPECT_THROW(skill::base_station_install::apply(*stock, bad),
                 skill::SkillError);

    bad.site_id       = "BTS-1";
    bad.equipment_kw  = 0.0;                 // invalid
    EXPECT_FALSE(skill::base_station_install::validate(*stock, bad).passed);

    bad.equipment_kw  = 5.0;
    bad.sectors_count = 0;                   // invalid
    EXPECT_FALSE(skill::base_station_install::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + site/power/sectors ───────────────────────
TEST(SkillBaseStationInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::base_station_install::Input in;
    in.site_id       = "BTS-CENTRAL-007";
    in.equipment_kw  = 12.5;
    in.sectors_count = 6;

    auto out = skill::base_station_install::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("base_station_install"));
    EXPECT_EQ(out.signature.pattern["site_id"].get<std::string>(),
              std::string("BTS-CENTRAL-007"));
    EXPECT_NEAR(out.signature.pattern["equipment_kw"].get<double>(), 12.5, 1e-9);
    EXPECT_EQ(out.signature.pattern["sectors_count"].get<int>(),    6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("site_install_crew"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillBaseStationInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::base_station_install::Input in;
    in.site_id       = "BTS-EAST-12";
    in.equipment_kw  = 7.5;
    in.sectors_count = 3;

    auto out = skill::base_station_install::apply(*stock, in);
    auto cands = skill::base_station_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["site_id"].get<std::string>(),
              std::string("BTS-EAST-12"));
    EXPECT_EQ(cands[0].recovered_params["sectors_count"].get<int>(), 3);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::base_station_install::recognize(raw).empty());
}

// ── 5. SPECIFIC: sectors_count ∉ {1,3,6} → BS-SECTORS info finding ───────
TEST(SkillBaseStationInstall, AtypicalSectorCountEmitsInfo)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::base_station_install::Input in;
    in.site_id       = "BTS-X";
    in.equipment_kw  = 4.0;
    in.sectors_count = 4;                   // atypical: not 1, 3, or 6

    auto r = skill::base_station_install::validate(*stock, in);
    EXPECT_TRUE(r.passed);                  // info does not block
    EXPECT_TRUE(hasFinding(r, "DFM-BS-SECTORS"));
    EXPECT_NO_THROW(skill::base_station_install::apply(*stock, in));

    // Typical 3-sector site emits NO sectors-info finding.
    skill::base_station_install::Input ok;
    ok.site_id       = "BTS-Y";
    ok.equipment_kw  = 4.0;
    ok.sectors_count = 3;
    auto r2 = skill::base_station_install::validate(*stock, ok);
    EXPECT_TRUE(r2.passed);
    EXPECT_FALSE(hasFinding(r2, "DFM-BS-SECTORS"));
}
