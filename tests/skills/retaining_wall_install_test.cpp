// @lat: [[process/test-strategy#skill round-trip]]
//
// retaining_wall_install skill — 5-case sweep covering identity-geometry
// synthesis, DFM rejection of invalid wall_type, metadata-only recognition,
// and the specific assertion that sheet-pile installs use the vibratory
// pile driver while gravity/reinforced use a crawler crane.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/retaining_wall_install.hpp"

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

skill::retaining_wall_install::Input goodInput()
{
    skill::retaining_wall_install::Input in;
    in.wall_type = "reinforced";
    in.height_m  = 4.0;
    in.length_m  = 20.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillRetainingWallInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::retaining_wall_install::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("retaining_wall_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillRetainingWallInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.wall_type = "soil_nail";          // not in {gravity, reinforced, sheet}
    auto r = skill::retaining_wall_install::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-WALL-TYPE"));
    EXPECT_THROW(skill::retaining_wall_install::apply(*stock, in),
                 skill::SkillError);

    auto in2 = goodInput();
    in2.height_m = 0.0;
    auto r2 = skill::retaining_wall_install::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    auto in3 = goodInput();
    in3.length_m = -5.0;
    auto r3 = skill::retaining_wall_install::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillRetainingWallInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.wall_type = "gravity";
    in.height_m  = 3.0;
    in.length_m  = 15.0;
    auto out = skill::retaining_wall_install::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("retaining_wall_install"));
    EXPECT_EQ(out.signature.pattern["wall_type"].get<std::string>(),
              std::string("gravity"));
    EXPECT_NEAR(out.signature.pattern["height_m"].get<double>(), 3.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["length_m"].get<double>(), 15.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["wall_face_area_m2"].get<double>(),
                45.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillRetainingWallInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::retaining_wall_install::apply(*stock, goodInput());

    auto cands = skill::retaining_wall_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["wall_type"].get<std::string>(),
              std::string("reinforced"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::retaining_wall_install::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "retaining_wall_install cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: equipment selection depends on wall_type ────────────────
TEST(SkillRetainingWallInstall, EquipmentSelectionByWallType)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto inSheet = goodInput();
    inSheet.wall_type = "sheet";
    auto outSheet = skill::retaining_wall_install::apply(*stock, inSheet);
    EXPECT_EQ(outSheet.signature.tooling.tool_type,
              std::string("vibratory_pile_driver"));

    auto inGrav = goodInput();
    inGrav.wall_type = "gravity";
    auto outGrav = skill::retaining_wall_install::apply(*stock, inGrav);
    EXPECT_EQ(outGrav.signature.tooling.tool_type,
              std::string("crawler_crane"));

    auto inReinf = goodInput();
    inReinf.wall_type = "reinforced";
    auto outReinf = skill::retaining_wall_install::apply(*stock, inReinf);
    EXPECT_EQ(outReinf.signature.tooling.tool_type,
              std::string("crawler_crane"));
}
