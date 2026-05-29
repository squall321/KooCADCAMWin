// @lat: [[process/test-strategy#skill round-trip]]
//
// connector_polish skill — ferrule end-face polish.  Covers:
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput  (positive return_loss_db → DFM error)
//   3. SignatureRecordsKind+params
//   4. RecognizeMetadataReplay
//   5. Specific assertion — APC class sets 8° endface angle

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/connector_polish.hpp"

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

// ─── 1. Apply: clones geometry unchanged ──────────────────────────────────
TEST(SkillConnectorPolish, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCylindricalStock(2.5, 10.0, "aluminum_6061");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::connector_polish::Input in;
    in.polish_class   = "UPC";
    in.return_loss_db = -50.0;

    auto out = skill::connector_polish::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("connector_polish"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input (positive RL) ──────────────────────────
TEST(SkillConnectorPolish, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(2.5, 10.0);

    skill::connector_polish::Input in;
    in.polish_class   = "UPC";
    in.return_loss_db = 10.0;     // positive — invalid (RL is negative dB)

    auto r = skill::connector_polish::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::connector_polish::apply(*stock, in), skill::SkillError);

    // Unknown class should also be a DFM error.
    skill::connector_polish::Input badClass;
    badClass.polish_class   = "XYZ";
    badClass.return_loss_db = -50.0;
    auto r2 = skill::connector_polish::validate(*stock, badClass);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-POLISH-CLASS"));
}

// ─── 3. Signature records kind + params ───────────────────────────────────
TEST(SkillConnectorPolish, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCylindricalStock(2.5, 10.0);

    skill::connector_polish::Input in;
    in.polish_class   = "UPC";
    in.return_loss_db = -50.0;

    auto out = skill::connector_polish::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("connector_polish"));
    EXPECT_EQ(out.signature.pattern["polish_class"].get<std::string>(),
              std::string("UPC"));
    EXPECT_NEAR(out.signature.pattern["return_loss_db"].get<double>(),
                -50.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["endface_angle_deg"].get<double>(),
                0.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("ferrule_polisher"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillConnectorPolish, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(2.5, 10.0);

    skill::connector_polish::Input in;
    in.polish_class   = "UPC";
    in.return_loss_db = -50.0;
    auto out = skill::connector_polish::apply(*stock, in);

    auto cands = skill::connector_polish::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["polish_class"].get<std::string>(),
              std::string("UPC"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::connector_polish::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "connector_polish cannot be recognized geometrically";
}

// ─── 5. Specific assertion — APC class sets endface_angle_deg = 8 ─────────
TEST(SkillConnectorPolish, ApcSetsAngledEndface)
{
    auto stock = skill::createCylindricalStock(2.5, 10.0);

    skill::connector_polish::Input in;
    in.polish_class   = "APC";
    in.return_loss_db = -65.0;

    auto out = skill::connector_polish::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["endface_angle_deg"].get<double>(),
                8.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["polish_class"].get<std::string>(),
              std::string("APC"));
}
