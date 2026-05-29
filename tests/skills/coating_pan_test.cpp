// @lat: [[process/test-strategy#skill round-trip]]
//
// coating_pan — drug-coating pan metadata stamp.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. UnknownCoatTypeEmitsInfo

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/coating_pan.hpp"

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

// ─── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillCoatingPan, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(25.0, 25.0, 4.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::coating_pan::Input in;
    in.coat_type      = "film";
    in.coat_pickup_mg = 8.0;
    in.drying_temp_c  = 45.0;

    auto out = skill::coating_pan::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillCoatingPan, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 3.0);

    // Non-positive coat pickup.
    skill::coating_pan::Input zeroP;
    zeroP.coat_type      = "film";
    zeroP.coat_pickup_mg = 0.0;
    zeroP.drying_temp_c  = 50.0;
    {
        auto r = skill::coating_pan::validate(*stock, zeroP);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::coating_pan::apply(*stock, zeroP),
                 skill::SkillError);

    // Drying temperature too low.
    skill::coating_pan::Input cold;
    cold.coat_type      = "film";
    cold.coat_pickup_mg = 10.0;
    cold.drying_temp_c  = 5.0;        // ≤ 10 → error
    {
        auto r = skill::coating_pan::validate(*stock, cold);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }

    // Drying temperature too high.
    skill::coating_pan::Input hot;
    hot.coat_type      = "film";
    hot.coat_pickup_mg = 10.0;
    hot.drying_temp_c  = 150.0;       // > 100 → error
    {
        auto r = skill::coating_pan::validate(*stock, hot);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
}

// ─── 3. Signature records kind + key params ────────────────────────────────
TEST(SkillCoatingPan, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::coating_pan::Input in;
    in.coat_type      = "enteric";
    in.coat_pickup_mg = 22.0;
    in.drying_temp_c  = 60.0;

    auto out = skill::coating_pan::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("coating_pan"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("coating_pan"));
    EXPECT_EQ(out.signature.pattern["coat_type"].get<std::string>(),
              std::string("enteric"));
    EXPECT_NEAR(out.signature.pattern["coat_pickup_mg"].get<double>(),
                22.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["drying_temp_c"].get<double>(),
                60.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 4. Recognize metadata replay ──────────────────────────────────────────
TEST(SkillCoatingPan, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::coating_pan::Input in;
    in.coat_type      = "sugar";
    in.coat_pickup_mg = 30.0;
    in.drying_temp_c  = 40.0;

    auto out   = skill::coating_pan::apply(*stock, in);
    auto cands = skill::coating_pan::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["coat_type"].get<std::string>(),
              std::string("sugar"));
    EXPECT_NEAR(cands[0].recovered_params["coat_pickup_mg"].get<double>(),
                30.0, 1e-9);

    // Stripped of metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::coating_pan::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Unknown coat type emits info finding, still proceeds ───────────────
TEST(SkillCoatingPan, UnknownCoatTypeEmitsInfo)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 2.0);

    skill::coating_pan::Input in;
    in.coat_type      = "exotic_polymer_blend";   // unknown → info
    in.coat_pickup_mg = 12.0;
    in.drying_temp_c  = 55.0;

    auto r = skill::coating_pan::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-CP-TYPE"));

    auto out = skill::coating_pan::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("coating_pan"));
}
