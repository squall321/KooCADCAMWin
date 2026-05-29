// @lat: [[process/test-strategy#skill round-trip]]
//
// carton_form — folded paperboard carton.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. AutoLockBottomShortensCycleTime

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/carton_form.hpp"

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
TEST(SkillCartonForm, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(80.0, 120.0, 40.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::carton_form::Input in;
    in.carton_size_mm = { 80.0, 120.0, 40.0 };
    in.fold_pattern   = "rte";
    in.glue_type      = "hot_melt";

    auto out = skill::carton_form::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("carton_form"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillCartonForm, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 30.0);

    // Zero / negative dimension.
    skill::carton_form::Input badSize;
    badSize.carton_size_mm = { 80.0, 0.0, 40.0 };
    badSize.fold_pattern   = "rte";
    badSize.glue_type      = "hot_melt";
    {
        auto r = skill::carton_form::validate(*stock, badSize);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::carton_form::apply(*stock, badSize),
                 skill::SkillError);

    // Empty fold pattern.
    skill::carton_form::Input noPattern;
    noPattern.carton_size_mm = { 80.0, 120.0, 40.0 };
    noPattern.fold_pattern   = "";
    noPattern.glue_type      = "hot_melt";
    {
        auto r = skill::carton_form::validate(*stock, noPattern);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }

    // Empty glue type.
    skill::carton_form::Input noGlue;
    noGlue.carton_size_mm = { 80.0, 120.0, 40.0 };
    noGlue.fold_pattern   = "rte";
    noGlue.glue_type      = "";
    {
        auto r = skill::carton_form::validate(*stock, noGlue);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillCartonForm, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(100.0, 150.0, 30.0);

    skill::carton_form::Input in;
    in.carton_size_mm = { 100.0, 150.0, 30.0 };
    in.fold_pattern   = "ste";
    in.glue_type      = "cold_glue";

    auto out = skill::carton_form::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("carton_form"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("carton_form"));
    EXPECT_EQ(out.signature.pattern["fold_pattern"].get<std::string>(),
              std::string("ste"));
    EXPECT_EQ(out.signature.pattern["glue_type"].get<std::string>(),
              std::string("cold_glue"));
    const auto& sz = out.signature.pattern["carton_size_mm"];
    ASSERT_EQ(sz.size(), 3u);
    EXPECT_NEAR(sz[0].get<double>(), 100.0, 1e-9);
    EXPECT_NEAR(sz[1].get<double>(), 150.0, 1e-9);
    EXPECT_NEAR(sz[2].get<double>(),  30.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillCartonForm, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 90.0, 30.0);

    skill::carton_form::Input in;
    in.carton_size_mm = { 60.0, 90.0, 30.0 };
    in.fold_pattern   = "alb";
    in.glue_type      = "tab_lock";

    auto out   = skill::carton_form::apply(*stock, in);
    auto cands = skill::carton_form::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["fold_pattern"].get<std::string>(),
              std::string("alb"));
    EXPECT_EQ(cands[0].recovered_params["glue_type"].get<std::string>(),
              std::string("tab_lock"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::carton_form::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: tab_lock glue type yields a SHORTER est cycle than hot_melt
TEST(SkillCartonForm, AutoLockBottomShortensCycleTime)
{
    auto stock = skill::createCuboidStock(80.0, 120.0, 40.0);

    skill::carton_form::Input tabLock;
    tabLock.carton_size_mm = { 80.0, 120.0, 40.0 };
    tabLock.fold_pattern   = "alb";
    tabLock.glue_type      = "tab_lock";

    skill::carton_form::Input hotMelt;
    hotMelt.carton_size_mm = { 80.0, 120.0, 40.0 };
    hotMelt.fold_pattern   = "rte";
    hotMelt.glue_type      = "hot_melt";

    auto outTL = skill::carton_form::apply(*stock, tabLock);
    auto outHM = skill::carton_form::apply(*stock, hotMelt);

    EXPECT_LT(outTL.signature.tooling.est_cycle_time_s,
              outHM.signature.tooling.est_cycle_time_s)
        << "tab_lock must be faster than hot_melt (no glue dwell)";
}
