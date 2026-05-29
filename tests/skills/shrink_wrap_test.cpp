// @lat: [[process/test-strategy#skill round-trip]]
//
// shrink_wrap — heat-shrink polymer film.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. ExcessOvenTempEmitsHighInfoButProceeds

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/shrink_wrap.hpp"

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
TEST(SkillShrinkWrap, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 30.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::shrink_wrap::Input in;
    in.film_type    = "pof";
    in.oven_temp_c  = 160.0;
    in.dwell_time_s = 8.0;

    auto out = skill::shrink_wrap::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("shrink_wrap"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillShrinkWrap, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    // Unknown film type.
    skill::shrink_wrap::Input badFilm;
    badFilm.film_type    = "pet";   // not in known catalog
    badFilm.oven_temp_c  = 160.0;
    badFilm.dwell_time_s = 8.0;
    {
        auto r = skill::shrink_wrap::validate(*stock, badFilm);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::shrink_wrap::apply(*stock, badFilm),
                 skill::SkillError);

    // oven_temp_c <= 0.
    skill::shrink_wrap::Input badTemp;
    badTemp.film_type    = "pof";
    badTemp.oven_temp_c  = 0.0;
    badTemp.dwell_time_s = 8.0;
    {
        auto r = skill::shrink_wrap::validate(*stock, badTemp);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }

    // dwell_time_s <= 0.
    skill::shrink_wrap::Input badDwell;
    badDwell.film_type    = "pof";
    badDwell.oven_temp_c  = 160.0;
    badDwell.dwell_time_s = 0.0;
    {
        auto r = skill::shrink_wrap::validate(*stock, badDwell);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillShrinkWrap, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(80.0, 50.0, 20.0);

    skill::shrink_wrap::Input in;
    in.film_type    = "pvc";
    in.oven_temp_c  = 140.0;
    in.dwell_time_s = 6.0;

    auto out = skill::shrink_wrap::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("shrink_wrap"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("shrink_wrap"));
    EXPECT_EQ(out.signature.pattern["film_type"].get<std::string>(),
              std::string("pvc"));
    EXPECT_NEAR(out.signature.pattern["oven_temp_c"].get<double>(),
                140.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["dwell_time_s"].get<double>(),
                6.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("shrink_tunnel"));
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillShrinkWrap, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 30.0);

    skill::shrink_wrap::Input in;
    in.film_type    = "pe";
    in.oven_temp_c  = 170.0;
    in.dwell_time_s = 10.0;

    auto out   = skill::shrink_wrap::apply(*stock, in);
    auto cands = skill::shrink_wrap::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["film_type"].get<std::string>(),
              std::string("pe"));
    EXPECT_NEAR(cands[0].recovered_params["oven_temp_c"].get<double>(),
                170.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::shrink_wrap::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: oven_temp_c > 220 emits HIGH info, does NOT block ───────
TEST(SkillShrinkWrap, ExcessOvenTempEmitsHighInfoButProceeds)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);

    skill::shrink_wrap::Input hot;
    hot.film_type    = "pof";
    hot.oven_temp_c  = 240.0;    // above 220 → info-only
    hot.dwell_time_s = 6.0;

    auto r = skill::shrink_wrap::validate(*stock, hot);
    EXPECT_TRUE(r.passed) << "excessive oven temp must be info-only";
    EXPECT_TRUE(hasFinding(r, "DFM-SHRINK-TEMP-HIGH"));

    EXPECT_NO_THROW(skill::shrink_wrap::apply(*stock, hot));
}
