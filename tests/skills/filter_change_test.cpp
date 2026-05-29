// @lat: [[process/test-strategy#skill round-trip]]
//
// filter_change skill — 5-case maintenance sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/filter_change.hpp"

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

skill::filter_change::Input goodInput()
{
    skill::filter_change::Input in;
    in.filter_type      = "oil";
    in.filter_id        = "P164375";
    in.hours_in_service = 500.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ─────────────────────────────────
TEST(SkillFilterChange, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::filter_change::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("filter_change"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ───────────────────────────────────────
TEST(SkillFilterChange, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.filter_id = "";                     // empty
    auto r = skill::filter_change::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::filter_change::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.hours_in_service = -1.0;           // negative
    auto r2 = skill::filter_change::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillFilterChange, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.filter_type      = "hydraulic";
    in.filter_id        = "HF-30200";
    in.hours_in_service = 1500.0;

    auto out = skill::filter_change::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("filter_change"));
    EXPECT_EQ(out.signature.pattern["filter_type"].get<std::string>(),
              std::string("hydraulic"));
    EXPECT_EQ(out.signature.pattern["filter_id"].get<std::string>(),
              std::string("HF-30200"));
    EXPECT_NEAR(out.signature.pattern["hours_in_service"].get<double>(),
                1500.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillFilterChange, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::filter_change::apply(*stock, goodInput());

    auto cands = skill::filter_change::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["filter_id"].get<std::string>(),
              std::string("P164375"));

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::filter_change::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "filter_change cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: overdue (>8760 h) → INFO finding, not error ────────────
TEST(SkillFilterChange, OverdueHoursIsInfoNotError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.hours_in_service = 12000.0;         // 1.37 years — overdue

    auto r = skill::filter_change::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "overdue hours should NOT block (info-only)";
    EXPECT_TRUE(hasFinding(r, "DFM-FILTER-LIFE"));
    EXPECT_NO_THROW(skill::filter_change::apply(*stock, in));
}
