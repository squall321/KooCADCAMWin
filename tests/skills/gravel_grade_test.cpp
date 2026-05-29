// @lat: [[process/test-strategy#skill round-trip]]
//
// gravel_grade skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection on empty class set, metadata-only recognition, and target
// class missing from distribution surfacing as a warning (non-blocking).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/gravel_grade.hpp"

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

skill::gravel_grade::Input goodInput()
{
    skill::gravel_grade::Input in;
    in.size_distribution_classes = { "5-10mm", "10-20mm", "20-40mm" };
    in.throughput_kg_h           = 30000.0;
    in.target_class              = "10-20mm";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillGravelGrade, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::gravel_grade::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("gravel_grade"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (empty class set) ─────────────────────
TEST(SkillGravelGrade, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.size_distribution_classes.clear();

    auto r = skill::gravel_grade::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::gravel_grade::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + throughput/classes/target ───────────────
TEST(SkillGravelGrade, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.size_distribution_classes = { "0-5mm", "5-10mm", "10-20mm", "20-40mm" };
    in.throughput_kg_h           = 50000.0;
    in.target_class              = "5-10mm";

    auto out = skill::gravel_grade::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("gravel_grade"));
    EXPECT_NEAR(out.signature.pattern["throughput_kg_h"].get<double>(),
                50000.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["target_class"].get<std::string>(),
              std::string("5-10mm"));
    EXPECT_EQ(out.signature.pattern["class_count"].get<std::size_t>(), 4u);
    EXPECT_EQ(out.signature.pattern["size_distribution_classes"].size(), 4u);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillGravelGrade, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::gravel_grade::apply(*stock, goodInput());

    auto cands = skill::gravel_grade::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["target_class"].get<std::string>(),
              std::string("10-20mm"));

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::gravel_grade::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "gravel_grade cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: target class missing from distribution → warning ───────
TEST(SkillGravelGrade, MissingTargetClassEmitsWarningButProceeds)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.size_distribution_classes = { "5-10mm", "10-20mm" };
    in.target_class              = "40-60mm";    // not in distribution

    auto r = skill::gravel_grade::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "missing target class should warn but not block";
    EXPECT_TRUE(hasFinding(r, "DFM-GRAVEL-TARGET"));
    EXPECT_NO_THROW(skill::gravel_grade::apply(*stock, in));
}
