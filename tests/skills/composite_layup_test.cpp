// @lat: [[process/test-strategy#skill round-trip]]
//
// composite_layup skill — metadata-only layup recording.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/composite_layup.hpp"

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

}  // namespace

// ─── 1. Apply handles input + geometry unchanged ──────────────────────────
TEST(SkillCompositeLayup, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 1.2);
    const double volBefore = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::composite_layup::Input in;
    in.ply_count        = 4;
    in.ply_thickness_mm = 0.15;
    in.orientation_deg  = { 0.0, 90.0, 90.0, 0.0 };   // symmetric, balanced
    in.matrix_material  = "epoxy";
    in.fiber_material   = "carbon_t300";

    auto out = skill::composite_layup::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("composite_layup"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate rejects bad input (orientation list mismatch) ────────────
TEST(SkillCompositeLayup, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 1.0);

    skill::composite_layup::Input in;
    in.ply_count        = 4;
    in.ply_thickness_mm = 0.15;
    in.orientation_deg  = { 0.0, 90.0 };    // size != ply_count
    in.matrix_material  = "epoxy";
    in.fiber_material   = "carbon_t300";

    auto r = skill::composite_layup::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LAYUP-ORIENT") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::composite_layup::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params + computed flags ──────────────
TEST(SkillCompositeLayup, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 1.0);

    skill::composite_layup::Input in;
    in.ply_count        = 4;
    in.ply_thickness_mm = 0.20;
    in.orientation_deg  = { 45.0, -45.0, -45.0, 45.0 };  // symmetric, balanced
    in.matrix_material  = "epoxy";
    in.fiber_material   = "carbon_t700";

    auto out = skill::composite_layup::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("composite_layup"));
    EXPECT_EQ(out.signature.pattern["ply_count"].get<int>(), 4);
    EXPECT_NEAR(out.signature.pattern["ply_thickness_mm"].get<double>(), 0.20, 1e-9);
    EXPECT_EQ(out.signature.pattern["matrix_material"].get<std::string>(), "epoxy");
    EXPECT_EQ(out.signature.pattern["fiber_material"].get<std::string>(), "carbon_t700");
    EXPECT_TRUE(out.signature.pattern["symmetric"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["balanced"].get<bool>());
    EXPECT_NEAR(out.signature.pattern["cpt_mm"].get<double>(), 0.80, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillCompositeLayup, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 1.0);

    skill::composite_layup::Input in;
    in.ply_count        = 2;
    in.ply_thickness_mm = 0.15;
    in.orientation_deg  = { 0.0, 90.0 };
    in.matrix_material  = "epoxy";
    in.fiber_material   = "carbon_t300";

    auto out = skill::composite_layup::apply(*stock, in);
    auto cands = skill::composite_layup::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["ply_count"].get<int>(), 2);
    EXPECT_EQ(cands[0].recovered_params["matrix_material"].get<std::string>(), "epoxy");

    // Stripped-metadata workpiece → recognize is empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::composite_layup::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific assertion: asymmetric stack-up emits SYM info finding ────
TEST(SkillCompositeLayup, AsymmetricStackEmitsInfoFinding)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 1.0);

    skill::composite_layup::Input in;
    in.ply_count        = 3;
    in.ply_thickness_mm = 0.15;
    in.orientation_deg  = { 0.0, 45.0, 90.0 };   // asymmetric
    in.matrix_material  = "epoxy";
    in.fiber_material   = "carbon_t300";

    auto r = skill::composite_layup::validate(*stock, in);
    EXPECT_TRUE(r.passed) << "asymmetric stack is info-only, not error";
    bool symInfo = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LAYUP-SYM" && f.severity == "info") {
            symInfo = true; break;
        }
    EXPECT_TRUE(symInfo);
}
