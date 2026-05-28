// @lat: [[process/test-strategy#skill round-trip]]
//
// radiographic_test — RT (X-ray) inspection skill (metadata-only).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/radiographic_test.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <gp_Dir.hxx>

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

// ── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillRadiographicTest, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "carbon_steel_1045");
    const double volBefore = volumeOf(stock->shape());

    skill::radiographic_test::Input in;
    in.target_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.source_kv         = 200.0;
    in.exposure_min      = 5.0;
    in.film_class        = "C3";

    auto out = skill::radiographic_test::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_TRUE(out.signature.pattern["is_inspection_only"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. ValidateRejectsBadInput: bad kV / exposure / film_class ───────────
TEST(SkillRadiographicTest, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::radiographic_test::Input badKv;
    badKv.target_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    badKv.source_kv         = 0.0;
    badKv.exposure_min      = 5.0;
    badKv.film_class        = "C3";
    auto r1 = skill::radiographic_test::validate(*stock, badKv);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::radiographic_test::apply(*stock, badKv),
                 skill::SkillError);

    skill::radiographic_test::Input badExp = badKv;
    badExp.source_kv    = 200.0;
    badExp.exposure_min = -1.0;
    auto r2 = skill::radiographic_test::validate(*stock, badExp);
    EXPECT_FALSE(r2.passed);

    skill::radiographic_test::Input badFilm = badKv;
    badFilm.source_kv    = 200.0;
    badFilm.exposure_min = 5.0;
    badFilm.film_class   = "C9";   // not a valid ISO class
    auto r3 = skill::radiographic_test::validate(*stock, badFilm);
    EXPECT_FALSE(r3.passed);
}

// ── 3. Low kV on dense material → info, not block ────────────────────────
TEST(SkillRadiographicTest, LowKvOnSteelEmitsInfoOnly)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "carbon_steel_1045");

    skill::radiographic_test::Input in;
    in.target_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.source_kv         = 80.0;     // < 150 → info on steel
    in.exposure_min      = 5.0;
    in.film_class        = "C3";

    auto r = skill::radiographic_test::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-RT-PEN"));
    EXPECT_NO_THROW(skill::radiographic_test::apply(*stock, in));
}

// ── 4. Signature records RT parameters + tooling ─────────────────────────
TEST(SkillRadiographicTest, SignatureRecordsParametersAndTooling)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::radiographic_test::Input in;
    in.target_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.source_kv         = 300.0;
    in.exposure_min      = 8.0;
    in.film_class        = "C4";

    auto out = skill::radiographic_test::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("radiographic_test"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("radiographic_test"));
    EXPECT_NEAR(out.signature.pattern["source_kv"].get<double>(), 300.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["exposure_min"].get<double>(), 8.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["film_class"].get<std::string>(),
              std::string("C4"));
    EXPECT_EQ(out.signature.pattern["iso_standard"].get<std::string>(),
              std::string("ISO_11699-1"));
    EXPECT_EQ(out.signature.pattern["ndt_category"].get<std::string>(),
              std::string("RT"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("x_ray_tube"));
    EXPECT_EQ(out.signature.tooling.tool_material,
              std::string("tungsten_target"));
    EXPECT_GT(out.signature.tooling.est_cycle_time_s, 0.0);
}

// ── 5. Recognize metadata replay round-trip ──────────────────────────────
TEST(SkillRadiographicTest, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::radiographic_test::Input in;
    in.target_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.source_kv         = 200.0;
    in.exposure_min      = 5.0;
    in.film_class        = "C3";

    auto out = skill::radiographic_test::apply(*stock, in);
    auto cands = skill::radiographic_test::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["film_class"].get<std::string>(),
              std::string("C3"));

    skill::Workpiece raw(out.workpiece->shape(), out.workpiece->material());
    EXPECT_TRUE(skill::radiographic_test::recognize(raw).empty());
}
