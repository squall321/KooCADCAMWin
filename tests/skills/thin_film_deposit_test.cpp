// @lat: [[process/test-strategy#skill round-trip]]
//
// thin_film_deposit — CVD/PVD/ALD metadata-only skill.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/thin_film_deposit.hpp"

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

// ── 1. Apply: geometry unchanged (clone, volume preserved) ───────────────
TEST(SkillThinFilmDeposit, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCylindricalStock(150.0, 0.775, "silicon_wafer");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::thin_film_deposit::Input in;
    in.process_type        = "CVD";
    in.thickness_nm        = 50.0;
    in.deposit_rate_nm_min = 10.0;

    auto out = skill::thin_film_deposit::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. ValidateRejectsBadInput: bad process, zero thickness, zero rate ──
TEST(SkillThinFilmDeposit, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(150.0, 0.775, "silicon_wafer");

    skill::thin_film_deposit::Input badProc;
    badProc.process_type        = "MBE";        // not in {CVD, PVD, ALD}
    badProc.thickness_nm        = 50.0;
    badProc.deposit_rate_nm_min = 10.0;
    auto r1 = skill::thin_film_deposit::validate(*stock, badProc);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::thin_film_deposit::apply(*stock, badProc),
                 skill::SkillError);

    skill::thin_film_deposit::Input zeroThk;
    zeroThk.process_type        = "PVD";
    zeroThk.thickness_nm        = 0.0;
    zeroThk.deposit_rate_nm_min = 10.0;
    auto r2 = skill::thin_film_deposit::validate(*stock, zeroThk);
    EXPECT_FALSE(r2.passed);

    skill::thin_film_deposit::Input zeroRate;
    zeroRate.process_type        = "PVD";
    zeroRate.thickness_nm        = 50.0;
    zeroRate.deposit_rate_nm_min = 0.0;
    auto r3 = skill::thin_film_deposit::validate(*stock, zeroRate);
    EXPECT_FALSE(r3.passed);
}

// ── 3. SignatureRecordsKind + key params ─────────────────────────────────
TEST(SkillThinFilmDeposit, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCylindricalStock(200.0, 0.775, "silicon_wafer");

    skill::thin_film_deposit::Input in;
    in.process_type        = "ALD";
    in.thickness_nm        = 20.0;
    in.deposit_rate_nm_min = 0.5;

    auto out = skill::thin_film_deposit::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("thin_film_deposit"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("thin_film_deposit"));
    EXPECT_EQ(out.signature.pattern["process_type"].get<std::string>(),
              std::string("ALD"));
    EXPECT_NEAR(out.signature.pattern["thickness_nm"].get<double>(),
                20.0, 1e-12);
    EXPECT_NEAR(out.signature.pattern["deposit_rate_nm_min"].get<double>(),
                0.5, 1e-12);
    EXPECT_TRUE(out.signature.pattern["is_additive"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("deposition_chamber"));
}

// ── 4. RecognizeMetadataReplay ───────────────────────────────────────────
TEST(SkillThinFilmDeposit, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(150.0, 0.775, "silicon_wafer");

    skill::thin_film_deposit::Input in;
    in.process_type        = "PVD";
    in.thickness_nm        = 100.0;
    in.deposit_rate_nm_min = 20.0;
    auto out = skill::thin_film_deposit::apply(*stock, in);

    auto cands = skill::thin_film_deposit::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["process_type"].get<std::string>(),
              std::string("PVD"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::thin_film_deposit::recognize(raw).empty());
}

// ── 5. SPECIFIC: ALD with thickness > 100 nm emits info ──────────────────
TEST(SkillThinFilmDeposit, AldThickEmitsInfo)
{
    auto stock = skill::createCylindricalStock(150.0, 0.775, "silicon_wafer");

    skill::thin_film_deposit::Input in;
    in.process_type        = "ALD";
    in.thickness_nm        = 250.0;           // > 100 nm
    in.deposit_rate_nm_min = 0.1;

    auto r = skill::thin_film_deposit::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "ALD slow throughput is advisory, not blocking";
    EXPECT_TRUE(hasFinding(r, "DFM-DEPO-ALD-THICK"));
    EXPECT_NO_THROW(skill::thin_film_deposit::apply(*stock, in));
}
