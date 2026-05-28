// @lat: [[process/test-strategy#skill round-trip]]
//
// calibration_check — gauge calibration verification.  Covers:
//   1. apply clones geometry unchanged
//   2. validate rejects bad input
//   3. signature records kind + gauge_id + last_cal_date + due_date + trace
//   4. recognize metadata replay
//   5. specific: overdue detection (current_date >= due_date)

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/calibration_check.hpp"

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

// ─── 1. Apply clones geometry unchanged ──────────────────────────────────
TEST(SkillCalibrationCheck, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::calibration_check::Input in;
    in.gauge_id      = "GAUGE-CMM-007";
    in.last_cal_date = "2026-01-15";
    in.due_date      = "2027-01-15";
    in.traceability  = "NIST";
    in.current_date  = "2026-05-28";

    auto out = skill::calibration_check::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillCalibrationCheck, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Empty gauge_id → error.
    skill::calibration_check::Input in;
    in.gauge_id      = "";          // INVALID
    in.last_cal_date = "2026-01-15";
    in.due_date      = "2027-01-15";
    in.traceability  = "NIST";
    auto r = skill::calibration_check::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::calibration_check::apply(*stock, in),
                 skill::SkillError);

    // due_date < last_cal_date → DFM-CAL-ORDER error.
    skill::calibration_check::Input in2;
    in2.gauge_id      = "GAUGE-X";
    in2.last_cal_date = "2026-06-01";
    in2.due_date      = "2026-05-01";       // before last_cal!
    in2.traceability  = "NIST";
    auto r2 = skill::calibration_check::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-CAL-ORDER"));

    // Empty last_cal_date → error.
    skill::calibration_check::Input in3;
    in3.gauge_id      = "GAUGE-Y";
    in3.last_cal_date = "";         // INVALID
    in3.due_date      = "2027-01-01";
    in3.traceability  = "NIST";
    auto r3 = skill::calibration_check::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillCalibrationCheck, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::calibration_check::Input in;
    in.gauge_id      = "MICRO-42";
    in.last_cal_date = "2026-03-10";
    in.due_date      = "2027-03-10";
    in.traceability  = "KRISS";
    in.current_date  = "2026-05-28";

    auto out = skill::calibration_check::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("calibration_check"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("calibration_check"));
    EXPECT_EQ(out.signature.pattern["gauge_id"].get<std::string>(),
              std::string("MICRO-42"));
    EXPECT_EQ(out.signature.pattern["last_cal_date"].get<std::string>(),
              std::string("2026-03-10"));
    EXPECT_EQ(out.signature.pattern["due_date"].get<std::string>(),
              std::string("2027-03-10"));
    EXPECT_EQ(out.signature.pattern["traceability"].get<std::string>(),
              std::string("KRISS"));
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["overdue"].get<bool>());
}

// ─── 4. Recognize metadata replay ────────────────────────────────────────
TEST(SkillCalibrationCheck, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::calibration_check::Input in;
    in.gauge_id      = "GAUGE-REP";
    in.last_cal_date = "2026-04-01";
    in.due_date      = "2027-04-01";
    in.traceability  = "NIST";
    auto out = skill::calibration_check::apply(*stock, in);

    auto cands = skill::calibration_check::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].skill_id, std::string("calibration_check"));
    EXPECT_EQ(cands[0].recovered_params["gauge_id"].get<std::string>(),
              std::string("GAUGE-REP"));
    EXPECT_EQ(cands[0].recovered_params["traceability"].get<std::string>(),
              std::string("NIST"));

    // Workpiece without metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape(), out.workpiece->material());
    EXPECT_TRUE(skill::calibration_check::recognize(raw).empty());
}

// ─── 5. Specific: overdue detection (current_date >= due_date) ───────────
TEST(SkillCalibrationCheck, OverdueDetection)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Not yet due — overdue should be false, no DFM-CAL-OVERDUE.
    skill::calibration_check::Input ok;
    ok.gauge_id      = "GAUGE-OK";
    ok.last_cal_date = "2026-01-01";
    ok.due_date      = "2027-01-01";
    ok.traceability  = "NIST";
    ok.current_date  = "2026-05-28";
    auto rOK = skill::calibration_check::validate(*stock, ok);
    EXPECT_TRUE(rOK.passed);
    EXPECT_FALSE(hasFinding(rOK, "DFM-CAL-OVERDUE"));
    auto outOK = skill::calibration_check::apply(*stock, ok);
    EXPECT_FALSE(outOK.signature.pattern["overdue"].get<bool>());

    // Overdue — current_date strictly after due_date.
    skill::calibration_check::Input late;
    late.gauge_id      = "GAUGE-LATE";
    late.last_cal_date = "2024-01-01";
    late.due_date      = "2025-01-01";
    late.traceability  = "NIST";
    late.current_date  = "2026-05-28";        // far past due
    auto rLate = skill::calibration_check::validate(*stock, late);
    EXPECT_TRUE(rLate.passed);                // info-only, still passes
    EXPECT_TRUE(hasFinding(rLate, "DFM-CAL-OVERDUE"));
    auto outLate = skill::calibration_check::apply(*stock, late);
    EXPECT_TRUE(outLate.signature.pattern["overdue"].get<bool>());

    // Boundary: current_date == due_date is overdue (>=).
    skill::calibration_check::Input today;
    today.gauge_id      = "GAUGE-TODAY";
    today.last_cal_date = "2025-05-28";
    today.due_date      = "2026-05-28";
    today.traceability  = "NIST";
    today.current_date  = "2026-05-28";
    auto rT = skill::calibration_check::validate(*stock, today);
    EXPECT_TRUE(hasFinding(rT, "DFM-CAL-OVERDUE"));

    // Unknown traceability → info finding.
    skill::calibration_check::Input bad;
    bad.gauge_id      = "GAUGE-WEIRD";
    bad.last_cal_date = "2026-01-01";
    bad.due_date      = "2027-01-01";
    bad.traceability  = "MADE-UP-LAB";
    bad.current_date  = "2026-05-28";
    auto rB = skill::calibration_check::validate(*stock, bad);
    EXPECT_TRUE(rB.passed);
    EXPECT_TRUE(hasFinding(rB, "DFM-CAL-TRACE"));
}
