#!/usr/bin/env python3
# @lat: [[process/test-strategy#skill-contract]]
#
# Feature-chain contract drift guard (breakthrough plan B2, ctest: feature_chain_guard).
#
# The Workpiece contract requires every skill apply() to carry the FULL prior
# feature history into its output.  ~320 skills silently violated this until
# the 2026-06 codemod; this guard makes the drift structurally impossible to
# reintroduce: it FAILS (exit 1) when a skill .cpp stamps a signature without
# a recognized propagation mechanism.
#
# Recognized compliant mechanisms:
#   1. explicit loop      : for (const auto& X : Y.features()) Z->addFeature(X);
#   2. batch copy         : setFeatures(Y.features())
#   3. shared helpers     : cloneWithSignature(...), stampNoOpCoating(...)
#   4. delegation         : output workpiece TAKEN FROM an inner skill's
#                           SkillOutput (inner skill propagates) — these files
#                           are whitelisted EXPLICITLY below; adding a file to
#                           the whitelist requires the same review as code.
#
# Site-level rule (catches the classic violation shape):
#   auto wpNew = std::make_shared<Workpiece>(...);   <- fresh construction
#   wpNew->addFeature(sig);                          <- stamp w/o propagation
# A propagation loop must sit between construction and stamp.
#
# Known residual blind spot (accepted): a file that propagates at one site
# but adds a second, non-adjacent fresh-construction branch later.  Reviews
# and the SkillContract runtime test cover that case.

import re
import sys
from pathlib import Path

SKILLS_DIR = Path(__file__).resolve().parents[2] / "src" / "skills"

# Non-skill infrastructure living in src/skills/.
INFRA = {"Workpiece.cpp", "Stock.cpp"}

# Delegation pattern: output workpiece comes from an inner skill's SkillOutput,
# which (post-codemod) already carries the full chain.  Each entry was verified
# manually on 2026-06-11; keep this list SHORT and justified.
DELEGATION_WHITELIST = {
    "adaptive_clearing.cpp",        # wpNew = pocketOut.workpiece
    "bolt_circle_pattern.cpp",      # current = drill_hole::apply(...).workpiece per hole
    "counterbore_ring_pattern.cpp", # current = counterbore::apply(...).workpiece per instance
    "countersink_ring_pattern.cpp", # current = countersink::apply(...).workpiece per instance
    "linear_hole_array.cpp",        # current = drill_hole::apply(...).workpiece per hole
    "rectangular_hole_grid.cpp",    # current = drill_hole::apply(...).workpiece per hole
    "coaxial_step_bore.cpp",        # current = drill_hole::apply(...).workpiece per step
    "bore_and_finish.cpp",          # wpNew = step3.workpiece
    "drill_and_tap.cpp",            # wpNew = drillOut.workpiece
    "drill_through_hole.cpp",       # wpNew = drillOut.workpiece (2-entry layout, documented)
    "hard_turning.cpp",             # wpNew = baseOut.workpiece
    "heli_coil.cpp",                # copies inner.workpiece->features() minus inner sig
    "lapping_op.cpp",               # explicit loop at the stamp site (branchy construction)
    "plunge_roughing.cpp",          # wpNew = pocketOut.workpiece
    "pocket_with_corner_relief.cpp",# seeds working copy with wp.features()
    "rest_milling.cpp",             # seeds working copy with wp.features()
    "trochoidal_mill.cpp",          # wpNew = pocketOut.workpiece
    "ultra_precision_diamond_turn.cpp",  # wpNew = baseOut.workpiece
}

PROP  = re.compile(r"for \(const auto& (\w+) : \w+(?:\.|->)features\(\)\)\s*\n?\s*\w+->addFeature\(\1\)")
SETF  = re.compile(r"setFeatures\(\w+(?:\.|->)features\(\)\)")
CONS  = re.compile(r"^\s*(?:auto|std::shared_ptr<Workpiece>)\s+(\w+)\s*=\s*std::make_shared<Workpiece>\(")
LOOPL = re.compile(r"for \(const auto& \w+ : \w+(?:\.|->)features\(\)\)")


def site_violations(lines):
    """Construction immediately stamped without a propagation loop between."""
    out = []
    for i, line in enumerate(lines):
        m = CONS.match(line)
        if not m:
            continue
        var = m.group(1)
        stamp = re.compile(rf"\s*{var}->addFeature\(\w+\);")
        for j in range(i + 1, min(i + 4, len(lines))):
            if LOOPL.search(lines[j]) or SETF.search(lines[j]):
                break                      # propagation present (loop or batch)
            if stamp.match(lines[j]):
                out.append(i + 1)          # 1-based line of the construction
                break
    return out


def main():
    violations = []
    for f in sorted(SKILLS_DIR.glob("*.cpp")):
        if f.name in INFRA:
            continue
        text = f.read_text(encoding="utf-8", errors="replace")
        if "addFeature(" not in text and "cloneWithSignature" not in text:
            continue                       # never stamps — out of scope
        if f.name in DELEGATION_WHITELIST:
            continue
        compliant = (PROP.search(text) or SETF.search(text)
                     or "cloneWithSignature" in text
                     or "stampNoOpCoating" in text)
        if not compliant:
            violations.append(f"{f.name}: stamps a signature without any "
                              f"recognized propagation mechanism")
            continue
        for ln in site_violations(text.split("\n")):
            violations.append(f"{f.name}:{ln}: fresh Workpiece stamped without "
                              f"a propagation loop between construction and "
                              f"addFeature")

    if violations:
        print("feature-chain contract violations:")
        for v in violations:
            print("  " + v)
        print(f"\n{len(violations)} violation(s). See Workpiece.hpp contract "
              f"and docs/breakthrough_plan.md track B2.")
        return 1
    print("feature-chain contract: all skill files compliant")
    return 0


if __name__ == "__main__":
    sys.exit(main())
