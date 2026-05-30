// @lat: [[engine/skills#Layer 5 LLM adapter]]
//
// Layer-5 LlmBridge — slice-1 four-case spec.
//
//   1. DefaultSystemPromptNonEmpty  — defaultSystemPrompt() returns a
//                                     non-empty string that mentions both
//                                     "EditOp" and "JSON".  This protects
//                                     against accidental schema drift in
//                                     the canonical prompt.
//   2. ParseValidEditOpJson         — a syntactically valid EditOp JSON
//                                     (here: ReplaceParams) parses cleanly
//                                     and the resulting fields match.
//   3. ParseInvalidEditOpJson       — garbage JSON returns nullopt; we
//                                     never throw out of parseEditOpJson.
//   4. FallbackPathInvoked          — LlmBridge::run with provider="mock"
//                                     and a stub-supported instruction
//                                     ("remove last step") returns
//                                     success=true with a populated
//                                     RemoveStep EditOp.

#include <gtest/gtest.h>

#include "adapt/EditOp.hpp"
#include "adapt/LlmBridge.hpp"

#include <cstdlib>   // _putenv_s, _dupenv_s, std::free, ::unsetenv
#include <string>

using namespace koocadcam;
using nlohmann::json;

namespace {

// Build a fully-formed StepInvocation for a drill_hole.  Used as a seed
// for the mock-fallback case so the plan is non-empty and "remove last
// step" has something to act on.
process::StepInvocation makeDrillStep(double x, double y,
                                       double dia, double depth)
{
    process::StepInvocation s;
    s.skill_id = "drill_hole";
    s.params = {
        { "position_x_mm", x },
        { "position_y_mm", y },
        { "diameter_mm",   dia },
        { "depth_mm",      depth },
        { "through_hole",  false },
    };
    return s;
}

}  // namespace

// ── 1. DefaultSystemPromptNonEmpty ───────────────────────────────────────
TEST(LayerFiveLlmBridge, DefaultSystemPromptNonEmpty)
{
    const std::string prompt = adapt::LlmBridge::defaultSystemPrompt();

    EXPECT_FALSE(prompt.empty())
        << "defaultSystemPrompt() must return non-empty text";
    EXPECT_NE(prompt.find("EditOp"), std::string::npos)
        << "system prompt must mention 'EditOp' so the model knows the schema name";
    EXPECT_NE(prompt.find("JSON"), std::string::npos)
        << "system prompt must mention 'JSON' so the model knows the output format";
}

// ── 2. ParseValidEditOpJson ──────────────────────────────────────────────
TEST(LayerFiveLlmBridge, ParseValidEditOpJson)
{
    // ReplaceParams with new params for step 0.
    const json valid = {
        { "kind",    "ReplaceParams" },
        { "index",   0 },
        { "payload", {
            { "diameter_mm", 4.5 },
            { "depth_mm",    7.0 },
        } },
    };

    const auto parsed = adapt::LlmBridge::parseEditOpJson(valid);
    ASSERT_TRUE(parsed.has_value())
        << "valid ReplaceParams JSON must parse to an EditOp";

    EXPECT_EQ(parsed->kind, adapt::EditKind::ReplaceParams);
    EXPECT_EQ(parsed->index, 0);
    ASSERT_TRUE(parsed->payload.is_object());
    EXPECT_DOUBLE_EQ(parsed->payload["diameter_mm"].get<double>(), 4.5);
    EXPECT_DOUBLE_EQ(parsed->payload["depth_mm"].get<double>(),    7.0);

    // Spot-check that AppendStep with a full step payload also parses.
    const json appendOp = {
        { "kind", "AppendStep" },
        { "payload", {
            { "skill_id", "drill_hole" },
            { "params",   { { "diameter_mm", 3.0 }, { "depth_mm", 5.0 } } },
            { "note",     "added by LLM" },
        } },
    };
    const auto parsedAppend = adapt::LlmBridge::parseEditOpJson(appendOp);
    ASSERT_TRUE(parsedAppend.has_value());
    EXPECT_EQ(parsedAppend->kind, adapt::EditKind::AppendStep);
    EXPECT_EQ(parsedAppend->payload["skill_id"].get<std::string>(),
              std::string("drill_hole"));
}

// ── 3. ParseInvalidEditOpJson ────────────────────────────────────────────
TEST(LayerFiveLlmBridge, ParseInvalidEditOpJson)
{
    // Not even an object — a bare array.
    const json arr = json::array({ 1, 2, 3 });
    EXPECT_FALSE(adapt::LlmBridge::parseEditOpJson(arr).has_value());

    // Missing "kind" entirely.
    const json missingKind = { { "index", 0 } };
    EXPECT_FALSE(adapt::LlmBridge::parseEditOpJson(missingKind).has_value());

    // Unknown "kind" string.
    const json badKind = { { "kind", "TeleportStep" }, { "index", 0 } };
    EXPECT_FALSE(adapt::LlmBridge::parseEditOpJson(badKind).has_value());

    // RemoveStep with no "index".
    const json missingIndex = { { "kind", "RemoveStep" } };
    EXPECT_FALSE(adapt::LlmBridge::parseEditOpJson(missingIndex).has_value());

    // AppendStep with malformed payload (no skill_id).
    const json noSkillId = {
        { "kind",    "AppendStep" },
        { "payload", { { "params", json::object() } } },
    };
    EXPECT_FALSE(adapt::LlmBridge::parseEditOpJson(noSkillId).has_value());

    // ReorderSteps missing destIndex.
    const json reorderNoDest = {
        { "kind",  "ReorderSteps" },
        { "index", 0 },
    };
    EXPECT_FALSE(adapt::LlmBridge::parseEditOpJson(reorderNoDest).has_value());
}

// ── 4. FallbackPathInvoked ───────────────────────────────────────────────
TEST(LayerFiveLlmBridge, FallbackPathInvoked)
{
    // Plan with one step so "remove last step" has a target.
    process::ProcessPlan plan;
    plan.append(makeDrillStep(10.0, 10.0, 3.0, 5.0));
    ASSERT_EQ(plan.size(), 1);

    adapt::LlmRequest req;
    req.config.provider = "mock";        // force the fallback path
    req.current_plan    = plan;
    req.instruction     = "remove last step";

    const auto resp = adapt::LlmBridge::run(req);

    EXPECT_TRUE(resp.success)
        << "mock-provider fallback must succeed for a stub-supported "
           "phrase; error: " << resp.error;
    ASSERT_TRUE(resp.edit.has_value())
        << "fallback must populate the EditOp on success";
    EXPECT_EQ(resp.edit->kind, adapt::EditKind::RemoveStep);
    EXPECT_EQ(resp.edit->index, 0)
        << "with a 1-step plan, 'remove last step' targets index 0";

    // No real LLM was called, so token_count should be zero.
    EXPECT_EQ(resp.token_count, 0);
}

// ── 5. MockFallbackWhenNoApiKey ──────────────────────────────────────────
//
// sendPrompt() must NEVER touch the network when ANTHROPIC_API_KEY is
// unset (CI / offline guarantee).  It returns a deterministic string
// starting with the literal marker "[mock]" so callers/tests can detect
// "no real LLM was called".
//
// We explicitly unset the env var here in case the developer running
// this test has one in their shell.  This keeps the test reproducible
// across machines.
TEST(LlmBridge, MockFallbackWhenNoApiKey)
{
    // Unset the key for this test scope.  Restore-on-exit not strictly
    // necessary — gtest runs each TEST in process scope, but other tests
    // in this binary do not depend on the env var.
#if defined(_WIN32)
    ::_putenv_s("ANTHROPIC_API_KEY", "");
#else
    ::unsetenv("ANTHROPIC_API_KEY");
#endif

    const std::string reply =
        adapt::LlmBridge::sendPrompt("hello, are you online?");

    // The mock string format is part of the public contract — callers
    // (and Tests in other binaries) rely on the "[mock]" prefix to
    // recognize the offline branch.
    EXPECT_FALSE(reply.empty());
    EXPECT_EQ(reply.rfind("[mock]", 0), 0u)
        << "without API key, sendPrompt() must return a string starting "
           "with the literal marker '[mock]'; got: " << reply;
}

// ── 6. RealHttpCallWhenApiKeySet (DISABLED) ─────────────────────────────
//
// Gated DISABLED because CI cannot ship a real API key.  Documents the
// real-call path: when ANTHROPIC_API_KEY is set and the network is
// reachable, sendPrompt() POSTs to https://api.anthropic.com/v1/messages
// and returns the model's text reply.  The reply should NOT begin with
// the "[mock]" marker.
//
// To run locally:
//   $env:ANTHROPIC_API_KEY = "<your key>"
//   .\build\tests\Debug\llm_bridge_test.exe --gtest_also_run_disabled_tests
TEST(LlmBridge, DISABLED_RealHttpCallWhenApiKeySet)
{
#if defined(_WIN32)
    // Read the developer's real key from the environment — we will NOT
    // set one for them; if it's missing the test deliberately fails.
    char* buf = nullptr;
    size_t len = 0;
    const bool present =
        (_dupenv_s(&buf, &len, "ANTHROPIC_API_KEY") == 0
         && buf != nullptr && std::string(buf).size() > 0);
    if (buf) std::free(buf);
    ASSERT_TRUE(present)
        << "DISABLED_RealHttpCallWhenApiKeySet requires a real "
           "ANTHROPIC_API_KEY in the environment.  Set it and re-run "
           "with --gtest_also_run_disabled_tests.";
#else
    GTEST_SKIP() << "real HTTP path is Windows-only (WinHTTP).";
#endif

    adapt::LlmConfig cfg;
    cfg.provider = "anthropic";
    cfg.model    = "claude-sonnet-4-6";
    cfg.api_key_env_var = "ANTHROPIC_API_KEY";

    const std::string reply = adapt::LlmBridge::sendPrompt(
        "Reply with the single word OK.", cfg);

    EXPECT_FALSE(reply.empty());
    // A real model reply must NOT carry the offline marker.
    EXPECT_NE(reply.rfind("[mock]", 0), 0u)
        << "real LLM call must not return the mock-fallback string; "
           "got: " << reply;
}
