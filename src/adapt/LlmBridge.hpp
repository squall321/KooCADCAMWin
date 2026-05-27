#pragma once
// @lat: [[engine/skills#Layer 5 LLM adapter]]
//
// LlmBridge — pluggable real-LLM bridge for the adapt/ layer.
//
// Design intent
// ─────────────
// `NaturalLanguageStub` is a rule-based regex matcher that handles a tiny
// vocabulary of canned phrases.  Eventually we want a real LLM (Claude,
// GPT, a local model) to translate arbitrary natural-language instructions
// into typed `EditOp`s against the current `ProcessPlan`.
//
// Slice-1 scope: this header + .cpp establish the **framework** for that
// pluggable bridge — config struct, request / response shapes, JSON-schema
// system prompt, JSON → EditOp parser, and a fallback path that delegates
// to `stubNaturalLanguageToEditOp` when no real provider is available.
//
// Slice-1 explicitly does NOT make any network call.  Real provider work
// (HTTP POST to api.anthropic.com/v1/messages or openai compatible) is a
// TODO and a follow-up that will introduce a curl-class dep.  Until then,
// `run()` either uses the stub fallback or returns a typed error.
//
// Why a bridge layer (not direct API calls scattered through the codebase)?
//
//   - **Single response shape**: `LlmResponse` is identical regardless of
//     which provider answered.  Callers (UI, tests, CLI) don't branch on
//     provider type.
//   - **Deterministic test path**: `provider = "mock"` always falls back
//     to the stub — tests stay reproducible without network.
//   - **Switchable**: an environment-driven config (api_key_env_var) means
//     swapping providers requires only setting `LlmConfig.provider`.
//   - **Safe failure mode**: a missing API key never aborts; it just
//     downgrades to the stub.  This preserves slice-1's offline guarantee.
//
// What the bridge does NOT do (yet):
//   - Streaming responses (token-by-token).
//   - Multi-turn conversation state.
//   - Function/tool calling (Anthropic tools API, OpenAI function_calling).
//   - Retries / backoff.
//   - HTTP itself — there is no curl dep, no socket; the "real" provider
//     branch returns `error = "real LLM provider not yet wired (slice-1
//     stub)"`.

#include "EditOp.hpp"
#include "NaturalLanguageStub.hpp"

#include <nlohmann/json.hpp>

#include <chrono>
#include <optional>
#include <string>

namespace koocadcam::adapt {

// ── LlmConfig ────────────────────────────────────────────────────────────
//
// Static configuration for a single LLM call.  Sensible defaults make it
// easy to spin up: `LlmConfig{}` selects deterministic Claude Sonnet with
// the Anthropic API key from ``ANTHROPIC_API_KEY``.
struct LlmConfig
{
    // "anthropic" | "openai" | "mock".  Anything else → error path.
    std::string provider = "anthropic";

    // Provider-specific model id.  For Anthropic, e.g.
    // "claude-sonnet-4-6" / "claude-opus-4-7".  For OpenAI, e.g.
    // "gpt-4o-2024-08-06".  Free-form string; the bridge does not
    // validate this against a known list.
    std::string model = "claude-sonnet-4-6";

    // Name of the environment variable holding the API key.  If unset,
    // `run()` falls back to the stub regardless of `provider`.
    std::string api_key_env_var = "ANTHROPIC_API_KEY";

    // Provider hyper-parameters.  Temperature 0.0 = deterministic, which
    // we want when the model is producing structured JSON.
    int    max_tokens = 4096;
    double temperature = 0.0;

    // Request timeout.  Once a real HTTP path is added, this will be
    // wired into the curl easy handle.  Today it's recorded for
    // forward-compatibility but not consulted.
    std::chrono::seconds timeout{30};
};

// ── LlmRequest ───────────────────────────────────────────────────────────
//
// A single bridge call: "given the current plan + this NL instruction,
// produce one EditOp".  `system_prompt` is overridable; empty means use
// `LlmBridge::defaultSystemPrompt()`.
struct LlmRequest
{
    LlmConfig            config;
    process::ProcessPlan current_plan;
    std::string          instruction;
    std::string          system_prompt;   // defaultSystemPrompt() if empty
};

// ── LlmResponse ──────────────────────────────────────────────────────────
//
// Uniform return shape — both real and fallback paths populate the same
// fields.  Inspect `success` first; on failure consult `error`.
//
// `raw_text` carries the model's textual reply so callers can log it.
// For the fallback path it's an empty string (the stub doesn't generate
// "text", it generates a typed EditOp directly).
struct LlmResponse
{
    bool                  success     = false;
    std::optional<EditOp> edit;            // present iff parse succeeded
    std::string           raw_text;        // model output (for debugging)
    std::string           error;           // populated iff !success
    int                   token_count = 0; // 0 for stub-fallback path
};

// ── LlmBridge ────────────────────────────────────────────────────────────
class LlmBridge
{
public:
    // Single-shot call.  Returns an LlmResponse populated according to the
    // routing rules:
    //
    //   - provider == "mock" OR (provider != "mock" AND API key env var is
    //     unset/empty) → call fallbackViaStub.
    //   - otherwise → return error: "real LLM provider not yet wired
    //     (slice-1 stub)".  (Real anthropic/openai HTTP comes in a
    //     follow-up; see provider envelope notes in LlmBridge.cpp.)
    //
    // Does NOT throw on stub fallback errors — they're packaged into
    // `error`/`success=false` so callers see one uniform shape.
    static LlmResponse run(const LlmRequest& req);

    // The default system prompt.  Returns a non-empty string that
    // describes the EditOp JSON schema and tells the model to respond
    // in JSON only.  See LlmBridge.cpp for the exact prompt text.
    static std::string defaultSystemPrompt();

    // Parse a model-produced JSON blob into an EditOp.  Returns nullopt
    // if the JSON does not match the expected schema (missing/invalid
    // "kind", missing required fields per kind, etc.).
    //
    // Accepted schema (matches the system prompt):
    //   {
    //     "kind": "AppendStep" | "InsertStepAt" | "RemoveStep" |
    //             "ReplaceParams" | "ReorderSteps" | "AnnotateStep",
    //     "index":     <int, optional except where required>,
    //     "destIndex": <int, ReorderSteps only>,
    //     "payload":   <object, kind-specific>
    //   }
    static std::optional<EditOp> parseEditOpJson(const nlohmann::json& j);

    // Fallback: delegate to the rule-based NaturalLanguageStub.  Always
    // returns an `LlmResponse`; the stub's `skill::SkillError` is caught
    // and packaged into `error`/`success=false`.
    static LlmResponse fallbackViaStub(const LlmRequest& req);
};

}  // namespace koocadcam::adapt
