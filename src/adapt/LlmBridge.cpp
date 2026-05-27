// @lat: [[engine/skills#Layer 5 LLM adapter]]
//
// LlmBridge.cpp — slice-1 implementation.
//
// What lives here today:
//   - defaultSystemPrompt(): one canonical string telling the model how to
//     emit an EditOp in JSON.
//   - parseEditOpJson():     JSON → typed EditOp, with full validation of
//                            kind + per-kind required fields.
//   - fallbackViaStub():     wraps NaturalLanguageStub in the LlmResponse
//                            shape; exceptions get repackaged.
//   - run():                 routing — mock / unset-key → fallback,
//                            real provider → typed "not yet wired" error.
//
// What is intentionally TODO:
//   - The actual HTTP POST to api.anthropic.com/v1/messages (or the
//     openai equivalent).  See `dispatchAnthropic()` below — the request
//     envelope (URL, headers, body shape) is documented in comments, but
//     the function returns an error response immediately because the
//     CMake build currently has no curl/libcpr/cpr/HttpClient dep.
//   - Once a transport library is chosen, ONLY `dispatchAnthropic` (and a
//     parallel `dispatchOpenAi`) need bodies.  The surrounding scaffold
//     (config, request/response, JSON parse, fallback) is provider-neutral
//     and stays unchanged.

#include "LlmBridge.hpp"

#include "skills/Skill.hpp"   // SkillError

#include <cstdlib>            // std::getenv
#include <exception>
#include <string>

namespace koocadcam::adapt {

namespace sk = koocadcam::skill;
using nlohmann::json;

// ── Default system prompt ────────────────────────────────────────────────
//
// One paragraph telling the model exactly how to format its reply.  We
// keep this verbose-ish on purpose: clarity beats brevity when the model
// is the audience.  Updates here MUST stay in sync with
// `parseEditOpJson()` below — they are two ends of the same contract.

namespace {

const char* const kDefaultSystemPrompt =
    "You are an AI assistant editing a CAM (computer-aided manufacturing) "
    "process plan.  A process plan is an ordered list of machining skill "
    "invocations.  Given the current plan and a natural-language "
    "instruction from the user, your job is to produce a single EditOp "
    "describing the change to apply.\n"
    "\n"
    "Respond with ONLY a JSON object — no prose, no markdown fences, no "
    "explanation.  The JSON object must conform to the EditOp schema "
    "below.\n"
    "\n"
    "EditOp JSON schema:\n"
    "{\n"
    "  \"kind\":      one of [\"AppendStep\", \"InsertStepAt\", "
    "\"RemoveStep\", \"ReplaceParams\", \"ReorderSteps\", "
    "\"AnnotateStep\"],\n"
    "  \"index\":     integer index of the target step (0-based); used by "
    "InsertStepAt, RemoveStep, ReplaceParams, ReorderSteps, AnnotateStep,\n"
    "  \"destIndex\": integer index of the destination slot; used ONLY by "
    "ReorderSteps,\n"
    "  \"payload\":   JSON object whose shape depends on \"kind\":\n"
    "                 - AppendStep / InsertStepAt: a full StepInvocation, "
    "                   i.e. { \"skill_id\": string, \"params\": object, "
    "                   \"depends_on\": [int...], \"note\": string };\n"
    "                 - ReplaceParams: the new params object (replaces the "
    "                   step's params verbatim);\n"
    "                 - AnnotateStep: { \"note\": string };\n"
    "                 - RemoveStep / ReorderSteps: payload is unused and "
    "                   may be omitted.\n"
    "}\n"
    "\n"
    "Fields not used by a given kind should be omitted.  Indices are "
    "0-based and must lie within the current plan's range (for "
    "InsertStepAt, \"index\" may equal the plan's length to append at "
    "the end).  If you cannot satisfy the instruction safely, respond "
    "with {\"kind\":\"AnnotateStep\",\"index\":-1,\"payload\":{\"note\":"
    "\"<reason>\"}} — never invent an unsafe edit.";

}  // namespace

std::string LlmBridge::defaultSystemPrompt()
{
    return kDefaultSystemPrompt;
}

// ── parseEditOpJson ──────────────────────────────────────────────────────
//
// Validates the JSON shape against the schema documented above + in the
// system prompt.  Returns nullopt on any mismatch.  We do NOT throw —
// callers want the option type so they can flag "model produced garbage"
// without unwinding.
std::optional<EditOp> LlmBridge::parseEditOpJson(const json& j)
{
    if (!j.is_object())                              return std::nullopt;
    if (!j.contains("kind") || !j["kind"].is_string()) return std::nullopt;

    const std::string kindStr = j["kind"].get<std::string>();

    EditOp op;

    if (kindStr == "AppendStep") {
        op.kind = EditKind::AppendStep;
        if (!j.contains("payload") || !j["payload"].is_object()) return std::nullopt;
        if (!j["payload"].contains("skill_id") ||
            !j["payload"]["skill_id"].is_string())
        {
            return std::nullopt;
        }
        op.payload = j["payload"];
    }
    else if (kindStr == "InsertStepAt") {
        op.kind = EditKind::InsertStepAt;
        if (!j.contains("index") || !j["index"].is_number_integer()) return std::nullopt;
        op.index = j["index"].get<int>();
        if (!j.contains("payload") || !j["payload"].is_object()) return std::nullopt;
        if (!j["payload"].contains("skill_id") ||
            !j["payload"]["skill_id"].is_string())
        {
            return std::nullopt;
        }
        op.payload = j["payload"];
    }
    else if (kindStr == "RemoveStep") {
        op.kind = EditKind::RemoveStep;
        if (!j.contains("index") || !j["index"].is_number_integer()) return std::nullopt;
        op.index = j["index"].get<int>();
    }
    else if (kindStr == "ReplaceParams") {
        op.kind = EditKind::ReplaceParams;
        if (!j.contains("index") || !j["index"].is_number_integer()) return std::nullopt;
        op.index = j["index"].get<int>();
        if (!j.contains("payload") || !j["payload"].is_object()) return std::nullopt;
        op.payload = j["payload"];
    }
    else if (kindStr == "ReorderSteps") {
        op.kind = EditKind::ReorderSteps;
        if (!j.contains("index")     || !j["index"].is_number_integer())     return std::nullopt;
        if (!j.contains("destIndex") || !j["destIndex"].is_number_integer()) return std::nullopt;
        op.index     = j["index"].get<int>();
        op.destIndex = j["destIndex"].get<int>();
    }
    else if (kindStr == "AnnotateStep") {
        op.kind = EditKind::AnnotateStep;
        if (!j.contains("index") || !j["index"].is_number_integer()) return std::nullopt;
        op.index = j["index"].get<int>();
        if (!j.contains("payload") || !j["payload"].is_object()) return std::nullopt;
        if (!j["payload"].contains("note") ||
            !j["payload"]["note"].is_string())
        {
            return std::nullopt;
        }
        op.payload = j["payload"];
    }
    else {
        // Unknown "kind" string.
        return std::nullopt;
    }

    return op;
}

// ── fallbackViaStub ──────────────────────────────────────────────────────
//
// Always returns a populated LlmResponse — never throws.  The stub may
// throw `skill::SkillError` for unrecognized phrases or invalid plans;
// those are caught and repackaged into `error`/`success=false`.
LlmResponse LlmBridge::fallbackViaStub(const LlmRequest& req)
{
    LlmResponse out;
    out.token_count = 0;   // no LLM was invoked
    out.raw_text.clear();
    try {
        EditOp op = stubNaturalLanguageToEditOp(req.instruction, req.current_plan);
        out.success = true;
        out.edit    = std::move(op);
        return out;
    }
    catch (const sk::SkillError& e) {
        out.success = false;
        out.error   = std::string("stub-fallback: ") + e.what();
        return out;
    }
    catch (const std::exception& e) {
        out.success = false;
        out.error   = std::string("stub-fallback (unexpected): ") + e.what();
        return out;
    }
}

// ── Real provider scaffolding (TODO) ─────────────────────────────────────
//
// When we add an HTTP transport (libcpr is the leading candidate — it's a
// thin curl wrapper, header-only-ish, MIT, easy CMake integration), the
// body of `dispatchAnthropic` will look approximately like:
//
//   cpr::Response r = cpr::Post(
//       cpr::Url{"https://api.anthropic.com/v1/messages"},
//       cpr::Header{
//           { "x-api-key",         api_key },
//           { "anthropic-version", "2023-06-01" },
//           { "content-type",      "application/json" },
//       },
//       cpr::Body{ json{
//           { "model",       req.config.model },
//           { "max_tokens",  req.config.max_tokens },
//           { "temperature", req.config.temperature },
//           { "system",      req.system_prompt.empty()
//                            ? defaultSystemPrompt()
//                            : req.system_prompt },
//           { "messages", json::array({
//               { { "role", "user" },
//                 { "content", json::array({ { { "type", "text" },
//                     { "text",
//                       "Current plan:\n" + req.current_plan.toJson().dump(2) +
//                       "\n\nInstruction:\n" + req.instruction } }}) } }
//           }) },
//       }.dump() },
//       cpr::Timeout{ static_cast<long>(req.config.timeout.count() * 1000) });
//
//   // r.text → { "content": [ { "type":"text", "text":"<json>" } ], ... }
//   // Extract the assistant's textual reply, json::parse it, run through
//   // parseEditOpJson, fill LlmResponse.
//
// `dispatchOpenAi` would target https://api.openai.com/v1/chat/completions
// with a different body shape (messages: [{role, content}], no top-level
// "system"; use the "developer" / "system" role in the messages array).
//
// Until a transport dep lands, both functions short-circuit to an error
// LlmResponse.

namespace {

LlmResponse notYetWired(const std::string& providerName)
{
    LlmResponse out;
    out.success = false;
    out.error   = "real LLM provider not yet wired (slice-1 stub): "
                  "provider=" + providerName;
    return out;
}

// Read an env var by name.  Returns empty string when unset (works
// uniformly across MSVC and gcc; std::getenv returns nullptr when unset).
std::string readEnv(const std::string& name)
{
    if (name.empty()) return {};
#if defined(_MSC_VER)
    // MSVC deprecates std::getenv; use _dupenv_s which honors /WX.
    char* buf = nullptr;
    size_t len = 0;
    if (_dupenv_s(&buf, &len, name.c_str()) != 0 || buf == nullptr) {
        return {};
    }
    std::string out(buf);
    std::free(buf);
    return out;
#else
    const char* v = std::getenv(name.c_str());
    return v ? std::string(v) : std::string();
#endif
}

}  // namespace

// ── run ──────────────────────────────────────────────────────────────────
//
// Routing rules (slice-1):
//
//   1. provider == "mock"                              → fallbackViaStub
//   2. api_key_env_var unset/empty                     → fallbackViaStub
//   3. provider == "anthropic" / "openai" / other      → "not yet wired"
//
// Future: rule 3 splits into per-provider dispatch.  Rules 1+2 stay as
// the offline / test-only escape hatch.
LlmResponse LlmBridge::run(const LlmRequest& req)
{
    // Rule 1: explicit mock provider always uses the stub.
    if (req.config.provider == "mock") {
        return fallbackViaStub(req);
    }

    // Rule 2: missing API key → stub fallback regardless of provider.
    // This lets developers run the bridge offline (e.g. CI without
    // secrets) and still exercise the simple patterns the stub knows.
    const std::string key = readEnv(req.config.api_key_env_var);
    if (key.empty()) {
        return fallbackViaStub(req);
    }

    // Rule 3: real provider — not yet wired.  We deliberately do NOT
    // fall back to the stub here, because the user explicitly asked for
    // a real LLM and we want the failure to be visible (the stub would
    // silently substitute and probably reject the instruction).
    return notYetWired(req.config.provider);
}

}  // namespace koocadcam::adapt
