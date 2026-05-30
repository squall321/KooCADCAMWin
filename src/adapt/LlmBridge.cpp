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

#include <cstdio>             // std::fprintf
#include <cstdlib>            // std::getenv
#include <exception>
#include <string>
#include <vector>

#if defined(_WIN32)
// Use WinHTTP for the Anthropic API call.  WinHTTP ships with Windows;
// no extra dep, no vcpkg port, no curl.  The Anthropic API uses TLS 1.2
// which WinHTTP supports out of the box on Win10+.
// NOTE: project CMake defines WIN32_LEAN_AND_MEAN globally; that strips
// stringapiset.h content (CP_UTF8, MultiByteToWideChar) which we need
// here.  Undef before pulling windows.h so the strings APIs are present.
#  ifdef WIN32_LEAN_AND_MEAN
#    undef WIN32_LEAN_AND_MEAN
#  endif
#  include <windows.h>
#  include <winhttp.h>
#  pragma comment(lib, "winhttp.lib")
#endif

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

// ── Real provider implementation ─────────────────────────────────────────
//
// Anthropic Claude API contract used here:
//
//   POST https://api.anthropic.com/v1/messages
//   Headers:
//     x-api-key:         <env value of api_key_env_var>
//     anthropic-version: 2023-06-01
//     content-type:      application/json
//   Body:
//     { "model":      "<config.model>",
//       "max_tokens": <config.max_tokens>,
//       "system":     "<system_prompt>",
//       "messages":   [ { "role": "user", "content": "<prompt text>" } ] }
//
// Response shape (success):
//     { "id": "...", "type": "message", "role": "assistant",
//       "content": [ { "type": "text", "text": "<reply>" }, ... ],
//       "usage":   { "input_tokens": N, "output_tokens": M }, ... }
//
// We extract the first text block from content[] as the model reply.
// `dispatchOpenAi` (different shape) is intentionally not implemented —
// add when needed.
//
// On any network / parse failure we log a warning to stderr and fall
// back to a deterministic mock string so the caller never sees an
// exception bubble out.

namespace {

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

// Deterministic mock body returned both when no API key is set AND when
// the real HTTP path errors out.  Always begins with the literal marker
// "[mock]" so callers and tests can detect "we didn't actually talk to a
// real LLM".  Kept distinct from `notYetWired` because callers of
// sendPrompt() want a usable string, not an error envelope.
std::string mockReplyFor(const std::string& promptText)
{
    std::string preview = promptText.size() > 80
        ? promptText.substr(0, 80) + "..."
        : promptText;
    return std::string("[mock] LlmBridge offline reply for prompt: \"")
         + preview + "\"";
}

#if defined(_WIN32)

// Convert a narrow UTF-8 string to a wide string for the WinHTTP APIs
// (URL host, path, headers).  Project CMake sets WIN32_LEAN_AND_MEAN
// globally which prevents stringapiset.h from exposing
// MultiByteToWideChar — so we do an ASCII-only byte→wchar widening here.
// All hosts/paths/headers we send are pure ASCII ("api.anthropic.com",
// "/v1/messages", standard header names), so this is safe.
std::wstring toWide(const std::string& s)
{
    std::wstring w;
    w.reserve(s.size());
    for (unsigned char c : s) {
        w.push_back(static_cast<wchar_t>(c));
    }
    return w;
}

// RAII wrapper for WinHTTP handles — closes on scope exit.
struct WinHttpHandle
{
    HINTERNET h = nullptr;
    WinHttpHandle() = default;
    explicit WinHttpHandle(HINTERNET hh) : h(hh) {}
    ~WinHttpHandle() { if (h) ::WinHttpCloseHandle(h); }
    WinHttpHandle(const WinHttpHandle&) = delete;
    WinHttpHandle& operator=(const WinHttpHandle&) = delete;
    operator HINTERNET() const { return h; }
};

// Perform a single POST to api.anthropic.com/v1/messages.
// On success returns the HTTP body as a std::string (the JSON envelope
// the Anthropic server returned).  On any WinHTTP failure returns
// std::nullopt and writes a single line to stderr identifying the
// failure point.
std::optional<std::string> httpsPostAnthropic(
    const std::string& apiKey,
    const std::string& jsonBody,
    int                timeoutSec)
{
    const std::wstring host = L"api.anthropic.com";
    const std::wstring path = L"/v1/messages";

    WinHttpHandle session(::WinHttpOpen(
        L"KooCADCAM-LlmBridge/1.0",
        WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY,
        WINHTTP_NO_PROXY_NAME,
        WINHTTP_NO_PROXY_BYPASS,
        0));
    if (!session) {
        std::fprintf(stderr, "[LlmBridge] WinHttpOpen failed: %lu\n",
                     ::GetLastError());
        return std::nullopt;
    }

    // Timeouts in milliseconds: resolve, connect, send, receive.
    const int ms = timeoutSec * 1000;
    ::WinHttpSetTimeouts(session, ms, ms, ms, ms);

    WinHttpHandle connect(::WinHttpConnect(
        session, host.c_str(), INTERNET_DEFAULT_HTTPS_PORT, 0));
    if (!connect) {
        std::fprintf(stderr, "[LlmBridge] WinHttpConnect failed: %lu\n",
                     ::GetLastError());
        return std::nullopt;
    }

    WinHttpHandle request(::WinHttpOpenRequest(
        connect, L"POST", path.c_str(),
        nullptr, WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES,
        WINHTTP_FLAG_SECURE));
    if (!request) {
        std::fprintf(stderr, "[LlmBridge] WinHttpOpenRequest failed: %lu\n",
                     ::GetLastError());
        return std::nullopt;
    }

    // Headers.  All three are required by Anthropic.
    const std::wstring headers =
        L"x-api-key: " + toWide(apiKey) + L"\r\n"
        L"anthropic-version: 2023-06-01\r\n"
        L"content-type: application/json\r\n";

    if (!::WinHttpSendRequest(
            request,
            headers.c_str(),
            static_cast<DWORD>(headers.size()),
            const_cast<char*>(jsonBody.data()),
            static_cast<DWORD>(jsonBody.size()),
            static_cast<DWORD>(jsonBody.size()),
            0))
    {
        std::fprintf(stderr, "[LlmBridge] WinHttpSendRequest failed: %lu\n",
                     ::GetLastError());
        return std::nullopt;
    }

    if (!::WinHttpReceiveResponse(request, nullptr)) {
        std::fprintf(stderr,
                     "[LlmBridge] WinHttpReceiveResponse failed: %lu\n",
                     ::GetLastError());
        return std::nullopt;
    }

    // Read the response body in chunks.
    std::string body;
    for (;;) {
        DWORD avail = 0;
        if (!::WinHttpQueryDataAvailable(request, &avail)) {
            std::fprintf(stderr,
                "[LlmBridge] WinHttpQueryDataAvailable failed: %lu\n",
                ::GetLastError());
            return std::nullopt;
        }
        if (avail == 0) break;

        std::vector<char> buf(avail);
        DWORD got = 0;
        if (!::WinHttpReadData(request, buf.data(), avail, &got)) {
            std::fprintf(stderr,
                "[LlmBridge] WinHttpReadData failed: %lu\n",
                ::GetLastError());
            return std::nullopt;
        }
        if (got == 0) break;
        body.append(buf.data(), got);
    }

    // Status check — anything other than 2xx is treated as a network
    // failure for fallback purposes.  We still return the body so the
    // caller can include the server's error message in logs.
    DWORD status = 0;
    DWORD statusSize = sizeof(status);
    if (::WinHttpQueryHeaders(
            request,
            WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
            WINHTTP_HEADER_NAME_BY_INDEX,
            &status, &statusSize,
            WINHTTP_NO_HEADER_INDEX))
    {
        if (status < 200 || status >= 300) {
            std::fprintf(stderr,
                "[LlmBridge] Anthropic API returned HTTP %lu: %.200s\n",
                static_cast<unsigned long>(status), body.c_str());
            return std::nullopt;
        }
    }

    return body;
}

#endif  // _WIN32

// Extract the first text block from an Anthropic API JSON response.
// Returns nullopt on any schema mismatch.
//
// Expected shape:
//   { "content": [ { "type": "text", "text": "<reply>" }, ... ], ... }
std::optional<std::string> extractAnthropicText(const std::string& bodyJson)
{
    try {
        json j = json::parse(bodyJson);
        if (!j.contains("content") || !j["content"].is_array()) {
            return std::nullopt;
        }
        for (const auto& block : j["content"]) {
            if (block.is_object()
                && block.contains("type")
                && block["type"].is_string()
                && block["type"].get<std::string>() == "text"
                && block.contains("text")
                && block["text"].is_string())
            {
                return block["text"].get<std::string>();
            }
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr,
            "[LlmBridge] failed to parse Anthropic JSON response: %s\n",
            e.what());
    }
    return std::nullopt;
}

// Common dispatch for the Anthropic provider — used by both run() (with
// system prompt + plan context) and sendPrompt() (plain text).
std::optional<std::string> dispatchAnthropic(
    const std::string& apiKey,
    const std::string& systemPrompt,
    const std::string& userText,
    const LlmConfig&   cfg)
{
#if !defined(_WIN32)
    (void)apiKey; (void)systemPrompt; (void)userText; (void)cfg;
    std::fprintf(stderr,
        "[LlmBridge] real HTTP path is Windows-only (WinHTTP); "
        "non-Windows builds fall back to mock.\n");
    return std::nullopt;
#else
    json body = {
        { "model",      cfg.model },
        { "max_tokens", cfg.max_tokens },
        { "messages",   json::array({
            json{ { "role", "user" }, { "content", userText } }
        }) },
    };
    if (!systemPrompt.empty()) {
        body["system"] = systemPrompt;
    }
    // Anthropic accepts an optional temperature; pass it only when the
    // caller set a non-default value.  (Default 0.0 → deterministic.)
    body["temperature"] = cfg.temperature;

    const std::string bodyStr = body.dump();
    auto resp = httpsPostAnthropic(
        apiKey, bodyStr, static_cast<int>(cfg.timeout.count()));
    if (!resp.has_value()) {
        return std::nullopt;
    }
    return extractAnthropicText(*resp);
#endif
}

LlmResponse notYetWired(const std::string& providerName)
{
    LlmResponse out;
    out.success = false;
    out.error   = "LLM provider not implemented: provider=" + providerName
                + " (only 'anthropic' and 'mock' are wired)";
    return out;
}

}  // namespace

// ── run ──────────────────────────────────────────────────────────────────
//
// Routing rules:
//
//   1. provider == "mock"                              → fallbackViaStub
//   2. api_key_env_var unset/empty                     → fallbackViaStub
//   3. provider == "anthropic"                         → real HTTP call;
//                                                        HTTP failure →
//                                                        fallbackViaStub
//                                                        + logged warning
//   4. provider == "openai" / other                    → "not yet wired"
//                                                        (visible error)
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

    // Rule 4: anything other than anthropic → typed error.  The user
    // explicitly asked for that provider and we want the failure to be
    // visible (silently substituting the stub would mask the gap).
    if (req.config.provider != "anthropic") {
        return notYetWired(req.config.provider);
    }

    // Rule 3: real Anthropic call.
    const std::string sysPrompt = req.system_prompt.empty()
        ? defaultSystemPrompt()
        : req.system_prompt;

    // Construct the user message: current plan as JSON + the instruction.
    // Keeping this textual (rather than two separate user-turn messages)
    // matches the system-prompt instructions which expect a single JSON
    // EditOp reply.
    const std::string userMsg =
        "Current plan:\n" + req.current_plan.toJson().dump(2)
        + "\n\nInstruction:\n" + req.instruction;

    auto replyText = dispatchAnthropic(key, sysPrompt, userMsg, req.config);
    if (!replyText.has_value()) {
        // HTTP / parse failure — log and fall back to the stub so the
        // caller still gets a usable EditOp.  The warning was already
        // written to stderr inside dispatchAnthropic / httpsPostAnthropic.
        std::fprintf(stderr,
            "[LlmBridge] Anthropic call failed; falling back to stub.\n");
        return fallbackViaStub(req);
    }

    // Parse the model reply as JSON → EditOp.
    LlmResponse out;
    out.raw_text = *replyText;
    try {
        json parsed = json::parse(*replyText);
        auto editOpt = parseEditOpJson(parsed);
        if (!editOpt.has_value()) {
            out.success = false;
            out.error   = "model produced JSON but it did not match the "
                          "EditOp schema";
            return out;
        }
        out.success = true;
        out.edit    = std::move(editOpt);
        return out;
    } catch (const std::exception& e) {
        out.success = false;
        out.error   = std::string("model reply was not valid JSON: ")
                    + e.what();
        return out;
    }
}

// ── sendPrompt ───────────────────────────────────────────────────────────
//
// Plain-text wrapper.  Mirrors run()'s routing but without any EditOp
// schema enforcement — caller gets the model's text reply verbatim, or
// the deterministic mock string when offline / on failure.
std::string LlmBridge::sendPrompt(const std::string& promptText,
                                  const LlmConfig&   cfg)
{
    // Mock provider: bypass the network unconditionally.
    if (cfg.provider == "mock") {
        return mockReplyFor(promptText);
    }

    // No key → mock fallback (silent, expected case for CI / offline dev).
    const std::string key = readEnv(cfg.api_key_env_var);
    if (key.empty()) {
        return mockReplyFor(promptText);
    }

    // Non-anthropic provider → mock fallback with a warning.  We don't
    // want sendPrompt() to throw, so visible-error semantics here
    // reduce to a stderr line.
    if (cfg.provider != "anthropic") {
        std::fprintf(stderr,
            "[LlmBridge] sendPrompt: provider '%s' not implemented; "
            "returning mock.\n", cfg.provider.c_str());
        return mockReplyFor(promptText);
    }

    // Real call — no system prompt, no plan context.  The caller has
    // pre-baked whatever framing it wanted into promptText.
    auto reply = dispatchAnthropic(key, std::string{}, promptText, cfg);
    if (!reply.has_value()) {
        std::fprintf(stderr,
            "[LlmBridge] sendPrompt: Anthropic call failed; "
            "returning mock.\n");
        return mockReplyFor(promptText);
    }
    return *reply;
}

}  // namespace koocadcam::adapt
