#pragma once
// @lat: [[engine/feature-watch#JSON Schema]]
//
// Driven-dimension pre-pass (modeling roadmap #3a).  Product specs are plain
// JSON whose numeric fields are independent values — there is no way to say
// "bezel_outer_dia = case_dia - 2".  resolveExpressions() adds that, opt-in and
// non-invasive:
//
//   * A STRING leaf whose value starts with '=' is an EXPRESSION; everything
//     else (plain strings like form_factor="round", and existing numbers) is
//     left exactly as-is, so no current spec is affected.
//   * An expression references other numeric fields by DOTTED PATH
//     (e.g. "= base.diameter_mm - 2 * bezel.width_mm") and supports + - * /,
//     parentheses and unary minus, with normal precedence.
//   * Chained references (a = "=b", b = "=c") resolve via a fixpoint; an
//     unresolvable reference or a reference cycle is a reported error.
//
// Header-only and depends only on nlohmann::json, so both koo_io and koo_engine
// can call it (buildAll runs it as a pre-pass) without a link dependency.

#include <nlohmann/json.hpp>

#include <cctype>
#include <stdexcept>
#include <string>
#include <vector>

namespace koocadcam::io {

namespace detail {

// Thrown when an expression references a field that is itself an as-yet
// unresolved expression — the fixpoint loop retries it next round.
struct ExprPending : std::runtime_error { using std::runtime_error::runtime_error; };
// Thrown on a genuine error (parse failure, unknown/non-numeric reference).
struct ExprError   : std::runtime_error { using std::runtime_error::runtime_error; };

// Navigate `root` by a dotted path ("base.diameter_mm"); returns a pointer to
// the JSON value or nullptr if any segment is missing.
inline const nlohmann::json* navigate(const nlohmann::json& root, const std::string& path)
{
    const nlohmann::json* cur = &root;
    std::size_t i = 0;
    while (i < path.size()) {
        std::size_t dot = path.find('.', i);
        const std::string key = path.substr(i, dot == std::string::npos ? dot : dot - i);
        if (!cur->is_object() || !cur->contains(key)) return nullptr;
        cur = &(*cur)[key];
        if (dot == std::string::npos) break;
        i = dot + 1;
    }
    return cur;
}

// Recursive-descent evaluator over a single "=..." expression body (the text
// after the '='), resolving field references against `root`.
class Evaluator
{
public:
    Evaluator(const std::string& src, const nlohmann::json& root) : s_(src), root_(root) {}

    double run()
    {
        const double v = parseExpr();
        skipWs();
        if (pos_ != s_.size()) throw ExprError("trailing tokens in expression: " + s_);
        return v;
    }

private:
    const std::string& s_;
    const nlohmann::json& root_;
    std::size_t pos_ = 0;

    void skipWs() { while (pos_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[pos_]))) ++pos_; }
    char peek()  { skipWs(); return pos_ < s_.size() ? s_[pos_] : '\0'; }

    double parseExpr()                       // + and - (lowest precedence)
    {
        double v = parseTerm();
        for (;;) {
            const char c = peek();
            if (c == '+') { ++pos_; v += parseTerm(); }
            else if (c == '-') { ++pos_; v -= parseTerm(); }
            else return v;
        }
    }

    double parseTerm()                       // * and /
    {
        double v = parseFactor();
        for (;;) {
            const char c = peek();
            if (c == '*') { ++pos_; v *= parseFactor(); }
            else if (c == '/') {
                ++pos_;
                const double d = parseFactor();
                if (d == 0.0) throw ExprError("division by zero in expression: " + s_);
                v /= d;
            }
            else return v;
        }
    }

    double parseFactor()                     // unary minus, parens, number, reference
    {
        const char c = peek();
        if (c == '-') { ++pos_; return -parseFactor(); }
        if (c == '+') { ++pos_; return parseFactor(); }
        if (c == '(') {
            ++pos_;
            const double v = parseExpr();
            if (peek() != ')') throw ExprError("missing ')' in expression: " + s_);
            ++pos_;
            return v;
        }
        if (std::isdigit(static_cast<unsigned char>(c)) || c == '.') return parseNumber();
        if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') return parseReference();
        throw ExprError("unexpected character in expression: " + s_);
    }

    double parseNumber()
    {
        skipWs();
        const std::size_t start = pos_;
        while (pos_ < s_.size() &&
               (std::isdigit(static_cast<unsigned char>(s_[pos_])) || s_[pos_] == '.'))
            ++pos_;
        return std::stod(s_.substr(start, pos_ - start));
    }

    double parseReference()
    {
        skipWs();
        const std::size_t start = pos_;
        while (pos_ < s_.size() &&
               (std::isalnum(static_cast<unsigned char>(s_[pos_])) ||
                s_[pos_] == '_' || s_[pos_] == '.'))
            ++pos_;
        const std::string path = s_.substr(start, pos_ - start);
        const nlohmann::json* v = navigate(root_, path);
        if (!v) throw ExprError("unknown field reference '" + path + "'");
        if (v->is_number()) return v->get<double>();
        if (v->is_string()) {
            const std::string& sv = v->get_ref<const std::string&>();
            if (!sv.empty() && sv[0] == '=') throw ExprPending(path);  // resolve later
        }
        throw ExprError("field reference '" + path + "' is not numeric");
    }
};

// Walk the tree collecting JSON pointers to every "=..." string leaf.
inline void collectExprLeaves(const nlohmann::json& node, nlohmann::json::json_pointer ptr,
                              std::vector<nlohmann::json::json_pointer>& out)
{
    if (node.is_object()) {
        for (auto it = node.begin(); it != node.end(); ++it)
            collectExprLeaves(it.value(), ptr / it.key(), out);
    } else if (node.is_array()) {
        for (std::size_t i = 0; i < node.size(); ++i)
            collectExprLeaves(node[i], ptr / static_cast<int>(i), out);
    } else if (node.is_string()) {
        const std::string& s = node.get_ref<const std::string&>();
        if (!s.empty() && s[0] == '=') out.push_back(ptr);
    }
}

}  // namespace detail

// Resolve every "=expr" string leaf in `spec` into a number, in place.  Plain
// strings and existing numbers are untouched.  Returns false + `err` on a parse
// error, an unknown/non-numeric reference, or a reference cycle.
inline bool resolveExpressions(nlohmann::json& spec, std::string& err)
{
    std::vector<nlohmann::json::json_pointer> pending;
    detail::collectExprLeaves(spec, nlohmann::json::json_pointer{}, pending);

    // Fixpoint: each round resolves every expression whose references are all
    // numeric.  An expression that references an unresolved one defers (Pending)
    // to the next round.  When a round resolves nothing yet work remains, the
    // remaining set is an unresolvable reference or a cycle.
    while (!pending.empty()) {
        std::vector<nlohmann::json::json_pointer> still;
        bool progressed = false;
        for (const auto& ptr : pending) {
            const std::string body = spec.at(ptr).get<std::string>().substr(1);  // drop '='
            try {
                const double v = detail::Evaluator(body, spec).run();
                spec.at(ptr) = v;
                progressed = true;
            } catch (const detail::ExprPending&) {
                still.push_back(ptr);                       // a ref not resolved yet
            } catch (const detail::ExprError& e) {
                err = std::string("spec expression at ") + ptr.to_string() + ": " + e.what();
                return false;
            }
        }
        if (!progressed) {
            err = "spec expression cycle or unresolvable reference at " + still.front().to_string();
            return false;
        }
        pending.swap(still);
    }
    return true;
}

}  // namespace koocadcam::io
