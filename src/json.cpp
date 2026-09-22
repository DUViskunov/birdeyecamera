#include "wz/json.hpp"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace wz {

double JsonValue::asNumber(double fallback) const {
    if (type_ == Type::Number) return num_;
    if (type_ == Type::Bool) return bool_ ? 1.0 : 0.0;
    return fallback;
}

bool JsonValue::asBool(bool fallback) const {
    if (type_ == Type::Bool) return bool_;
    if (type_ == Type::Number) return std::fabs(num_) > 1e-12;
    return fallback;
}

std::string JsonValue::asString(const std::string& fallback) const {
    if (type_ == Type::String) return str_;
    return fallback;
}

const JsonValue* JsonValue::find(const std::string& key) const {
    if (type_ != Type::Object) return nullptr;
    for (const auto& kv : object_) {
        if (kv.first == key) return &kv.second;
    }
    return nullptr;
}

double JsonValue::numberAt(const std::string& key, double fallback) const {
    const JsonValue* v = find(key);
    return v ? v->asNumber(fallback) : fallback;
}

bool JsonValue::boolAt(const std::string& key, bool fallback) const {
    const JsonValue* v = find(key);
    return v ? v->asBool(fallback) : fallback;
}

std::string JsonValue::stringAt(const std::string& key, const std::string& fallback) const {
    const JsonValue* v = find(key);
    return v ? v->asString(fallback) : fallback;
}

void JsonValue::push(JsonValue v) {
    if (type_ != Type::Array) {
        type_ = Type::Array;
        array_.clear();
    }
    array_.push_back(std::move(v));
}

void JsonValue::set(const std::string& key, JsonValue v) {
    if (type_ != Type::Object) {
        type_ = Type::Object;
        object_.clear();
    }
    for (auto& kv : object_) {
        if (kv.first == key) {
            kv.second = std::move(v);
            return;
        }
    }
    object_.emplace_back(key, std::move(v));
}

std::string jsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (unsigned char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (c < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    // UTF-8 пропускаем как есть: экранирование не требуется.
                    out += static_cast<char>(c);
                }
        }
    }
    return out;
}

namespace {

void appendNumber(std::string& out, double v) {
    // Целые печатаем без дробной части - конфигурация читабельнее.
    if (std::fabs(v - std::floor(v)) < 1e-9 && std::fabs(v) < 1e15) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.0f", v);
        out += buf;
        return;
    }
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%.10g", v);
    out += buf;
}

void newlineIndent(std::string& out, int indent, int depth) {
    if (indent < 0) return;
    out += '\n';
    out.append(static_cast<size_t>(indent) * static_cast<size_t>(depth), ' ');
}

}  // namespace

void JsonValue::dumpTo(std::string& out, int indent, int depth) const {
    switch (type_) {
        case Type::Null: out += "null"; break;
        case Type::Bool: out += bool_ ? "true" : "false"; break;
        case Type::Number: appendNumber(out, num_); break;
        case Type::String:
            out += '"';
            out += jsonEscape(str_);
            out += '"';
            break;
        case Type::Array: {
            if (array_.empty()) {
                out += "[]";
                break;
            }
            out += '[';
            for (size_t i = 0; i < array_.size(); ++i) {
                if (i) out += ',';
                newlineIndent(out, indent, depth + 1);
                array_[i].dumpTo(out, indent, depth + 1);
            }
            newlineIndent(out, indent, depth);
            out += ']';
            break;
        }
        case Type::Object: {
            if (object_.empty()) {
                out += "{}";
                break;
            }
            out += '{';
            for (size_t i = 0; i < object_.size(); ++i) {
                if (i) out += ',';
                newlineIndent(out, indent, depth + 1);
                out += '"';
                out += jsonEscape(object_[i].first);
                out += "\": ";
                object_[i].second.dumpTo(out, indent, depth + 1);
            }
            newlineIndent(out, indent, depth);
            out += '}';
            break;
        }
    }
}

std::string JsonValue::dump(int indent) const {
    std::string out;
    dumpTo(out, indent, 0);
    return out;
}

namespace {

// Рекурсивный спуск. Дополнительно допускаются строчные комментарии //:
// строгий JSON их запрещает, но для файла настроек они полезны.
class Parser {
public:
    explicit Parser(const std::string& text) : s_(text) {}

    bool parse(JsonValue& out, std::string* error) {
        skipWs();
        if (!parseValue(out)) {
            if (error) *error = fail_;
            return false;
        }
        skipWs();
        if (pos_ != s_.size()) {
            if (error) *error = "лишние символы после значения, позиция " + std::to_string(pos_);
            return false;
        }
        return true;
    }

private:
    bool error(const std::string& msg) {
        if (fail_.empty()) fail_ = msg + ", позиция " + std::to_string(pos_);
        return false;
    }

    void skipWs() {
        while (pos_ < s_.size()) {
            const char c = s_[pos_];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
                ++pos_;
            } else if (c == '/' && pos_ + 1 < s_.size() && s_[pos_ + 1] == '/') {
                while (pos_ < s_.size() && s_[pos_] != '\n') ++pos_;
            } else {
                break;
            }
        }
    }

    bool literal(const char* text, JsonValue value, JsonValue& out) {
        const size_t n = std::char_traits<char>::length(text);
        if (s_.compare(pos_, n, text) != 0) return error("неизвестный литерал");
        pos_ += n;
        out = std::move(value);
        return true;
    }

    bool parseValue(JsonValue& out) {
        if (pos_ >= s_.size()) return error("неожиданный конец документа");
        const char c = s_[pos_];
        if (c == '{') return parseObject(out);
        if (c == '[') return parseArray(out);
        if (c == '"') {
            std::string str;
            if (!parseString(str)) return false;
            out = JsonValue(std::move(str));
            return true;
        }
        if (c == 't') return literal("true", JsonValue(true), out);
        if (c == 'f') return literal("false", JsonValue(false), out);
        if (c == 'n') return literal("null", JsonValue(), out);
        return parseNumber(out);
    }

    bool parseObject(JsonValue& out) {
        out = JsonValue::object();
        ++pos_;  // '{'
        skipWs();
        if (pos_ < s_.size() && s_[pos_] == '}') {
            ++pos_;
            return true;
        }
        while (true) {
            skipWs();
            std::string key;
            if (!parseString(key)) return false;
            skipWs();
            if (pos_ >= s_.size() || s_[pos_] != ':') return error("ожидалось ':'");
            ++pos_;
            skipWs();
            JsonValue value;
            if (!parseValue(value)) return false;
            out.set(key, std::move(value));
            skipWs();
            if (pos_ >= s_.size()) return error("незакрытый объект");
            if (s_[pos_] == ',') {
                ++pos_;
                continue;
            }
            if (s_[pos_] == '}') {
                ++pos_;
                return true;
            }
            return error("ожидалось ',' или '}'");
        }
    }

    bool parseArray(JsonValue& out) {
        out = JsonValue::array();
        ++pos_;  // '['
        skipWs();
        if (pos_ < s_.size() && s_[pos_] == ']') {
            ++pos_;
            return true;
        }
        while (true) {
            skipWs();
            JsonValue value;
            if (!parseValue(value)) return false;
            out.push(std::move(value));
            skipWs();
            if (pos_ >= s_.size()) return error("незакрытый массив");
            if (s_[pos_] == ',') {
                ++pos_;
                continue;
            }
            if (s_[pos_] == ']') {
                ++pos_;
                return true;
            }
            return error("ожидалось ',' или ']'");
        }
    }

    bool parseString(std::string& out) {
        if (pos_ >= s_.size() || s_[pos_] != '"') return error("ожидалась строка");
        ++pos_;
        out.clear();
        while (pos_ < s_.size()) {
            const char c = s_[pos_++];
            if (c == '"') return true;
            if (c != '\\') {
                out += c;
                continue;
            }
            if (pos_ >= s_.size()) break;
            const char esc = s_[pos_++];
            switch (esc) {
                case '"': out += '"'; break;
                case '\\': out += '\\'; break;
                case '/': out += '/'; break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'n': out += '\n'; break;
                case 'r': out += '\r'; break;
                case 't': out += '\t'; break;
                case 'u': {
                    if (pos_ + 4 > s_.size()) return error("оборванная \\u-последовательность");
                    const unsigned code = static_cast<unsigned>(
                        std::strtoul(s_.substr(pos_, 4).c_str(), nullptr, 16));
                    pos_ += 4;
                    // Кодируем в UTF-8.
                    if (code < 0x80) {
                        out += static_cast<char>(code);
                    } else if (code < 0x800) {
                        out += static_cast<char>(0xC0 | (code >> 6));
                        out += static_cast<char>(0x80 | (code & 0x3F));
                    } else {
                        out += static_cast<char>(0xE0 | (code >> 12));
                        out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
                        out += static_cast<char>(0x80 | (code & 0x3F));
                    }
                    break;
                }
                default: return error("недопустимое экранирование");
            }
        }
        return error("незакрытая строка");
    }

    bool parseNumber(JsonValue& out) {
        const size_t start = pos_;
        if (pos_ < s_.size() && (s_[pos_] == '-' || s_[pos_] == '+')) ++pos_;
        while (pos_ < s_.size() && (std::isdigit(static_cast<unsigned char>(s_[pos_])) ||
                                    s_[pos_] == '.' || s_[pos_] == 'e' || s_[pos_] == 'E' ||
                                    s_[pos_] == '-' || s_[pos_] == '+')) {
            ++pos_;
        }
        if (pos_ == start) return error("ожидалось число");

        const std::string token = s_.substr(start, pos_ - start);
        char* end = nullptr;
        const double v = std::strtod(token.c_str(), &end);
        if (end != token.c_str() + token.size()) return error("некорректное число");
        out = JsonValue(v);
        return true;
    }

    const std::string& s_;
    size_t pos_ = 0;
    std::string fail_;
};

}  // namespace

bool JsonValue::parse(const std::string& text, JsonValue& out, std::string* error) {
    Parser p(text);
    return p.parse(out, error);
}

bool JsonValue::parseFile(const std::string& path, JsonValue& out, std::string* error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        if (error) *error = "не удалось открыть " + path;
        return false;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return parse(ss.str(), out, error);
}

}  // namespace wz
