// Минимальный разбор и вывод JSON: конфигурация и ответы веб-интерфейса.
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace wz {

class JsonValue {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };

    JsonValue() = default;
    explicit JsonValue(bool b) : type_(Type::Bool), bool_(b) {}
    explicit JsonValue(double n) : type_(Type::Number), num_(n) {}
    explicit JsonValue(int n) : type_(Type::Number), num_(n) {}
    explicit JsonValue(const char* s) : type_(Type::String), str_(s) {}
    explicit JsonValue(std::string s) : type_(Type::String), str_(std::move(s)) {}

    static JsonValue array() {
        JsonValue v;
        v.type_ = Type::Array;
        return v;
    }
    static JsonValue object() {
        JsonValue v;
        v.type_ = Type::Object;
        return v;
    }

    Type type() const { return type_; }
    bool isNull() const { return type_ == Type::Null; }
    bool isNumber() const { return type_ == Type::Number; }
    bool isString() const { return type_ == Type::String; }
    bool isArray() const { return type_ == Type::Array; }
    bool isObject() const { return type_ == Type::Object; }

    // Чтение с запасным значением - конфигурация не должна падать из-за
    // отсутствующего необязательного поля.
    double asNumber(double fallback = 0.0) const;
    bool asBool(bool fallback = false) const;
    std::string asString(const std::string& fallback = std::string()) const;

    const JsonValue* find(const std::string& key) const;
    double numberAt(const std::string& key, double fallback) const;
    bool boolAt(const std::string& key, bool fallback) const;
    std::string stringAt(const std::string& key, const std::string& fallback) const;

    const std::vector<JsonValue>& items() const { return array_; }
    size_t size() const { return type_ == Type::Array ? array_.size() : object_.size(); }

    void push(JsonValue v);
    void set(const std::string& key, JsonValue v);

    std::string dump(int indent = -1) const;

    static bool parse(const std::string& text, JsonValue& out, std::string* error);
    static bool parseFile(const std::string& path, JsonValue& out, std::string* error);

private:
    void dumpTo(std::string& out, int indent, int depth) const;

    Type type_ = Type::Null;
    bool bool_ = false;
    double num_ = 0.0;
    std::string str_;
    std::vector<JsonValue> array_;
    std::vector<std::pair<std::string, JsonValue>> object_;
};

std::string jsonEscape(const std::string& s);

}  // namespace wz
