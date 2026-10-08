#include "common/config.h"

#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace {

std::string trim(const std::string& s) {
    const char* ws = " \t\r\n";
    std::size_t b = s.find_first_not_of(ws);
    if (b == std::string::npos) return "";
    std::size_t e = s.find_last_not_of(ws);
    return s.substr(b, e - b + 1);
}

std::string fmtDouble(double v) {
    std::ostringstream os;
    os.precision(12);
    os << v;
    return os.str();
}

}

Config Config::fromFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open config " + path);
    std::stringstream ss;
    ss << in.rdbuf();
    return fromString(ss.str(), path);
}

Config Config::fromString(const std::string& text, const std::string& source) {
    Config cfg;
    std::istringstream in(text);
    std::string line;
    int lineNo = 0;
    while (std::getline(in, line)) {
        ++lineNo;
        std::string comment;
        std::size_t hash = line.find('#');
        if (hash != std::string::npos) {
            comment = trim(line.substr(hash + 1));
            line = line.substr(0, hash);
        }
        line = trim(line);
        if (line.empty()) continue;
        std::size_t eq = line.find('=');
        std::string where = source + ":" + std::to_string(lineNo);
        if (eq == std::string::npos) throw std::runtime_error(where + ": expected 'key = value'");
        std::string key = trim(line.substr(0, eq));
        std::string value = trim(line.substr(eq + 1));
        if (key.empty()) throw std::runtime_error(where + ": empty key");
        if (cfg.entries_.count(key)) throw std::runtime_error(where + ": duplicate key '" + key + "'");
        cfg.entries_[key] = Entry{value, comment, where};
    }
    return cfg;
}

void Config::set(const std::string& key, const std::string& value) {
    auto it = entries_.find(key);
    if (it == entries_.end()) {
        entries_[key] = Entry{value, "", "override"};
    } else {
        it->second.value = value;
        it->second.origin = "override";
    }
}

void Config::setFromAssignment(const std::string& assignment) {
    std::size_t eq = assignment.find('=');
    if (eq == std::string::npos) throw std::runtime_error("override must be key=value: " + assignment);
    set(trim(assignment.substr(0, eq)), trim(assignment.substr(eq + 1)));
}

bool Config::has(const std::string& key) const { return entries_.count(key) != 0; }

const Config::Entry* Config::find(const std::string& key) const {
    auto it = entries_.find(key);
    if (it == entries_.end()) return nullptr;
    it->second.used = true;
    return &it->second;
}

const Config::Entry& Config::require(const std::string& key) const {
    const Entry* e = find(key);
    if (!e) throw std::runtime_error("config key missing: " + key);
    return *e;
}

double Config::getDouble(const std::string& key) const {
    const Entry& e = require(key);
    errno = 0;
    char* end = nullptr;
    double v = std::strtod(e.value.c_str(), &end);
    if (e.value.empty() || *end != '\0' || errno == ERANGE)
        throw std::runtime_error(e.origin + ": '" + key + "' is not a number: " + e.value);
    return v;
}

long Config::getInt(const std::string& key) const {
    const Entry& e = require(key);
    errno = 0;
    char* end = nullptr;
    long v = std::strtol(e.value.c_str(), &end, 10);
    if (e.value.empty() || *end != '\0' || errno == ERANGE)
        throw std::runtime_error(e.origin + ": '" + key + "' is not an integer: " + e.value);
    return v;
}

std::string Config::getString(const std::string& key) const { return require(key).value; }

bool Config::getBool(const std::string& key) const {
    const Entry& e = require(key);
    if (e.value == "true" || e.value == "1" || e.value == "yes") return true;
    if (e.value == "false" || e.value == "0" || e.value == "no") return false;
    throw std::runtime_error(e.origin + ": '" + key + "' is not a bool: " + e.value);
}

double Config::getDouble(const std::string& key, double def) const {
    if (has(key)) return getDouble(key);
    defaultsUsed_[key] = fmtDouble(def);
    return def;
}

long Config::getInt(const std::string& key, long def) const {
    if (has(key)) return getInt(key);
    defaultsUsed_[key] = std::to_string(def);
    return def;
}

std::string Config::getString(const std::string& key, const std::string& def) const {
    if (has(key)) return getString(key);
    defaultsUsed_[key] = def;
    return def;
}

bool Config::getBool(const std::string& key, bool def) const {
    if (has(key)) return getBool(key);
    defaultsUsed_[key] = def ? "true" : "false";
    return def;
}

std::string Config::dumpUsed() const {
    std::map<std::string, std::string> lines;
    for (const auto& [key, e] : entries_) {
        if (!e.used) continue;
        std::string l = key + " = " + e.value;
        if (!e.comment.empty()) l += "    # " + e.comment;
        if (e.origin == "override") l += "    (override)";
        lines[key] = l;
    }
    for (const auto& [key, v] : defaultsUsed_) lines[key] = key + " = " + v + "    (default)";
    std::string out;
    for (const auto& kv : lines) out += kv.second + "\n";
    return out;
}

std::vector<std::string> Config::unusedKeys() const {
    std::vector<std::string> keys;
    for (const auto& [key, e] : entries_)
        if (!e.used) keys.push_back(key);
    return keys;
}
