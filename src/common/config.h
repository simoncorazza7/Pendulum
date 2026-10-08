#ifndef PENDEL_CONFIG_H
#define PENDEL_CONFIG_H

#include <map>
#include <string>
#include <vector>

// Run configuration (Stage 0.4). File format, one entry per line:
//   key = value    # unit and/or comment
// Blank lines and lines starting with '#' are ignored. Values are SI unless the comment says
// otherwise. Every key that is read is recorded (with defaults that were used), so a run can
// save exactly the configuration it ran with.
class Config {
public:
    static Config fromFile(const std::string& path);   // throws std::runtime_error (with line no.)
    static Config fromString(const std::string& text, const std::string& source = "<string>");

    // Override or add a value, e.g. from the command line ("key=value").
    void set(const std::string& key, const std::string& value);
    void setFromAssignment(const std::string& assignment);   // "key=value"

    bool has(const std::string& key) const;

    // Required values: throw std::runtime_error if the key is missing or malformed.
    double getDouble(const std::string& key) const;
    long getInt(const std::string& key) const;
    std::string getString(const std::string& key) const;
    bool getBool(const std::string& key) const;

    // Optional values: the default is recorded as used.
    double getDouble(const std::string& key, double def) const;
    long getInt(const std::string& key, long def) const;
    std::string getString(const std::string& key, const std::string& def) const;
    bool getBool(const std::string& key, bool def) const;

    // "key = value  # comment" for every key read so far, sorted; defaults marked "(default)".
    std::string dumpUsed() const;
    // Keys present in the file/overrides that were never read (likely typos).
    std::vector<std::string> unusedKeys() const;

private:
    struct Entry {
        std::string value;
        std::string comment;
        std::string origin;          // "file:line" or "override"
        mutable bool used = false;
    };
    const Entry* find(const std::string& key) const;
    const Entry& require(const std::string& key) const;

    std::map<std::string, Entry> entries_;
    mutable std::map<std::string, std::string> defaultsUsed_;
};

#endif
