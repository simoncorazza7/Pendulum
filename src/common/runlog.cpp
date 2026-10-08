#include "common/runlog.h"

#include <cerrno>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <sys/utsname.h>
#include <unistd.h>

#ifndef GIT_HASH
#define GIT_HASH "unknown"
#endif
#ifndef GIT_DIRTY
#define GIT_DIRTY 1
#endif

namespace runlog {
namespace {

void mkdirOrExisting(const std::string& path) {
    if (mkdir(path.c_str(), 0775) != 0 && errno != EEXIST)
        throw std::runtime_error("mkdir " + path + ": " + std::strerror(errno));
}

std::string jsonEscape(const std::string& s) {
    std::string out;
    for (char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) out += ' ';
                else out += c;
        }
    }
    return out;
}

std::string timeString(const char* fmt, bool utc) {
    std::time_t now = std::time(nullptr);
    std::tm tm {};
    if (utc) gmtime_r(&now, &tm); else localtime_r(&now, &tm);
    char buf[64];
    std::strftime(buf, sizeof buf, fmt, &tm);
    return buf;
}

}

std::string gitCommit() { return GIT_HASH; }
bool gitDirty() { return GIT_DIRTY != 0; }

std::string makeRunDir(const std::string& base, const std::string& mode) {
    mkdirOrExisting(base);
    std::string stem = base + "/" + timeString("%Y-%m-%dT%H-%M-%S", false) + "_" + mode;
    std::string dir = stem;
    for (int i = 2; mkdir(dir.c_str(), 0775) != 0; ++i) {   // same second: add a suffix
        if (errno != EEXIST) throw std::runtime_error("mkdir " + dir + ": " + std::strerror(errno));
        dir = stem + "_" + std::to_string(i);
    }
    return dir;
}

void writeMeta(const std::string& dir, const std::string& mode, const std::string& commandLine,
               const Config& cfg, const std::string& configPath, const Fields& extra) {
    utsname un {};
    uname(&un);
    Fields f = {
        {"mode", mode},
        {"git_commit", gitCommit()},
        {"git_dirty", gitDirty() ? "true" : "false"},
        {"build_time", std::string(__DATE__) + " " + __TIME__},
        {"host", un.nodename},
        {"kernel", std::string(un.release) + " " + un.version},
        {"machine", un.machine},
        {"written_utc", timeString("%Y-%m-%dT%H:%M:%SZ", true)},
        {"command_line", commandLine},
        {"config_path", configPath},
    };
    f.insert(f.end(), extra.begin(), extra.end());

    std::ofstream meta(dir + "/meta.json");
    if (!meta) throw std::runtime_error("cannot write " + dir + "/meta.json");
    meta << "{\n";
    for (std::size_t i = 0; i < f.size(); ++i)
        meta << "  \"" << jsonEscape(f[i].first) << "\": \"" << jsonEscape(f[i].second) << "\""
             << (i + 1 < f.size() ? "," : "") << "\n";
    meta << "}\n";

    std::ofstream used(dir + "/config_used.conf");
    used << "# Config values read by this run (git " << gitCommit()
         << (gitDirty() ? ", dirty" : "") << ")\n" << cfg.dumpUsed();

    if (!configPath.empty()) {
        std::ifstream src(configPath);
        std::ofstream copy(dir + "/config_source.conf");
        copy << src.rdbuf();
    }
}

}
