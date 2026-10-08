#ifndef PENDEL_RUNLOG_H
#define PENDEL_RUNLOG_H

#include <string>
#include <utility>
#include <vector>

#include "common/config.h"

// One directory per run (Stage 0.4):
//   <base>/<YYYY-mm-ddTHH-MM-SS>_<mode>/
//     data.csv            the logged records (written by the caller)
//     meta.json           build (git commit, dirty flag), host, kernel, times, command line, results
//     config_used.conf    every config value the run read, including defaults
//     config_source.conf  copy of the config file as given (if any)
namespace runlog {

// Creates the run directory (and <base> if needed) and returns its path. Throws on failure.
std::string makeRunDir(const std::string& base, const std::string& mode);

// Key/value pairs written into meta.json next to the automatic fields. Values are strings;
// numbers should be formatted by the caller.
using Fields = std::vector<std::pair<std::string, std::string>>;

// Writes meta.json, config_used.conf and (if configPath is non-empty) config_source.conf.
void writeMeta(const std::string& dir, const std::string& mode, const std::string& commandLine,
               const Config& cfg, const std::string& configPath, const Fields& extra);

std::string gitCommit();   // compile-time, from the Makefile (-DGIT_HASH)
bool gitDirty();           // true if tracked files differed from the commit at build time

}

#endif
