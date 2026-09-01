/**
 * @file shipped_config.hpp
 * @brief Minimal reader that binds a regression suite to a shipped yaml file
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 *
 * The configuration users copy must be the configuration the suites defend, so
 * a scenario READS the shipped yaml rather than mirroring it. A hand-copied
 * duplicate would let a yaml edit ship a configuration that fails the very
 * suite meant to protect it.
 *
 * The guard is bound to the FILE, not to a list of key names (R14 H1): every
 * entry the file contains must be either consumed by the scenario or named in
 * its allow-list, keys may not repeat, and the values the scenario uses must
 * live in the block users copy. A key the guard neither reads nor allows - and
 * a second block that re-declares the same leaves further down the file - is a
 * failure, not a silent pass.
 *
 * Deliberately a minimal `key: value` reader, not a YAML library: the files are
 * flat parameter blocks, and the point is to have no build dependency between
 * a regression suite and the configuration it defends.
 */

#pragma once

#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace bac_sim
{

struct ConfigEntry
{
  std::string value;
  std::string section;  // the block header this entry appeared under
  int line = 0;
};

struct ConfigFile
{
  bool readable = false;
  std::map<std::string, ConfigEntry> entries;  // `key: value` lines
  std::vector<std::string> sections;           // `key:` lines (block headers)
  std::vector<std::string> duplicates;         // any key that appears twice
};

inline ConfigFile
readConfigFile(const std::string &path)
{
  ConfigFile config;
  std::ifstream file(path);
  if (!file)
  {
    return config;
  }
  config.readable = true;

  const auto trim = [](std::string &text) {
    const std::size_t first = text.find_first_not_of(" \t\r\n");
    const std::size_t last = text.find_last_not_of(" \t\r\n");
    text = (first == std::string::npos) ? std::string{} : text.substr(first, last - first + 1);
  };

  std::set<std::string> seen;
  std::string section;
  std::string line;
  int line_number = 0;
  while (std::getline(file, line))
  {
    ++line_number;
    const std::size_t comment = line.find('#');
    if (comment != std::string::npos)
    {
      line.erase(comment);
    }
    const std::size_t colon = line.find(':');
    if (colon == std::string::npos)
    {
      continue;
    }
    std::string key = line.substr(0, colon);
    std::string value = line.substr(colon + 1);
    trim(key);
    trim(value);
    if (key.empty())
    {
      continue;
    }
    if (!seen.insert(key).second)
    {
      // Last-wins would let a healthy duplicate block mask a broken one.
      config.duplicates.push_back(key + " (line " + std::to_string(line_number) + ")");
      continue;
    }
    if (value.empty())
    {
      config.sections.push_back(key);
      section = key;
      continue;
    }
    ConfigEntry entry;
    entry.value = value;
    entry.section = section;
    entry.line = line_number;
    config.entries.emplace(key, entry);
  }
  return config;
}

inline bool
isAllowedUnconsumed(const std::string &key, const std::vector<std::string> &allowed)
{
  for (const std::string &entry : allowed)
  {
    if (key == entry)
    {
      return true;
    }
  }
  return false;
}

}  // namespace bac_sim
