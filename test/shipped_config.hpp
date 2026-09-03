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
 *
 * checkShippedConfig() below IS that guard - one implementation, three callers.
 * A suite supplies only what differs: the path, the wording of the file, the
 * motion_model.type it must select, the key-to-field table, and the allow-list.
 * Until this was shared the holonomic and Ackermann suites carried a guard
 * each, and they had already drifted: the holonomic one rejected a value
 * hoisted out of the FollowPath block at READ time (leaving the field at its
 * default and failing "every value ... was read"), the Ackermann one assigned
 * the value anyway and only complained afterwards. The shared guard keeps the
 * stricter reading.
 *
 * REACH, so that it is not read as more than it is. All three files are checked
 * for structure; only two of them are then RUN. The holonomic and Ackermann
 * suites drive their scenarios from the Params this fills in, so a value edited
 * in those files moves a measured trajectory. test/scenarios.cpp runs
 * bac::Params defaults with a v_max override, so for config/bac_controller.yaml
 * this is a check on the FILE - every key bound or allow-listed, none repeated,
 * every bound value a plain finite number inside the block users copy - and not
 * a closed-loop run of the shipped differential-drive values.
 */

#pragma once

#include <cmath>
#include <cstdlib>
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

/// One `key: value` line a suite consumes, and the field its value lands in.
/// Exactly one of the two pointers is set; build these with bindNumber /
/// bindInteger rather than by hand.
struct ConfigBinding
{
  const char *key;
  float      *number;
  int        *integer;
};

inline ConfigBinding
bindNumber(const char *key, float &field)
{
  return ConfigBinding{key, &field, nullptr};
}

inline ConfigBinding
bindInteger(const char *key, int &field)
{
  return ConfigBinding{key, nullptr, &field};
}

/// Everything one suite's guard needs that differs between the shipped files.
struct ShippedConfigSpec
{
  std::string                path;        ///< from the BAC_*_CONFIG_PATH define
  std::string                label;       ///< "holonomic" / "Ackermann" / "differential-drive"
  std::string                model_name;  ///< the motion_model.type the file must select
  std::vector<ConfigBinding> bindings;    ///< every key the suite consumes
  std::vector<std::string>   allowed;     ///< keys and blocks it may carry unconsumed
};

struct ShippedConfigVerdict
{
  bool readable = false;  ///< the file opened and held at least one `key: value`
  bool complete = false;  ///< every bound value came from the block users copy
};

/// The guard itself. Reports through the caller's own `expect` so a failure
/// lands in that suite's failure count and in the CI log shape it always had.
inline ShippedConfigVerdict
checkShippedConfig(const ShippedConfigSpec &spec, void (*expect)(bool, const std::string &))
{
  ShippedConfigVerdict verdict;
  const ConfigFile config = readConfigFile(spec.path);
  if (!config.readable || config.entries.empty())
  {
    expect(false, "the shipped " + spec.label + " configuration is readable at " + spec.path);
    return verdict;
  }
  verdict.readable = true;

  bool complete = true;
  std::set<std::string> consumed;

  // A value hoisted out of FollowPath is not the value the plugin would load,
  // so it is not read: the field keeps its default and the file is incomplete.
  const auto value_of = [&](const std::string &key) -> const ConfigEntry * {
    consumed.insert(key);
    const auto found = config.entries.find(key);
    if (found == config.entries.end())
    {
      expect(false, "the shipped configuration declares " + key);
      complete = false;
      return nullptr;
    }
    if (found->second.section != "FollowPath")
    {
      expect(false, "the shipped configuration keeps " + key +
                        " inside the FollowPath block users copy (found under '" +
                        found->second.section + "' on line " +
                        std::to_string(found->second.line) + ")");
      complete = false;
      return nullptr;
    }
    return &found->second;
  };

  // The loader takes this one as a STRING, so `"omni"` is legal YAML for it and
  // means the same as the bare form - unlike a quoted number, which is a type
  // error there too. Only a matched outer pair is stripped: `"omni` and `omni"`
  // are not the same value and are not accepted as one.
  const ConfigEntry *model = value_of("motion_model.type");
  std::string model_value = (model == nullptr) ? std::string("<missing>") : model->value;
  if (model_value.size() >= 2U && model_value.front() == '"' && model_value.back() == '"')
  {
    model_value = model_value.substr(1, model_value.size() - 2U);
  }
  if (model == nullptr || model_value != spec.model_name)
  {
    expect(false, "the shipped configuration selects the " + spec.label + " model (got '" +
                      model_value + "')");
    complete = false;
  }

  for (const ConfigBinding &binding : spec.bindings)
  {
    const ConfigEntry *entry = value_of(binding.key);
    if (entry == nullptr)
    {
      continue;
    }
    // A quoted or unit-suffixed number is a type error for the real parameter
    // loader, and `.inf` / `nan` are values no vehicle can drive; report the KEY
    // rather than letting std::stof abort the process without one (R14 L5).
    const char *begin = entry->value.c_str();
    char       *end   = nullptr;
    const float parsed = std::strtof(begin, &end);
    if (end == begin || *end != '\0' || !std::isfinite(parsed))
    {
      expect(false, "the shipped configuration gives " + std::string(binding.key) +
                        " a plain finite number (got '" + entry->value + "' on line " +
                        std::to_string(entry->line) + ")");
      complete = false;
      continue;
    }
    if (binding.number != nullptr)
    {
      *binding.number = parsed;
    }
    else
    {
      *binding.integer = static_cast<int>(parsed);
    }
  }

  // --- the guard is on the FILE, not on the key list above (R14 H1) ---
  for (const std::string &duplicate : config.duplicates)
  {
    expect(false, "the shipped configuration declares " + duplicate +
                      " only once; a repeated key means a second block is "
                      "masking the one users copy");
  }
  for (const std::string &section : config.sections)
  {
    expect(isAllowedUnconsumed(section, spec.allowed),
           "the shipped configuration block '" + section + "' is one this suite knows about");
  }
  for (const auto &entry : config.entries)
  {
    if (consumed.count(entry.first) != 0U)
    {
      continue;
    }
    expect(isAllowedUnconsumed(entry.first, spec.allowed),
           "the shipped configuration key " + entry.first + " (line " +
               std::to_string(entry.second.line) +
               ") is exercised by this suite or listed in kAllowedUnconsumedKeys on purpose");
  }

  verdict.complete = complete;
  return verdict;
}

}  // namespace bac_sim
