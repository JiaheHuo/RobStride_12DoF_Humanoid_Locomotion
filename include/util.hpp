#pragma once
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <cstdint>

inline std::vector<std::string> split_csv(const std::string& s) {
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item.erase(item.begin(), std::find_if(item.begin(), item.end(), [](unsigned char ch){ return !std::isspace(ch); }));
    item.erase(std::find_if(item.rbegin(), item.rend(), [](unsigned char ch){ return !std::isspace(ch); }).base(), item.end());
    if (!item.empty()) out.push_back(item);
  }
  return out;
}

inline uint32_t parse_u32_hex_or_dec(const std::string& s) {
  std::string t = s;
  t.erase(std::remove_if(t.begin(), t.end(), ::isspace), t.end());
  if (t.rfind("0x", 0) == 0 || t.rfind("0X", 0) == 0) {
    return static_cast<uint32_t>(std::stoul(t, nullptr, 16));
  }
  return static_cast<uint32_t>(std::stoul(t, nullptr, 10));
}
