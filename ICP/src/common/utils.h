//
// Created by lukas on 02.05.24.
//

#ifndef ICP_UTILS_H
#define ICP_UTILS_H

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <chrono>
#include <filesystem>
#include <cstring>
#include <string>

namespace icp {
constexpr bool isPowerOf2(size_t v) {
  return (v != 0 && ((v & (v - 1)) == 0));
}

template <typename T> T read(const std::byte* buffer, off_t offset) {
  T value;
  memcpy(&value, buffer + offset, sizeof(T));
  return value;
}

template <typename ClockTime, typename Timeout>
inline bool isTimeout(const ClockTime& start, Timeout timeoutSeconds) {
  return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start).count() > timeoutSeconds;
}

inline bool endsWith(const std::string& value, const std::string& suffix) {
  return value.size() >= suffix.size() && value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

inline std::string toLower(const std::string& string) {
  std::string lower = string;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return std::tolower(c); });
  return std::move(lower);
}

inline bool hasLazExtension(const std::string& path) {
  return endsWith(toLower(path), "laz");
}

inline bool hasLasOrLazExtension(const std::string& path) {
  auto ext = toLower(path);
  return endsWith(ext, "las") || endsWith(ext, "laz");
}

size_t maxOpenFileHandles();
}

#endif //ICP_UTILS_H
