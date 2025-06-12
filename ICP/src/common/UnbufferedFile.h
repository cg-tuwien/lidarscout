//
// Created by lherzberger on 06.05.24.
//

#pragma once

#ifndef ICP_UNBUFFEREDFILE_H
#define ICP_UNBUFFEREDFILE_H

#include <filesystem>
#include <string>
#include <stdexcept>

#ifdef __linux__
#include <fcntl.h>
#elif defined(WIN32) || defined(_WIN32) || defined(__WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

#include "utils.h"

namespace icp {
#ifdef __linux__
using FileHandle = int;
#elif defined(WIN32) || defined(_WIN32) || defined(__WIN32)
using FileHandle = HANDLE;
#endif

class UnbufferedFile {
public:
  const std::string path;
  const size_t size;
  const FileHandle handle;

  explicit UnbufferedFile(const std::string &path)
      : path(path),
        size(std::filesystem::file_size(path)),
        handle(UnbufferedFile::createHandle(path)) {}

  virtual ~UnbufferedFile() {
#ifdef __linux__
    close(handle);
#elif defined(WIN32) || defined(_WIN32) || defined(__WIN32)
    CloseHandle(handle);
#endif
  }

 private:
  static FileHandle createHandle(const std::string& path) {
#ifdef __linux__
    auto handle = open(path.c_str(), O_RDONLY | O_DIRECT, 0644);
    if (handle < 0) {
      throw std::runtime_error("could not open file");
    }
    return handle;
#elif defined(WIN32) || defined(_WIN32) || defined(__WIN32)
    const auto accessMode = GENERIC_READ;
    const auto shareMode = FILE_SHARE_READ;
    const auto createDisposition = OPEN_EXISTING;
    const auto flags =
        FILE_ATTRIBUTE_NORMAL |
        FILE_FLAG_POSIX_SEMANTICS |
        FILE_FLAG_OVERLAPPED |
        FILE_FLAG_NO_BUFFERING |
        FILE_FLAG_RANDOM_ACCESS;
    auto handle = CreateFile(
        path.c_str(),
        accessMode,
        shareMode,
        nullptr,
        createDisposition,
        flags,
        nullptr);
    if (handle == INVALID_HANDLE_VALUE) {
      throw std::runtime_error("could not open file");
    }
    return handle;
#endif
  }

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32)
  std::optional<UINT32> registrationIndex = std::nullopt;
#endif
};
}

#endif  // ICP_UNBUFFEREDFILE_H
