#pragma once

#include <string>

void loadFileNative(
    const std::string& file,
    uint64_t firstByte,
    uint64_t numBytes,
    void* target,
    uint64_t* out_padding);
