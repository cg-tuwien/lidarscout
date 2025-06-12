//
// Created by lherzberger on 23.05.24.
//

#pragma once

#ifndef ICP_LASFILE_H
#define ICP_LASFILE_H

#include <filesystem>
#include <utility>

#include "LasHeader.h"
#include "UnbufferedFile.h"
#include "ReadRequestType.h"

namespace icp {
class LasFile {
 public:
  LasFile() = delete;

  explicit LasFile(const std::string& path) :
        path(path),
        file_(std::weak_ptr<UnbufferedFile>()),
        header_(nullptr),
        chunkPointRequestsSubmitted(0),
        headerRequestState(NOT_CLAIMED),
        chunkTableRequestState(NOT_CLAIMED)
  {
    if (!std::filesystem::exists(path)) {
      throw std::runtime_error("file does not exist: " + path);
    }
  }

  /**
   * Initializes the header for this LasFile from a given buffer.
   * If the file is not compressed (i.e., not an LAZ file), the chunk table is initialized directly.
   * If the first chunk point is present within the given buffer, it is read and returned to the caller.
   * @param headerBuffer the buffer to construct the header from
   * @param bufferSize the size of the buffer - reading headerBuffer[bufferSize - 1] must be valid.
   * @return the first chunk point, if it is present in the given buffer, an empty optional otherwise.
   * @throws std::runtime_error if the header has already been initialized
   */
  std::optional<Point> initializeHeader(const std::byte* headerBuffer, size_t bufferSize) {
    if (hasHeader()) {
      throw std::runtime_error("[LasFile::initializeHeader]: header already initialized");
    }

    header_ = std::make_unique<LasHeader>(headerBuffer, bufferSize, hasLazExtension(path));

    // if we're not dealing with an LAZ file, the chunk table is built on header construction
    if (!header_->isCompressed) {
      chunkTableRequestState = RESOLVED;
    }

    std::optional<Point> firstPoint = std::nullopt;
    if (!header_->chunkOffsets.empty()) {
      const bool bufferContainsFirstPoint = (header_->chunkOffsets[0] + header_->pointDataRecordLength) < bufferSize;
      if (bufferContainsFirstPoint) {
        firstPoint = std::make_optional<Point>(header_->parsePoint(headerBuffer + header_->chunkOffsets[chunkPointRequestsSubmitted++]));
        addChunkPointSample(firstPoint.value());
      }
    }

    headerRequestState = RESOLVED;

    return firstPoint;
  }

  /**
   * Initializes the chunk table for this LasFile from a given buffer.
   * @param chunkTableBuffer the buffer to construct the header from
   * @param bufferSize the size of the buffer - reading chunkTableBuffer[bufferSize - 1] must be valid.
   * @throws std::runtime_error if the header has not been initialized yet
   * @throws std::runtime_error if the chunk table has already been initialized
   */
  void initializeChunkTable(const std::byte* chunkTableBuffer, size_t bufferSize) {
    if (!hasHeader()) {
      throw std::runtime_error("[LasFile::initializeChunkTable]: header not initialized");
    }
    if (hasChunkTable()) {
      throw std::runtime_error("[LasFile::initializeChunkTable]: chunk table already initialized");
    }
    header_->parseChunkOffsets(chunkTableBuffer, bufferSize);
    chunkTableRequestState = RESOLVED;
  }

  std::shared_ptr<UnbufferedFile> file() {
    if (auto f = file_.lock(); f) {
      return f;
    }
    std::lock_guard<std::mutex> lock(fileOpenMutex);
    // check if other thread has opened the file in the meantime
    if (auto f = file_.lock(); f) {
      return f;
    }
    auto f = std::make_shared<UnbufferedFile>(path);
    file_ = f;
    return f;
  }

  const LasHeader& header() const { return *header_; }

  std::optional<std::pair<ReadRequestType, off_t>> nextRequest() {
    // todo: if is compressed and chunk table offset missing, try to find it at the end of the file
    // todo: if is compressed and chunk table is missing, produce next chunk read
    // I think this should be it: https://github.com/LASzip/LASzip/blob/1b98e594fc10f39c8ad12b0adcc2183df1a555ca/src/lasreadpoint.cpp#L474


    if (!hasHeader()) {
      int expectedState = NOT_CLAIMED;
      if (headerRequestState != WAITING && headerRequestState.compare_exchange_weak(expectedState, WAITING)) {
        return std::make_optional<std::pair<ReadRequestType, off_t>>({HEADER_REQUEST, 0});
      }
    } else if (!hasChunkTable()) {
      int expectedState = NOT_CLAIMED;
      if (chunkTableRequestState != WAITING && chunkTableRequestState.compare_exchange_weak(expectedState, WAITING)) {
        return std::make_optional<std::pair<ReadRequestType, off_t>>({CHUNK_TABLE_REQUEST, header_->chunkTableOffset.value()});
      }
    } else if (auto pointIndex = chunkPointRequestsSubmitted++; pointIndex < header_->chunkOffsets.size()) {
      return std::make_optional<std::pair<ReadRequestType, off_t>>({CHUNK_POINT_REQUEST, header_->chunkOffsets[pointIndex]});
    }
    return std::nullopt;
  }

  bool hasHeader() const {
    return headerRequestState == RESOLVED;
  }

  bool hasChunkTable() const {
    return chunkTableRequestState == RESOLVED;
  }

  std::optional<size_t> numChunkPoints() const {
    if (hasChunkTable()) {
      return std::make_optional<size_t>(header_->chunkOffsets.size());
    }
    return std::nullopt;
  }

  size_t numChunkPointRequestsSubmitted() const {
    return std::min(chunkPointRequestsSubmitted.load(), numChunkPoints().value_or(0));
  }

  bool allRequestsSubmitted() const {
    return hasHeader() && hasChunkTable() && chunkPointRequestsSubmitted >= header_->chunkOffsets.size();
  }

  void addChunkPointSample(const Point& p) {
    accumulatedR += static_cast<uint32_t>(p.rgba[0]);
    accumulatedG += static_cast<uint32_t>(p.rgba[1]);
    accumulatedB += static_cast<uint32_t>(p.rgba[2]);
    ++numChunkPointsLoaded;
  }

  uint32_t averageColor() const {
    // todo: that's stupid: samples might be added concurrently...
    Point p{};
    if (numChunkPointsLoaded != 0) {
      p.rgba[0] = accumulatedR / numChunkPointsLoaded;
      p.rgba[1] = accumulatedG / numChunkPointsLoaded;
      p.rgba[2] = accumulatedB / numChunkPointsLoaded;
    }
    return p.color;
  }

  std::optional<ChunkBounds> asChunkBounds() const {
    if (hasHeader()) {
      ChunkBounds b{};
      b.min.x = header_->min[0];
      b.min.y = header_->min[1];
      b.min.z = header_->min[2];
      b.max.x = header_->max[0];
      b.max.y = header_->max[1];
      b.max.z = header_->max[2];
      b.color = averageColor();
      return std::make_optional<ChunkBounds>(b);
    }
    return std::nullopt;
  }

  const std::string path;

 private:
  enum RequestState {
    NOT_CLAIMED = 0,
    WAITING = 1,
    RESOLVED = 2,
  };

  std::mutex fileOpenMutex;
  std::weak_ptr<UnbufferedFile> file_;

  std::unique_ptr<LasHeader> header_;

  std::atomic<int> headerRequestState;
  std::atomic<int> chunkTableRequestState;
  std::atomic<size_t> chunkPointRequestsSubmitted;

  // todo: maybe add priority here? only change in controlling thread, workers only read

  std::atomic<uint32_t> accumulatedR;
  std::atomic<uint32_t> accumulatedG;
  std::atomic<uint32_t> accumulatedB;
  std::atomic<size_t> numChunkPointsLoaded;
};
}

#endif  //ICP_LASFILE_H
