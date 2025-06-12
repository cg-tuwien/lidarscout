//
// Created by Lukas on 13.05.2024.
//

#pragma once

#ifndef ICP_POINTLOADERJOB_H
#define ICP_POINTLOADERJOB_H

#include <memory>

#include "LasHeader.h"
#include "UnbufferedFile.h"
#include "ReadRequestType.h"
#include "LasFile.h"

namespace icp {
struct InternalJobData;

struct PointLoaderJob {
  std::shared_ptr<LasFile> lasFile = nullptr;
  std::shared_ptr<UnbufferedFile> file = nullptr;
  // todo: header is now part of LasFile
  std::shared_ptr<LasHeader> header = nullptr;
  std::unique_ptr<InternalJobData> internal = nullptr;

  /**
   * the byte offset into the file, multiple of 4096 (or whatever sector size is)
   */
  off_t fileOffset = 0;

  /**
   * byte offset into the local buffer that will contain the point cloud data once the job is completed
   */
  off_t bufferOffset = 0;

  /**
   * The data buffer that wil contain the point cloud data read from file
   */
  std::byte* data = nullptr;

  /**
   * The size of the data buffer. Accessing this->data[this->dataSize] may result in a segmentation fault.
   */
  size_t dataSize = 0;


  /**
   * the byte offset of this job's chunk in the file (offset of uncompressed point & size of chunk)
   */
  off_t chunkOffset = 0;

  /**
   * if a chunk point overlaps two sectors, we need to read both (at least for LAZ files, because we need to get the size of the chunk to produce the next job)
   */
  bool usesTwoSectors = false;

  bool enqueued = false;

  ReadRequestType requestType = HEADER_REQUEST;

  //constexpr bool isHeaderRequest() const { return fileOffset == 0; }

  constexpr bool isHeaderRequest() const { return requestType == HEADER_REQUEST; }
  constexpr bool isChunkPointsRequest() const { return requestType == CHUNK_POINT_REQUEST; }
  constexpr bool isChunkTableRequest() const { return requestType == CHUNK_TABLE_REQUEST; }

  constexpr off_t chunkPointOffset() const { return (chunkOffset - fileOffset) + bufferOffset; }
};
}

#endif  //ICP_POINTLOADERJOB_H
