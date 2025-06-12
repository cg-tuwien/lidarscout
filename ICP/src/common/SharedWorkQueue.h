//
// Created by lherzberger on 23.05.24.
//

#pragma once

#ifndef ICP_SHAREDWORKQUEUE_H
#define ICP_SHAREDWORKQUEUE_H

#include <atomic>
#include <deque>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <vector>

#include "PointLoaderJob.h"

namespace icp {
template <typename T>
class FixedSizeWorkQueue {
 public:
  explicit FixedSizeWorkQueue(std::vector<T> work) : work(std::move(work)), begin(0) {}

  size_t size() const { return work.size() - std::min(work.size(), begin.load()); }

  size_t empty() const { return size() == 0; }

  std::optional<T> pop() {
    if (!work.empty()) {
      if (size_t i = begin++; i < work.size()) {
        return std::make_optional<T>(work[i]);
      }
    }
    return std::nullopt;
  }

 private:
  std::vector<T> work;
  std::atomic_size_t begin = 0;
};

template <typename T>
class ResizableWorkQueue {
 public:
  explicit ResizableWorkQueue(std::deque<T> work) : work(std::move(work)) {}

  size_t size() const {
    std::shared_lock lock(mutex);
    return work.size();
  }

  size_t empty() const { return size() == 0; }

  void push(T element) {
    std::unique_lock lock(mutex);
    work.push_back(std::move(element));
  }

  void push(std::vector<T>& elements) {
    std::unique_lock lock(mutex);
    for (auto& element : elements) {
      work.push_back(std::move(element));
    }
  }

  std::optional<T> pop() {
    std::unique_lock lock(mutex);
    if (work.empty()) {
      return std::nullopt;
    } else {
      const auto file = work.front();
      work.pop_front();
      return std::make_optional<T>(file);
    }
  }

  std::vector<T> pop_n(size_t n) {
    std::vector<T> result;
    result.reserve(n);
    std::unique_lock lock(mutex);
    while (!work.empty() && result.size() < n) {
      result.emplace_back(work.front());
      work.pop_front();
    }
    return result;
  }

  void sort(const std::function<bool(const T&, const T&)>& compareOp) {
    std::deque<T> toSort{};
    {
      std::shared_lock lock(mutex);
      toSort = std::deque<T>(work.begin(), work.end());
    }
    std::sort(toSort.begin(), toSort.end(), compareOp);

    std::unique_lock lock(mutex);
    work = toSort;
  }

 private:
  std::deque<T> work;
  mutable std::shared_mutex mutex;
};

class SharedWorkManager {
 public:
  SharedWorkManager() = delete;

  SharedWorkManager(std::vector<std::string> files, size_t numThreads)
      : files(std::move(files)),
        numThreads(numThreads),
        preparedJobs({}),
        noPendingHeaderRequests(0),
        numStarvingThreads(0) {}

  std::optional<std::string> tryGetHeaderRequest() { return files.pop(); }

  std::optional<std::shared_ptr<PointLoaderJob>> tryGetJob() { return preparedJobs.pop(); }

  std::vector<std::shared_ptr<PointLoaderJob>> tryGetJobs() {
    return preparedJobs.pop_n(std::max(static_cast<int>(preparedJobs.size() / numThreads), 1));
  }

  void shareJobs(std::vector<std::shared_ptr<PointLoaderJob>> jobs) { preparedJobs.push(jobs); }

  bool allowedToExit() const {
    return noPendingHeaderRequests == numThreads && preparedJobs.empty() && files.empty();
  }

  void reportNoPendingHeaderRequests() { ++noPendingHeaderRequests; }

  void reportStarving() { ++numStarvingThreads; }

  void reportNotStarvingAnymore() { --numStarvingThreads; }

  bool hasStarving() const { return numStarvingThreads > 0; }

 private:
  const size_t numThreads;
  std::atomic_int noPendingHeaderRequests;
  std::atomic_int numStarvingThreads;
  FixedSizeWorkQueue<std::string> files;
  ResizableWorkQueue<std::shared_ptr<PointLoaderJob>> preparedJobs;
};
}

#endif  //ICP_SHAREDWORKQUEUE_H
