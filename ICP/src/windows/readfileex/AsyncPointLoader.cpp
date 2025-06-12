//
// Created by lherzberger on 14.05.24.
//

#include "AsyncPointLoader.h"

namespace icp {
recycle::shared_pool<InternalOverlapped> makeOverlappedPool() {
  return {
      []() { return std::make_shared<InternalOverlapped>(); }
  };
}

size_t AsyncPointLoader::submit(PointLoaderJob* job) {
  if (numPendingReads() > queueDepth) {
    SleepEx(0, TRUE);
    //return 0;
  }

  job->internal = std::make_unique<InternalJobData>();
  job->internal->buffer = getOrCreateBuffer(job);
  job->data = job->internal->buffer->data();
  job->internal->overlapped = overlappedPool.allocate();
  job->internal->overlapped->o = {};
  job->internal->complete = false;
  job->internal->success = false;
  job->internal->completedJobs = completedJobs.get();

  LPOVERLAPPED lpOvl = &job->internal->overlapped->o;
  lpOvl->Internal = 0;
  lpOvl->InternalHigh = 0;
  lpOvl->Offset =      static_cast<unsigned long long>(job->fileOffset)           & 0xffffffffllu;
  lpOvl->OffsetHigh = (static_cast<unsigned long long>(job->fileOffset) >> 32llu) & 0xffffffffllu;
  lpOvl->hEvent = job;

  bool success = ReadFileEx(
      job->file->handle,
      job->data,
      job->internal->buffer->size(),
      lpOvl,
      [](DWORD errorCode, DWORD numBytesRead, LPOVERLAPPED overlapped){
        auto job = static_cast<PointLoaderJob*>(overlapped->hEvent);
        if (errorCode != 0) {
          spdlog::error("Could not read {} at {}", job->file->path, job->fileOffset);
        } else if (numBytesRead < job->internal->buffer->size() && (job->fileOffset + numBytesRead) < job->file->size) {
          spdlog::error("Could only read {} of {} bytes of {} at offset {}", numBytesRead, job->internal->buffer->size(), job->file->path, job->fileOffset);
        } else {
          job->internal->success = true;
        }
        job->internal->complete = true;
        job->internal->completedJobs->push(job);
      });

  job->enqueued = success;// || GetLastError() == ERROR_MORE_DATA;
  if (job->enqueued) {
    ++numPendingReads_;
    return 1;
  } else {
    spdlog::warn("could not enqueue read, error code {}", std::system_category().message(GetLastError()));
    job->internal = nullptr;
    return 0;
  }
}

size_t AsyncPointLoader::submitRemaining() {
  return 0;
}

std::vector<PointLoaderJob*> AsyncPointLoader::reap(int minEvents) {
  std::vector<PointLoaderJob*> jobsDone;
  int maxEvents = std::min(numPendingReads(), queueDepth);
  if (maxEvents == 0) {
    return jobsDone;
  }
  PointLoaderJob* job;
  // todo: this is dumb if queueDepth, i.e. maxEvents, is large, because we don't get to submit until we emptied our queue
  while (hasPendingReads() && jobsDone.size() < maxEvents) {
     if (completedJobs->empty()) {
       if (jobsDone.size() < minEvents) {
         SleepEx(0, TRUE);
       } else {
         break;
       }
     } else {
       if (completedJobs->try_pop(job)) {
         if (job->internal->success) {
           jobsDone.push_back(job);
         } else {
           spdlog::warn("unsuccessful read");
         }
         --numPendingReads_;
       }
     }
  }
  return jobsDone;
}

size_t AsyncPointLoader::numPendingReads() const {
  return numPendingReads_;
}
}
