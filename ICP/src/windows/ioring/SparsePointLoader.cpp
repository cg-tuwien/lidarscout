//
// Created by lherzberger on 23.05.24.
//

#include "spdlog/spdlog.h"

#include "SparsePointLoader.h"
#include "PointLoaderJob.h"

namespace icp {
void SparsePointLoader::loadChunkPointsInner(std::vector<Point>& chunkPoints, std::unordered_map<std::string, ChunkBoundsSampleAccumulator>& chunkBounds, bool includeChunkBounds) {
  size_t numExpected = 0;
  size_t numPendingHeaderRequests = 0;
  bool reportedDone = false;
  starving = false;

  greedilyClaimFiles();
  innerLoader.preRegisterResources(localWorkQueue);

  while (!isIdle() || !globalWork->allowedToExit()) {
    size_t newPendingReads = 0;
    if (!innerLoader.isQueueFull() || true) {
      if (auto job = tryGetWork(); job.has_value()) {
        std::shared_ptr<PointLoaderJob> toSubmit = *job;
        newPendingReads = innerLoader.submit(toSubmit.get());
        if (toSubmit->enqueued) {
          if (toSubmit->isHeaderRequest() || toSubmit->isChunkTableRequest()) {
            ++numPendingHeaderRequests;
          }
          jobsInProcess[toSubmit.get()] = toSubmit;
        } else {
          spdlog::warn("could not enqueue read request, trying again later");
          localWorkQueue.push_back(toSubmit);
        }
      } else {
        newPendingReads = innerLoader.submitRemaining();
      }
    }

    if (innerLoader.hasPendingReads()) {
      bool forceReaping = innerLoader.isQueueFull() || newPendingReads == 0;
      for (PointLoaderJob* job : innerLoader.reap(forceReaping ? 1 : 0)) {
        if (job->isHeaderRequest() || job->isChunkTableRequest()) {
          --numPendingHeaderRequests;
        }
        handleResult(*job, chunkPoints, chunkBounds, includeChunkBounds);
        jobsInProcess.erase(job);
      }
    }

    bool shareWork = (localWorkQueue.size() > innerLoader.queueDepth * 2) && globalWork->hasStarving();
    if (shareWork) {
      size_t shareWorkSize = localWorkQueue.size() - innerLoader.queueDepth * 2;
      std::vector<std::shared_ptr<PointLoaderJob>> jobsToShare;
      jobsToShare.insert(
          jobsToShare.begin(),
          std::make_move_iterator(localWorkQueue.begin()),
          std::make_move_iterator(localWorkQueue.begin() + static_cast<long long>(shareWorkSize)));
      localWorkQueue.erase(localWorkQueue.begin(), localWorkQueue.begin() + static_cast<long long>(shareWorkSize));
      globalWork->shareJobs(jobsToShare);
    }

    if (numPendingHeaderRequests == 0 && !reportedDone) {
      reportedDone = true;
      globalWork->reportNoPendingHeaderRequests();
    }
    if (!starving && jobsInProcess.empty() && localWorkQueue.empty() && newPendingReads == 0) {
      starving = true;
      globalWork->reportStarving();
    }
  }
  spdlog::debug("num chunk points {} of expected {}", chunkPoints.size(), numExpected);
}
}
