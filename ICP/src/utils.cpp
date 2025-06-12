//
// Created by lherzberger on 12.06.24.
//

#include "utils.h"

namespace icp {
size_t maxOpenFileHandles() {
  // todo: return the actual maximum number of open file handles supported by the od
  return std::numeric_limits<size_t>::max();
}
}