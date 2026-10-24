#include "core/common/Types.h"

namespace vacuum {

const char* toString(GotoStatus status) {
  switch (status) {
    case GotoStatus::None: return "none";
    case GotoStatus::Running: return "running";
    case GotoStatus::Reached: return "reached";
    case GotoStatus::Bumped: return "bumped";
    case GotoStatus::Cancelled: return "cancelled";
    case GotoStatus::Preempted: return "preempted";
    case GotoStatus::Unknown: break;
  }
  return "unknown";
}

GotoStatus gotoStatusFromString(const std::string& text) {
  if (text == "running") return GotoStatus::Running;
  if (text == "reached") return GotoStatus::Reached;
  if (text == "bumped") return GotoStatus::Bumped;
  if (text == "cancelled") return GotoStatus::Cancelled;
  if (text == "preempted") return GotoStatus::Preempted;
  return GotoStatus::Unknown;
}

}  // namespace vacuum
