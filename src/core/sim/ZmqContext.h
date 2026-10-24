#pragma once

// Internal header: only include from .cpp files that already use cppzmq.

#include <zmq.hpp>

namespace vacuum {

// One process-wide ZeroMQ context (thread-safe); sockets are per thread.
zmq::context_t& sharedZmqContext();

}  // namespace vacuum
