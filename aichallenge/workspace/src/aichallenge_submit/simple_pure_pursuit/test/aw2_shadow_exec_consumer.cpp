#include "simple_pure_pursuit/aw2_shadow_ipc.hpp"

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <sys/types.h>

namespace shadow = simple_pure_pursuit::aw2_shadow;

namespace {

bool parseUnsigned(const char *text, std::uint64_t &value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  char *end = nullptr;
  errno = 0;
  value = std::strtoull(text, &end, 10);
  return errno == 0 && end != text && *end == '\0';
}

} // namespace

int main(int argc, char **argv) {
  std::uint64_t producer_pid = 0U;
  std::uint64_t nonce_high = 0U;
  std::uint64_t nonce_low = 0U;
  if (argc != 5 || !parseUnsigned(argv[2], producer_pid) ||
      !parseUnsigned(argv[3], nonce_high) ||
      !parseUnsigned(argv[4], nonce_low) ||
      producer_pid >
          static_cast<std::uint64_t>(std::numeric_limits<pid_t>::max())) {
    return 2;
  }
  auto bootstrap = shadow::receiveBootstrap(
      argv[1], static_cast<pid_t>(producer_pid), nonce_high, nonce_low);
  if (!bootstrap.has_value()) {
    return 3;
  }
  const bool consumed =
      shadow::consumeOneInChild(bootstrap->data_fd, bootstrap->ack_fd, 1U);
  shadow::closeBootstrap(bootstrap.value());
  return consumed ? 0 : 4;
}
