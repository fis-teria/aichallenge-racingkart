#ifndef SIMPLE_PURE_PURSUIT__AW2_SHADOW_IPC_HPP_
#define SIMPLE_PURE_PURSUIT__AW2_SHADOW_IPC_HPP_

#include "simple_pure_pursuit/aw2_shadow_transport.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <sys/types.h>

namespace simple_pure_pursuit::aw2_shadow {

inline constexpr std::uint64_t kPublisherAddressSpaceLimitBytes =
    1024ULL * 1024ULL * 1024ULL;
inline constexpr std::uint64_t kPublisherOpenFileLimit = 128U;
inline constexpr int kSupervisorUnconfirmedCleanupExitCode = 7;
inline constexpr int kSupervisorForcedCleanupExitCode = 8;

struct PeerCredentials {
  pid_t pid{-1};
  uid_t uid{static_cast<uid_t>(-1)};
  gid_t gid{static_cast<gid_t>(-1)};
};

enum class ChildTerminationOutcome : std::uint8_t {
  kAlreadyExited,
  kGraceful,
  kTerminated,
  kKilled,
  kKillUnconfirmed,
  kError,
};

enum class BootstrapFailureStage : std::uint8_t {
  kNone,
  kSpawn,
  kHandshakeTimeout,
  kAccept,
  kAuthentication,
  kBootstrapThread,
};

struct BootstrapFailureDetails {
  BootstrapFailureStage stage{BootstrapFailureStage::kNone};
  pid_t spawned_supervisor_pid{-1};
  ChildTerminationOutcome termination_outcome{
      ChildTerminationOutcome::kAlreadyExited};
};

class DirectionalMemfdChannels {
public:
  static std::optional<DirectionalMemfdChannels>
  create(std::uint64_t ring_generation,
         std::uint64_t controller_instance_id) noexcept;

  DirectionalMemfdChannels() = default;
  ~DirectionalMemfdChannels();
  DirectionalMemfdChannels(const DirectionalMemfdChannels &) = delete;
  DirectionalMemfdChannels &
  operator=(const DirectionalMemfdChannels &) = delete;
  DirectionalMemfdChannels(DirectionalMemfdChannels &&other) noexcept;
  DirectionalMemfdChannels &
  operator=(DirectionalMemfdChannels &&other) noexcept;

  SharedDataRegion *data() noexcept { return data_mapping_; }
  SharedAckRegion *ack() noexcept { return ack_mapping_; }
  const SharedDataRegion *data() const noexcept { return data_mapping_; }
  const SharedAckRegion *ack() const noexcept { return ack_mapping_; }

  int producerDataFd() const noexcept { return producer_data_fd_; }
  int producerAckFd() const noexcept { return producer_ack_fd_; }
  int shadowDataFd() const noexcept { return shadow_data_fd_; }
  int shadowAckFd() const noexcept { return shadow_ack_fd_; }

  bool validate() const noexcept;
  void closeShadowAliases() noexcept;

private:
  int producer_data_fd_{-1};
  int producer_ack_fd_{-1};
  int shadow_data_fd_{-1};
  int shadow_ack_fd_{-1};
  SharedDataRegion *data_mapping_{nullptr};
  SharedAckRegion *ack_mapping_{nullptr};
};

class ProducerSession {
public:
  static std::optional<ProducerSession>
  start(const std::string &supervisor_executable, std::uint64_t ring_generation,
        std::uint64_t controller_instance_id,
        std::chrono::milliseconds handshake_timeout,
        BootstrapFailureDetails *failure_details = nullptr) noexcept;

  ProducerSession() = default;
  ~ProducerSession();
  ProducerSession(const ProducerSession &) = delete;
  ProducerSession &operator=(const ProducerSession &) = delete;
  ProducerSession(ProducerSession &&other) noexcept;
  ProducerSession &operator=(ProducerSession &&other) noexcept;

  void setProducerDigests(
      const std::array<std::uint8_t, 32U> &build_digest,
      const std::array<std::uint8_t, 32U> &config_digest) noexcept;
  bool tryCapture(const FixedSnapshot &snapshot) noexcept;
  bool valid() const noexcept;
  std::uint64_t consumedCount() const noexcept;
  pid_t supervisorPid() const noexcept { return supervisor_pid_; }

private:
  DirectionalMemfdChannels channels_;
  int control_socket_fd_{-1};
  pid_t supervisor_pid_{-1};
  std::string socket_directory_;
  std::string socket_path_;
};

class AsyncProducerSession {
public:
  static std::unique_ptr<AsyncProducerSession>
  start(const std::string &supervisor_executable, std::uint64_t ring_generation,
        std::uint64_t controller_instance_id,
        const std::array<std::uint8_t, 32U> &build_digest,
        const std::array<std::uint8_t, 32U> &config_digest,
        std::chrono::milliseconds handshake_timeout) noexcept;

  ~AsyncProducerSession();
  AsyncProducerSession(const AsyncProducerSession &) = delete;
  AsyncProducerSession &operator=(const AsyncProducerSession &) = delete;

  bool tryCapture(const FixedSnapshot &snapshot) noexcept;
  bool ready() const noexcept;

private:
  struct State;
  explicit AsyncProducerSession(State *state) noexcept : state_(state) {}
  State *state_{nullptr};
};

std::uint32_t activeAsyncBootstrapCount() noexcept;

struct ReceivedBootstrap {
  int control_socket_fd{-1};
  int data_fd{-1};
  int ack_fd{-1};
  pid_t producer_pid{-1};
  std::uint64_t nonce_high{0U};
  std::uint64_t nonce_low{0U};
};

std::optional<ReceivedBootstrap>
receiveBootstrap(const std::string &socket_path, pid_t expected_producer_pid,
                 std::uint64_t nonce_high, std::uint64_t nonce_low) noexcept;
void closeBootstrap(ReceivedBootstrap &bootstrap) noexcept;

std::optional<PeerCredentials> getPeerCredentials(int socket_fd) noexcept;
bool peerMatchesExpected(const PeerCredentials &credentials, pid_t expected_pid,
                         uid_t expected_uid, gid_t expected_gid) noexcept;

bool consumeOneInChild(int data_fd, int ack_fd,
                       std::uint64_t expected_sequence) noexcept;
bool applyPublisherParentDeathSignal(pid_t expected_supervisor_pid) noexcept;
bool applyPublisherResourceIsolation() noexcept;

ChildTerminationOutcome
terminateOneShotChild(pid_t child, std::chrono::milliseconds shutdown_deadline,
                      std::chrono::milliseconds terminate_grace) noexcept;
int supervisorExitCodeForChildTermination(
    ChildTerminationOutcome outcome) noexcept;

} // namespace simple_pure_pursuit::aw2_shadow

#endif // SIMPLE_PURE_PURSUIT__AW2_SHADOW_IPC_HPP_
