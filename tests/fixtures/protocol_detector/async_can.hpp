// Offline receive-lifetime fixture. No sockets, ROS or hardware access.
#pragma once
#include <linux/can.h>
#include <functional>
#include <memory>
#include <string>
namespace westonrobot {
class AsyncCAN : public std::enable_shared_from_this<AsyncCAN> {
 public:
  explicit AsyncCAN(std::string) {}
  static std::shared_ptr<AsyncCAN>& reader() {
    static std::shared_ptr<AsyncCAN> value;
    return value;
  }
  void SetReceiveCallback(std::function<void(can_frame*)> callback) {
    callback_ = callback;
  }
  bool Open() {
    // A pending AsyncCAN read retains shared_from_this, independently of the
    // detector's shared_ptr, just like the real asynchronous receive path.
    reader() = shared_from_this();
    open_ = true;
    return true;
  }
  void Close() { open_ = false; }
  bool IsOpened() const { return open_; }
  void Deliver() {
    if (!open_) return;
    can_frame frame{};
    frame.can_id = 0x221;
    callback_(&frame);
  }
 private:
  bool open_ = false;
  std::function<void(can_frame*)> callback_;
};
}
