#include <ugv_sdk/utilities/protocol_detector.hpp>

int main() {
  using namespace westonrobot;
  {
    ProtocolDetector detector;
    if (!detector.Connect("offline_fixture")) return 2;
    AsyncCAN::reader()->Deliver();
  }
  // Before the fix this calls ParseCANFrame on an expired stack detector.
  // After the fix the receiver has stopped, so nothing calls the old pointer.
  AsyncCAN::reader()->Deliver();
  bool still_open = AsyncCAN::reader()->IsOpened();
  AsyncCAN::reader().reset();
  return still_open ? 3 : 0;
}
