#include "../src/runtime.hpp"
#include "../src/flow_roi.hpp"

#include <iostream>
#include <mutex>
#include <set>
#include <sys/syscall.h>
#include <unistd.h>
#include <time.h>

namespace {

FeatureRoi g_feature_roi{};
int g_max_features = 500;
double g_fb_shadow_max_px = 0.0;
bool g_obs_shadow_enabled = true;

// runtime.hpp уже подключён вне namespace.
// Повторное подключение не создаёт вложенный std.
#include "../src/flow_core.hpp"

} // namespace

int main() {
    std::cout << "BF FLOW CORE READY\n";
    return 0;
}
