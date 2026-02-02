#pragma once

#include <utility>

namespace viam {

/// Simple RAII scope guard that runs a cleanup function on destruction.
/// Can be dismissed to prevent cleanup from running.
template <typename Func>
class ScopeGuard {
   public:
    explicit ScopeGuard(Func&& func) : func_(std::move(func)), active_(true) {}
    ~ScopeGuard() {
        if (active_) {
            func_();
        }
    }
    void dismiss() { active_ = false; }

    ScopeGuard(const ScopeGuard&) = delete;
    ScopeGuard& operator=(const ScopeGuard&) = delete;
    ScopeGuard(ScopeGuard&&) = delete;
    ScopeGuard& operator=(ScopeGuard&&) = delete;

   private:
    Func func_;
    bool active_;
};

/// Helper to create a ScopeGuard with type deduction
template <typename Func>
ScopeGuard<Func> make_scope_guard(Func&& func) {
    return ScopeGuard<Func>(std::forward<Func>(func));
}

}  // namespace viam
