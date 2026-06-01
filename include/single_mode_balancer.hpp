#pragma once

#include <balancer_interface.hpp>
#include <memory>
#include <vector>

#include "battery_monitor.hpp"

class SingleModeBalancer : public IBalancer {
   public:
    SingleModeBalancer(long balance_time_ms, long relax_time_ms);
    std::vector<bool> balance(const std::vector<float>& voltages) override;

   private:
    enum class BalancerState {
        Idle,
        Balancing,
        Relaxing,
    };

    long _balance_time_ms;
    long _relax_time_ms;

    long _balance_start_timestamp;
    long _relax_start_timestamp;

    float _cut_off_voltage;

    BalancerState _balancer_state;
    std::vector<bool> _balance_bits;

    void reset_balance_bits();
    void select_cells_to_balance(const std::vector<float>& voltages);
    float min_voltage(const std::vector<float>& voltages) const;
};