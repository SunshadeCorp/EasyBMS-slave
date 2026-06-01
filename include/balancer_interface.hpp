#pragma once

#include <vector>

class IBalancer {
   public:
    virtual std::vector<bool> balance(const std::vector<float>& voltages) = 0;
};