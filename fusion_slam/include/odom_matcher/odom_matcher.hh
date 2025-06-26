#pragma once

namespace slam {
class OdomMatcher {
   public:
    void Align();
    virtual ~OdomMatcher();
};
}  // namespace slam