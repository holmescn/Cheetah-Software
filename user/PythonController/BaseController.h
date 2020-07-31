#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H
#include <memory>

class ProxyController;
class LegProxy;
class StateProxy;

class BaseController {
    ProxyController *_proxy;
public:
    BaseController() = default;
    ~BaseController() = default;

    virtual void initialize() = 0;
    virtual void run() = 0;

    inline void SetProxy(ProxyController *proxy) { _proxy = proxy; }
    std::unique_ptr<LegProxy> GetLeg(size_t leg) const;
    std::unique_ptr<StateProxy> GetState() const;
};

class PyBaseController : public BaseController {
public:
    void initialize() override;
    void run() override;
};

#endif // BASE_CONTROLLER_H