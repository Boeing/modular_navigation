#ifndef GRIDMAP_TIME_DECAY_H
#define GRIDMAP_TIME_DECAY_H

#include <gridmap/data_source.h>

#include <atomic>
#include <thread>

#include <opencv2/imgproc.hpp>

namespace gridmap
{

class TimeDecay : public DataSource
{
  public:
    TimeDecay();
    virtual ~TimeDecay() override;

    virtual void onInitialize(const XmlRpc::XmlRpcValue& parameters) override;

    virtual void matchSize() override;

  private:
    void timeDecayThread(const double frequency, const double log_odds_decay);

    double frequency_;
    double log_odds_decay_;

    std::atomic<bool> running_;
    std::thread time_decay_thread_;
};
}

#endif
