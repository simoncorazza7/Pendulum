#ifndef PENDEL_RUNTIME_REMOTE_H
#define PENDEL_RUNTIME_REMOTE_H

#include <cstdint>

// Stage 2b command source (docs/remote.md): the controller runs on a PC that answers every sample the Pi
// streams. BalanceRuntime applies, at the tick with sequence number seq(), the PC's command computed from the
// sample of tick seq() - remote_delay (constant delay). Portable interface; the UDP implementation is
// app::RemoteLink (Pi only), tests use a fake.
namespace rt {

class RemoteSource {
public:
    virtual ~RemoteSource() = default;
    virtual std::uint32_t seq() const = 0;                          // sequence number of this tick's sample
    virtual bool command(std::uint32_t seq, double* a) const = 0;   // the PC's command from sample `seq`, if here
    virtual bool linkUp() const = 0;   // replies with the current params arrived within the link timeout
};

}

#endif
