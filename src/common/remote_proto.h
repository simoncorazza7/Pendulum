#ifndef PENDEL_COMMON_REMOTE_PROTO_H
#define PENDEL_COMMON_REMOTE_PROTO_H

#include <cstddef>
#include <cstdint>
#include <cstring>

// Stage 2b UDP protocol between the Pi (pendctl: balance.remote, remote-sim) and the PC (py/remote_ctrl.py).
// docs/remote.md. Little-endian, fixed sizes, packed byte by byte (no struct layout or host byte order assumed).
// Mirror: py/remote_ctrl.py (struct formats); tests/golden/remote_packets.txt holds bytes both sides must produce.
//
//   PC -> Pi  HELLO   'PHEL' u16 version, u16 0                                                          8 bytes
//   Pi -> PC  PARAMS  'PPRM' u16 version, u16 delay_ticks, u32 params_id, f64 ts, kp, ki, kd, a_max, v_max,
//                     v_taper, i_max, i_delay, theta_ref, vel_fc, cart_kx, cart_kv, cart_theta_max          124 bytes
//   Pi -> PC  SAMPLE  'PSMP' u16 version, u16 flags, u32 seq, u32 epoch, u32 params_id, i64 t_ns, i32 counts,
//                     i32 pos_usteps, f64 theta, omega, x, v, u8 mode, u8 phase, u16 delay_ticks, f64 i0  80 bytes
//   PC -> Pi  REPLY   'PCMD' u16 version, u16 flags, u32 seq, u32 epoch, u32 params_id, f64 a_cmd,
//                     u32 pc_proc_ns, u32 0                                                                36 bytes
//
// seq = the Pi's loop tick of the sample; a REPLY carries the seq (and epoch) of the sample it was computed from.
// epoch counts engagements: the PC resets its controller state when it changes, with the integrator starting at i0
// (rad*s; the momentum pre-load theta'/w0^2 of a swing-up hand-over, else 0). Version 2 added i0 (2026-11-03): a
// version-1 PC program is ignored, so the link never comes up instead of catching without the pre-load. Version 3
// added the Stage 3 outer-loop gains to PARAMS (the PC runs core::cascadeStep / control.cascade_step on x, v).
// params_id identifies the PARAMS (FNV-1a of their packed bytes); the Pi only uses replies computed with its
// current params.
namespace proto {

constexpr std::uint16_t kVersion = 3;
constexpr std::uint32_t kMagicHello = 0x4C454850;    // bytes 'P' 'H' 'E' 'L'
constexpr std::uint32_t kMagicParams = 0x4D525050;   // 'P' 'P' 'R' 'M'
constexpr std::uint32_t kMagicSample = 0x504D5350;   // 'P' 'S' 'M' 'P'
constexpr std::uint32_t kMagicReply = 0x444D4350;    // 'P' 'C' 'M' 'D'
constexpr std::size_t kHelloSize = 8, kParamsSize = 124, kSampleSize = 80, kReplySize = 36;

// SAMPLE flags
constexpr std::uint16_t kSampleController = 1;       // the supervisor runs the controller on this sample (ENGAGED)
constexpr std::uint16_t kSampleFinishing = 2;        // the Pi is ending the session
// REPLY flags
constexpr std::uint16_t kReplyParamsOk = 1;          // computed with the params named by params_id
constexpr std::uint16_t kReplyComputed = 2;          // the controller ran (sample flagged kSampleController)

struct Params {
    std::uint16_t delay_ticks = 0;
    std::uint32_t params_id = 0;
    double ts = 0, kp = 0, ki = 0, kd = 0, a_max = 0, v_max = 0, v_taper = 0, i_max = 0, i_delay = 0,
           theta_ref = 0, vel_fc = 0, cart_kx = 0, cart_kv = 0, cart_theta_max = 0;
};

struct Sample {
    std::uint16_t flags = 0;
    std::uint32_t seq = 0, epoch = 0, params_id = 0;
    std::int64_t t_ns = 0;
    std::int32_t counts = 0, pos_usteps = 0;
    double theta = 0, omega = 0, x = 0, v = 0;
    std::uint8_t mode = 0, phase = 0;
    std::uint16_t delay_ticks = 0;
    double i0 = 0;                  // rad*s, the controller integrator at the start of this epoch
};

struct Reply {
    std::uint16_t flags = 0;
    std::uint32_t seq = 0, epoch = 0, params_id = 0;
    double a_cmd = 0;
    std::uint32_t pc_proc_ns = 0;
};

class Writer {
public:
    explicit Writer(std::uint8_t* p) : p_(p) {}
    void u8(std::uint8_t v) { *p_++ = v; }
    void u16(std::uint16_t v) { put(v, 2); }
    void u32(std::uint32_t v) { put(v, 4); }
    void i32(std::int32_t v) { put(static_cast<std::uint32_t>(v), 4); }
    void i64(std::int64_t v) { put(static_cast<std::uint64_t>(v), 8); }
    void f64(double v) {
        std::uint64_t b;
        std::memcpy(&b, &v, 8);
        put(b, 8);
    }

private:
    void put(std::uint64_t v, int n) {
        for (int i = 0; i < n; ++i) *p_++ = static_cast<std::uint8_t>(v >> (8 * i));
    }
    std::uint8_t* p_;
};

class Reader {
public:
    explicit Reader(const std::uint8_t* p) : p_(p) {}
    std::uint8_t u8() { return *p_++; }
    std::uint16_t u16() { return static_cast<std::uint16_t>(get(2)); }
    std::uint32_t u32() { return static_cast<std::uint32_t>(get(4)); }
    std::int32_t i32() { return static_cast<std::int32_t>(static_cast<std::uint32_t>(get(4))); }
    std::int64_t i64() { return static_cast<std::int64_t>(get(8)); }
    double f64() {
        const std::uint64_t b = get(8);
        double v;
        std::memcpy(&v, &b, 8);
        return v;
    }

private:
    std::uint64_t get(int n) {
        std::uint64_t v = 0;
        for (int i = 0; i < n; ++i) v |= static_cast<std::uint64_t>(*p_++) << (8 * i);
        return v;
    }
    const std::uint8_t* p_;
};

inline std::size_t packHello(std::uint8_t* out) {
    Writer w(out);
    w.u32(kMagicHello);
    w.u16(kVersion);
    w.u16(0);
    return kHelloSize;
}

inline std::size_t packParams(const Params& p, std::uint8_t* out) {
    Writer w(out);
    w.u32(kMagicParams);
    w.u16(kVersion);
    w.u16(p.delay_ticks);
    w.u32(p.params_id);
    for (double d : {p.ts, p.kp, p.ki, p.kd, p.a_max, p.v_max, p.v_taper, p.i_max, p.i_delay, p.theta_ref, p.vel_fc,
                     p.cart_kx, p.cart_kv, p.cart_theta_max})
        w.f64(d);
    return kParamsSize;
}

inline std::size_t packSample(const Sample& s, std::uint8_t* out) {
    Writer w(out);
    w.u32(kMagicSample);
    w.u16(kVersion);
    w.u16(s.flags);
    w.u32(s.seq);
    w.u32(s.epoch);
    w.u32(s.params_id);
    w.i64(s.t_ns);
    w.i32(s.counts);
    w.i32(s.pos_usteps);
    w.f64(s.theta);
    w.f64(s.omega);
    w.f64(s.x);
    w.f64(s.v);
    w.u8(s.mode);
    w.u8(s.phase);
    w.u16(s.delay_ticks);
    w.f64(s.i0);
    return kSampleSize;
}

inline std::size_t packReply(const Reply& r, std::uint8_t* out) {
    Writer w(out);
    w.u32(kMagicReply);
    w.u16(kVersion);
    w.u16(r.flags);
    w.u32(r.seq);
    w.u32(r.epoch);
    w.u32(r.params_id);
    w.f64(r.a_cmd);
    w.u32(r.pc_proc_ns);
    w.u32(0);
    return kReplySize;
}

// The type of a received datagram: its magic, if the size and version match (else 0).
inline std::uint32_t kindOf(const std::uint8_t* d, std::size_t n) {
    if (n < 8) return 0;
    Reader r(d);
    const std::uint32_t m = r.u32();
    if (r.u16() != kVersion) return 0;
    if ((m == kMagicHello && n == kHelloSize) || (m == kMagicParams && n == kParamsSize) ||
        (m == kMagicSample && n == kSampleSize) || (m == kMagicReply && n == kReplySize))
        return m;
    return 0;
}

inline bool unpackParams(const std::uint8_t* d, std::size_t n, Params* p) {
    if (kindOf(d, n) != kMagicParams) return false;
    Reader r(d + 6);
    p->delay_ticks = r.u16();
    p->params_id = r.u32();
    double* f[] = {&p->ts, &p->kp, &p->ki, &p->kd, &p->a_max, &p->v_max, &p->v_taper, &p->i_max, &p->i_delay,
                   &p->theta_ref, &p->vel_fc, &p->cart_kx, &p->cart_kv, &p->cart_theta_max};
    for (double* x : f) *x = r.f64();
    return true;
}

inline bool unpackSample(const std::uint8_t* d, std::size_t n, Sample* s) {
    if (kindOf(d, n) != kMagicSample) return false;
    Reader r(d + 6);
    s->flags = r.u16();
    s->seq = r.u32();
    s->epoch = r.u32();
    s->params_id = r.u32();
    s->t_ns = r.i64();
    s->counts = r.i32();
    s->pos_usteps = r.i32();
    s->theta = r.f64();
    s->omega = r.f64();
    s->x = r.f64();
    s->v = r.f64();
    s->mode = r.u8();
    s->phase = r.u8();
    s->delay_ticks = r.u16();
    s->i0 = r.f64();
    return true;
}

inline bool unpackReply(const std::uint8_t* d, std::size_t n, Reply* p) {
    if (kindOf(d, n) != kMagicReply) return false;
    Reader r(d + 6);
    p->flags = r.u16();
    p->seq = r.u32();
    p->epoch = r.u32();
    p->params_id = r.u32();
    p->a_cmd = r.f64();
    p->pc_proc_ns = r.u32();
    return true;
}

// FNV-1a over the packed PARAMS with params_id = 0: the id the Pi puts in PARAMS, SAMPLE and expects in REPLY.
inline std::uint32_t paramsId(Params p) {
    p.params_id = 0;
    std::uint8_t b[kParamsSize];
    packParams(p, b);
    std::uint32_t h = 2166136261u;
    for (std::uint8_t c : b) h = (h ^ c) * 16777619u;
    return h == 0 ? 1 : h;
}

}

#endif
