#include "ekf/state.h"

using namespace Eigen;
using namespace xform;
using namespace quat;
using namespace std;

namespace roscopter
{
namespace ekf
{

ErrorState::ErrorState() :
    x(arr.data()+DX),
    p(arr.data()+DP),
    q(arr.data()+DQ),
    v(arr.data()+DV),
    ba(arr.data()+DBA),
    bg(arr.data()+DBG),
    bb(*(arr.data()+DBB)),
#ifdef RELATIVE
    ref(*(arr.data()+DREF)),
    qREL(arr.data()+DQREL),
    sv(arr.data()+DSV)
#else
    ref(*(arr.data()+DREF))
#endif
{
    arr.setConstant(NAN);
}

ErrorState::ErrorState(const ErrorState& other) :
    ErrorState()
{
    arr = other.arr;
}

ErrorState& ErrorState::operator=(const ErrorState& obj)
{
    arr = obj.arr;
    return *this;
}

ErrorState ErrorState::operator* (const double& s) const
{
    ErrorState out;
    out.arr = s * arr;
    return out;
}

ErrorState ErrorState::operator/ (const double& s) const
{
    ErrorState out;
    out.arr = arr/s;
    return out;
}

ErrorState& ErrorState::operator*= (const double& s)
{
    arr = s * arr;
    return *this;
}

ErrorState ErrorState::operator+ (const ErrorState& obj) const
{
    ErrorState out;
    out.arr = obj.arr + arr;
    return out;
}

ErrorState ErrorState::operator+ (const Matrix<double, SIZE, 1>& v) const
{
    ErrorState out;
    out.arr = v + arr;
    return out;
}

ErrorState& ErrorState::operator+= (const Matrix<double, SIZE, 1>& v)
{
    arr += v;
    return *this;
}

ErrorState& ErrorState::operator+= (const ErrorState& obj)
{
    arr = obj.arr + arr;
    return *this;
}
ErrorState ErrorState::operator- (const ErrorState& obj) const
{
    ErrorState out;
    out.arr = arr - obj.arr;
    return out;
}

ErrorState ErrorState::operator- (const Matrix<double, SIZE, 1>& v) const
{
    ErrorState out;
    out.arr = arr - v;
    return out;
}

ErrorState& ErrorState::operator-= (const Matrix<double, SIZE, 1>& v)
{
    arr -= v;
    return *this;
}

ErrorState& ErrorState::operator-= (const ErrorState& obj)
{
    arr = arr - obj.arr;
    return *this;
}

State::State() :
    t(*(arr.data()+T)),
    x(arr.data()+X),
    p(arr.data()+P),
    q(arr.data()+Q),
    v(arr.data()+V),
    ba(arr.data()+BA),
    bg(arr.data()+BG),
    bb(*(arr.data()+BB)),
    ref(*(arr.data()+REF)),
#ifdef RELATIVE
    qREL(arr.data()+QREL),
    sv(arr.data()+SV),
#endif
    imu(arr.data()+1+NX),
    a(arr.data()+A),
    w(arr.data()+W)
{
//#ifndef NDEBUG
    // to help with tracking down uninitialized memory, in debug mode fill with nans
    arr.setConstant(NAN);
//#endif
}

State::State(const State &other) :
    State()
{
    arr = other.arr;
}

State& State::operator= (const State& other)
{
    arr = other.arr;
    return *this;
}

State State::operator+(const ErrorState& dx) const
{
    State xp;
    xp.p = p + dx.p;
    xp.q = q + dx.q;
    xp.v = v + dx.v;
    xp.ba = ba + dx.ba;
    xp.bg = bg + dx.bg;
    xp.bb = bb + dx.bb;
    xp.ref = ref + dx.ref;
#ifdef RELATIVE
    xp.qREL = qREL + dx.qREL;
    xp.sv = sv + dx.sv;
#endif
    return xp;
}

State State::operator+(const Matrix<double, ErrorState::SIZE, 1>& dx) const
{
    State xp;
    xp.p = p + dx.segment<3>(ErrorState::DP);
    xp.q = q + dx.segment<3>(ErrorState::DQ);
    xp.v = v + dx.segment<3>(ErrorState::DV);
    xp.ba = ba + dx.segment<3>(ErrorState::DBA);
    xp.bg = bg + dx.segment<3>(ErrorState::DBG);
    xp.bb = bb + dx(ErrorState::DBB);
    xp.ref = ref + dx(ErrorState::DREF);
#ifdef RELATIVE
    xp.qREL = qREL + dx.segment<3>(ErrorState::DQREL);
    xp.sv = sv + dx.segment<3>(ErrorState::DSV);
#endif
    return xp;
}

State& State::operator+=(const VectorXd& dx)
{
    p += dx.segment<3>(ErrorState::DP);
    q += dx.segment<3>(ErrorState::DQ);
    v += dx.segment<3>(ErrorState::DV);
    ba += dx.segment<3>(ErrorState::DBA);
    bg += dx.segment<3>(ErrorState::DBG);
    bb += dx(ErrorState::DBB);
    ref += dx(ErrorState::DREF);
#ifdef RELATIVE
    qREL += dx.segment<3>(ErrorState::DQREL);
    sv += dx.segment<3>(ErrorState::DSV);
#endif

    return *this;
}

State& State::operator+=(const ErrorState& dx)
{
    p = p + dx.p;
    q = q + dx.q;
    v = v + dx.v;
    ba = ba + dx.ba;
    bg = bg + dx.bg;
    bb = bb + dx.bb;
    ref = ref + dx.ref;
#ifdef RELATIVE
    qREL = qREL + dx.qREL;
    sv = sv + dx.sv;
#endif

    return *this;
}

ErrorState State:: operator-(const State& dx) const
{
    ErrorState del;
    del.p = p - dx.p;
    del.q = q - dx.q;
    del.v = v - dx.v;
    del.ba = ba - dx.ba;
    del.bg = bg - dx.bg;
    del.bb = bb - dx.bb;
    del.ref = ref - dx.ref;
#ifdef RELATIVE
    del.qREL = qREL - dx.qREL;
    del.sv = sv - dx.sv;
#endif

    return del;
}

StateBuf::StateBuf(int _size) :
    buf(_size),
    size(_size),
    head(0),
    tail(0)
{
//#ifndef NDEBUG
    // to help with tracking down uninitialized memory, in debug mode fill with nans
    for (int i = 0; i < buf.size(); i++)
    {
        buf[i].x.arr.setConstant(NAN);
        buf[i].P.setConstant(NAN);
    }
//#endif
}

State& StateBuf::x()
{
    return buf[head].x;
}

const State& StateBuf::x() const
{
    return buf[head].x;
}

dxMat& StateBuf::P()
{
    return buf[head].P;
}

const dxMat& StateBuf::P() const
{
    return buf[head].P;
}

StateBuf::Snapshot& StateBuf::next()
{
    return buf[(head + 1) % size];
}

void StateBuf::advance()
{
    head = (head+1) % size;
    if (head == tail)
        tail = (tail + 1) % size;
}

bool StateBuf::rewind(double t)
{
    int tmp = head;
    while (buf[tmp].x.t > t)
    {
        tmp = (tmp + size - 1) % size;
        if (tmp == head)
            return false;
    }
    head = tmp;
    tail = (head + 1) % size;
    return true;
}

StateBuf::Snapshot& StateBuf::begin()
{
    return buf[tail];
}
}
}
