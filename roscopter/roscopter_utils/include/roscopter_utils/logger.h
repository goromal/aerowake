#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <fstream>
#include <thread>
#include <unistd.h>
#include <iostream>
#include <Eigen/Dense>
#include <utility>

#define LOGGER_BUFFER_SIZE 1024*1024

namespace roscopter
{

template <typename Arg, typename... Args>
void doPrint(std::ostream& out, Arg&& arg, Args&&... args)
{
    out << std::forward<Arg>(arg);
    using expander = int[];
    (void)expander{0, (void(out << ',' << std::forward<Args>(args)), 0)...};
}

class Logger
{
public:
    Logger() {}

    Logger(const std::string filename)
    {
        open(filename);
    }

    void open(const std::string& filename)
    {
        file_.open(filename);
    }

    ~Logger()
    {
        file_.close();
    }
    template <typename... T>
    void log(T... data)
    {
        int dummy[sizeof...(data)] = { (file_.write((char*)&data, sizeof(T)), 1)... };
    }

    template <typename... T>
    void logVectors(T... data)
    {
        int dummy[sizeof...(data)] = { (file_.write((char*)data.data(), sizeof(typename T::Scalar)*data.rows()*data.cols()), 1)... };
//        doPrint(std::cout, data...);
    }

    std::ofstream file_;
};

}


