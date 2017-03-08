// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef mavlink_utils_Utils_hpp
#define mavlink_utils_Utils_hpp

#include "StrictMode.hpp"
#define _USE_MATH_DEFINES
#include <cmath>
#include <chrono>
#include <thread>
#include <memory>
#include <string>
#include <cstdarg>
#include <cstring>
#include <array>
#include <sstream>
#include <fstream>
#include <vector>
#include <iomanip>
#include "type_utils.hpp"
#include <limits>

#ifndef _WIN32
#include <limits.h> // needed for CHAR_BIT used below
#endif

//Stubs for C++17 optional type
#if (defined __cplusplus) && (__cplusplus >= 201700L)
#include <optional>
#else
#include "optional.hpp"
#endif

#if (defined __cplusplus) && (__cplusplus >= 201700L)
using std::optional;
#else
using std::experimental::optional;
#endif

#ifndef M_PIf
#define M_PIf static_cast<float>(3.1415926535897932384626433832795028841972)
#endif

#ifndef M_PI
#define M_PI static_cast<double>(3.1415926535897932384626433832795028841972)
#endif

#ifndef M_PIl
#define M_PIl static_cast<long double>(3.1415926535897932384626433832795028841972)
#endif

#define EARTH_RADIUS (6378137.0f)

/*
    This file is collection of routines that can be included in ANY project just
    by dropping in common_utils.hpp. Therefore there should not be any dependency 
    in the code below other than STL. The code should be able to compilable on
    all major platforms.
*/


namespace mavlink_utils {

class Utils {
private:
    typedef std::chrono::system_clock system_clock;
    typedef std::chrono::steady_clock steady_clock;
    typedef std::string string;
    typedef std::stringstream stringstream;
	//this is not required for most compilers
	typedef unsigned int uint;
	template <typename T>
    using time_point = std::chrono::time_point<T>;    

public:
    static void enableImmediateConsoleFlush() {
        //disable buffering
        setbuf(stdout, NULL);
    }
	static double degreesToRadians(double degrees) {
		return static_cast<double>(degrees*(M_PIl / 180.0));
	}
	static float degreesToRadians(float degrees) {
		return static_cast<float>(degrees*(M_PI / 180.0f));
	}

    static void logMessage(const char* message, ...);

    static void logError(const char* message, ...);

    template <typename T>
    static int sign(T val) {
        return T(0) < val ? 1 : (T(0) > val ? -1 : 0);
    }   

    /// Limits absolute value whole preserving sign
    template <typename T> 
    static T limitAbsValue(T val, T min_value, T max_value) {
        T val_abs = std::abs(val);
        T val_limited = std::max(val_abs, min_value);
        val_limited = std::min(val_limited, max_value);
        return sign(val) * val_limited;
    }
    
    template<typename Range>
    static const string printRange(Range&& range, const string& delim = ", ",
        const string& prefix="(", const string& suffix=")")
    {
        return printRange(std::begin(range), std::end(range), delim, prefix, suffix);
    }
    template<typename Iterator>
    static const string printRange(Iterator start, Iterator last, const string& delim = ", ",
        const string& prefix="(", const string& suffix=")")
    {
        stringstream ss;
        ss << prefix;

        for (Iterator i = start; i != last; ++i)
        {
            if (i != start)
                ss << delim;
            ss << *i;
        }

        ss << suffix;
        return ss.str();         
    }

    static string stringf(const char* format, ...);

    static string getFileExtension(const string str);

    static string trim(const string& str, char ch);

	static std::vector<std::string> split(std::string s, const char* splitChars, int numSplitChars)
	{
		auto start = s.begin();
		std::vector<std::string> result;
		for (auto it = s.begin(); it != s.end(); it++)
		{
			bool split = false;
			for (int i = 0; i < numSplitChars; i++)
			{
				if (*it == splitChars[i]) {
					split = true;
					break;
				}
			}
			if (split)
			{
				if (start < it)
				{
					result.push_back(std::string(start, it));
				}
				start = it;
				start++;
			}
		}
		if (start < s.end())
		{
			result.push_back(std::string(start, s.end()));
		}
		return result;
	}

#ifdef _WIN32
	static string toLower(const std::string str)
	{
		int len = static_cast<int>(str.size());
		char* buf = new char[len + 1];
		str.copy(buf, len, 0);
		buf[len] = '\0';
		_strlwr_s(buf, len + 1);
		string lower = buf;
		delete buf;
		return lower;
	}
#else
	static std::string toLower(const string str)
	{
		int len = str.size();
		char* buf = new char[len + 1];
		str.copy(buf, len, 0);
		char* p = buf;
		for (int i = len; i > 0; i--)
		{
			*p = tolower(*p);
			p++;
		}
		*p = '\0';
		string lower = buf;
		delete buf;
		return lower;
	}
#endif

	//http://stackoverflow.com/a/28703383/207661
	template <typename R>
	static constexpr R bitmask(unsigned int const onecount)
	{
		//  return (onecount != 0)
		//      ? (static_cast<R>(-1) >> ((sizeof(R) * CHAR_BIT) - onecount))
		//      : 0;
		return static_cast<R>(-(onecount != 0))
			& (static_cast<R>(-1) >> ((sizeof(R) * CHAR_BIT) - onecount));
	}

    static inline int floorToInt(float x)
    {
        return static_cast<int> (std::floor(x));
    }

    static constexpr float maxFloat() {
        return std::numeric_limits<float>::max();
    }
    static constexpr uint maxUInt() {
        return std::numeric_limits<uint>::max();
    }    
    static constexpr float minFloat() {
        return std::numeric_limits<float>::max();
    }

    static constexpr float nanFloat() {
        return std::numeric_limits<float>::quiet_NaN();
    }

	static constexpr float DegreesToRadians(float degrees) {
		return static_cast<float>(M_PI * degrees / 180);
	}

	static void saveToFile(string file_name, const char* data, uint size)
	{
		std::ofstream file(file_name, std::ios::binary);
		file.write(data, size);
	}

    template<typename Container>
    static typename std::enable_if<type_utils::is_container<Container>::value, void>::type
    append(Container& to, const Container& from)
    {
        using std::begin;
        using std::end;
        to.insert(end(to), begin(from), end(from));
    }

	template<typename Container>
	static typename std::enable_if<type_utils::is_container<Container>::value, void>::type
		copy(const Container& from, Container& to)
	{
		using std::begin;
		using std::end;
		std::copy(begin(from), end(from), begin(to));
	}

	template<typename T>
	static void copy(const T* from, T* to, uint count)
	{
		std::copy(from, from + count, to);
	}

	static int to_integer(std::string s)
	{
		return atoi(s.c_str());
	}

    static const char* to_string(time_point<steady_clock> t)
    {
        time_t tt = system_clock::to_time_t(std::chrono::time_point_cast<system_clock::duration>(system_clock::now() + (t - steady_clock::now())));
        return ctime(&tt);
    }

    static time_point<system_clock> now()
    {
        return system_clock::now();
    }

    static string to_string(time_point<system_clock> time)
    {
		return to_string(time, "%Y-%m-%d-%H-%M-%S");
    }

	static string to_string(time_point<system_clock> time, const char* format)
	{
		time_t tt = system_clock::to_time_t(time);
		char str[1024];
		if (std::strftime(str, sizeof(str), format, std::localtime(&tt)))
			return string(str);
		else return string();
	}

    static string getEnv(const string& var)
    {
        char* ptr = std::getenv(var.c_str());
        return ptr ? ptr : "";
    }

};

} //namespace
#endif
