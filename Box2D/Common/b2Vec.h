#ifndef B2VEC_H
#define B2VEC_H
#pragma once

#include <Box2D/Common/b2MathFunctions.h>
#include <Box2D/Common/b2Settings.h>
#include <array>
#include <utility>

namespace box2d
{

constexpr int b2VecX = 0;
constexpr int b2VecY = 1;
constexpr int b2VecZ = 2;

template<typename CONTAINER_TYPE, std::size_t N>
class b2Vec
{
    std::array<CONTAINER_TYPE, N> data;
public:

    b2Vec() = default;

    template< typename... Args, typename Enabled_ = typename std::enable_if< sizeof...(Args) == N, void >::type >
    b2Vec(Args... args)
    {
        data = {args...};
    }

    b2Vec(b2Vec<CONTAINER_TYPE, N>&& other) : data(std::move(other.data))
    {
    }

    b2Vec(const b2Vec<CONTAINER_TYPE, N>& other) : data(other.data)
    {
    }

    b2Vec(const std::initializer_list<CONTAINER_TYPE>& list)
    {
        std::size_t i{};
        for (auto item : list)
        {
            data[i++] = item;
        }
    }

    auto operator=(b2Vec<CONTAINER_TYPE, N>&& other)
    {
        data = std::move(other.data);
        return *this;
    }

    auto operator=(const b2Vec<CONTAINER_TYPE, N>& other)
    {
        data = other.data;
        return *this;
    }

    decltype(N) size() const { return N; }
    auto begin() { return data.begin(); }
    auto end() { return data.end(); }

    CONTAINER_TYPE& operator[](std::size_t pos)
    {
        return data[pos];
    }

    const CONTAINER_TYPE& operator[](std::size_t pos) const
    {
        return data[pos];
    }

    template <typename T>
    inline auto operator*(T&& t) const
    {
        b2Vec<CONTAINER_TYPE, N> ret;
        for (std::size_t i = 0; i < data.size(); ++i)
        {
            ret[i] = data[i] * t;
        }

        return ret;
    }

    template <typename T>
    inline auto& operator*=(T&& t)
    {
        for (auto& value : data)
        {
            value *= t;
        }

        return *this;
    }

    template <typename T>
    inline auto operator/(T&& t) const
    {
        b2Vec<CONTAINER_TYPE, N> ret;
        for (std::size_t i = 0; i < data.size(); ++i)
        {
            ret[i] = data[i] / t;
        }

        return ret;
    }


    template <typename T>
    inline auto& operator/=(T&& t)
    {
        for (auto& value : data)
        {
            value /= t;
        }

        return *this;
    }

    inline auto operator+(const b2Vec<CONTAINER_TYPE,N>& other) const
    {
        b2Vec<CONTAINER_TYPE,N> ret;
        for (std::size_t i = 0; i < data.size(); ++i)
        {
            ret[i] = data[i] + other[i];
        }

        return ret;
    }

    inline b2Vec<CONTAINER_TYPE,N> operator-(const b2Vec<CONTAINER_TYPE,N>& other) const
    {
        b2Vec<CONTAINER_TYPE,N> ret;
        for (std::size_t i = 0; i < data.size(); ++i)
        {
            ret[i] = data[i] - other[i];
        }

        return ret;
    }

    inline b2Vec<CONTAINER_TYPE,N>& operator+=(const b2Vec<CONTAINER_TYPE,N>& other)
    {
        for (std::size_t i = 0; i < data.size(); ++i)
        {
            data[i] += other[i];
        }

        return *this;
    }

    inline b2Vec<CONTAINER_TYPE,N>& operator-=(const b2Vec<CONTAINER_TYPE,N>& other)
    {
        for (std::size_t i = 0; i < data.size(); ++i)
        {
            data[i] -= other[i];
        }

        return *this;
    }

    inline auto operator==(const b2Vec<CONTAINER_TYPE,N>& other)
    {
        bool ret = std::equal(data.begin(), data.end(), other.data.begin(), other.data.end());
        return ret;
    }

    inline auto LengthSquared()
    {
        CONTAINER_TYPE ret{};
        for (auto value : data)
        {
            ret += value * value;
        }
        return ret;
    }

    inline auto Length()
    {
        auto ret = b2Sqrt(LengthSquared());
        return ret;
    }


    inline auto operator-() const
    {
        b2Vec<CONTAINER_TYPE, N> ret;
        for (std::size_t i = 0; i < data.size(); ++i)
        {
            ret[i] = -data[i];
        }

        return ret;
    }

    inline auto Negate() const
    {

        return -(*this);
    }

    /// Convert this vector into a unit vector. Returns the length.
    inline auto Normalize()
    {
        float length = Length();
        if (length < EPSILON)
        {
            return 0.0f;
        }
        float invLength = 1.0f / length;
        *this *= invLength;

        return length;
    }

    /// Does this vector contain finite coordinates?
    inline bool IsValid() const
    {
        bool ret = true;
        for (const auto& value : data)
        {
            ret = ret && b2IsValid(value);
        }
        return ret;
    }

    inline bool operator<=(const box2d::b2Vec<CONTAINER_TYPE, N>& other) const
    {
        bool ret = true;
        for (std::size_t i = 0; i < data.size(); ++i)
        {
            ret = ret && (data[i] <= other[i]);
        }
        return ret;
    }

    inline bool operator>=(const box2d::b2Vec<CONTAINER_TYPE, N>& other) const
    {
        bool ret = true;
        for (std::size_t i = 0; i < data.size(); ++i)
        {
            ret = ret && (data[i] >= other[i]);
        }
        return ret;
    }

    inline bool operator>(const box2d::b2Vec<CONTAINER_TYPE, N>& other) const
    {
        bool ret = true;
        for (std::size_t i = 0; i < data.size(); ++i)
        {
            ret = ret && (data[i] > other[i]);
        }
        return ret;
    }

    inline bool operator<(const box2d::b2Vec<CONTAINER_TYPE, N>& other) const
    {
        bool ret = true;
        for (std::size_t i = 0; i < data.size(); ++i)
        {
            ret = ret && (data[i] < other[i]);
        }
        return ret;
    }

    inline bool operator<=(float f) const
    {
        bool ret = true;
        for (const auto& value : data)
        {
            ret = ret && (value <= f);
        }
        return ret;
    }

    inline bool operator>=(float f) const
    {
        bool ret = true;
        for (const auto& value : data)
        {
            ret = ret && (value >= f);
        }
        return ret;
    }

    inline bool operator>(float f) const
    {
        bool ret = true;
        for (const auto& value : data)
        {
            ret = ret && (value > f);
        }
        return ret;
    }

    inline bool operator<(float f) const
    {
        bool ret = true;
        for (const auto& value : data)
        {
            ret = ret && (value < f);
        }
        return ret;
    }

};

/// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
inline b2Vec<float, 2> Skew(const b2Vec<float, 2>& vec)
{
    return b2Vec<float, 2>{{-vec[b2VecY], vec[b2VecX]}};
}

}


template <typename T, typename CONTAINER_TYPE, std::size_t N>
inline box2d::b2Vec<CONTAINER_TYPE, N> operator*(T&& t, const box2d::b2Vec<CONTAINER_TYPE, N>& vec)
{
    box2d::b2Vec<CONTAINER_TYPE, N> ret;
    for (std::size_t i = 0; i < vec.size(); ++i)
    {
        ret[i] = t * vec[i];
    }

    return ret;
}

template <typename T, typename CONTAINER_TYPE, std::size_t N>
inline auto operator/(T&& t, const box2d::b2Vec<CONTAINER_TYPE,N>& vec)
{
    box2d::b2Vec<CONTAINER_TYPE, N> ret;
    for (std::size_t i = 0; i < vec.size(); ++i)
    {
        ret[i] = t / vec[i];
    }

    return ret;
}

#endif
