#pragma once

#include <vector>
#include <sstream>
#include <limits>
#include <cmath>

template <typename T>
struct vec3
{
    T x;
    T y;
    T z;

    friend vec3 operator*(vec3 lhs, vec3 rhs)
    {
        return {lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z};
    }

    friend vec3 operator+(vec3 lhs, vec3 rhs)
    {
        return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
    }

    friend vec3 operator-(vec3 lhs, vec3 rhs)
    {
        return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
    }

    friend vec3 operator*(vec3 lhs, T rhs)
    {
        return {lhs.x * rhs, lhs.y * rhs, lhs.z * rhs};
    }

    friend vec3 operator-(vec3 lhs, T rhs)
    {
        return {lhs.x - rhs, lhs.y - rhs, lhs.z - rhs};
    }

    friend vec3 operator/(vec3 lhs, T rhs)
    {
        return {lhs.x / rhs, lhs.y / rhs, lhs.z / rhs};
    }
};

template <typename T>
vec3<T> parse_vec3(const std::string &line)
{
    std::istringstream iss(line);

    std::vector<T> values;
    T value;

    while (iss >> value)
    {
        char c;
        if (iss.get(c) && !std::isspace(c))
        {
            throw std::invalid_argument("Invalid character detected in vec3 values");
        }

        values.push_back(value);
    }
    if (values.size() != 3)
    {
        throw std::invalid_argument("Invalid number of values for vec3");
    }

    vec3<T> point;
    point.x = values[0];
    point.y = values[1];
    point.z = values[2];

    return point;
}

template <typename T>
T dot(vec3<T> a, vec3<T> b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

template <typename T>
vec3<T> cross(const vec3<T> &a, const vec3<T> &b)
{
    vec3<T> result;
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;
    return result;
}

template <typename T>
T len_squared(vec3<T> vec)
{
    return dot(vec, vec);
}

template <typename T>
T len(vec3<T> vec)
{
    return std::sqrt(len_squared(vec));
}

template <typename T>
vec3<T> normalize(vec3<T> vector)
{
    float normal = len(vector);

    vector.x /= normal;
    vector.y /= normal;
    vector.z /= normal;

    return vector;
}
