#pragma once

#include <vector>
#include <cassert>
#include <algorithm>

class KinectFrame
{
private:
    size_t _width, _height;
    std::vector<std::vector<int>> _array;

    int& operator()(unsigned int x, unsigned int y)
    {
        return _array[x][y];
    }

public:
    KinectFrame() = delete;
    KinectFrame(size_t width, size_t height)
    : _width(width), _height(height)
    {
        _array.assign(_width, std::vector<int>(_height, -1));
    }

    size_t width()  const { return _width; }
    size_t height() const { return _height; }

    void update(unsigned int x, unsigned int y, int v)
    {
        if (x >= width() || y >= height()) return;
        if (v < 0) return;
        if ((*this)(x, y) == -1) (*this)(x, y) = v;
        else                     (*this)(x, y) = std::min((*this)(x, y), v);
    }

    int get(unsigned int x, unsigned int y) const
    {
        assert(x < width() && y < height() && "bad frame coordinates");
        return _array[x][y];
    }
};