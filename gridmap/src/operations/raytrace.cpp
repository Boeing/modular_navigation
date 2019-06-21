#include <gridmap/operations/raytrace.h>

namespace gridmap
{

std::vector<Eigen::Array2i> drawLine(int x, int y, int x2, int y2)
{
    // THE EXTREMELY FAST LINE ALGORITHM Variation E (Addition Fixed Point PreCalc)

    std::vector<Eigen::Array2i> line;
    bool yLonger = false;
    int shortLen = y2 - y;
    int longLen = x2 - x;
    if (abs(shortLen) > abs(longLen))
    {
        int swap = shortLen;
        shortLen = longLen;
        longLen = swap;
        yLonger = true;
    }
    int decInc;
    if (longLen == 0)
        decInc = 0;
    else
        decInc = (shortLen << 16) / longLen;

    if (yLonger)
    {
        if (longLen > 0)
        {
            longLen += y;
            for (int j = 0x8000 + (x << 16); y <= longLen; ++y)
            {
                line.push_back({j >> 16, y});
                j += decInc;
            }
            return line;
        }
        longLen += y;
        for (int j = 0x8000 + (x << 16); y >= longLen; --y)
        {
            line.push_back({j >> 16, y});
            j -= decInc;
        }
        return line;
    }

    if (longLen > 0)
    {
        longLen += x;
        for (int j = 0x8000 + (y << 16); x <= longLen; ++x)
        {
            line.push_back({x, j >> 16});
            j += decInc;
        }
        return line;
    }
    longLen += x;
    for (int j = 0x8000 + (y << 16); x >= longLen; --x)
    {
        line.push_back({x, j >> 16});
        j -= decInc;
    }
    return line;
}
}
