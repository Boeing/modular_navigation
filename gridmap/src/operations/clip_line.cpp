#include <gridmap/operations/clip_line.h>

namespace gridmap
{

namespace
{

static const int INSIDE = 0;
static const int LEFT = 1;
static const int RIGHT = 2;
static const int BOTTOM = 4;
static const int TOP = 8;

int computeOutCode(const int x, const int y, const int max_x, const int max_y)
{
    int code = INSIDE;
    if (x < 0)
        code |= LEFT;
    else if (x > max_x)
        code |= RIGHT;
    if (y < 0)
        code |= BOTTOM;
    else if (y > max_y)
        code |= TOP;
    return code;
}
}  // namespace

void cohenSutherlandLineClipEnd(const int x0, const int y0, int& x1, int& y1, const int max_x, const int max_y)
{
    int outcode = computeOutCode(x1, y1, max_x, max_y);

    while (true)
    {
        if (!outcode)
        {
            return;
        }
        else
        {
            // Now find the intersection point;
            // use formulas:
            //   slope = (y1 - y0) / (x1 - x0)
            //   x = x0 + (1 / slope) * (ym - y0), where ym is ymin or ymax
            //   y = y0 + slope * (xm - x0), where xm is xmin or xmax
            // No need to worry about divide-by-zero because, in each case, the
            // outcode bit being tested guarantees the denominator is non-zero
            if (outcode & TOP)
            {
                // point is above the clip window
                x1 = x0 + (x1 - x0) * (max_y - y0) / (y1 - y0);
                y1 = max_y;
            }
            else if (outcode & BOTTOM)
            {
                // point is below the clip window
                x1 = x0 + (x1 - x0) * (0 - y0) / (y1 - y0);
                y1 = 0;
            }
            else if (outcode & RIGHT)
            {
                // point is to the right of clip window
                y1 = y0 + (y1 - y0) * (max_x - x0) / (x1 - x0);
                x1 = max_x;
            }
            else if (outcode & LEFT)
            {
                // point is to the left of clip window
                y1 = y0 + (y1 - y0) * (0 - x0) / (x1 - x0);
                x1 = 0;
            }

            outcode = computeOutCode(x1, y1, max_x, max_y);
        }
    }
}
}  // namespace gridmap
