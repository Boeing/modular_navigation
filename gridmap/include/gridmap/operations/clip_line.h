#ifndef GRIDMAP_CLIP_LINE_H
#define GRIDMAP_CLIP_LINE_H

namespace gridmap {

void cohenSutherlandLineClipEnd(const int x0, const int y0, int &x1, int &y1,
                                const int max_x, const int max_y);
}

#endif
