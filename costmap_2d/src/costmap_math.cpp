/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <costmap_2d/costmap_math.h>

double distanceToLine(const double pX, const double pY, const double x0, const double y0, const double x1,
                      const double y1)
{
    const double A = pX - x0;
    const double B = pY - y0;
    const double C = x1 - x0;
    const double D = y1 - y0;

    const double dot = A * C + B * D;
    const double len_sq = C * C + D * D;
    const double param = dot / len_sq;

    double xx;
    double yy;

    if (param < 0)
    {
        xx = x0;
        yy = y0;
    }
    else if (param > 1)
    {
        xx = x1;
        yy = y1;
    }
    else
    {
        xx = x0 + param * C;
        yy = y0 + param * D;
    }

    return distance(pX, pY, xx, yy);
}

bool intersects(const std::vector<geometry_msgs::Point>& polygon, const float testx, const float testy)
{
    bool c = false;
    int i;
    int j;
    const int nvert = polygon.size();
    for (i = 0, j = nvert - 1; i < nvert; j = i++)
    {
        float yi = polygon[i].y;
        float yj = polygon[j].y;
        float xi = polygon[i].x;
        float xj = polygon[j].x;

        if (((yi > testy) != (yj > testy)) && (testx < (xj - xi) * (testy - yi) / (yj - yi) + xi))
            c = !c;
    }
    return c;
}

bool intersects_helper(const std::vector<geometry_msgs::Point>& polygon1,
                       const std::vector<geometry_msgs::Point>& polygon2)
{
    for (unsigned int i = 0; i < polygon1.size(); i++)
        if (intersects(polygon2, polygon1[i].x, polygon1[i].y))
            return true;
    return false;
}

bool intersects(const std::vector<geometry_msgs::Point>& polygon1, const std::vector<geometry_msgs::Point>& polygon2)
{
    return intersects_helper(polygon1, polygon2) || intersects_helper(polygon2, polygon1);
}
