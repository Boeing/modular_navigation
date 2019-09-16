import OpenGL
import OpenGL.GL
import OpenGL.GLU


def is_edge(a, b, bounds):
    """Returns true if a and b are adjacent within a single path."""
    span = find_bound_tuple(a, b, bounds)
    if span is not None:
        return is_adjacent(a, b, span)
    else:
        return False


def is_adjacent(a, b, span):
    ret = False
    lower = min(a, b)
    upper = max(a, b)
    diff = upper - lower
    if diff == 1:
        ret = True
    elif lower is span[0] and upper is span[1]:
        ret = True
    return ret


def find_bound_tuple(a, b, bounds):
    """If a and b are both included in a bounds tuple, return it.
    Otherwise return None.
    """

    def inside(num, spanish):
        return num >= spanish[0] and num <= spanish[1]

    for span in bounds:
        if inside(a, span) and inside(b, span):
            return span
    return None


def cross(a, b):
    return (a[0] * b[1]) - (a[1] * b[0])


def vec(start, end):
    return end[0] - start[0], end[1] - start[1]


class GlData(object):
    def __init__(self):
        self.tess_style = 0
        self.current_shape = list()
        self.triangles = list()


def triangulate(polygon):
    gl_data = GlData()

    #
    # Define several callback functions.
    #
    def cb_vert(v):
        gl_data.current_shape.append(v)

    def cb_begin(style):
        gl_data.tess_style = style

    def cb_end():
        if gl_data.tess_style == OpenGL.GL.GL_TRIANGLE_FAN:
            c = gl_data.current_shape.pop(0)
            p1 = gl_data.current_shape.pop(0)
            while gl_data.current_shape:
                p2 = gl_data.current_shape.pop(0)
                gl_data.triangles.append([c, p1, p2])
                p1 = p2
        elif gl_data.tess_style == OpenGL.GL.GL_TRIANGLE_STRIP:
            p1 = gl_data.current_shape.pop(0)
            p2 = gl_data.current_shape.pop(0)
            while gl_data.current_shape:
                p3 = gl_data.current_shape.pop(0)
                gl_data.triangles.append([p1, p2, p3])
                p1 = p2
                p2 = p3
        elif gl_data.tess_style == OpenGL.GL.GL_TRIANGLES:
            # each three points constitute a triangle, no sharing
            while gl_data.current_shape:
                p1 = gl_data.current_shape.pop(0)
                p2 = gl_data.current_shape.pop(0)
                p3 = gl_data.current_shape.pop(0)
                gl_data.triangles.append([p1, p2, p3])
        else:
            print "Unknown tessellation style:", gl_data.tess_style
        gl_data.tess_style = None
        gl_data.current_shape = []

    def cb_error(what):
        print "error:", what

    def cb_combine(c, v, weight):
        print "combine:", c, v, weight, "(this will probably cause problems)"
        return c[0], c[1], c[2]

    tess = OpenGL.GLU.gluNewTess()

    OpenGL.GLU.gluTessCallback(tess, OpenGL.GLU.GLU_TESS_VERTEX, cb_vert)
    OpenGL.GLU.gluTessCallback(tess, OpenGL.GLU.GLU_TESS_BEGIN, cb_begin)
    OpenGL.GLU.gluTessCallback(tess, OpenGL.GLU.GLU_TESS_END, cb_end)
    OpenGL.GLU.gluTessCallback(tess, OpenGL.GLU.GLU_TESS_ERROR, cb_error)
    OpenGL.GLU.gluTessCallback(tess, OpenGL.GLU.GLU_TESS_COMBINE, cb_combine)

    OpenGL.GLU.gluTessBeginPolygon(tess, None)
    OpenGL.GLU.gluTessBeginContour(tess)
    for i, pt in enumerate(polygon):
        OpenGL.GLU.gluTessVertex(tess, pt, i)
    OpenGL.GLU.gluTessEndContour(tess)
    OpenGL.GLU.gluTessEndPolygon(tess)

    return gl_data.triangles


if __name__ == '__main__':
    polygon = [(0, 0, 0), (1, 1, 0), (1, 9, 0), (0, 0, 0)]

    triangulate(polygon)


class Triangle(object):
    def __init__(self, tri, bounds, flattened_points):
        """Given three input indexes (e.g. 7, 3, 12) and a list of boundary
        tuples, create a triangle whose surface normal points the
        correct way and knows which edges are user-defined.
        Surface normal: the winding order of the points will on exit
        be defined such that it is in a right-handed coordinate
        system. Specifically: let vec1 run from p0 to p1, and vec2
        from p0 to p2. vec1 x vec2 is positive.
        Edges: Each edge of a triangle might be on the boundary of the
        shape, or it might be internal to the shape. An edge is True
        if it is on the boundary of the shape. We know it is on the
        boundary if the vertex indices are adjacent within a single
        path.
        Say we have an input paths list with indexes like this:
            [ [0, 1, 2, 3, 4], [5, 6, 7], [8, 9, 10] ]
        Use Shape's make_bound_tuples method to generate the list of
        tuples that looks like this: [ (0, 4), (5, 7), (8, 10) ], and
        pass that in as the 'bounds' parameter.

        Points 1 and 2 are adjacent (as are 2 and 1). Points 4 and 0
        are adjacent because lists are considered circular. Points 4
        and 5 are NOT adjacent because they come from different lists.
        With these path bounds and triangle verts at 6, 7, 9, the
        resulting triangle will have edges [True, False, False] since
        the first edge from 6 to 7 is a boundary, while the other two
        are not.
        """
        self.degenerate = False
        self.points = [None] * 3  # because that's easy to understand, right?
        self.points[0] = tri[0]
        self.points[1] = tri[1]
        self.points[2] = tri[2]
        self.edges = [None] * 3
        self.edges[0] = is_edge(tri[0], tri[1], bounds)
        self.edges[1] = is_edge(tri[1], tri[2], bounds)
        self.edges[2] = is_edge(tri[2], tri[0], bounds)
        # Ensure the winding order is correct. Cross product must be positive.
        # Swap things around if it is not.
        #
        # make vectors from 0 to 1 and 0 to 2
        v1 = vec(flattened_points[tri[0]], flattened_points[tri[1]])
        v2 = vec(flattened_points[tri[0]], flattened_points[tri[2]])
        c = cross(v1, v2)
        if abs(c) < 0.0001:
            self.degenerate = True
        elif c < 0:
            # swap points 0 and 2
            tmpPt = self.points[0]
            self.points[0] = self.points[2]
            self.points[2] = tmpPt
            # swap edges 0 and 1
            tmpEdge = self.edges[0]
            self.edges[0] = self.edges[1]
            self.edges[1] = tmpEdge

    def __str__(self):
        def tf(b):
            if b:
                return "t"
            else:
                return "f"

        return str(self.points[0]) + " " + str(self.points[1]) + " " + str(self.points[2]) + " " + tf(
            self.edges[0]) + " " + tf(self.edges[1]) + " " + tf(self.edges[2])


class Shape(object):

    def __init__(self):
        self.paths = []

    def print_paths(self):
        for path in self.paths:
            for pt in path:
                print str(pt[0]) + ", " + str(pt[1])

    def make_bound_tuples(self):
        """Returns a list of tuples. Each has the lower and upper inclusive bounds of a path.
        Example input: [ [0, 1, 2, 3, 4], [5, 6, 7], [8, 9, 10] ]
        Example output: [ (0, 4), (5, 7), (8, 10) ]
        """
        ret = []
        low = 0
        for path in self.paths:
            high = low + len(path) - 1
            ret.append((low, high))
            low = high + 1
        return ret

    def flattened_points(self):
        ret = []
        for sublist in self.paths:
            for item in sublist:
                ret.append(item)
        return ret
