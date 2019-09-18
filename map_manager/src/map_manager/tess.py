import OpenGL
import OpenGL.GL
import OpenGL.GLU


class GlData(object):
    def __init__(self):
        self.tess_style = 0
        self.current_shape = list()
        self.triangles = list()


def triangulate(polygon):
    gl_data = GlData()

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
