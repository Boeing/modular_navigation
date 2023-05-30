#ifndef SIMPLE_SVG_HPP
#define SIMPLE_SVG_HPP

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace svg
{
// Utility XML/String Functions.
template <typename T>
inline std::string attribute(std::string const& attribute_name, T const& value, std::string const& unit = "")
{
    std::stringstream ss;
    ss << attribute_name << "=\"" << value << unit << "\" ";
    return ss.str();
}
inline std::string elemStart(std::string const& element_name)
{
    return "\t<" + element_name + " ";
}
inline std::string elemEnd(std::string const& element_name)
{
    return "</" + element_name + ">\n";
}
inline std::string emptyElemEnd()
{
    return "/>\n";
}

// Quick optional return type.  This allows functions to return an invalid
//  value if no good return is possible.  The user checks for validity
//  before using the returned value.
template <typename T> class optional
{
  public:
    explicit optional<T>(T const& type) : valid(true), type(type)
    {
    }
    optional<T>() : valid(false), type(T())
    {
    }
    T* operator->()
    {
        // If we try to access an invalid value, an exception is thrown.
        if (!valid)
            throw std::exception();

        return &type;
    }
    // Test for validity.
    bool operator!() const
    {
        return !valid;
    }

  private:
    bool valid;
    T type;
};

struct Dimensions
{
    Dimensions(double width, double height) : width(width), height(height)
    {
    }
    explicit Dimensions(double combined = 0) : width(combined), height(combined)
    {
    }
    double width;
    double height;
};

struct Point
{
    Point(double x = 0, double y = 0) : x(x), y(y)
    {
    }
    double x;
    double y;
};
inline optional<Point> getMinPoint(std::vector<Point> const& points)
{
    if (points.empty())
        return optional<Point>();

    Point min = points[0];
    for (unsigned i = 0; i < points.size(); ++i)
    {
        if (points[i].x < min.x)
            min.x = points[i].x;
        if (points[i].y < min.y)
            min.y = points[i].y;
    }
    return optional<Point>(min);
}
inline optional<Point> getMaxPoint(std::vector<Point> const& points)
{
    if (points.empty())
        return optional<Point>();

    Point max = points[0];
    for (unsigned i = 0; i < points.size(); ++i)
    {
        if (points[i].x > max.x)
            max.x = points[i].x;
        if (points[i].y > max.y)
            max.y = points[i].y;
    }
    return optional<Point>(max);
}

struct ViewBox
{
    ViewBox(const double min_x, const double min_y, const double width, const double height)
        : min_x(min_x), min_y(min_y), width(width), height(height)
    {
    }
    double min_x;
    double min_y;
    double width;
    double height;
};

// Defines the dimensions, scale, origin, and origin offset of the document.
struct Layout
{
    enum Origin
    {
        TopLeft,
        BottomLeft,
        TopRight,
        BottomRight,
    };

    Layout(Dimensions const& dimensions = Dimensions(0, 0), Origin origin = BottomLeft, double scale = 1,
           Point const& origin_offset = Point(0, 0))
        : dimensions(dimensions), scale(scale), origin(origin), origin_offset(origin_offset)
    {
    }
    Dimensions dimensions;
    double scale;
    Origin origin;
    Point origin_offset;
};

// Convert coordinates in user space to SVG native space.
inline double translateX(double x, Layout const& layout)
{
    if (layout.origin == Layout::BottomRight || layout.origin == Layout::TopRight)
        return layout.dimensions.width - ((x + layout.origin_offset.x) * layout.scale);
    else
        return (layout.origin_offset.x + x) * layout.scale;
}

inline double translateY(double y, Layout const& layout)
{
    if (layout.origin == Layout::BottomLeft || layout.origin == Layout::BottomRight)
        return layout.dimensions.height - ((y + layout.origin_offset.y) * layout.scale);
    else
        return (layout.origin_offset.y + y) * layout.scale;
}
inline double translateScale(double dimension, Layout const& layout)
{
    return dimension * layout.scale;
}

class Serializeable
{
  public:
    Serializeable()
    {
    }
    virtual ~Serializeable(){};
    virtual std::string toString(Layout const& layout) const = 0;
};

class Color : public Serializeable
{
  public:
    Color(int r, int g, int b) : transparent(false), red(r), green(g), blue(b)
    {
    }

    Color(int r, int g, int b, bool transparent) : transparent(transparent), red(r), green(g), blue(b)
    {
    }

    static Color Aqua()
    {
        return Color(0, 255, 255);
    }

    static Color Black()
    {
        return Color(0, 0, 0);
    }

    static Color Blue()
    {
        return Color(0, 0, 255);
    }

    static Color Brown()
    {
        return Color(165, 42, 42);
    }

    static Color Cyan()
    {
        return Color(0, 255, 255);
    }

    static Color Fuchsia()
    {
        return Color(255, 0, 255);
    }

    static Color Green()
    {
        return Color(0, 128, 0);
    }

    static Color Lime()
    {
        return Color(0, 255, 0);
    }

    static Color Magenta()
    {
        return Color(255, 0, 255);
    }

    static Color Orange()
    {
        return Color(255, 165, 0);
    }

    static Color Purple()
    {
        return Color(128, 0, 128);
    }

    static Color Red()
    {
        return Color(255, 0, 0);
    }

    static Color Silver()
    {
        return Color(192, 192, 192);
    }

    static Color White()
    {
        return Color(255, 255, 255);
    }

    static Color Yellow()
    {
        return Color(255, 255, 0);
    }

    static Color Transparent()
    {
        return Color(0, 0, 0, true);
    }

    virtual ~Color()
    {
    }

    std::string toString(Layout const&) const override
    {
        std::stringstream ss;
        if (transparent)
            ss << "none";
        else
            ss << "rgb(" << red << "," << green << "," << blue << ")";
        return ss.str();
    }

  private:
    bool transparent;
    int red;
    int green;
    int blue;
};

class Fill : public Serializeable
{
  public:
    explicit Fill(Color color = Color::Transparent()) : color(color)
    {
    }
    std::string toString(Layout const& layout) const override
    {
        std::stringstream ss;
        ss << attribute("fill", color.toString(layout));
        return ss.str();
    }

  private:
    Color color;
};

class Stroke : public Serializeable
{
  public:
    Stroke(double width = -1, Color color = Color::Transparent(), bool nonScalingStroke = false)
        : width(width), color(color), nonScaling(nonScalingStroke)
    {
    }
    std::string toString(Layout const& layout) const override
    {
        // If stroke width is invalid.
        if (width < 0)
            return std::string();

        std::stringstream ss;
        ss << attribute("stroke-width", translateScale(width, layout)) << attribute("stroke", color.toString(layout));
        if (nonScaling)
            ss << attribute("vector-effect", "non-scaling-stroke");
        return ss.str();
    }

  private:
    double width;
    Color color;
    bool nonScaling;
};

class Font : public Serializeable
{
  public:
    Font(double size = 12, std::string const& family = "Verdana") : size(size), family(family)
    {
    }
    std::string toString(Layout const& layout) const override
    {
        std::stringstream ss;
        ss << attribute("font-size", translateScale(size, layout)) << attribute("font-family", family);
        return ss.str();
    }

  private:
    double size;
    std::string family;
};

class Shape : public Serializeable
{
  public:
    Shape(Fill const& fill = Fill(), Stroke const& stroke = Stroke()) : fill(fill), stroke(stroke)
    {
    }
    virtual ~Shape()
    {
    }
    virtual std::string toString(Layout const& layout) const override = 0;
    virtual void offset(Point const& offset) = 0;

  protected:
    Fill fill;
    Stroke stroke;
};
template <typename T> inline std::string vectorToString(std::vector<T> collection, Layout const& layout)
{
    std::string combination_str;
    for (unsigned i = 0; i < collection.size(); ++i)
        combination_str += collection[i].toString(layout);

    return combination_str;
}

class Circle : public Shape
{
  public:
    Circle(Point const& center, double diameter, Fill const& fill, Stroke const& stroke = Stroke())
        : Shape(fill, stroke), center(center), radius(diameter / 2)
    {
    }
    std::string toString(Layout const& layout) const override
    {
        std::stringstream ss;
        ss << elemStart("circle") << attribute("cx", translateX(center.x, layout))
           << attribute("cy", translateY(center.y, layout)) << attribute("r", translateScale(radius, layout))
           << fill.toString(layout) << stroke.toString(layout) << emptyElemEnd();
        return ss.str();
    }
    void offset(Point const& offset) override
    {
        center.x += offset.x;
        center.y += offset.y;
    }

  private:
    Point center;
    double radius;
};

class Elipse : public Shape
{
  public:
    Elipse(Point const& center, double width, double height, Fill const& fill = Fill(), Stroke const& stroke = Stroke())
        : Shape(fill, stroke), center(center), radius_width(width / 2), radius_height(height / 2)
    {
    }
    std::string toString(Layout const& layout) const override
    {
        std::stringstream ss;
        ss << elemStart("ellipse") << attribute("cx", translateX(center.x, layout))
           << attribute("cy", translateY(center.y, layout)) << attribute("rx", translateScale(radius_width, layout))
           << attribute("ry", translateScale(radius_height, layout)) << fill.toString(layout) << stroke.toString(layout)
           << emptyElemEnd();
        return ss.str();
    }
    void offset(Point const& offset) override
    {
        center.x += offset.x;
        center.y += offset.y;
    }

  private:
    Point center;
    double radius_width;
    double radius_height;
};

class Rectangle : public Shape
{
  public:
    Rectangle(Point const& edge, double width, double height, Fill const& fill = Fill(),
              Stroke const& stroke = Stroke())
        : Shape(fill, stroke), edge(edge), width(width), height(height)
    {
    }
    std::string toString(Layout const& layout) const override
    {
        std::stringstream ss;
        ss << elemStart("rect") << attribute("x", translateX(edge.x, layout))
           << attribute("y", translateY(edge.y, layout)) << attribute("width", translateScale(width, layout))
           << attribute("height", translateScale(height, layout)) << fill.toString(layout) << stroke.toString(layout)
           << emptyElemEnd();
        return ss.str();
    }
    void offset(Point const& offset) override
    {
        edge.x += offset.x;
        edge.y += offset.y;
    }

  private:
    Point edge;
    double width;
    double height;
};

class Line : public Shape
{
  public:
    Line(Point const& start_point, Point const& end_point, Stroke const& stroke = Stroke())
        : Shape(Fill(), stroke), start_point(start_point), end_point(end_point)
    {
    }
    std::string toString(Layout const& layout) const override
    {
        std::stringstream ss;
        ss << elemStart("line") << attribute("x1", translateX(start_point.x, layout))
           << attribute("y1", translateY(start_point.y, layout)) << attribute("x2", translateX(end_point.x, layout))
           << attribute("y2", translateY(end_point.y, layout)) << stroke.toString(layout) << emptyElemEnd();
        return ss.str();
    }
    void offset(Point const& offset) override
    {
        start_point.x += offset.x;
        start_point.y += offset.y;

        end_point.x += offset.x;
        end_point.y += offset.y;
    }

  private:
    Point start_point;
    Point end_point;
};

class Polygon : public Shape
{
  public:
    Polygon(Fill const& fill = Fill(), Stroke const& stroke = Stroke()) : Shape(fill, stroke)
    {
    }
    explicit Polygon(Stroke const& stroke = Stroke()) : Shape(Fill(Color::Transparent()), stroke)
    {
    }
    Polygon& operator<<(Point const& point)
    {
        points.push_back(point);
        return *this;
    }
    std::string toString(Layout const& layout) const override
    {
        std::stringstream ss;
        ss << elemStart("polygon");

        ss << "points=\"";
        for (unsigned i = 0; i < points.size(); ++i)
            ss << translateX(points[i].x, layout) << "," << translateY(points[i].y, layout) << " ";
        ss << "\" ";

        ss << fill.toString(layout) << stroke.toString(layout) << emptyElemEnd();
        return ss.str();
    }
    void offset(Point const& offset) override
    {
        for (unsigned i = 0; i < points.size(); ++i)
        {
            points[i].x += offset.x;
            points[i].y += offset.y;
        }
    }

  private:
    std::vector<Point> points;
};

class Path : public Shape
{
  public:
    Path(Fill const& fill = Fill(), Stroke const& stroke = Stroke()) : Shape(fill, stroke)
    {
        startNewSubPath();
    }
    explicit Path(Stroke const& stroke = Stroke()) : Shape(Fill(Color::Transparent()), stroke)
    {
        startNewSubPath();
    }
    Path& operator<<(Point const& point)
    {
        paths.back().push_back(point);
        return *this;
    }

    void startNewSubPath()
    {
        if (paths.empty() || 0 < paths.back().size())
            paths.emplace_back();
    }

    std::string toString(Layout const& layout) const override
    {
        std::stringstream ss;
        ss << elemStart("path");

        ss << "d=\"";
        for (auto const& subpath : paths)
        {
            if (subpath.empty())
                continue;

            ss << "M";
            for (auto const& point : subpath)
                ss << translateX(point.x, layout) << "," << translateY(point.y, layout) << " ";
            ss << "z ";
        }
        ss << "\" ";
        ss << "fill-rule=\"evenodd\" ";

        ss << fill.toString(layout) << stroke.toString(layout) << emptyElemEnd();
        return ss.str();
    }

    void offset(Point const& offset) override
    {
        for (auto& subpath : paths)
            for (auto& point : subpath)
            {
                point.x += offset.x;
                point.y += offset.y;
            }
    }

  private:
    std::vector<std::vector<Point>> paths;
};

class Polyline : public Shape
{
  public:
    Polyline(Fill const& fill = Fill(), Stroke const& stroke = Stroke()) : Shape(fill, stroke)
    {
    }
    explicit Polyline(Stroke const& stroke = Stroke()) : Shape(Fill(Color::Transparent()), stroke)
    {
    }
    explicit Polyline(std::vector<Point> const& points, Fill const& fill = Fill(), Stroke const& stroke = Stroke())
        : Shape(fill, stroke), points(points)
    {
    }
    Polyline& operator<<(Point const& point)
    {
        points.push_back(point);
        return *this;
    }
    std::string toString(Layout const& layout) const override
    {
        std::stringstream ss;
        ss << elemStart("polyline");

        ss << "points=\"";
        for (unsigned i = 0; i < points.size(); ++i)
            ss << translateX(points[i].x, layout) << "," << translateY(points[i].y, layout) << " ";
        ss << "\" ";

        ss << fill.toString(layout) << stroke.toString(layout) << emptyElemEnd();
        return ss.str();
    }
    void offset(Point const& offset) override
    {
        for (unsigned i = 0; i < points.size(); ++i)
        {
            points[i].x += offset.x;
            points[i].y += offset.y;
        }
    }
    std::vector<Point> points;
};

class Text : public Shape
{
  public:
    Text(Point const& origin, std::string const& content, Fill const& fill = Fill(), Font const& font = Font(),
         Stroke const& stroke = Stroke())
        : Shape(fill, stroke), origin(origin), content(content), font(font)
    {
    }
    std::string toString(Layout const& layout) const override
    {
        std::stringstream ss;
        ss << elemStart("text") << attribute("x", translateX(origin.x, layout))
           << attribute("y", translateY(origin.y, layout)) << fill.toString(layout) << stroke.toString(layout)
           << font.toString(layout) << ">" << content << elemEnd("text");
        return ss.str();
    }
    void offset(Point const& offset) override
    {
        origin.x += offset.x;
        origin.y += offset.y;
    }

  private:
    Point origin;
    std::string content;
    Font font;
};

// Sample charting class.
class LineChart : public Shape
{
  public:
    LineChart(Dimensions margin = Dimensions(), double scale = 1,
              Stroke const& axis_stroke = Stroke(.5, Color::Purple()))
        : axis_stroke(axis_stroke), margin(margin), scale(scale)
    {
    }
    LineChart& operator<<(Polyline const& polyline)
    {
        if (polyline.points.empty())
            return *this;

        polylines.push_back(polyline);
        return *this;
    }
    std::string toString(Layout const& layout) const override
    {
        if (polylines.empty())
            return "";

        std::string ret;
        for (unsigned i = 0; i < polylines.size(); ++i)
            ret += polylineToString(polylines[i], layout);

        return ret + axisString(layout);
    }
    void offset(Point const& offset) override
    {
        for (unsigned i = 0; i < polylines.size(); ++i)
            polylines[i].offset(offset);
    }

  private:
    Stroke axis_stroke;
    Dimensions margin;
    double scale;
    std::vector<Polyline> polylines;

    optional<Dimensions> getDimensions() const
    {
        if (polylines.empty())
            return optional<Dimensions>();

        optional<Point> min = getMinPoint(polylines[0].points);
        optional<Point> max = getMaxPoint(polylines[0].points);
        for (unsigned i = 0; i < polylines.size(); ++i)
        {
            if (getMinPoint(polylines[i].points)->x < min->x)
                min->x = getMinPoint(polylines[i].points)->x;
            if (getMinPoint(polylines[i].points)->y < min->y)
                min->y = getMinPoint(polylines[i].points)->y;
            if (getMaxPoint(polylines[i].points)->x > max->x)
                max->x = getMaxPoint(polylines[i].points)->x;
            if (getMaxPoint(polylines[i].points)->y > max->y)
                max->y = getMaxPoint(polylines[i].points)->y;
        }

        return optional<Dimensions>(Dimensions(max->x - min->x, max->y - min->y));
    }
    std::string axisString(Layout const& layout) const
    {
        optional<Dimensions> dimensions = getDimensions();
        if (!dimensions)
            return "";

        // Make the axis 10% wider and higher than the data points.
        double width = dimensions->width * 1.1;
        double height = dimensions->height * 1.1;

        // Draw the axis.
        Polyline axis(Fill(Color::Transparent()), axis_stroke);
        axis << Point(margin.width, margin.height + height) << Point(margin.width, margin.height)
             << Point(margin.width + width, margin.height);

        return axis.toString(layout);
    }
    std::string polylineToString(Polyline const& polyline, Layout const& layout) const
    {
        Polyline shifted_polyline = polyline;
        shifted_polyline.offset(Point(margin.width, margin.height));

        std::vector<Circle> vertices;
        for (unsigned i = 0; i < shifted_polyline.points.size(); ++i)
            vertices.push_back(
                Circle(shifted_polyline.points[i], getDimensions()->height / 30.0, Fill(Color::Black())));

        return shifted_polyline.toString(layout) + vectorToString(vertices, layout);
    }
};

class Document
{
  public:
    Document() : layout_(nullptr), viewbox_(nullptr)
    {
    }

    explicit Document(const std::shared_ptr<ViewBox>& viewbox) : layout_(nullptr), viewbox_(viewbox)
    {
    }

    Document(const std::shared_ptr<Layout>& layout, const std::shared_ptr<ViewBox>& viewbox)
        : layout_(layout), viewbox_(viewbox)
    {
    }

    Document& operator<<(Shape const& shape)
    {
        if (layout_)
            body_nodes_str_list_.push_back(shape.toString(*layout_));
        else
            body_nodes_str_list_.push_back(shape.toString(Layout()));
        return *this;
    }
    std::string toString() const
    {
        std::stringstream ss;
        writeToStream(ss);
        return ss.str();
    }
    bool save(std::string const& file_name) const
    {
        std::ofstream ofs(file_name.c_str());
        if (!ofs.good())
            return false;

        writeToStream(ofs);
        ofs.close();
        return true;
    }

    std::shared_ptr<ViewBox> viewbox() const
    {
        return viewbox_;
    }

  private:
    void writeToStream(std::ostream& str) const
    {
        str << "<?xml " << attribute("version", "1.0") << attribute("standalone", "no")
            << "?>\n<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" "
            << "\"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n<svg ";

        if (layout_)
            str << attribute("width", layout_->dimensions.width, "px")
                << attribute("height", layout_->dimensions.height, "px");

        if (viewbox_)
            str << "viewBox=\"" << viewbox_->min_x << " " << viewbox_->min_y << " " << viewbox_->width << " "
                << viewbox_->height << "\" ";

        str << attribute("xmlns", "http://www.w3.org/2000/svg") << attribute("version", "1.1") << ">\n";
        for (const auto& body_node_str : body_nodes_str_list_)
        {
            str << body_node_str;
        }
        str << elemEnd("svg");
    }

  private:
    std::shared_ptr<Layout> layout_;
    std::shared_ptr<ViewBox> viewbox_;

    std::vector<std::string> body_nodes_str_list_;
};
}  // namespace svg

#endif
