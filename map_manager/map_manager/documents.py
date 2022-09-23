import calendar
import logging
from datetime import datetime, date
from io import BytesIO, TextIOWrapper

import math6d
import mongoengine
import numpy
#import rospy
import rclpy
import typing
from PIL import Image
from PIL.ImageOps import invert
from geometry_msgs.msg import Point as PointMsg
from geometry_msgs.msg import Point32 as Point32Msg
from geometry_msgs.msg import Polygon as PolygonMsg
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import Quaternion as QuaternionMsg
from mongoengine import DateTimeField, StringField
from mongoengine import Document, EmbeddedDocument, FileField
from mongoengine import EmbeddedDocumentField, EmbeddedDocumentListField
from mongoengine import FloatField, IntField, BooleanField
from mongoengine import GridFSProxy
from nav_msgs.msg import MapMetaData as MapMetaDataMsg
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from std_msgs.msg import ColorRGBA as ColorMsg

from map_manager.msg import MapInfo as MapInfoMsg
from graph_map.msg import Zone as ZoneMsg
from graph_map.msg import Region as RegionMsg

logger = logging.getLogger(__name__)


class Quaternion(EmbeddedDocument):
    w = FloatField(default=1)  # type: float
    x = FloatField(default=0)  # type: float
    y = FloatField(default=0)  # type: float
    z = FloatField(default=0)  # type: float

    @classmethod
    def pre_save(cls, sender, document, **kwargs):
        logging.debug('Normalising quaternion')
        qt = math6d.Quaternion(document.w, document.x, document.y, document.z)
        document.w = qt.w
        document.x = qt.x
        document.y = qt.y
        document.z = qt.z

    def get_euler(self):
        qt = math6d.Quaternion(self.w, self.x, self.y, self.z)
        return qt.to_euler(math6d.Axis.AXIS_X, math6d.Axis.AXIS_Y, math6d.Axis.AXIS_Z)

    def get_msg(self):
        # type: () -> QuaternionMsg
        qt = math6d.Quaternion(self.w, self.x, self.y, self.z)
        return QuaternionMsg(
            x=qt.x,
            y=qt.y,
            z=qt.z,
            w=qt.w
        )


mongoengine.signals.pre_save.connect(Quaternion.pre_save, sender=Quaternion)


class Point(EmbeddedDocument):
    x = FloatField(default=0)  # type: float
    y = FloatField(default=0)  # type: float
    z = FloatField(default=0)  # type: float

    def get_msg(self):
        # type: () -> PointMsg
        return PointMsg(
            x=self.x,
            y=self.y,
            z=self.z,
        )


class Polygon(EmbeddedDocument):
    points = EmbeddedDocumentListField(Point)  # type: typing.List[Point]

    def get_msg(self):
        # type: () -> PointMsg
        return PolygonMsg(
            points=[Point32Msg(x=p.x, y=p.y, z=p.z) for p in self.points]
        )


class Color(EmbeddedDocument):
    r = FloatField(default=0)  # type: float
    g = FloatField(default=0)  # type: float
    b = FloatField(default=0)  # type: float
    a = FloatField(default=0)  # type: float

    def get_msg(self):
        # type: () -> ColorMsg
        return ColorMsg(
            r=self.r,
            g=self.g,
            b=self.b,
            a=self.a
        )


class Region(EmbeddedDocument):
    polygon = EmbeddedDocumentField(Polygon)  # type: Polygon
    color = EmbeddedDocumentField(Color)

    def get_msg(self):
        # type: () -> PointMsg
        return RegionMsg(
            polygon=self.polygon.get_msg(),
            color=self.color.get_msg()
        )


class Pose(EmbeddedDocument):
    position = EmbeddedDocumentField(Point, default=Point)  # type: Point
    quaternion = EmbeddedDocumentField(Quaternion, default=Quaternion)  # type: Quaternion

    def get_msg(self):
        # type: () -> PoseMsg
        return PoseMsg(
            position=self.position.get_msg(),
            orientation=self.quaternion.get_msg()
        )


class DocumentMixin(object):
    description = StringField()  # type: str

    modified = DateTimeField(default=datetime.utcnow)  # type: date
    created = DateTimeField(default=datetime.utcnow)  # type: date

    @classmethod
    def pre_save(cls, sender, document, **kwargs):
        document.modified = datetime.utcnow()


class Zone(EmbeddedDocument):
    id = StringField(max_length=256, required=True)  # type: str
    display_name = StringField(max_length=256, required=True)  # type: str

    attr = StringField()  # type: str
    drivable = BooleanField(required=True)  # type: bool
    cost = FloatField(required=True)  # type: float

    regions = EmbeddedDocumentListField(Region)  # type: typing.List[Region]

    @classmethod
    def from_msg(cls, zone_msg: ZoneMsg):
        assert isinstance(zone_msg, ZoneMsg)
        return Zone(
            id=zone_msg.id,
            display_name=zone_msg.display_name,
            attr=zone_msg.attr,
            drivable=zone_msg.drivable,
            cost=zone_msg.cost,
            regions=[
                Region(
                    polygon=Polygon(points=[Point(x=p.x, y=p.y) for p in region.polygon.points]),
                    color=Color(r=region.color.r, g=region.color.g, b=region.color.b, a=region.color.a)
                )
                for region in zone_msg.regions
            ]
        )

    def get_msg(self):
        # type: () -> ZoneMsg
        return ZoneMsg(
            id=str(self.id),
            display_name=str(self.display_name),
            attr=str(self.attr),
            drivable=bool(self.drivable),
            cost=float(self.cost),
            regions=[region.get_msg() for region in self.regions]
        )


class Map(Document, DocumentMixin):
    @classmethod
    def from_msg(cls, map_info_msg, zone_msgs, occupancy_grid_msg, pbstream_msg, node_graph_json, area_tree_json):
        assert isinstance(map_info_msg, MapInfoMsg)
        assert isinstance(occupancy_grid_msg, CompressedImage)

        map_obj = Map()
        map_obj.name = map_info_msg.name
        map_obj.description = map_info_msg.description
        map_obj.width = map_info_msg.meta_data.width
        map_obj.height = map_info_msg.meta_data.height
        map_obj.resolution = map_info_msg.meta_data.resolution
        map_obj.origin = Pose(
            position=Point(
                x=map_info_msg.meta_data.origin.position.x,
                y=map_info_msg.meta_data.origin.position.y,
                z=map_info_msg.meta_data.origin.position.z),
            quaternion=Quaternion(
                x=map_info_msg.meta_data.origin.orientation.x,
                y=map_info_msg.meta_data.origin.orientation.y,
                z=map_info_msg.meta_data.origin.orientation.z,
                w=map_info_msg.meta_data.origin.orientation.w
            )
        )

        for zone in zone_msgs:
            map_obj.zones.append(Zone.from_msg(zone))

        if occupancy_grid_msg.format == 'raw':
            assert (len(occupancy_grid_msg.data) == map_info_msg.meta_data.width * map_info_msg.meta_data.height)
            im = Image.fromarray(numpy.fromstring(occupancy_grid_msg.data, dtype='uint8').reshape(
                map_info_msg.meta_data.height, map_info_msg.meta_data.width))
        else:
            b = BytesIO(occupancy_grid_msg.data)
            im = Image.open(b)  # type: Image
            if im.size != (map_info_msg.meta_data.width, map_info_msg.meta_data.height):
                raise Exception('Map info size does not match compressed data')

        map_obj.image.new_file()
        im.save(fp=map_obj.image, format='PPM')
        map_obj.image.close()

        map_obj.thumbnail.new_file()
        im.thumbnail(size=(400, 400))
        im.save(fp=map_obj.thumbnail, format='PPM')
        map_obj.thumbnail.close()

        if pbstream_msg:
            map_obj.pbstream.new_file()
            map_obj.pbstream.write(pbstream_msg)
            map_obj.pbstream.close()

        map_obj.node_graph = node_graph_json
        map_obj.area_tree = area_tree_json

        return map_obj

    name = StringField(max_length=256, required=True, primary_key=True)  # type: str

    resolution = FloatField()  # type: float
    width = IntField()  # type: int
    height = IntField()  # type: int
    origin = EmbeddedDocumentField(Pose)  # type: Pose

    image = FileField()  # type: GridFSProxy
    thumbnail = FileField()  # type: GridFSProxy

    pbstream = FileField()  # type: GridFSProxy

    zones = EmbeddedDocumentListField(Zone)  # type: typing.List[Zone]

    node_graph = StringField()
    area_tree = StringField()

    def get_png(self, buff):
        # type: (TextIOWrapper) -> None
        self.image.seek(0)
        im = Image.open(fp=self.image)
        inverted = invert(im)
        inverted.save(buff, format='PNG', compress_level=1)

    def get_thumbnail_png(self, buff):
        # type: (TextIOWrapper) -> None
        self.thumbnail.seek(0)
        im = Image.open(fp=self.thumbnail)
        im.save(buff, format='PNG', compress_level=1)

    def get_occupancy_grid_msg(self, node):
        if node is None:
            logger.info('No node passed to documents.Map.')
            return
        # type: () -> OccupancyGridMsg
        self.image.seek(0)
        return OccupancyGridMsg(
            #header=Header(seq=0, stamp=rospy.Time.now(), frame_id='map'),
            header=Header(#seq=0,
                          stamp=node.get_clock().now().to_msg(),
                          frame_id='map'),
            info=self.get_map_meta_data_msg(),
            data=numpy.array(Image.open(fp=self.image), dtype=numpy.int8).ravel()
        )

    def get_map_meta_data_msg(self):
        # type: () -> MapMetaDataMsg
        return MapMetaDataMsg(
            #map_load_time=rospy.Time(0),
            map_load_time=rclpy.time.Time(seconds=0).to_msg(),
            resolution=self.resolution,
            width=self.width,
            height=self.height,
            origin=self.origin.get_msg()
        )

    def get_map_info_msg(self):
        # type: () -> MapInfoMsg
        return MapInfoMsg(
            name=str(self.name),
            description=str(self.description),
            #created=rospy.Time(calendar.timegm(self.created.timetuple())),
            #modified=rospy.Time(calendar.timegm(self.modified.timetuple())),
            created=rclpy.time.Time(seconds=calendar.timegm(self.created.timetuple())).to_msg(),
            modified=rclpy.time.Time(seconds=calendar.timegm(self.modified.timetuple())).to_msg(),
            meta_data=self.get_map_meta_data_msg()
        )


mongoengine.signals.pre_save.connect(Map.pre_save, sender=Map)
