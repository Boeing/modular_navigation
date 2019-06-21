import calendar
import logging
from datetime import datetime, date
from io import BytesIO

import math3d
import mongoengine
import numpy
import rospy
import typing
from PIL import Image
from geometry_msgs.msg import Point as PointMsg
from geometry_msgs.msg import Point32 as Point32Msg
from geometry_msgs.msg import Polygon as PolygonMsg
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import Quaternion as QuaternionMsg
from mongoengine import DateTimeField, StringField
from mongoengine import Document, EmbeddedDocument, FileField
from mongoengine import EmbeddedDocumentField, EmbeddedDocumentListField
from mongoengine import FloatField, IntField
from mongoengine import GridFSProxy
from nav_msgs.msg import MapMetaData as MapMetaDataMsg
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

from hd_map.msg import Map as MapMsg
from hd_map.msg import MapInfo as MapInfoMsg
from hd_map.msg import Marker as MarkerMsg
from hd_map.msg import Zone as ZoneMsg

logger = logging.getLogger(__name__)


class Quaternion(EmbeddedDocument):
    w = FloatField(default=1)  # type: float
    x = FloatField(default=0)  # type: float
    y = FloatField(default=0)  # type: float
    z = FloatField(default=0)  # type: float

    @classmethod
    def pre_save(cls, sender, document, **kwargs):
        logging.debug('Normalising quaternion')
        qt = math3d.Versor(document.w, document.x, document.y, document.z)
        document.w = qt[0]
        document.x = qt[1]
        document.y = qt[2]
        document.z = qt[3]

    def get_euler(self):
        qt = math3d.Versor(self.w, self.x, self.y, self.z)
        return qt.get_orientation().to_euler('XYZ')

    def get_msg(self):
        # type: () -> QuaternionMsg
        qt = math3d.Versor(self.w, self.x, self.y, self.z)
        return QuaternionMsg(
            x=qt[1],
            y=qt[2],
            z=qt[3],
            w=qt[0]
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


class Marker(EmbeddedDocument, DocumentMixin):
    name = StringField(max_length=256, required=True, unique=True)  # type: str

    marker_type = IntField(required=True)  # type: int

    pose = EmbeddedDocumentField(Pose)  # type: Pose

    def get_msg(self):
        # type: () -> MarkerMsg
        return MarkerMsg(
            name=str(self.name),
            description=str(self.description),
            created=rospy.Time(calendar.timegm(self.created.timetuple())),
            modified=rospy.Time(calendar.timegm(self.modified.timetuple())),

            marker_type=self.marker_type,
            pose=self.pose.get_msg()
        )


class Zone(EmbeddedDocument, DocumentMixin):
    name = StringField(max_length=256, required=True, unique=True)  # type: str

    zone_type = IntField(required=True)  # type: int

    polygon = EmbeddedDocumentListField(Point)  # type: typing.List[Point]

    def get_msg(self):
        # type: () -> ZoneMsg
        return ZoneMsg(
            name=str(self.name),
            description=str(self.description),
            created=rospy.Time(calendar.timegm(self.created.timetuple())),
            modified=rospy.Time(calendar.timegm(self.modified.timetuple())),

            zone_type=self.zone_type,
            polygon=PolygonMsg(
                points=[Point32Msg(x=p.x, y=p.y, z=p.z) for p in self.polygon]
            )
        )


class Map(Document, DocumentMixin):
    @classmethod
    def from_msg(cls, map_msg, occupancy_grid_msg):
        assert isinstance(map_msg, MapMsg)
        assert isinstance(occupancy_grid_msg, CompressedImage)
        map_obj = Map()
        map_obj.name = map_msg.info.name
        map_obj.description = map_msg.info.description
        map_obj.width = map_msg.info.meta_data.width
        map_obj.height = map_msg.info.meta_data.height
        map_obj.resolution = map_msg.info.meta_data.resolution
        map_obj.origin = Pose(
            position=Point(
                x=map_msg.info.meta_data.origin.position.x,
                y=map_msg.info.meta_data.origin.position.y,
                z=map_msg.info.meta_data.origin.position.z),
            quaternion=Quaternion(
                x=map_msg.info.meta_data.origin.orientation.x,
                y=map_msg.info.meta_data.origin.orientation.y,
                z=map_msg.info.meta_data.origin.orientation.z,
                w=map_msg.info.meta_data.origin.orientation.w
            )
        )

        for marker in map_msg.markers:
            assert isinstance(marker, MarkerMsg)
            m_obj = Marker(
                name=marker.name,
                description=marker.description,
                marker_type=marker.marker_type,
                pose=Pose(
                    position=Point(
                        x=marker.pose.position.x,
                        y=marker.pose.position.y,
                        z=marker.pose.position.z
                    ),
                    quaternion=Quaternion(
                        w=marker.pose.orientation.w,
                        x=marker.pose.orientation.x,
                        y=marker.pose.orientation.y,
                        z=marker.pose.orientation.z
                    )
                )
            )
            map_obj.markers.append(m_obj)

        for zone in map_msg.zones:
            assert isinstance(zone, ZoneMsg)
            z_obj = Zone(
                name=zone.name,
                description=zone.description,
                zone_type=zone.zone_type,
                polygon=[Point(x=point.x, y=point.y, z=point.z) for point in zone.polygon.points]
            )
            map_obj.zones.append(z_obj)

        if occupancy_grid_msg.format == 'raw':
            assert (len(occupancy_grid_msg.data) == map_msg.info.meta_data.width * map_msg.info.meta_data.height)
            im = Image.fromarray(numpy.fromstring(occupancy_grid_msg.data, dtype='uint8').reshape(
                map_msg.info.meta_data.height, map_msg.info.meta_data.width))
        else:
            b = BytesIO(occupancy_grid_msg.data)
            im = Image.open(b)  # type: Image
            if im.size != (map_msg.info.meta_data.width, map_msg.info.meta_data.height):
                raise Exception('Map info size does not match compressed data')
        map_obj.image.new_file()
        im.save(fp=map_obj.image, format='PPM')
        map_obj.image.close()

        map_obj.thumbnail.new_file()
        im.thumbnail(size=(400, 400))
        im.save(fp=map_obj.thumbnail, format='PPM')
        map_obj.thumbnail.close()

        return map_obj

    name = StringField(max_length=256, required=True, primary_key=True)  # type: str

    resolution = FloatField()  # type: float
    width = IntField()  # type: int
    height = IntField()  # type: int
    origin = EmbeddedDocumentField(Pose)  # type: Pose

    image = FileField()  # type: GridFSProxy
    thumbnail = FileField()  # type: GridFSProxy

    map_data = FileField()  # type: GridFSProxy

    markers = EmbeddedDocumentListField(Marker)  # type: typing.List[Marker]
    zones = EmbeddedDocumentListField(Zone)  # type: typing.List[Zone]

    def get_png(self, buff):
        # type: () -> None
        self.image.seek(0)
        im = Image.open(fp=self.image)
        im.save(buff, format='PNG', compress_level=1)

    def get_thumbnail_png(self, buff):
        # type: () -> None
        self.thumbnail.seek(0)
        im = Image.open(fp=self.thumbnail)
        im.save(buff, format='PNG', compress_level=1)

    def get_occupancy_grid_msg(self):
        # type: () -> OccupancyGridMsg
        self.image.seek(0)
        return OccupancyGridMsg(
            header=Header(seq=0, stamp=rospy.Time.now(), frame_id='map'),
            info=self.get_map_meta_data_msg(),
            data=numpy.array(Image.open(fp=self.image), dtype=numpy.int8).ravel()
        )

    def get_map_meta_data_msg(self):
        # type: () -> MapMetaDataMsg
        return MapMetaDataMsg(
            map_load_time=rospy.Time(0),
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
            created=rospy.Time(calendar.timegm(self.created.timetuple())),
            modified=rospy.Time(calendar.timegm(self.modified.timetuple())),
            meta_data=self.get_map_meta_data_msg()
        )

    def get_map_msg(self):
        # type: () -> MapMsg
        return MapMsg(
            info=self.get_map_info_msg(),
            markers=[marker.get_msg() for marker in self.markers],
            zones=[zone.get_msg() for zone in self.zones],
        )


mongoengine.signals.pre_save.connect(Marker.pre_save, sender=Marker)
mongoengine.signals.pre_save.connect(Zone.pre_save, sender=Zone)
mongoengine.signals.pre_save.connect(Map.pre_save, sender=Map)
