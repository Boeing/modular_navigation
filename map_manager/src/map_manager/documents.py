import calendar
import logging
from datetime import datetime, date

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
from mongoengine import DateTimeField, StringField, BooleanField
from mongoengine import Document, EmbeddedDocument, FileField
from mongoengine import EmbeddedDocumentField, EmbeddedDocumentListField
from mongoengine import FloatField, IntField
from mongoengine import GridFSProxy
from nav_msgs.msg import MapMetaData as MapMetaDataMsg
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from std_msgs.msg import Header

from hd_map.msg import Map as MapMsg
from hd_map.msg import Marker as MarkerMsg
from hd_map.msg import Markers as MarkersMsg
from hd_map.msg import Zone as ZoneMsg
from hd_map.msg import Zones as ZonesMsg

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


class OccupancyGrid(EmbeddedDocument):
    image = FileField()  # type: GridFSProxy

    resolution = FloatField()  # type: float
    width = IntField()  # type: int
    height = IntField()  # type: int
    origin = EmbeddedDocumentField(Pose)  # type: Pose
    negate = BooleanField()  # type: bool
    occupied_thresh = FloatField()  # type: float
    free_thresh = FloatField()  # type: float

    def get_map_meta_data_msg(self):
        # type: () -> MapMetaDataMsg
        return MapMetaDataMsg(
            map_load_time=rospy.Time(0),
            resolution=self.resolution,
            width=self.width,
            height=self.height,
            origin=self.origin.get_msg()
        )

    def get_occupancy_grid_msg(self):
        # type: () -> OccupancyGridMsg
        self.image.seek(0)
        return OccupancyGridMsg(
            header=Header(seq=0, stamp=rospy.Time.now(), frame_id='map'),
            info=self.get_map_meta_data_msg(),
            data=numpy.array(Image.open(fp=self.image), dtype=numpy.int8).ravel()
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
    name = StringField(max_length=256, required=True, primary_key=True)  # type: str

    map = EmbeddedDocumentField(OccupancyGrid)  # type: OccupancyGrid
    markers = EmbeddedDocumentListField(Marker)  # type: typing.List[Marker]
    zones = EmbeddedDocumentListField(Zone)  # type: typing.List[Zone]

    def get_zones_msg(self):
        # type: () -> ZonesMsg
        zones = [zone.get_msg() for zone in self.zones]
        return ZonesMsg(
            zones=zones
        )

    def get_markers_msg(self):
        # type: () -> MarkersMsg
        markers = [marker.get_msg() for marker in self.markers]
        return MarkersMsg(
            markers=markers
        )

    def get_msg(self):
        # type: () -> MapMsg
        return MapMsg(
            name=str(self.name),
            description=str(self.description),
            created=rospy.Time(calendar.timegm(self.created.timetuple())),
            modified=rospy.Time(calendar.timegm(self.modified.timetuple())),

            markers=self.get_markers_msg().markers,
            zones=self.get_zones_msg().zones,
            map_info=self.map.get_map_meta_data_msg()
        )


mongoengine.signals.pre_save.connect(Marker.pre_save, sender=Marker)
mongoengine.signals.pre_save.connect(Zone.pre_save, sender=Zone)
mongoengine.signals.pre_save.connect(Map.pre_save, sender=Map)
