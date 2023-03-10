import io
from functools import wraps

from flask import abort, send_file, render_template
from flask import jsonify, Blueprint
from jinja2 import TemplateNotFound
from mongoengine import DoesNotExist, ValidationError
from requests.status_codes import codes

from map_manager.msg import MapInfo as MapMsg
from map_manager.documents import Map
from map_manager.http_utils.cross_domain import cross_domain

map_api = Blueprint('map_api', __name__, template_folder='templates')


def exception_wrapper(fn):
    @wraps(fn)
    def wrapper(*args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except Exception as e:
            return jsonify({'msg': 'Exception: {}'.format(str(e))}), codes.bad

    return wrapper


@map_api.route('/')
def home():
    try:
        map_docs = [m for m in Map.objects]  # type: Map
        return render_template('index.html', maps=map_docs)
    except TemplateNotFound:
        abort(404)


@map_api.route('/maps/<map_name>/occupancy_grid.msg', methods=['GET'])
@cross_domain('*')
@exception_wrapper
def get_occupancy_grid_msg(map_name, node):
    try:
        map_doc = Map.objects.get(name=map_name)  # type: Map

    except (DoesNotExist, ValidationError):
        raise abort(404)

    map_msg = map_doc.get_occupancy_grid_msg(node)  # type: MapMsg

    fp = io.BytesIO()
    map_msg.serialize(fp)
    fp.flush()
    fp.seek(0)

    return send_file(
        path_or_file=fp,
        mimetype='application/octet-stream',
        attachment_filename='{}.msg'.format(map_doc.name),
        cache_timeout=1
    )


@map_api.route('/maps/<map_name>/occupancy_grid.png', methods=['GET'])
@cross_domain('*')
@exception_wrapper
def get_png(map_name):
    try:
        map_doc = Map.objects.get(name=map_name)  # type: Map

    except (DoesNotExist, ValidationError):
        raise abort(404)

    fp = io.BytesIO()
    map_doc.get_png(fp)
    fp.flush()
    fp.seek(0)

    return send_file(
        path_or_file=fp,
        mimetype='image/png',
        attachment_filename='{}.png'.format(map_doc.name),
        cache_timeout=1
    )


@map_api.route('/maps/<map_name>/thumbnail.png', methods=['GET'])
@cross_domain('*')
@exception_wrapper
def get_thumbnail_png(map_name):
    try:
        map_doc = Map.objects.get(name=map_name)  # type: Map

    except (DoesNotExist, ValidationError):
        raise abort(404)

    fp = io.BytesIO()
    map_doc.get_thumbnail_png(fp)
    fp.flush()
    fp.seek(0)

    return send_file(
        path_or_file=fp,
        mimetype='image/png',
        attachment_filename='{}.png'.format(map_doc.name),
        cache_timeout=1
    )
