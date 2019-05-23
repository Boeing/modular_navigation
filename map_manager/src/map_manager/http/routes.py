import StringIO
import typing
from flask import abort, send_file
from mongoengine import DoesNotExist, ValidationError
from functools import partial
from map_manager.documents import Map
from map_manager.http.cross_domain import cross_domain


@cross_domain('*')
def get_occupancy_grid(map_name):
    try:
        map_doc = Map.objects.get(name=map_name)

    except (DoesNotExist, ValidationError):
        raise abort(404)

    map_msg = map_doc.get_msg()

    fp = StringIO.StringIO()
    map_msg.serialize(fp)
    fp.flush()
    fp.seek(0)

    return send_file(
        filename_or_fp=fp,
        mimetype='application/octet-stream',
        attachment_filename='{}.msg'.format(map_doc.name)
    )


def setup_routes(app):

    app.add_url_rule(
        rule='/maps/<map_name>/occupancy_grid',
        endpoint='get_occupancy_grid',
        view_func=partial(get_occupancy_grid),
        methods=['GET']
    )
