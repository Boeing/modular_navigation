#!/usr/bin/env python3

import argparse
import logging

import mongoengine

from map_manager.config import DATABASE_NAME
from map_manager.documents import Map

logger = logging.getLogger(__name__)

name = 'map_manager'

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('map_name', help='map_name', type=str)
    parser.add_argument('pbstream', help='pbstream', type=str)
    args = parser.parse_args()

    database = mongoengine.connect(db=DATABASE_NAME, host='localhost', port=27017)
    map_obj = Map.objects(name=args.map_name).get()

    with open(args.pbstream, 'w') as f:
        f.write(map_obj.map_data.read())
