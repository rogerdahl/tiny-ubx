#!/usr/bin/env python

"""ubx.hjson -> ubx.toml"""

import logging
import sys
import hjson
import toml

log = logging.getLogger(__name__)


def main():
    d = toml.load(open('./ubx.toml'))
    d = hjson.dump(d, open('./ubx.hjson', 'w'))


if __name__ == '__main__':
    sys.exit(main())
