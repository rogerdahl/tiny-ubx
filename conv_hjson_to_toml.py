#!/usr/bin/env python

"""ubx.hjson -> ubx.toml"""

import logging
import sys
import hjson
import toml

log = logging.getLogger(__name__)


def main():
    d = hjson.load(open('./ubx.hjson'))
    toml.dump(d, open('./ubx.toml', 'w'))


if __name__ == '__main__':
    sys.exit(main())
