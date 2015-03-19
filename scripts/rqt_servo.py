#!/usr/bin/env python

from rqt_gui.main import Main

import sys
import roslib

pkg_name = 'rqt_servo'
roslib.load_manifest(pkg_name)

main = Main()
sys.exit(main.main(sys.argv, standalone=pkg_name))
