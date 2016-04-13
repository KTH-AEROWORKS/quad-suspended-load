#!/usr/bin/env python

import sys

from gui.saver_mavros import saver_mavrosPlugin
from rqt_gui.main import Main

plugin = 'saver_mavros'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
