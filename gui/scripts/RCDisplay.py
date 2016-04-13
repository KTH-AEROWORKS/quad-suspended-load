#!/usr/bin/env python

import sys

from gui.RCDisplay import RCDisplayPlugin
from rqt_gui.main import Main

plugin = 'RCDisplay'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
