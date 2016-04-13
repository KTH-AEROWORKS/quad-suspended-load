#!/usr/bin/env python

import sys

from gui.saver import saverPlugin
from rqt_gui.main import Main

plugin = 'saver'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
