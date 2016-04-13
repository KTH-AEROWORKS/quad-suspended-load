#!/usr/bin/env python

import sys

from gui.positionPlot import positionPlotPlugin
from rqt_gui.main import Main

plugin = 'positionPlot'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
