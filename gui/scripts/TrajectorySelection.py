#!/usr/bin/env python

import sys

from gui.TrajectorySelection import TrajectorySelectionPlugin
from rqt_gui.main import Main

plugin = 'TrajectorySelection'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
