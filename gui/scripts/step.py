#!/usr/bin/env python

import sys

from gui.step import StepPlugin
from rqt_gui.main import Main

plugin = 'step'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))