#!/usr/bin/env python

import sys

from gui.ChooseSimulator import ChooseSimulator
from rqt_gui.main import Main

plugin = 'ChooseSimulator'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
