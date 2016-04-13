#!/usr/bin/env python

import sys

from gui.ChooseController import ChooseController
from rqt_gui.main import Main

plugin = 'ChooseController'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
