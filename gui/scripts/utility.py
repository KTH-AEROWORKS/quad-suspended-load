#!/usr/bin/env python

import sys

from gui.utility import utilityPlugin
from rqt_gui.main import Main

plugin = 'utility'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
