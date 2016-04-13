#!/usr/bin/env python

import sys

from gui.pointInput import pointInputPlugin
from rqt_gui.main import Main

plugin = 'pointInput'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
