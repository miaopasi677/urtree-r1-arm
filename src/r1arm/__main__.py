# -*- coding: utf-8 -*-
"""支持 `python -m r1arm`（需 PYTHONPATH 指向 src/）；日常请用 r1ctl.py。"""

import sys

from .cli import main

if __name__ == "__main__":
    sys.exit(main())
