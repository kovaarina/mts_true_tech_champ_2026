#!/usr/bin/env python3
"""Referee controller entry point — thin wrapper over the shared judge (common/referee).
Do not edit; the referee is fixed and runs in its own supervisor process."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "common", "referee"))
from referee_core import Referee   # noqa: E402

Referee().run()
