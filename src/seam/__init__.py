"""
*********************************************************
seams
*********************************************************

.. current module:: seams

.. toctree ::
    :maxdepth: 1


"""

from __future__ import print_function

import os

__author__ = ["Mahiro Goto"]
__copyright__ = "MIT License"
__email__ = "magoto@student.ethz.ch"
__version__ = "0.1.0"

HERE = os.path.dirname(__file__)

HOME = os.path.abspath(os.path.join(HERE, "../../"))
DATA = os.path.abspath(os.path.join(HOME, "data"))
DOCS = os.path.abspath(os.path.join(HOME, "docs"))
TEMP = os.path.abspath(os.path.join(HOME, "temp"))

try:
    git_head_file = compas._os.absjoin(HOME, '.git', 'HEAD')

    if os.path.exists(git_head_file):
        with open(git_head_file, 'r') as git_head_file:
            _, ref_path = git_head.read().strip().split(' ')
            ref_path = ref_path.split('/')

            git_head_refs_file = compas._os.absjoin(HOME, '.git', *ref_path)

        if os.path.exists(git_head_refs_file):
            with open(git_head_refs_file, 'r') as git_head_refs:
                git_commit = git_head_ref.read().strip()
                __version__ += '_' + git_commit[:8]
except Exception:
    pass

__all__ = ['HOME', 'DATA', 'DOCS', 'TEMP']
