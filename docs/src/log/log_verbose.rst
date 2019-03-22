.. _log_verbose:

Enabled detailed level
=======================

.. tip::

  If import glog to build.

The general configuration of the log is in the head file `logger.h <https://github.com/slightech/MYNT-EYE-S-SDK/blob/master/include/mynteye/logger.h>`_ .

Uncomment ``FLAGS_v = 2`` ; and recompile to enable the detail levels, the log is printed by ``VLOG(n)``

For information on how to use the log library, such as how to configure, print, etc., please open its document and learn more:

.. code-block:: bash

  $ ./scripts/open.sh third_party/glog/doc/glog.html
