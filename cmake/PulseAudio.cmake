# SPDX-License-Identifier: GPL-3.0-only
# The session-audio client is supplied by the host package manager.
find_package(PkgConfig REQUIRED)
pkg_check_modules(BLOFELD_PULSE REQUIRED IMPORTED_TARGET GLOBAL libpulse)
