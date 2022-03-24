# SPDX-License-Identifier: Apache-2.0

zephyr_library()

#zephyr_library_sources_ifdef(MODE_N_T icp10125.c)
#zephyr_library_sources_ifdef(MODE_N_P icp10125.c)
zephyr_library_sources(icp10125.c)
