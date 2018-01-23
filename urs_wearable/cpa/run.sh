#!/bin/sh

gnome-terminal \
--tab --working-directory=$PWD/cpa1 -e "./cpa+" \
--tab --working-directory=$PWD/cpa2 -e "./cpa+"
