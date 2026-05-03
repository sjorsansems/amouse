#!/usr/bin/env bash
set -euo pipefail

PROJECT="/mnt/c/Users/Sjors/OneDrive/Project/amouse - Final/dos"

if [ -d "$HOME/watcom/ow/binl" ]; then
  WROOT="$HOME/watcom/ow"
elif [ -d "$HOME/watcom/binl" ]; then
  WROOT="$HOME/watcom"
else
  WROOT="$HOME"
fi

export WATCOM="$WROOT"
export PATH="$WATCOM/binl:$WATCOM/binw:$PATH"
export INCLUDE="$WATCOM/h"

cd "$PROJECT"

echo "Building AMCFG.EXE (simple single-port tool)..."
wcl -q -ms -bt=dos amcfg.c -fe=AMCFG.EXE
ls -lh AMCFG.EXE

echo "Building AMCONFIG.EXE (advanced multi-port scanner)..."
wcl -q -ms -bt=dos amconfig.c -fe=AMCONFIG.EXE
ls -lh AMCONFIG.EXE

echo "Done!"
ls -l AMCFG.EXE
