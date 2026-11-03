# Build entry points delegate to the portable scripts shipped with this package.
SHELL := /bin/bash
HERE := $(CURDIR)/
.PHONY: all sender receiver capture pattern check
all: receiver
sender:
	bash "$(HERE)../../../build/build_ui.sh" sender
receiver:
	bash "$(HERE)../../../build/build_ui.sh" receiver
capture: sender
pattern:
	bash "$(HERE)../../../build/build_ui.sh" base
check:
	python3 "$(HERE)../../../build/check_project.py"
