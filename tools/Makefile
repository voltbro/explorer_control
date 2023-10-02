CODE = $(shell pwd)
PACKAGES = network_controller motor_controller utils.py

# Allow setting run level from cli
RUN_LEVEL = dev
L = RUN_LEVEL=$(RUN_LEVEL)
# Add source dir to pythonpath
PYTHONPATH = PYTHONPATH=./:$(CODE)
# Executables
PYTEST = $(PYTHONPATH) pytest
TEST = $(PYTEST) --verbosity=2 --showlocals --strict

SHELL = bash

.PHONY: lint format test test-failed test-cov validate

# Actions

test:
	$(TEST) --cov --cov-fail-under=95

test-failed:
	$(TEST) --last-failed

test-cov:
	$(TEST) --cov --cov-report html

lint:
	ruff check $(PACKAGES) tests
	black --skip-string-normalization --check $(PACKAGES) tests

format:
	isort $(CODE)
	black --skip-string-normalization $(CODE)

validate: lint test
