PROJECT_FILES = $(shell find src -iname *.h -o -iname *.cpp -o -iname *.c)

CATKIN_MAKE = catkin_make -DSETUPTOOLS_DEB_LAYOUT=OFF

.PHONY: build rebuild lint format

rebuild:
	$(CATKIN_MAKE) rebuild_cache
	$(MAKE) -C build

build:
	$(CATKIN_MAKE)
	$(MAKE) -C build install

lint:
	ruff check src
	black --skip-string-normalization --check src
	clang-tidy-17

format:
	isort --dont-follow-links src
	black --skip-string-normalization src
	clang-format-17 -i $(PROJECT_FILES)
