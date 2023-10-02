PYTHON=python3

_SETUP = set -e ; source setup.sh
SHELL = bash

.PHONY: help run-script command

run-script:  ## Меню запуска скриптов
	$(PYTHON) -m scripts $(SCRIPT)

compile-types:  ## Компиляция dsdl
	$(PYTHON) ./scripts/dev/compile.py

build-drivers:  ## Собрать все пакеты драйверов
	{ \
	$(_SETUP);\
	cd external/drivers; \
	make build;\
	}

help:  ## Показать это сообщение
	@egrep -h '\s##\s' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'

# --------------------------------------------------------------------------- #
# -----                       СПЕЦИАЛЬНЫЕ КОМАНДЫ                       ----- #
# --------------------------------------------------------------------------- #

tools-test:  ## Прогнать тесты для скриптов в tools
	{ \
	$(_SETUP) ;\
	$(MAKE) -C tools test;\
	}

tools-format:  ## Отформатировать код в tools
	{ \
	$(_SETUP) ;\
	$(MAKE) -C tools format;\
	}
