# Makefile

define HELP_MESSAGE
kscale-onshape-library

# Installing

1. Create a new Conda environment: `conda create --name kscale-onshape-library python=3.11`
2. Activate the environment: `conda activate kscale-onshape-library`
3. Install the package: `make install-dev`

# Running Tests

1. Run autoformatting: `make format`
2. Run static checks: `make static-checks`
3. Run unit tests: `make test`

endef
export HELP_MESSAGE

all:
	@echo "$$HELP_MESSAGE"
.PHONY: all

# ------------------------ #
#          Build           #
# ------------------------ #

install:
	@pip install --verbose -e .
.PHONY: install

install-dev:
	@pip install --verbose -e '.[dev]'
.PHONY: install

build-ext:
	@python setup.py build_ext --inplace
.PHONY: build-ext

clean:
	rm -rf build dist *.so **/*.so **/*.pyi **/*.pyc **/*.pyd **/*.pyo **/__pycache__ *.egg-info .eggs/ .ruff_cache/
.PHONY: clean

# ------------------------ #
#       Static Checks      #
# ------------------------ #


format:
	@isort --profile black kol
	@black kol
	@ruff format kol
	@isort kol
.PHONY: format

static-checks:
	@isort --profile black --check --diff kol
	@black --diff --check kol
	@ruff check kol
	@mypy --install-types --non-interactive kol
.PHONY: lint

mypy-daemon:
	@dmypy run -- $(py-files)
.PHONY: mypy-daemon

# ------------------------ #
#        Unit tests        #
# ------------------------ #

test:
	python -m pytest
.PHONY: test
