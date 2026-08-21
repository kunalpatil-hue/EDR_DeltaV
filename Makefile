PY ?= python3

.PHONY: all build test validate sweep clean scenarios
all: build test

build:
	$(PY) sim/build.py

strict:
	$(PY) sim/build.py --clean --strict

test: build
	$(PY) -m pytest tests/ -q

validate: build
	$(PY) sim/validate.py

sweep: build
	$(PY) sim/validate.py --sweep --rates --seeds 5

scenarios:
	$(PY) sim/run_sim.py --list

clean:
	rm -rf build out __pycache__ sim/__pycache__ tests/__pycache__ .pytest_cache
