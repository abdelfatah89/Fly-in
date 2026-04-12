install:
	pip install -r requirements.txt

run:
	python3 fly-in.py map.txt

debug:
	python3 -m pdb fly-in.py map.txt

clean:
	rm -rf __pycache__ mypy_cache/ .mypy_cache/ */__pycache__ */mypy_cache/ */.mypy_cache/

lint:
	flake8 .
	mypy . --warn-return-any \
	--warn-unused-ignores \
	--ignore-missing-imports \
	--disallow-untyped-defs \
	--check-untyped-defs \
	--exclude trash

lint-strict:
	flake8 .
	mypy . --strict --exclude trash
