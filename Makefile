.PHONY: check test test-simulation

check:
	uv run --no-project python -B scripts/check_repository.py

test: test-simulation

test-simulation:
	docker build --platform linux/amd64 -f docker/Dockerfile -t drone-navigation-tests .
	docker run --rm --platform linux/amd64 --network none --cap-drop ALL --security-opt no-new-privileges --memory 2g --cpus 2 -v "$(CURDIR):/project:ro" drone-navigation-tests
