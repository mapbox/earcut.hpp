BUILDTYPE ?= Release

.PHONY: all
all: test
	@make -C build

build/Makefile:
	@deps/run_gyp earcut.gyp --depth=. -Goutput_dir=. --generator-output=./build -f make

.PHONY: test
test: build/Makefile
	@make -C build test

.PHONY: bench
bench: build/Makefile
	@make -C build bench

.PHONY: xcode
xcode:
	deps/run_gyp earcut.gyp --depth=. -Goutput_dir=. --generator-output=./build -f xcode
	open build/earcut.xcodeproj

.PHONY: clean
clean:
	rm -rf build
