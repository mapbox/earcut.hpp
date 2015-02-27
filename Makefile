BUILDTYPE ?= Release

.PHONY: all
all: build/test/Makefile
	make -C build/test

build-viz: build/viz/Makefile
	make -C build/viz viz

build-%:
	make -C build/test $*

run-viz: build-viz
	build/viz/$(BUILDTYPE)/viz

run-%: build-%
	build/test/$(BUILDTYPE)/$*

config.gypi:
	./configure

build/test/Makefile:
	deps/run_gyp earcut.gyp --depth=. -Goutput_dir=. --generator-output=./build/test -f make

build/viz/Makefile: config.gypi
	deps/run_gyp test/viz.gyp -Iconfig.gypi --depth=. -Goutput_dir=. --generator-output=./build/viz -f make

.PHONY: xcode
xcode: config.gypi
	deps/run_gyp earcut.gyp -Iconfig.gypi --depth=. -Goutput_dir=. --generator-output=./build -f xcode
	open build/earcut.xcodeproj

.PHONY: clean
clean:
	-rm -f config.gypi
	-rm -rf build
