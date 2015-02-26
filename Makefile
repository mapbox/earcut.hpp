BUILDTYPE ?= Release

.PHONY: all
all: build/Makefile
	make -C build

build-%: build/Makefile
	make -C build $*

run-%: build-%
	build/$(BUILDTYPE)/$*

config.gypi:
	./configure

build/Makefile: config.gypi
	deps/run_gyp earcut.gyp -Iconfig.gypi --depth=. -Goutput_dir=. --generator-output=./build -f make

.PHONY: xcode
xcode: config.gypi
	deps/run_gyp earcut.gyp -Iconfig.gypi --depth=. -Goutput_dir=. --generator-output=./build -f xcode
	open build/earcut.xcodeproj

.PHONY: clean
clean:
	-rm config.gypi
	-rm -rf build
