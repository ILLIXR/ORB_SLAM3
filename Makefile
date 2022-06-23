
CXX := clang++-10
CC := clang-10

.PHONY: plugin.dbg.so
plugin.dbg.so: build/Debug/Makefile
	./build.sh && \
	make -C build/Debug  && \
	rm -f $@ && \
	ln -s lib/libplugin.so plugin.dbg.so && \
	true

.PHONY: plugin.opt.so
plugin.opt.so: build/Release/Makefile
	./build.sh && \
	make -C build/Release  && \
	rm -f $@ && \
	ln -s lib/libplugin.so plugin.opt.so && \
	true

build/Debug/Makefile:
	mkdir -p build/Debug && \
	cd build/Debug && \
	cmake -DCMAKE_BUILD_TYPE=Debug  -DCMAKE_CXX_COMPILER=$(CXX) -DCMAKE_C_COMPILER=$(CC) ../.. && \
	true

build/Release/Makefile:
	mkdir -p build/Release && \
	cd build/Release && \
	cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_COMPILER=$(CXX) -DCMAKE_C_COMPILER=$(CC) ../.. && \
	true

tests/run:
tests/gdb:

.PHONY: clean
clean:
	touch build && rm -rf build *.so
