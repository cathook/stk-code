CPP = g++

MACROS = -DVR_FINAL_TEST -DTRACKER_TEST $(OPTS)

CFLAGS = -std=gnu++0x -Wall -I.. $$(pkg-config --cflags --libs opencv) -pthread -lpthread

CPPS = $(shell ls *.cpp)


test: $(CPPS)
	$(CPP) $^ $(MACROS) $(CFLAGS) -o $@

run: test
	./test

clean:
	-rm test
