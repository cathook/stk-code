CPP = g++

CFLAGS = -std=gnu++0x -DVR_FINAL_TEST -Wall -I.. $$(pkg-config --cflags --libs opencv)

CPPS = $(shell ls *.cpp)


test: $(CPPS)
	$(CPP) $^ $(CFLAGS) -o $@

run: test
	./test

clean:
	-rm test
