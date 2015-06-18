CPP = g++

CFLAGS = -std=gnu++0x -DVR_FINAL_TEST -Wall -I..

CPPS = $(shell ls *.cpp)


test: $(CPPS)
	$(CPP) $(CFLAGS) -o $@ $^

run: test
	./test

clean:
	-rm test
