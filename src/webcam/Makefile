CPP = g++

CFLAGS = -std=gnu++0x -DVR_FINAL_TEST -Wall -I..


all: test_util test_church_mathod

run: test_util test_church_method
	./test_util
	./test_church

clean:
	-rm test_util
	-rm test_church

test_util: test_util.cpp
	$(CPP) $(CFLAGS) $^ -o $@


test_church_method: test_church_method.cpp church_method_method.cpp
	$(CPP) $(CFLAGS) $^ -o $@
