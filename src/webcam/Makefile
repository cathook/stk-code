all: test test_church

run: test test_church
	./test
	./test_church

clean:
	-rm test
	-rm test_church

test: test.cpp
	g++ -DVR_FINAL_TEST -Wall -I.. -std=gnu++0x $^ -o $@


test_church: test_church.cpp church_method.cpp
	g++ -DVR_FINAL_TEST -Wall -I.. -std=gnu++0x $^ -o $@


test_dot: test_dot.cpp
	g++ -DVR_FINAL_TEST -Wall -I.. $(pkg-config --cflags --libs) -std=gnu++0x $^ -o $@
  
