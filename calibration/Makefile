
EXE=capture.out
objects= build/main.o

# change to installation folder of vimba
VIMBA_DIR = /opt/Vimba_2_1
# remember to copy  $(VIMBA_DIR)/VimbaCPP/DynamicLib/x86_64bit/*.so to /usr/lib
LIBS = -lVimbaCPP -lVimbaC

CXX = g++
CXXFLAGS = --std=c++11 -I$(VIMBA_DIR)


default: $(EXE)


build/%.o: ./%.cpp
	$(CXX) -c  $(CXXFLAGS) $^ -o $@

$(EXE): $(objects)
	$(CXX) $(CXXFLAGS) $(objects) -o $(EXE) $(LIBS)


clean:
	rm $(EXE) $(objects)
