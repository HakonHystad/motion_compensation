#makefile for particle_filter

EXE=filter.out

CU=nvcc
CXX = g++
CXXFLAGS = --std=c++11 -O3
CUFLAGS = -D_FORCE_INLINES -std=c++11 -arch=sm_50 --maxrregcount=32 -O3 `pkg-config --cflags --libs opencv` --use_fast_math #-g -G
objects= build/main.o build/filter.o

default: $(EXE)


build/main.o: ./main.cu
	$(CU) -dc  $(CUFLAGS) $^ -o $@	

build/%.o: ./filter/%.cu
	$(CU) -dc  $(CUFLAGS) $^ -o $@

$(EXE): $(objects)
	$(CU) $(CUFLAGS) $(objects) -o $(EXE) -lcudadevrt


clean:
	rm $(EXE) $(objects)
