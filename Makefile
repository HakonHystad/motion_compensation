#makefile for particle_filter

EXE=filter.out

VIMBA_DIR = /opt/Vimba_2_1
#VIMBA_DIR = /home/minion/Nedlastinger/Vimba_2_0
# For cuda 9 grab the lastest repo for eigen not release
KDL = -I/usr/local/include/eigen3 -lpthread  -lorocos-kdl

CU=nvcc
CUFLAGS = -D_FORCE_INLINES -std=c++11 -arch=sm_50 --maxrregcount=32 -O3  --use_fast_math -I$(VIMBA_DIR) -lboost_system -lboost_date_time -lVimbaCPP -lVimbaC $(KDL)#-g -G `pkg-config --cflags --libs opencv`
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
