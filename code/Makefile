CC=mpic++
FLAGS = -O3 -Wall -std=c++11 -fPIC -fopenmp
INCS = -I$(BULLET_ROOT)/include/bullet -I/usr/include/bullet
LIBS = -L$(BULLET_ROOT)/lib -lBulletDynamics -lBulletCollision -lLinearMath -lBulletSoftBody -lsqlite3 -lglut -lGL -lGLU 

OFILES = GL_ShapeDrawer.o tools.o rnn.o driver.o main.o
GFILES = _GL_ShapeDrawer.o _tools.o _rnn.o _driver.o _main.o

evaluator: $(OFILES)
	$(CC) $(FLAGS) $(OFILES) $(LIBS) -o evaluator

g: $(GFILES)
	$(CC) $(FLAGS) $(GFILES) $(LIBS) -o gevaluator

_%.o: %.cpp %.h
	$(CC) -c -D GRAPHICS $(INCS) $(FLAGS) $< -o $@

%.o : %.cpp %.h
	$(CC) -c $(INCS) $(FLAGS) $< -o $@

_%.o : %.cpp
	$(CC) -c -D GRAPHICS $(INCS) $(FLAGS) $< -o $@

%.o : %.cpp
	$(CC) -c $(INCS) $(FLAGS) $< -o $@

clean:
	rm -rf *.o *~ runner evaluator gevaluator
