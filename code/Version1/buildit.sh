g++ -g3 -g -Wall -fsanitize=undefined,address main.cpp gl_frontEnd.cpp utils.cpp -lm -lGL -lglut -lpthread -o Test

./Test 45 40 8 10