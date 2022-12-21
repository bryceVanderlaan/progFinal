g++ -g3 -g -Wall  main.cpp gl_frontEnd.cpp utils.cpp -lm -lGL -lglut -lpthread -o Test

#-fsanitize=undefined,address

./Test 45 40 1 10