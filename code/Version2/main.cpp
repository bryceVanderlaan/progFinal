//
//  main.c
//  Final Project CSC412
//
//  Created by Jean-Yves Hervé on 2020-12-01
//	This is public domain code.  By all means appropriate it and change is to your
//	heart's content.
#include <iostream>
#include <string>
#include <random>
#include <thread>
#include <vector>
#include "math.h"
#include <list>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <climits>
#include <bits/stdc++.h>
#include <unistd.h>
#include <mutex>
//
#include "gl_frontEnd.h"

//	feel free to "un-use" std if this is against your beliefs.
using namespace std;

// struct to hold relevant info for pathfinding algo
struct Cell
{
	bool isObstacle = false;
	bool visited = false;

	float fGlobalScore;
	float fLocalScore;

	unsigned int row;
	unsigned int col;

	vector<Cell *> neighbors;
	Cell *parent;
};

//==================================================================================
//	Function prototypes
//==================================================================================
void initializeApplication(void);
GridPosition getNewFreePosition(void);
Direction newDirection(Direction forbiddenDir = Direction::NUM_DIRECTIONS);
TravelerSegment newTravelerSegment(const TravelerSegment &currentSeg, bool &canAdd);
void initTravelers(unsigned int numTravelers, float **colorList);
void travelerThreadMain(unsigned int index);
void initCellInfo(Cell **cellInfo);
void findTheExit(const GridPosition &source, Cell **cellInfo);
vector<Cell> buildPath(const GridPosition &source, Cell **cellInfo);
void destroyCellInfo(Cell **cellInfo);
bool inBounds(unsigned int row, unsigned int col);
double calculateDist(const Cell *source, const Cell *destination);
bool atEndOfPath(unsigned int index);
void travelerUpdate(vector<Cell> &pathToExit, GridPosition &pos, unsigned int index);
void travelerExit(unsigned int index);
void generateWalls(void);
void generatePartitions(void);

//==================================================================================
//	Application-level global variables
//==================================================================================

//	Don't rename any of these variables
//-------------------------------------
//	The state grid and its dimensions (arguments to the program)
SquareType **grid;
unsigned int numRows = 0;	   //	height of the grid
unsigned int numCols = 0;	   //	width
unsigned int numTravelers = 0; //	initial number
unsigned int numTravelersDone = 0;
unsigned int numLiveThreads = 0; //	the number of live traveler threads
vector<Traveler> travelerList;
vector<SlidingPartition> partitionList;
GridPosition exitPos; //	location of the exit

unsigned int maximumTailLength = 1;
unsigned int stepsUntilAddSegment = 0;
thread **TravelerThreads;

//	travelers' sleep time between moves (in microseconds)
const int MIN_SLEEP_TIME = 1000;
int travelerSleepTime = 150000;

//	An array of C-string where you can store things you want displayed
//	in the state pane to display (for debugging purposes?)
//	Dont change the dimensions as this may break the front end
const int MAX_NUM_MESSAGES = 8;
const int MAX_LENGTH_MESSAGE = 32;
char **message;
time_t launchTime;

//	Random generators:  For uniform distributions
const unsigned int MAX_NUM_INITIAL_SEGMENTS = 6;
random_device randDev;
default_random_engine engine(randDev());
uniform_int_distribution<unsigned int> unsignedNumberGenerator(0, numeric_limits<unsigned int>::max());
uniform_int_distribution<unsigned int> segmentNumberGenerator(0, MAX_NUM_INITIAL_SEGMENTS);
uniform_int_distribution<unsigned int> segmentDirectionGenerator(0, static_cast<unsigned int>(Direction::NUM_DIRECTIONS) - 1);
uniform_int_distribution<unsigned int> headsOrTails(0, 1);
uniform_int_distribution<unsigned int> rowGenerator;
uniform_int_distribution<unsigned int> colGenerator;

//==================================================================================
//	These are the functions that tie the simulation with the rendering.
//	Some parts are "don't touch."  Other parts need your intervention
//	to make sure that access to critical section is properly synchronized
//==================================================================================

void drawTravelers(void)
{
	//-----------------------------
	//	You may have to sychronize things here
	//-----------------------------
	for (unsigned int k = 0; k < travelerList.size(); k++)
	{
		//	here I would test if the traveler thread is still live
		drawTraveler(travelerList[k]);
	}
}

void updateMessages(void)
{
	//	Here I hard-code a few messages that I want to see displayed
	//	in my state pane.  The number of live robot threads will
	//	always get displayed.  No need to pass a message about it.
	unsigned int numMessages = 4;
	sprintf(message[0], "We created %d travelers", numTravelers);
	sprintf(message[1], "%d travelers solved the maze", numTravelersDone);
	sprintf(message[2], "I like cheese");
	sprintf(message[3], "Simulation run time: %ld s", time(NULL) - launchTime);

	//---------------------------------------------------------
	//	This is the call that makes OpenGL render information
	//	about the state of the simulation.
	//
	//	You *must* synchronize this call.
	//---------------------------------------------------------
	drawMessages(numMessages, message);
}

void handleKeyboardEvent(unsigned char c, int x, int y)
{
	int ok = 0;

	switch (c)
	{
	//	'esc' to quit
	case 27:
		exit(0);
		break;

	//	slowdown
	case ',':
		slowdownTravelers();
		ok = 1;
		break;

	//	speedup
	case '.':
		speedupTravelers();
		ok = 1;
		break;

	default:
		ok = 1;
		break;
	}
	if (!ok)
	{
		//	do something?
	}
}

//------------------------------------------------------------------------
//	You shouldn't have to touch this one.  Definitely if you don't
//	add the "producer" threads, and probably not even if you do.
//------------------------------------------------------------------------
void speedupTravelers(void)
{
	//	decrease sleep time by 20%, but don't get too small
	int newSleepTime = (8 * travelerSleepTime) / 10;

	if (newSleepTime > MIN_SLEEP_TIME)
	{
		travelerSleepTime = newSleepTime;
	}
}

void slowdownTravelers(void)
{
	//	increase sleep time by 20%.  No upper limit on sleep time.
	//	We can slow everything down to admistrative pace if we want.
	travelerSleepTime = (12 * travelerSleepTime) / 10;
}

//------------------------------------------------------------------------
//	You shouldn't have to change anything in the main function besides
//	initialization of the various global variables and lists
//------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	//	We know that the arguments  of the program  are going
	//	to be the width (number of columns) and height (number of rows) of the
	//	grid, the number of travelers, etc.
	//	So far, I hard code-some values
	// KENNY HAS CHANGED THESE VALUES TO THE ARG VALUES
	numRows = stoi(argv[2]);
	numCols = stoi(argv[1]);
	numTravelers = stoi(argv[3]);
	TravelerThreads = new std::thread *[numTravelers];

	// If no 4th argument is given, then the maximum grow
	// distance of the traveler is equal to maximumTailLength == 1
	if (argc > 4)
	{
		stepsUntilAddSegment = stoi(argv[4]);
	}
	else
	{
		stepsUntilAddSegment = maximumTailLength;
	}

	numLiveThreads = 0;
	numTravelersDone = 0;

	//	Even though we extracted the relevant information from the argument
	//	list, I still need to pass argc and argv to the front-end init
	//	function because that function passes them to glutInit, the required call
	//	to the initialization of the glut library.
	initializeFrontEnd(argc, argv);

	//	Now we can do application-level initialization
	initializeApplication();

	launchTime = time(NULL);

	//	Now we enter the main loop of the program and to a large extend
	//	"lose control" over its execution.  The callback functions that
	//	we set up earlier will be called when the corresponding event
	//	occurs
	glutMainLoop();

	// Wait for threads to finish
	for (unsigned int i = 0; i < numTravelers; i++)
	{
		TravelerThreads[i]->join();
	}

	//	Free allocated resource before leaving (not absolutely needed, but
	//	just nicer.  Also, if you crash there, you know something is wrong
	//	in your code.
	for (unsigned int i = 0; i < numRows; i++)
		free(grid[i]);
	free(grid);
	for (int k = 0; k < MAX_NUM_MESSAGES; k++)
		free(message[k]);
	free(message);

	//	This will probably never be executed (the exit point will be in one of the
	//	call back functions).
	return 0;
}

//==================================================================================
//
//	This is a function that you have to edit and add to.
//
//==================================================================================

void initializeApplication(void)
{
	//	Initialize some random generators
	rowGenerator = uniform_int_distribution<unsigned int>(0, numRows - 1);
	colGenerator = uniform_int_distribution<unsigned int>(0, numCols - 1);

	//	Allocate the grid
	grid = new SquareType *[numRows];
	for (unsigned int i = 0; i < numRows; i++)
	{
		grid[i] = new SquareType[numCols];
		for (unsigned int j = 0; j < numCols; j++)
			grid[i][j] = SquareType::FREE_SQUARE;
	}

	message = new char *[MAX_NUM_MESSAGES];
	for (unsigned int k = 0; k < MAX_NUM_MESSAGES; k++)
		message[k] = new char[MAX_LENGTH_MESSAGE + 1];

	//---------------------------------------------------------------
	//	All the code below to be replaced/removed
	//	I initialize the grid's pixels to have something to look at
	//---------------------------------------------------------------
	//	Yes, I am using the C random generator after ranting in class that the C random
	//	generator was junk.  Here I am not using it to produce "serious" data (as in a
	//	real simulation), only wall/partition location and some color
	srand((unsigned int)time(NULL));

	//	generate a random exit
	exitPos = getNewFreePosition();
	grid[exitPos.row][exitPos.col] = SquareType::EXIT;

	//	Generate walls and partitions
	generateWalls();
	generatePartitions();

	float **travelerColor = createTravelerColors(numTravelers);

	initTravelers(numTravelers, travelerColor);

	// INITILIZE LIST OF THREADS
	// At this point, we need 1 thread for each traveler.
	for (unsigned int i = 0; i < numTravelers; i++)
	{
		TravelerThreads[i] = new thread(travelerThreadMain, i);
		numLiveThreads += 1;
	}

	// free array of colors
	for (unsigned int k = 0; k < numTravelers; k++)
	{
		delete[] travelerColor[k];
	}
	delete[] travelerColor;
}

//------------------------------------------------------
#if 0
#pragma mark -
#pragma mark Generation Helper Functions
#endif
//------------------------------------------------------

GridPosition getNewFreePosition(void)
{
	GridPosition pos;

	bool noGoodPos = true;
	while (noGoodPos)
	{
		unsigned int row = rowGenerator(engine);
		unsigned int col = colGenerator(engine);
		if (grid[row][col] == SquareType::FREE_SQUARE)
		{
			pos.row = row;
			pos.col = col;
			noGoodPos = false;
		}
	}
	return pos;
}

Direction newDirection(Direction forbiddenDir)
{
	bool noDir = true;

	Direction dir = Direction::NUM_DIRECTIONS;
	while (noDir)
	{
		dir = static_cast<Direction>(segmentDirectionGenerator(engine));
		noDir = (dir == forbiddenDir);
	}
	return dir;
}

TravelerSegment newTravelerSegment(const TravelerSegment &currentSeg, bool &canAdd)
{
	TravelerSegment newSeg;
	switch (currentSeg.dir)
	{
	case Direction::NORTH:
		if (currentSeg.row < numRows - 1 &&
			grid[currentSeg.row + 1][currentSeg.col] == SquareType::FREE_SQUARE)
		{
			newSeg.row = currentSeg.row + 1;
			newSeg.col = currentSeg.col;
			newSeg.dir = newDirection(Direction::SOUTH);
			grid[newSeg.row][newSeg.col] = SquareType::TRAVELER;
			canAdd = true;
		}
		//	no more segment
		else
			canAdd = false;
		break;

	case Direction::SOUTH:
		if (currentSeg.row > 0 &&
			grid[currentSeg.row - 1][currentSeg.col] == SquareType::FREE_SQUARE)
		{
			newSeg.row = currentSeg.row - 1;
			newSeg.col = currentSeg.col;
			newSeg.dir = newDirection(Direction::NORTH);
			grid[newSeg.row][newSeg.col] = SquareType::TRAVELER;
			canAdd = true;
		}
		//	no more segment
		else
			canAdd = false;
		break;

	case Direction::WEST:
		if (currentSeg.col < numCols - 1 &&
			grid[currentSeg.row][currentSeg.col + 1] == SquareType::FREE_SQUARE)
		{
			newSeg.row = currentSeg.row;
			newSeg.col = currentSeg.col + 1;
			newSeg.dir = newDirection(Direction::EAST);
			grid[newSeg.row][newSeg.col] = SquareType::TRAVELER;
			canAdd = true;
		}
		//	no more segment
		else
			canAdd = false;
		break;

	case Direction::EAST:
		if (currentSeg.col > 0 &&
			grid[currentSeg.row][currentSeg.col - 1] == SquareType::FREE_SQUARE)
		{
			newSeg.row = currentSeg.row;
			newSeg.col = currentSeg.col - 1;
			newSeg.dir = newDirection(Direction::WEST);
			grid[newSeg.row][newSeg.col] = SquareType::TRAVELER;
			canAdd = true;
		}
		//	no more segment
		else
			canAdd = false;
		break;

	default:
		canAdd = false;
	}

	return newSeg;
}

void initTravelers(unsigned int numTravelers, float **colorList)
{

	for (unsigned int i = 0; i < numTravelers; i++)
	{
		//	Initialize traveler info structs
		GridPosition pos = getNewFreePosition();
		//	Note that treating an enum as a sort of integer is increasingly
		//	frowned upon, as C++ versions progress
		Direction dir = static_cast<Direction>(segmentDirectionGenerator(engine));

		TravelerSegment seg = {pos.row, pos.col, dir};
		Traveler traveler;
		traveler.startingPos = pos;
		traveler.segmentList.push_back(seg);
		grid[pos.row][pos.col] = SquareType::TRAVELER;

		//	Changed to make it so argv[4] segments will be added to the travelers,
		// and for no argv[4] you cannot add any segments to the tail

		bool canAddSegment;
		unsigned int numAddSegments = segmentNumberGenerator(engine);
		TravelerSegment currSeg = traveler.segmentList[0];

		if (stepsUntilAddSegment > 1)
		{
			// cout << "Grow Tail Distance: " << stepsUntilAddSegment << endl;
			canAddSegment = true;
		}
		else
		{
			// cout << "Grow Tail Distance: " << stepsUntilAddSegment << endl;
			canAddSegment = false;
		}

		// cout << "Traveler " << index << " at (row=" << pos.row << ", col=" <<
		//	pos.col << "), direction: " << dirStr(dir) << ", with up to " << numAddSegments << " additional segments" << endl;
		// cout << "End position is: (row=" << exitPos.row << ", col=" << exitPos.col << ")\n";

		for (unsigned int c = 0; c < 4; c++)
		{
			traveler.rgba[c] = colorList[i][c];
		}

		// add the traveler to the traveler list
		travelerList.push_back(traveler);

		for (unsigned int s = 0; s < numAddSegments && canAddSegment; s++)
		{
			TravelerSegment newSeg = newTravelerSegment(currSeg, canAddSegment);
			if (canAddSegment)
			{
				travelerList[i].segmentList.push_back(newSeg);
				currSeg = newSeg;
			}
		}
	}
}

void travelerThreadMain(unsigned int index)
{

	// create a matrix of cells the same size as the grid to find
	// the path to the exit
	Cell **cellInfo = new Cell *[numRows];
	for (unsigned int i = 0; i < numRows; i++)
	{
		cellInfo[i] = new Cell[numCols];
	}

	// initialize all the cell structs in the matrix
	initCellInfo(cellInfo);

	GridPosition pos = travelerList[index].startingPos;

	// pass the starting position and the grid to find the a path to the exit point
	findTheExit(pos, cellInfo);

	// build the path to the exit and store it in a vector
	vector<Cell> pathToExit = buildPath(pos, cellInfo);

	// deallocate the Cell matrix
	destroyCellInfo(cellInfo);

	// the path will always come back in reverse order, we need to reverse it
	reverse(pathToExit.begin(), pathToExit.end());

	// update the traveler until it reaches the exit point
	travelerUpdate(pathToExit, pos, index);

	// update the traveler until every segment has reached the exit point
	travelerExit(index);

	// Traveler Finished Maze
	numTravelersDone += 1;

	// Thread is dead after this
	numLiveThreads -= 1;
}

void initCellInfo(Cell **cellInfo)
{
	// init each cell in the matrix
	for (unsigned int i = 0; i < numRows; i++)
	{
		for (unsigned int j = 0; j < numCols; j++)
		{
			// assign row and col
			cellInfo[i][j].row = i;
			cellInfo[i][j].col = j;

			// if the square type at this index on the grid is of
			// type FREE_SQUARE, it is not an obstacle
			if (grid[i][j] == SquareType::FREE_SQUARE)
			{
				cellInfo[i][j].isObstacle = false;
			}
			else
			{
				// otherwise it is
				cellInfo[i][j].isObstacle = true;
			}

			// these are max unsigned int by default since any
			// euclidian distances between neighbors or the end pt will be less
			cellInfo[i][j].fLocalScore = UINT_MAX;
			cellInfo[i][j].fGlobalScore = UINT_MAX;

			// by default, no cell has a parent and no cell has been visited
			cellInfo[i][j].parent = nullptr;
			cellInfo[i][j].visited = false;

			// four possible moves (NSEW)
			int moveRow[] = {1, -1, 0, 0};
			int moveCol[] = {0, 0, 1, -1};

			for (int k = 0; k < 4; k++)
			{
				unsigned int neighborRow = i + moveRow[k];
				unsigned int neighborCol = j + moveCol[k];

				// if the current neighbor does not go OOB
				if (inBounds(neighborRow, neighborCol))
				{
					// add it to the vector of neighbors
					cellInfo[i][j].neighbors.push_back(&(cellInfo[neighborRow][neighborCol]));
				}
				else
				{
					// otherwise skip it
					continue;
				}
			}
		}
	}
}

void findTheExit(const GridPosition &source, Cell **cellInfo)
{
	// to start, the current cell is the starting position of the traveler
	Cell *currCell = &(cellInfo[source.row][source.col]);
	// end position will remain constant
	Cell *endCell = &(cellInfo[exitPos.row][exitPos.col]);
	// set local score to zero since we have not found the distance for any neighbors yet
	currCell->fLocalScore = 0.0f;
	// calulate euclidian distance between starting cell and end cell
	currCell->fGlobalScore = calculateDist(currCell, endCell);

	// init a list of Cell pointers to keep track of the cells
	list<Cell *> untestedCells;
	// the starting cell is the first cell in the list
	untestedCells.push_back(currCell);

	// as long as there are elements in the list and we have not yet reached the end
	while (!untestedCells.empty() && currCell != endCell)
	{
		// sort the list in order of lowest global score
		//(this represents the lowest euclidian distance between an element and the end cell)
		untestedCells.sort([](const Cell *a, const Cell *b)
						   { return a->fGlobalScore < b->fGlobalScore; });

		// get rid of any cells that have already been evaluated
		while (!untestedCells.empty() && untestedCells.front()->visited)
		{
			untestedCells.pop_front();
		}

		// stop the loop if there are no more cells to process
		if (untestedCells.empty())
		{
			break;
		}

		// due to the sort, we can safely assume that the first element in the list
		// is the next best move
		currCell = untestedCells.front();
		// we mark this cell as visited, so as not to process it twice
		currCell->visited = true;

		// assess each neighbor of the current cell
		for (auto neighbor : currCell->neighbors)
		{
			// if it hasn't been assessed yet and it's not an obstacle
			if (!neighbor->visited && !neighbor->isObstacle)
			{
				// put it in the list to test at some point
				untestedCells.push_back(neighbor);
			}

			float potentiallyBetterLocalScore = currCell->fLocalScore + calculateDist(currCell, neighbor);

			// if the euclidian distance between the current cell and the current neighbor is less
			// than the current neigbor's local score
			if (potentiallyBetterLocalScore < neighbor->fLocalScore)
			{
				// the current cell is now the parent of this neighbor
				neighbor->parent = currCell;
				neighbor->fLocalScore = potentiallyBetterLocalScore;
				neighbor->fGlobalScore = neighbor->fLocalScore + calculateDist(neighbor, endCell);
			}
		}
	}
}

vector<Cell> buildPath(const GridPosition &source, Cell **cellInfo)
{
	// init and empty vector
	vector<Cell> pathToExit{};
	// get pointers to the starting cell and ending cell
	Cell *currCell = &(cellInfo[exitPos.row][exitPos.col]);
	Cell *startCell = &(cellInfo[source.row][source.col]);
	// as long as the current cell is not the start of the path
	while (currCell != startCell)
	{
		// add that cell to the path
		pathToExit.push_back(*currCell);
		// move to the parent of the current cell
		currCell = currCell->parent;
	}

	// return the full path in backwards order
	return pathToExit;
}

void destroyCellInfo(Cell **cellInfo)
{
	for (unsigned int i = 0; i < numRows; i++)
	{
		delete[] cellInfo[i];
	}
	delete[] cellInfo;
}

bool inBounds(unsigned int row, unsigned int col)
{
	// returns true if the index provided as argument is within the grid
	return (row >= 0 && row < numRows && col >= 0 && col < numCols);
}

double calculateDist(const Cell *source, const Cell *destination)
{
	// pythagorean theorem for distance (aka Euclidian aka as the crow flies)
	return sqrtf((source->row - destination->row) * (source->row - destination->row) + (source->col - destination->col) * (source->col - destination->col));
}

bool atEndOfPath(unsigned int index)
{
	return (travelerList[index].segmentList[0].row == exitPos.row && travelerList[index].segmentList[0].col == exitPos.col);
}

void travelerUpdate(vector<Cell> &pathToExit, GridPosition &pos, unsigned int index)
{
	unsigned int stepsTaken = 0;

	TravelerSegment currSeg = travelerList[index].segmentList[0];
	bool canAddSegment;

	if (stepsUntilAddSegment > 1)
	{
		// cout << "Every " << stepsUntilAddSegment << " steps, add a segment.\n";
		canAddSegment = true;
	}
	else
	{
		// cout << "Every " << stepsUntilAddSegment << " steps, add a segment.\n";
		canAddSegment = false;
	}

	while (stepsTaken < pathToExit.size())
	{
		usleep(travelerSleepTime);

		if (stepsTaken % stepsUntilAddSegment == 0 && !atEndOfPath(index))
		{
			TravelerSegment newSeg = newTravelerSegment(currSeg, canAddSegment);
			travelerList[index].segmentList.push_back(newSeg);
			currSeg = newSeg;
		}

		// cout << "Traveler 1 is at position: (" << pos.row << ", " << pos.col << ")\n";
		// cout << "Now moving to position: (" << pathToExit[stepsTaken].row << ", " << pathToExit[stepsTaken].col << ")\n";
		// cout << "Traveler Size is: " << travelerList[index].segmentList.size() << endl;
		// cout << "=======================================================================\n";

		Direction nextMoveDirection;
		// look at current position, and then compare it to the next position to get direction
		if (pathToExit[stepsTaken].row < pos.row)
		{
			// This is a north move
			nextMoveDirection = Direction::NORTH;
		}
		else if (pathToExit[stepsTaken].row > pos.row)
		{
			// This is a south move
			nextMoveDirection = Direction::SOUTH;
		}
		else if (pathToExit[stepsTaken].col < pos.col)
		{
			// This is a West Move
			nextMoveDirection = Direction::WEST;
		}
		else if (pathToExit[stepsTaken].col > pos.col)
		{
			// This is an EAST move
			nextMoveDirection = Direction::EAST;
		}
		// We calculate the new direction for the head of the traveler

		// Next, if The traveler is going to have more than 1 segment,
		// Then each segment after gets the segment ahead of its direction
		// so segment 1 would get the direction of segment 0
		for (unsigned int i = travelerList[index].segmentList.size() - 1; i > 0; i--)
		{
			travelerList[index].segmentList[i].dir = travelerList[index].segmentList[i - 1].dir;
		}

		travelerList[index].segmentList[0].dir = nextMoveDirection;
		travelerList[index].segmentList[0].row = pathToExit[stepsTaken].row;
		travelerList[index].segmentList[0].col = pathToExit[stepsTaken].col;
		pos.row = pathToExit[stepsTaken].row;
		pos.col = pathToExit[stepsTaken].col;

		drawTraveler(travelerList[index]);
		stepsTaken++;
	}
}

void travelerExit(unsigned int index)
{
	// Loop for tail to disappear
	// while there is at least one segment in the list
	while (!travelerList[index].segmentList.empty())
	{
		usleep(travelerSleepTime);

		// create a temporary list that holds the current state of the segmentList
		vector<TravelerSegment> tempSegmentList = travelerList[index].segmentList;

		// each element of the segment list gets the coords of the element in front of it
		for (size_t i = 1; i < tempSegmentList.size(); i++)
		{
			travelerList[index].segmentList[i] = tempSegmentList[i - 1];
		}

		// remove the front
		travelerList[index].segmentList.erase(travelerList[index].segmentList.begin());

		drawTraveler(travelerList[index]);
	}
}

void generateWalls(void)
{
	const unsigned int NUM_WALLS = (numCols + numRows) / 4;

	//	I decide that a wall length  cannot be less than 3  and not more than
	//	1/4 the grid dimension in its Direction
	const unsigned int MIN_WALL_LENGTH = 3;
	const unsigned int MAX_HORIZ_WALL_LENGTH = numCols / 3;
	const unsigned int MAX_VERT_WALL_LENGTH = numRows / 3;
	const unsigned int MAX_NUM_TRIES = 20;

	bool goodWall = true;

	//	Generate the vertical walls
	for (unsigned int w = 0; w < NUM_WALLS; w++)
	{
		goodWall = false;

		//	Case of a vertical wall
		if (headsOrTails(engine))
		{
			//	I try a few times before giving up
			for (unsigned int k = 0; k < MAX_NUM_TRIES && !goodWall; k++)
			{
				//	let's be hopeful
				goodWall = true;

				//	select a column index
				unsigned int HSP = numCols / (NUM_WALLS / 2 + 1);
				unsigned int col = (1 + unsignedNumberGenerator(engine) % (NUM_WALLS / 2 - 1)) * HSP;
				unsigned int length = MIN_WALL_LENGTH + unsignedNumberGenerator(engine) % (MAX_VERT_WALL_LENGTH - MIN_WALL_LENGTH + 1);

				//	now a random start row
				unsigned int startRow = unsignedNumberGenerator(engine) % (numRows - length);
				for (unsigned int row = startRow, i = 0; i < length && goodWall; i++, row++)
				{
					if (grid[row][col] != SquareType::FREE_SQUARE)
						goodWall = false;
				}

				//	if the wall first, add it to the grid
				if (goodWall)
				{
					for (unsigned int row = startRow, i = 0; i < length && goodWall; i++, row++)
					{
						grid[row][col] = SquareType::WALL;
					}
				}
			}
		}
		// case of a horizontal wall
		else
		{
			goodWall = false;

			//	I try a few times before giving up
			for (unsigned int k = 0; k < MAX_NUM_TRIES && !goodWall; k++)
			{
				//	let's be hopeful
				goodWall = true;

				//	select a column index
				unsigned int VSP = numRows / (NUM_WALLS / 2 + 1);
				unsigned int row = (1 + unsignedNumberGenerator(engine) % (NUM_WALLS / 2 - 1)) * VSP;
				unsigned int length = MIN_WALL_LENGTH + unsignedNumberGenerator(engine) % (MAX_HORIZ_WALL_LENGTH - MIN_WALL_LENGTH + 1);

				//	now a random start row
				unsigned int startCol = unsignedNumberGenerator(engine) % (numCols - length);
				for (unsigned int col = startCol, i = 0; i < length && goodWall; i++, col++)
				{
					if (grid[row][col] != SquareType::FREE_SQUARE)
						goodWall = false;
				}

				//	if the wall first, add it to the grid
				if (goodWall)
				{
					for (unsigned int col = startCol, i = 0; i < length && goodWall; i++, col++)
					{
						grid[row][col] = SquareType::WALL;
					}
				}
			}
		}
	}
}

void generatePartitions(void)
{
	const unsigned int NUM_PARTS = (numCols + numRows) / 4;

	//	I decide that a partition length  cannot be less than 3  and not more than
	//	1/4 the grid dimension in its Direction
	const unsigned int MIN_PARTITION_LENGTH = 3;
	const unsigned int MAX_HORIZ_PART_LENGTH = numCols / 3;
	const unsigned int MAX_VERT_PART_LENGTH = numRows / 3;
	const unsigned int MAX_NUM_TRIES = 20;

	bool goodPart = true;

	for (unsigned int w = 0; w < NUM_PARTS; w++)
	{
		goodPart = false;

		//	Case of a vertical partition
		if (headsOrTails(engine))
		{
			//	I try a few times before giving up
			for (unsigned int k = 0; k < MAX_NUM_TRIES && !goodPart; k++)
			{
				//	let's be hopeful
				goodPart = true;

				//	select a column index
				unsigned int HSP = numCols / (NUM_PARTS / 2 + 1);
				unsigned int col = (1 + unsignedNumberGenerator(engine) % (NUM_PARTS / 2 - 2)) * HSP + HSP / 2;
				unsigned int length = MIN_PARTITION_LENGTH + unsignedNumberGenerator(engine) % (MAX_VERT_PART_LENGTH - MIN_PARTITION_LENGTH + 1);

				//	now a random start row
				unsigned int startRow = unsignedNumberGenerator(engine) % (numRows - length);
				for (unsigned int row = startRow, i = 0; i < length && goodPart; i++, row++)
				{
					if (grid[row][col] != SquareType::FREE_SQUARE)
						goodPart = false;
				}

				//	if the partition is possible,
				if (goodPart)
				{
					//	add it to the grid and to the partition list
					SlidingPartition part;
					part.isVertical = true;
					for (unsigned int row = startRow, i = 0; i < length && goodPart; i++, row++)
					{
						grid[row][col] = SquareType::VERTICAL_PARTITION;
						GridPosition pos = {row, col};
						part.blockList.push_back(pos);
					}
				}
			}
		}
		// case of a horizontal partition
		else
		{
			goodPart = false;

			//	I try a few times before giving up
			for (unsigned int k = 0; k < MAX_NUM_TRIES && !goodPart; k++)
			{
				//	let's be hopeful
				goodPart = true;

				//	select a column index
				unsigned int VSP = numRows / (NUM_PARTS / 2 + 1);
				unsigned int row = (1 + unsignedNumberGenerator(engine) % (NUM_PARTS / 2 - 2)) * VSP + VSP / 2;
				unsigned int length = MIN_PARTITION_LENGTH + unsignedNumberGenerator(engine) % (MAX_HORIZ_PART_LENGTH - MIN_PARTITION_LENGTH + 1);

				//	now a random start row
				unsigned int startCol = unsignedNumberGenerator(engine) % (numCols - length);
				for (unsigned int col = startCol, i = 0; i < length && goodPart; i++, col++)
				{
					if (grid[row][col] != SquareType::FREE_SQUARE)
						goodPart = false;
				}

				//	if the wall first, add it to the grid and build SlidingPartition object
				if (goodPart)
				{
					SlidingPartition part;
					part.isVertical = false;
					for (unsigned int col = startCol, i = 0; i < length && goodPart; i++, col++)
					{
						grid[row][col] = SquareType::HORIZONTAL_PARTITION;
						GridPosition pos = {row, col};
						part.blockList.push_back(pos);
					}
				}
			}
		}
	}
}