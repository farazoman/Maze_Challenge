#include "MazeGenerator.h"
#include "MazeSolver.h"

#include <cstdio>
#include <ctime>
#include <thread>
#include <cmath>
#include <cstdlib>

std::mutex MazeSolver::deadEndsMutex;
std::set<int> MazeSolver::treatedDeadEnds;
bool isIntersection(unsigned int cell, MazeSolver::Maze* walls);

//#define DEBUG
#ifdef DEBUG
	#define		D(...)	do { printf( __VA_ARGS__ ); fflush(stdout); } while (0)
#else
	#define		D(...) 	((void)0)
#endif

void backtrackFromDeadEnds(int i, int mazeDimension, MazeSolver::Maze* walls, 
				std::stack<int>* deadEnds, std::stack<NewDeadEnd>* locallyTreatedDeadEnds) 
{
	while(true) {
		//Try to obtain a dead end
		//D("%d: Obtaining lock...\n", i);
		MazeSolver::deadEndsMutex.lock();
		if(deadEnds->size() == 0) {
			//No more dead ends; stop the thread.
			MazeSolver::deadEndsMutex.unlock();
			//D("%d, Stopping thread, no more dead ends\n", i);
			break;
		}
		int deadEnd = deadEnds->top();
		deadEnds->pop();
		//D("%d: Working with dead end at %d\n", i, deadEnd);
		MazeSolver::deadEndsMutex.unlock();
		
		//Go back up the path to the first intersection
		Direction directionComingFrom; //Direction we come from, entering the current cell
		unsigned int currentCell = deadEnd;
		unsigned int previousCell = currentCell;
		while(!isIntersection(currentCell, walls)) {			
			//Could we avoid branching here?
			if(((*walls)[currentCell][0] == 0) && (previousCell != currentCell - 1)) {
				previousCell = currentCell;
				currentCell--;
				directionComingFrom = RIGHT;
			} else if(((*walls)[currentCell][1] == 0) && (previousCell != currentCell - mazeDimension)) {
				previousCell = currentCell;
				currentCell -= mazeDimension;
				directionComingFrom = DOWN;
			} else if(((*walls)[currentCell][2] == 0) && (previousCell != currentCell + 1)) {
				previousCell = currentCell;
				currentCell++;
				directionComingFrom = LEFT;
			} else if(((*walls)[currentCell][3] == 0) && (previousCell != currentCell + mazeDimension)) {
				previousCell = currentCell;
				currentCell += mazeDimension;
				directionComingFrom = UP;
			} else {
				D("%d: Couldn't move anywhere\n", i);
				exit(1); //Something wrong happened, we are surrounded by walls
			}
		}		
		
		//We are done with this dead end, move on to the next.
		NewDeadEnd nde = {directionComingFrom, currentCell};
		locallyTreatedDeadEnds->push(nde);
	}
}

bool isIntersection(unsigned int cell, MazeSolver::Maze* walls) {		
	//Consider the beginning and end as intersections
	if(cell == 0 || cell == walls->size() - 1)
		return true;
		
	return 	(((*walls)[cell][0] == 0) + 
			((*walls)[cell][1] == 0) +
			((*walls)[cell][2] == 0) +
			((*walls)[cell][3] == 0) > 2);
}

std::vector<int> MazeSolver::SolveMaze(Maze walls)
{
	bool isDeadEndsEmpty = false;
	std::stack<int> deadEnds;
	int mazeDimension = sqrt(walls.size());
	
	std::thread threads[MazeSolver::NUM_THREADS];
	std::stack<NewDeadEnd> newDeadEnds[MazeSolver::NUM_THREADS];
	
	if(deadEnds.empty())
		break;
	/* While we can, find dead ends through the maze. Backtrack from
	them to the nearest intersection and block that way. When there
	are no more dead ends to be found, we have a solution. */
	while(!isDeadEndsEmpty) {
		findDeadEnds(deadEnds, walls);
		D("Found %d dead ends\n", deadEnds.size());
	
		for(int i = 0 ; i < MazeSolver::NUM_THREADS ; i++) {
			threads[i] = std::thread(backtrackFromDeadEnds, i, mazeDimension, &walls, &deadEnds, newDeadEnds + i);
		}
		
		D("Threads started\n");
			
		for(int i = 0 ; i < MazeSolver::NUM_THREADS ; i++) {
			D("Joining thread %d... ", i);
			threads[i].join();
			D("Done\n");
		}
		
		//Is it worth it to multithread this too?
		for(int i = 0 ; i < MazeSolver::NUM_THREADS ; i++) {
			while(!newDeadEnds[i].empty()) {
				//Add "virtual walls" to the maze to block off the dead ends we came from.
				NewDeadEnd deadEnd = newDeadEnds[i].top();
				walls[deadEnd.id][deadEnd.direction] = -1;
				newDeadEnds[i].pop();
			}
		}
		
	}
	
	MazeSolver::treatedDeadEnds.clear();

	//Go through the maze to construct the solution.
	std::vector<int> path;
	unsigned int cell = 0, previous = 0;
	while(cell != walls.size() - 1) {
		path.push_back(cell);
		if((walls[cell][0] == 0) && (previous != cell - 1)) {
			previous = cell;
			cell--;
		} else if((walls[cell][1] == 0) && (previous != cell - mazeDimension)) {
			previous = cell;
			cell -= mazeDimension;
		} else if((walls[cell][2] == 0) && (previous != cell + 1)) {
			previous = cell;
			cell++;
		} else if((walls[cell][3] == 0) && (previous != cell + mazeDimension)) {
			previous = cell;
			cell += mazeDimension;
		} else {
			D("This solution is broken.\n");
			exit(1);
		}
	}
	
	path.push_back(walls.size() -1);
	return path;
}

void MazeSolver::findDeadEnds(std::stack<int>& outDeadEnds, Maze& walls)
{
	//Ignore first and last cell (start and end)
	for(unsigned int i = 1 ; i < walls.size() -1 ; i++) {
		if(	((walls[i][0] != 0) + 
			(walls[i][1] != 0) + 
			(walls[i][2] != 0) + 
			(walls[i][3] != 0)) > 2) {
			if(MazeSolver::treatedDeadEnds.find(i) == MazeSolver::treatedDeadEnds.end()) {
				outDeadEnds.push(i);
				MazeSolver::treatedDeadEnds.insert(i);
				//D("Found dead end at %d\n", i);
			}
		}
	}
}

// Validate the path for a maze
// Returns true if the path is valid, false otherwise
bool MazeSolver::ValidatePath(int dimension, Maze walls, std::vector<int> path)
{
	// Get the path length and total number of cells in a maze
	int pathLength = path.size();
	int totalCells = walls.size();

	// First simple check
	// Check the start and end cell
	if (path[0] != 0 || path[pathLength - 1] != totalCells - 1) {
		return false;
	}

	// Check along the path to see if it counters any walls
	for (int i = 0; i < pathLength - 1; i++) {
		// The difference of IDs between next cell and current cell
		// Used to determine the relative position of next cell
		int difference = path[i + 1] - path[i];
		
		if (difference == 1) {
			// The next cell is right to current cell and there is a wall to the right
			if (walls[path[i]][2] == 1) {
				return false;
			}
		} else if (difference == -1) {
			// The next cell is left to current cell and there is a wall to the left
			if (walls[path[i]][0] == 1) {
				return false;
			}
		} else if (difference == dimension) {
			// The next cell is lower to current cell
			if (walls[path[i]][3] == 1) {
				return false;
			}
		} else if (difference == 0 - dimension) {
			// The next cell is upper to current cell
			if (walls[path[i]][1] == 1) {
				return false;
			}
		} else {
			return false;
		}
	}

	// If the path passes validation then it is good
	return true;
}

int main(int argc,char *argv[])
{
	// Initialize random seed
	time_t seed = time(NULL);
	std::cout << "Seed: " << seed << std::endl;
	srand(seed);
	
	if(argc != 3) {
		std::cout << "Usage: maze <dimension> <tries>" << std::endl;
		exit(1);
	}
	// The dimension of the maze
	int dimension = atoi(argv[1]);
	int tries = atoi(argv[2]);
	
	if(dimension < 1 || tries < 1) {
		std::cout << "Invalid input, please retry" << std::endl;
		exit(1);
	}

	double totalDuration = 0.d;
	
	for(int i = 0 ; i < tries ; i++) {
		std::cout << "Starting run #" << (i + 1) << std::endl;
		// Generate walls for the maze given the dimension
		MazeSolver::Maze walls = MazeGenerator::GenerateMaze(dimension);

		#ifdef DEBUG
		for(unsigned int i = 0 ; i < walls.size() ; i++) {
			D("%d %d %d %d\n", walls[i][0], walls[i][1], walls[i][2], walls[i][3]);
		}
		D("\n");
		#endif
		
		std::clock_t startTime;
		startTime = std::clock();

		// Get the path that solves the maze
		std::vector<int> path = MazeSolver::SolveMaze(walls);

		double duration = (std::clock() - startTime) / (double) CLOCKS_PER_SEC;

		// Path validation
		if(!MazeSolver::ValidatePath(dimension, walls, path)) {
			std::cout << "Your solution for this run is invalid. Please check your algorithm." << std::endl;
			exit(1);
		}
		
		totalDuration += duration;
		std::cout << "Run #" << (i + 1) << " done." << std::endl;
	}
	
	std::cout << "Done! Your average time was " << (totalDuration / tries) << "s, over " 
				<< tries << " runs on mazes of dimension " << dimension << std::endl;

	return 0;
}
