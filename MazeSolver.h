#ifndef MAZESOLVER_H
#define MAZESOLVER_H

#include <vector>
#include <set>
#include <mutex>

enum Direction {LEFT = 0, UP, RIGHT, DOWN};

typedef struct 
{
	Direction direction;
	unsigned int id;
} NewDeadEnd;

class MazeSolver
{
	private:
		static const int NUM_THREADS = 8;
		
	public:
		typedef std::vector<std::vector<int> > Maze; //Just to save some typing
		
		static std::vector<int> SolveMaze(Maze walls);
		static bool ValidatePath(int dimension, Maze walls, std::vector<int> path);
		static void findDeadEnds(std::stack<int>& outDeadEnds, Maze& walls);
		
		static std::mutex deadEndsMutex;
		static std::set<int> treatedDeadEnds;
};

#endif
