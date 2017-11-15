/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/

#include <math.h>   
#include "mex.h"
#include <queue>
#include <unordered_map>

/* Input Arguments */
#define	BLOCKSV_IN      prhs[0]
#define	TRIANGLESV_IN      prhs[1]
#define	TABLEINDEX_IN      prhs[2]
#define	ONVSTART_IN      prhs[3]
#define	CLEARVSTART_IN      prhs[4]
#define	ONVGOAL_IN      prhs[5]
#define	CLEARVGOAL_IN      prhs[6]
#define	MOVEACTIONINDEX_IN      prhs[7]
#define	MOVETOTABLEACTIONINDEX_IN      prhs[8]

using namespace std;

/* Output Arguments */
#define	PLAN_OUT	plhs[0]

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

struct State {

    State* parent;
    int* on;
    bool* clear;
    int id;
    int numItems;
    int gValue;
    int hValue;
    double weight;

    int item;
    int from;
    int to;

    // Use this to create successor states
    State(State* parent, int item, int from, int to,
        int** onV_goal, int onV_goal_length,
        int* clearV_goal, int numofclear_goal, double weight=1.0) {

        this->parent = parent;
        this->numItems = parent->numItems;
        this->on = (int*) malloc(numItems * sizeof(int));
        this->clear = (bool*) malloc(numItems * sizeof(bool));
        memcpy(this->on, parent->on, numItems * sizeof(int));
        memcpy(this->clear, parent->clear, numItems * sizeof(bool));

        on[item] = to;
        if (from != numItems) {
            clear[from] = true;
        }
        if (to != numItems) {
            clear[to] = false;
        }

        this->setId();
        this->gValue = this->parent->gValue + 1;
        this->weight = weight;
        this->computeHValue(onV_goal, onV_goal_length, clearV_goal, numofclear_goal);

        this->from = from;
        this->to = to;
        this->item = item;
    }

    // Use this to create start state
    State(int numofblocks, int numoftriangles,
        int** onV, int onV_length,
        int* clearV, int numofclear, double weight=1.0) {
        this->numItems = numofblocks + numoftriangles;
        this->on = (int*) malloc((numofblocks + numoftriangles) * sizeof(int));
        this->clear = (bool*) malloc((numofblocks + numoftriangles) * sizeof(bool));
        this->setId();
        this->setOn(onV, onV_length);
        this->setClear(clearV, numofclear);

        this->gValue = 0;
        this->weight = weight;
        this->hValue = 1;
    }

    bool isGoal(int** onV_goal, int onV_goal_length, int* clearV_goal, int numofclear_goal) {

        for (int i = 0; i < onV_goal_length; i++) {
            if (this->on[onV_goal[i][0]] != onV_goal[i][1])
                return false;
        }
        
        for (int i = 0; i < numofclear_goal; i++) {
            if (this->clear[clearV_goal[i]] != true)
                return false;
        }
        return true;
    }

    void computeHValue(int** onV_goal, int onV_goal_length, int* clearV_goal, int numofclear_goal) {
        this->hValue = 0;
        for (int i = 0; i < onV_goal_length; i++) {
            if (this->on[onV_goal[i][0]] != onV_goal[i][1])
                this->hValue++;
        }
        for (int i = 0; i < numofclear_goal; i++) {
            if (this->clear[clearV_goal[i]] != true)
                this->hValue++;
        }
    }

    int getF() {
        return this->gValue + (this->weight * this->hValue);
    }

    void setId() {
        for (int i = 0; i < this->numItems; i++) {
            this->id += (on[i] * pow(this->numItems, i));
        }
    }

    void setOn(int** onV, int onV_length) {
        for (int i = 0; i < this->numItems; i++) {
            this->on[i] = this->numItems;
        }
        for (int i = 0; i < onV_length; i++) {
            this->on[onV[i][0]] = onV[i][1];
        }
    }

    void setClear(int* clearV, int numofclear) {
        for (int i = 0; i < this->numItems; i++) {
            this->clear[i] = false;
        }
        for (int i = 0; i < numofclear; i++) {
            this->clear[clearV[i]] = true;
        }
    }
};

struct CompareState {

  bool operator()(State *thisState, State *otherState) const {
    if(thisState->getF()==otherState->getF())
    {
        return thisState->hValue > otherState->hValue;
    }
    return thisState->getF() > otherState->getF();
  }

};

static void planner(int* blocksV, int numofblocks, int* trianglesV, int numoftriangles, int TableIndex, 
            int** onV_start, int onV_start_length, int* clearV_start, int numofclear_start, 
            int** onV_goal, int onV_goal_length, int* clearV_goal, int numofclear_goal,
            int moveActionIndex, int moveToTableActionIndex, int*** plan, int* planlength) 
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;

    priority_queue<State*, std::vector<State*>, CompareState> queue;
    std::unordered_map <int, bool> closed;

    bool isTriangle[numoftriangles + numofblocks] = {false};
    for(int i = 0; i < numoftriangles; i++) {
        isTriangle[trianglesV[i]] = true;
    }
    
    // *planlength = numofsteps;
    State* startState = new State(numofblocks, numoftriangles, onV_start, onV_start_length, clearV_start, numofclear_start);

    queue.push(startState);
    State* currState;
    State* nextState;

    while(true) {

        currState = queue.top();
        queue.pop();
        if (closed[currState->id])
            continue;
        closed[currState->id] = true;

        if(currState->isGoal(onV_goal, onV_goal_length, clearV_goal, numofclear_goal))
            break;

        // Two rules to get successors:
        
        // Rule # 1: Any item that is clear can be moved to table, unless it's already on table
        for (int i = 0; i < currState->numItems; i++) {
            if (currState->clear[i] && currState->on[i] != currState->numItems) {
                nextState = new State(currState, i, currState->on[i], currState->numItems, onV_goal, onV_goal_length, clearV_goal, numofclear_goal);
                if(closed.find(nextState->id) == closed.end())
                    queue.push(nextState);
            }
        }
        // Rule # 2: Any item that is clear can be moved to another clear block
        for (int i = 0; i < currState->numItems; i++) {
            if (currState->clear[i]) {
                for (int j = 0; j < currState->numItems; j++) {
                    if (currState->clear[j] && i != j && !isTriangle[j]) {
                        nextState = new State(currState, i, currState->on[i], j, onV_goal, onV_goal_length, clearV_goal, numofclear_goal);
                        if(closed.find(nextState->id)==closed.end())
                            queue.push(nextState);
                    }
                }
            }
        }
    }

    *planlength = currState->gValue;
    *plan = (int**) malloc(*planlength * sizeof(int*));
    for (int i = *planlength - 1; i >= 0; i--){
        (*plan)[i] = (int*) malloc(4*sizeof(int));

        (*plan)[i][0] = (currState->to == currState->numItems);
        (*plan)[i][1] = currState->item;
        (*plan)[i][2] = currState->from;
        (*plan)[i][3] = currState->to;
        currState = currState->parent;
    }
    return;
}

//prhs contains input parameters (9): 
//1st is blocksV - is an array of block indices (that is, if b is in blocks, then b
//is a block
//2nd is trianglesV - is an array of triangle indices
//3rd is TableIndex - index that corresponds to the table
//4th is onV_start - a vector of On(x,y) statements that are true at start (each row is a statement, so
//OnV[0] says that item OnV[0][0] is on top of OnV[0][1]
//5th is clearV_start - is an array of items that are clear at start state (note Table is always clear
//by default)
//6th is onV_goal - a vector of On(x,y) statements that are true at goal 
//7th i clearV_goal - is an array of items that are clear at goal state 
//8th is moveActionIndex - index of the move(x,y,z) action that moves x from y to z
//(note that y could be a table but z should NOT be a table)
//9th is moveToTableActionIndex - index of the moveToTable(x,y,z) action that moves x
//from y to z, where z is ALWAYS an index of the table

//plhs should contain output parameters (1): 
//plan - an array of action with their parameters. plan(i,:) is ith action.
//plan[i][0] - index of the action, where plan[i][1] - first argument, plan[i][2] - second argument, plan[i][3] - third argument 
void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{
    /* Check for proper number of arguments */    
    if (nrhs != 9) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Nine input arguments required."); 
    } else if (nlhs != 1) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 

    int i = 0;

    /* get the blocks */
    double* blocksV_double = mxGetPr(BLOCKSV_IN);
    int numofblocks = (int) (MAX(mxGetM(BLOCKSV_IN), mxGetN(BLOCKSV_IN)));
    if(numofblocks < 2)
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumofBlocks",
                "At least two blocks are required.");         
    int *blocksV = (int*) malloc(sizeof(int)*numofblocks);
    for(i = 0; i < numofblocks; i++)
    {
        blocksV[i] = (int) blocksV_double[i];
    }

    /* get the triangles */
    double* trianglesV_double = mxGetPr(TRIANGLESV_IN);
    int numoftriangles = (int) (MAX(mxGetM(TRIANGLESV_IN), mxGetN(TRIANGLESV_IN)));
    int *trianglesV = (int*) malloc(sizeof(int)*numoftriangles);
    for(i = 0; i < numoftriangles; i++)
    {
        trianglesV[i] = (int) trianglesV_double[i];
    }

    /*get the table index */
    int TableIndex = (int)(*mxGetPr(TABLEINDEX_IN));

    /*get the onV for start*/
    int onV_start_length = (int) mxGetM(ONVSTART_IN);
    int onV_start_cols = (int) mxGetN(ONVSTART_IN);
    double* onv_start_double = mxGetPr(ONVSTART_IN);
    if(onV_start_cols != 2){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidonv_start",
                "each onv_start statement should have 3 parameters");         
    }

    int **onV_start = (int**) malloc(sizeof(int*)*onV_start_length);
    for(i = 0; i < onV_start_length; i++)
    {
        onV_start[i] = (int*)(malloc(sizeof(int)*2));
        onV_start[i][0] = (int)onv_start_double[0*onV_start_length + i];
        onV_start[i][1] = (int)onv_start_double[1*onV_start_length + i];
    }

    /*get the clearV for start*/
    double* clearV_start_double = mxGetPr(CLEARVSTART_IN);
    int numofclear_start = (int) (MAX(mxGetM(CLEARVSTART_IN), mxGetN(CLEARVSTART_IN)));
    int *clearstartV = (int*) malloc(sizeof(int)*numofclear_start);
    for(i = 0; i < numofclear_start; i++)
    {
        clearstartV[i] = (int) clearV_start_double[i];
    }

    /*get the onV for goal*/
    int onV_goal_length = (int) mxGetM(ONVGOAL_IN);
    int onV_goal_cols = (int) mxGetN(ONVGOAL_IN);
    double* onv_goal_double = mxGetPr(ONVGOAL_IN);
    if(onV_goal_cols != 2){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidonv_goal",
                "each onv_goal statement should have 3 parameters");         
    }

    int **onV_goal = (int**) malloc(sizeof(int*)*onV_goal_length);
    for(i = 0; i < onV_goal_length; i++)
    {
        onV_goal[i] = (int*) malloc(sizeof(int)*2);
        onV_goal[i][0] = (int)onv_goal_double[0*onV_goal_length + i];
        onV_goal[i][1] = (int)onv_goal_double[1*onV_goal_length + i];
    }

    /*get the clearV for goal*/
    double* clearV_goal_double = mxGetPr(CLEARVGOAL_IN);
    int numofclear_goal = (int) (MAX(mxGetM(CLEARVGOAL_IN), mxGetN(CLEARVGOAL_IN)));
    int *cleargoalV = (int*) malloc(sizeof(int)*numofclear_goal);
    for(i = 0; i < numofclear_goal; i++)
    {
        cleargoalV[i] = (int) clearV_goal_double[i];
    }
    
    /*get the moveAction index */
    int moveActionIndex = (int)(*mxGetPr(MOVEACTIONINDEX_IN));
           
    /*get the moveToTableAction index */
    int moveToTableActionIndex = (int)(*mxGetPr(MOVETOTABLEACTIONINDEX_IN));
    
    //call the planner
    int** plan = NULL;
    int planlength = 0;
    
    planner(blocksV, numofblocks, trianglesV, numoftriangles, TableIndex, 
            onV_start, onV_start_length, clearstartV, numofclear_start, onV_goal, onV_goal_length, cleargoalV, numofclear_goal, 
            moveActionIndex, moveToTableActionIndex, &plan, &planlength);

    /* Create return values */
    if(planlength > 0) {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)4, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
        int i,j;
        for(i = 0; i < planlength; i++) {
            for (j = 0; j < 4; j++) {
                plan_out[j*planlength + i] = (double)plan[i][j];
            }
        }
    }
    else {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);
        *plan_out = 0;
    }

    return;
}
