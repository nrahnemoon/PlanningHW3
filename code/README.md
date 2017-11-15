Instead of using any of the structures provided in the skeleton code, I reimplemented everything from scratch using my own struct.  I did this because it would be a lot easier and faster.  The State struct essentially contains two main objects, the "on" array and the "clear" array.  Given an "item", "to", "from" and a parent state, it can generate a child State.

There are two main rules for getting a state's successor.  Those rules are simply:

1) Any item that is clear can be moved to table, unless it's already on table
2) Any item that is clear can be moved to another clear block

I used this logic to generate a state's successors.  To determine which state to explore next, I used a priority queue that pops the item with the lowest f-value.  The heuristic I used was simply how many items in the on array is different than the items in the clear array.  I thought I would have to implement a more complex heuristic like keeping track of all literals and determining the number of nodes that need to be popped until all the goal literals are reached, or removing all precondition conditions to determine how long it would take to reach the goal.  I did some back of the envelope calculations and realized that although these heuristics are better, they take a lot more time to compute.  In my opinion, the best heursitic is the simplest one.  While, my heuristic can't solve a complex 26 block problem, I don't think any of the other heurisitcs would solve it better.

Finally, I should also mention I keep track of what States I have closed so that I don't re-explore a closed state.  I only close a state when it is popped off the priority queue (as opposed to closing a state when I create it).  Why?  Because although I may reach a state quickly, it may not be the state with the lowest f-value... so, I need to wait until it's popped off the queue to close it.

To run my code, simply use the following commands:

> mex planner.cpp
> test1
> test2
> test5
> test6
 