% Test case written by Fahad

blocksV = [0 1 3];
trianglesV = [2];
TableIndex = 4;
onV_start = [0 1; 2 3];
clearV_start = [0 2];
onV_goal = [2 1];
clearV_goal = [2];
moveActionIndex = 0;
moveToTableActionIndex = 1;
runtest(blocksV, trianglesV, TableIndex, onV_start, clearV_start, onV_goal, clearV_goal, moveActionIndex, moveToTableActionIndex);

% Nima's result:

% plan of length 2 was found
% moveToTable block 0 from block 1 to table
% move triangle 2 from block 3 to block 1