% Test case written by Fahad

blocksV = [0 1 3 5 6];
trianglesV = [2 4];
TableIndex = 7;
onV_start = [0 1; 2 0; 5 3; 6 5; 4 6];
clearV_start = [2 4];
onV_goal = [1 6; 0 1; 4 0];
clearV_goal = [4];
moveActionIndex = 0;
moveToTableActionIndex = 1;
runtest(blocksV, trianglesV, TableIndex, onV_start, clearV_start, onV_goal, clearV_goal, moveActionIndex, moveToTableActionIndex);

% Nima's result:

% plan of length 6 was found
% moveToTable triangle 4 from block 6 to table
% moveToTable triangle 2 from block 0 to table
% moveToTable block 0 from block 1 to table
% move block 1 from table to block 6
% move block 0 from table to block 1
% move triangle 4 from table to block 0