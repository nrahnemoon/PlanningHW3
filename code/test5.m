% Test case written by Nima

blocksV = [0 1 2 4 5 6 7];
trianglesV = [3 8];
TableIndex = 9;
onV_start = [2 1; 3 2; 5 4; 6 5; 7 6; 8 7];
clearV_start = [0 3 8];
onV_goal = [4 7; 0 4; 2 0; 1 2; 8 1; 6 5; 3 6];
clearV_goal = [ 3 8 ];
moveActionIndex = 0;
moveToTableActionIndex = 1;
runtest(blocksV, trianglesV, TableIndex, onV_start, clearV_start, onV_goal, clearV_goal, moveActionIndex, moveToTableActionIndex);

% Nima's result:

% plan of length 11 was found
% moveToTable triangle 8 from block 7 to table
% moveToTable block 7 from block 6 to table
% moveToTable block 6 from block 5 to table
% moveToTable block 5 from block 4 to table
% move block 4 from table to block 7
% move block 0 from table to block 4
% move block 6 from table to block 5
% move triangle 3 from block 2 to block 6
% move block 2 from block 1 to block 0
% move block 1 from table to block 2
% move triangle 8 from table to block 1
