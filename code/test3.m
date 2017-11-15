% Test case written by Nima

blocksV = [0 1 2 4 5 6 7 9 10 12 13 14 15 16 17 18 19 21 22 23 25];
trianglesV = [3 8 11 20 24];
TableIndex = 26;
onV_start = [2 1; 3 2; 5 4; 6 5; 7 6; 8 7; 10 9; 11 10; 13 12; 14 13; 16 15; 17 16; 18 17; 19 18; 20 19; 22 21; 23 22; 0 26; 1 26; 4 26; 9 26; 12 26; 15 26; 21 26; 24 26; 25 26 ];
clearV_start = [0 3 8 11 14 20 23 24 25];
onV_goal = [21 19; 12 21; 9 12; 24 9; 4 7; 13 4; 18 13; 3 18; 1 2; 22 1; 11 22];
clearV_goal = [24 3 11];
moveActionIndex = 0;
moveToTableActionIndex = 1;
runtest(blocksV, trianglesV, TableIndex, onV_start, clearV_start, onV_goal, clearV_goal, moveActionIndex, moveToTableActionIndex);

% Nima's result:

% Does not converge